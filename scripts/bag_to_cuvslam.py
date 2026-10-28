#!/usr/bin/env python3
"""
bag_to_cuvslam.py - Converts a ROS 2 bag (.db3 / .mcap) recorded with Intel RealSense
cameras (e.g. D455) into the EDEX format read by cuVSLAM (https://github.com/nvidia-isaac/cuVSLAM).

Does not require ROS: it reads the sqlite3 database (.db3) or the MCAP file directly and
decodes the required CDR messages itself (Image, CameraInfo, Imu, TFMessage).

For each camera (e.g. cam1, cam2) it produces a folder containing:

    <output>/<cam>/
    ├── stereo.edex             # rig: 0 = RGB (brown5k), 1 = infra1, 2 = infra2 (pinhole) [+ IMU]
    ├── frame_metadata.jsonl    # one frame per RGB image; IR present only when synchronized
    ├── IMU.jsonl               # only for the camera that owns the IMU (default: cam2)
    ├── imu.csv                 # same IMU data as human-readable CSV
    ├── sync.csv                # timestamps of each frame and RGB<->IR offset
    ├── calibration.json        # original camera_info and transforms used
    └── images/
        ├── rgb/000000.jpg ...
        ├── infra1/000000.png ...   (same index as the RGB frame they are matched to)
        └── infra2/000000.png ...

Synchronization (per camera, independent of the others):
  * all RGB frames are kept and define the EDEX frames (camera 0);
  * infra1 and infra2 are first paired with each other (tolerance --stereo-tolerance-ms);
  * each IR pair is matched to the nearest RGB frame within --tolerance-ms
    (1:1 matching, starting from the smallest offsets). Pairs with no nearby RGB
    are discarded; RGB frames without a pair stay RGB-only (cuVSLAM accepts partial frames).
  * The time used is the header stamp (default) or the bag receive time
    (--time receive). The same time is written to the EDEX and to the IMU.

Example:
    python3 bag_to_cuvslam.py ~/dotlumen/Captures/3_Staircases/staircases_102 \\
        --output ~/dotlumen/Captures/3_Staircases/staircases_102/edex

See `--help` for all options.
"""

import argparse
import bisect
import csv
import glob
import io
import json
import math
import os
import re
import shutil
import sqlite3
import statistics
import struct
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    sys.exit("OpenCV is missing: install the dependencies with `pip install -r requirements.txt`")

NS = 1_000_000_000


# =============================================================================
# CDR (XCDR1) decoding of the required messages
# =============================================================================
class Cdr:
    """Minimal CDR reader. Alignment is relative to the start of the payload
    (after the 4-byte encapsulation header)."""

    def __init__(self, buf):
        if len(buf) < 4:
            raise ValueError("CDR payload too short")
        self.buf = memoryview(buf)
        self.e = "<" if (buf[1] & 0x01) else ">"
        self.pos = 4

    def _align(self, n):
        r = (self.pos - 4) % n
        if r:
            self.pos += n - r

    def _unpack(self, fmt, size):
        self._align(size)
        v = struct.unpack_from(self.e + fmt, self.buf, self.pos)[0]
        self.pos += size
        return v

    def u8(self):
        v = self.buf[self.pos]
        self.pos += 1
        return v

    def i32(self):
        return self._unpack("i", 4)

    def u32(self):
        return self._unpack("I", 4)

    def f64(self):
        return self._unpack("d", 8)

    def f64s(self, n):
        self._align(8)
        v = struct.unpack_from(self.e + "%dd" % n, self.buf, self.pos)
        self.pos += 8 * n
        return list(v)

    def string(self):
        n = self.u32()
        s = bytes(self.buf[self.pos:self.pos + n]).split(b"\x00", 1)[0].decode("utf-8", "replace")
        self.pos += n
        return s

    def bytes_seq(self):
        n = self.u32()
        b = self.buf[self.pos:self.pos + n]
        self.pos += n
        return b

    def header(self):
        sec = self.i32()
        nsec = self.u32()
        frame_id = self.string()
        return sec * NS + nsec, frame_id


def header_stamp(prefix):
    """Header stamp from the first 12 bytes of the payload (or None)."""
    if prefix is None or len(prefix) < 12:
        return None
    e = "<" if (prefix[1] & 0x01) else ">"
    sec, nsec = struct.unpack_from(e + "iI", prefix, 4)
    if sec <= 0 or nsec >= NS:
        return None
    return sec * NS + nsec


def decode_image(buf):
    c = Cdr(buf)
    stamp, frame_id = c.header()
    height = c.u32()
    width = c.u32()
    encoding = c.string()
    _bigendian = c.u8()
    step = c.u32()
    data = c.bytes_seq()
    return dict(stamp=stamp, frame_id=frame_id, height=height, width=width,
                encoding=encoding, step=step, data=data)


def decode_camera_info(buf):
    c = Cdr(buf)
    stamp, frame_id = c.header()
    height = c.u32()
    width = c.u32()
    model = c.string()
    nd = c.u32()
    d = c.f64s(nd) if nd else []
    k = c.f64s(9)
    r = c.f64s(9)
    p = c.f64s(12)
    return dict(stamp=stamp, frame_id=frame_id, height=height, width=width,
                distortion_model=model, d=d, k=k, r=r, p=p)


def decode_imu(buf):
    c = Cdr(buf)
    stamp, frame_id = c.header()
    c.f64s(4)   # orientation
    c.f64s(9)   # orientation covariance
    gyro = c.f64s(3)
    c.f64s(9)
    acc = c.f64s(3)
    return dict(stamp=stamp, frame_id=frame_id, gyro=gyro, acc=acc)


def decode_tf(buf):
    c = Cdr(buf)
    n = c.u32()
    out = []
    for _ in range(n):
        _stamp, parent = c.header()
        child = c.string()
        t = c.f64s(3)
        q = c.f64s(4)  # x, y, z, w
        out.append((parent.lstrip("/"), child.lstrip("/"), t, q))
    return out


# =============================================================================
# Reading bags (.db3 / .mcap) without ROS
# =============================================================================
class Db3File:
    def __init__(self, path):
        self.path = path
        self.con = sqlite3.connect(f"file:{path}?mode=ro", uri=True, check_same_thread=False)
        self.topics = {}
        for tid, name, typ in self.con.execute("SELECT id, name, type FROM topics"):
            self.topics[tid] = (name, typ)
        self.blobopen = hasattr(self.con, "blobopen")

    def topic_types(self):
        return {n: t for n, t in self.topics.values()}

    def iterate(self, topics, prefix_only=False):
        """Generator of (topic, recv_ns, key, payload). With prefix_only the payload
        contains only the first 12 bytes (to read the stamp without loading images)."""
        ids = [tid for tid, (n, _) in self.topics.items() if n in topics]
        if not ids:
            return
        ph = ",".join("?" * len(ids))
        if prefix_only and self.blobopen:
            q = f"SELECT id, topic_id, timestamp FROM messages WHERE topic_id IN ({ph}) ORDER BY timestamp, id"
            for mid, tid, ts in self.con.execute(q, ids):
                with self.con.blobopen("messages", "data", mid, readonly=True) as b:
                    data = b.read(12)
                yield self.topics[tid][0], ts, (self.path, mid), data
        else:
            col = "substr(data, 1, 12)" if prefix_only else "data"
            q = (f"SELECT id, topic_id, timestamp, {col} FROM messages "
                 f"WHERE topic_id IN ({ph}) ORDER BY timestamp, id")
            for mid, tid, ts, data in self.con.execute(q, ids):
                yield self.topics[tid][0], ts, (self.path, mid), data

    def fetch(self, mid):
        row = self.con.execute("SELECT data FROM messages WHERE id=?", (mid,)).fetchone()
        return row[0] if row else None


class McapFile:
    MAGIC = b"\x89MCAP0\r\n"

    def __init__(self, path):
        self.path = path
        self._types = None

    @staticmethod
    def _str(buf, off):
        (n,) = struct.unpack_from("<I", buf, off)
        off += 4
        return bytes(buf[off:off + n]).decode("utf-8", "replace"), off + n

    @staticmethod
    def _decompress(comp, data, size):
        if comp == "":
            return data
        if comp == "zstd":
            try:
                import zstandard
            except ImportError:
                sys.exit("zstd-compressed MCAP: install `zstandard` (pip install -r requirements.txt)")
            return zstandard.ZstdDecompressor().decompress(bytes(data), max_output_size=size)
        if comp == "lz4":
            try:
                import lz4.frame
            except ImportError:
                sys.exit("lz4-compressed MCAP: install `lz4` (pip install -r requirements.txt)")
            return lz4.frame.decompress(bytes(data))
        sys.exit(f"Unsupported MCAP compression: {comp}")

    def _records(self, stream):
        while True:
            head = stream.read(9)
            if len(head) < 9:
                return
            op, length = struct.unpack("<BQ", head)
            yield op, stream.read(length)

    def _messages(self):
        schemas, channels = {}, {}

        def handle(op, body):
            if op == 0x03:
                sid, = struct.unpack_from("<H", body, 0)
                schemas[sid], _ = self._str(body, 2)
            elif op == 0x04:
                cid, sid = struct.unpack_from("<HH", body, 0)
                topic, off = self._str(body, 4)
                enc, _ = self._str(body, off)
                channels[cid] = (topic, schemas.get(sid, "?"), enc)
                yield ("channel", channels[cid])
            elif op == 0x05:
                cid, _seq, log_time, _pub = struct.unpack_from("<HIQQ", body, 0)
                if cid in channels:
                    yield ("msg", (channels[cid][0], log_time, body[22:]))
            elif op == 0x06:
                off = 16
                size, = struct.unpack_from("<Q", body, off)
                off += 12
                comp, off = self._str(body, off)
                rlen, = struct.unpack_from("<Q", body, off)
                off += 8
                inner = io.BytesIO(self._decompress(comp, body[off:off + rlen], size))
                for op2, body2 in self._records(inner):
                    yield from handle(op2, body2)

        with open(self.path, "rb") as f:
            if f.read(8) != self.MAGIC:
                sys.exit(f"{self.path}: not a valid MCAP file")
            for op, body in self._records(f):
                if op == 0x02:  # footer
                    break
                yield from handle(op, body)

    def topic_types(self):
        if self._types is None:
            self._types = {}
            for kind, val in self._messages():
                if kind == "channel":
                    self._types[val[0]] = val[1]
        return self._types

    def iterate(self, topics, prefix_only=False):
        counters = {}
        for kind, val in self._messages():
            if kind != "msg" or val[0] not in topics:
                continue
            topic, log_time, data = val
            k = counters.get(topic, 0)
            counters[topic] = k + 1
            yield topic, log_time, (self.path, topic, k), (data[:12] if prefix_only else data)


class Bag:
    def __init__(self, path):
        path = os.path.expanduser(path)
        if os.path.isdir(path):
            files = sorted(glob.glob(os.path.join(path, "*.db3")) +
                           glob.glob(os.path.join(path, "*.mcap")))
        elif os.path.isfile(path):
            files = [path]
        else:
            sys.exit(f"Bag not found: {path}")
        if not files:
            sys.exit(f"No .db3/.mcap files in {path}")
        self.files = [Db3File(f) if f.endswith(".db3") else McapFile(f) for f in files]

    def topic_types(self):
        out = {}
        for f in self.files:
            out.update(f.topic_types())
        return out

    def iterate(self, topics, prefix_only=False):
        for f in self.files:
            yield from f.iterate(topics, prefix_only)


# =============================================================================
# Geometry: static TFs and EDEX conventions
# =============================================================================
def quat_to_rot(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TfTree:
    def __init__(self):
        self.edges = {}  # frame -> list of (other, T_frame_from_other)

    def add(self, parent, child, t, q):
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(q)
        T[:3, 3] = t
        self.edges.setdefault(parent, []).append((child, T))              # parent_from_child
        self.edges.setdefault(child, []).append((parent, np.linalg.inv(T)))

    def frames(self):
        return sorted(self.edges)

    def lookup(self, target, source):
        """T_target_from_source (maps points from the source frame to the target frame)."""
        target, source = target.lstrip("/"), source.lstrip("/")
        if target == source:
            return np.eye(4)
        if target not in self.edges or source not in self.edges:
            return None
        # BFS from target: T_target_from_X
        seen = {target: np.eye(4)}
        queue = [target]
        while queue:
            cur = queue.pop(0)
            for nxt, T_cur_from_nxt in self.edges.get(cur, []):
                if nxt in seen:
                    continue
                seen[nxt] = seen[cur] @ T_cur_from_nxt
                if nxt == source:
                    return seen[nxt]
                queue.append(nxt)
        return None


# EDEX files store extrinsics in the legacy cuVSLAM frame (y up, z backward),
# while ROS uses OpenCV optical frames (y down, z forward). See
# libs/common/coordinate_system.h (LegacyEdexIsometryToOpenCV / LegacyEdexImuExtrinsicToOpenCV).
K_FLIP = np.diag([1.0, -1.0, -1.0])


def edex_camera_transform(T_rig_from_cam_opencv):
    R = T_rig_from_cam_opencv[:3, :3]
    t = T_rig_from_cam_opencv[:3, 3]
    R2 = K_FLIP @ R @ K_FLIP
    t2 = K_FLIP @ t
    return [[float(R2[i, 0]), float(R2[i, 1]), float(R2[i, 2]), float(t2[i])] for i in range(3)]


def edex_imu_transform(T_rig_from_imu_opencv):
    # Only the target frame (rig) changes convention; the IMU stays in its own frame.
    M = np.eye(4)
    M[:3, :3] = K_FLIP
    T = M @ T_rig_from_imu_opencv
    return [[float(T[i, j]) for j in range(4)] for i in range(3)]


def ros_d_to_brown5k(d):
    """ROS plumb_bob: [k1, k2, p1, p2, k3] -> cuVSLAM brown5k: [k1, k2, k3, p1, p2]."""
    d = list(d) + [0.0] * (5 - len(d))
    k1, k2, p1, p2, k3 = d[:5]
    return [k1, k2, k3, p1, p2]


# =============================================================================
# Image conversion
# =============================================================================
def image_to_cv(img):
    enc = img["encoding"].lower()
    h, w, step = img["height"], img["width"], img["step"]
    raw = np.frombuffer(img["data"], dtype=np.uint8)
    if enc in ("mono8", "8uc1"):
        return raw.reshape(h, step)[:, :w]
    if enc in ("rgb8", "bgr8", "8uc3"):
        a = raw.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if enc == "rgb8" else a
    if enc in ("rgba8", "bgra8", "8uc4"):
        a = raw.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR)
    if enc in ("mono16", "16uc1"):
        a = raw.reshape(h, step)[:, :w * 2].copy().view("<u2").reshape(h, w)
        return (a >> 8).astype(np.uint8)
    if enc in ("yuv422_yuy2", "yuyv", "yuv422"):
        a = raw.reshape(h, step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(a, cv2.COLOR_YUV2BGR_YUYV if enc != "yuv422" else cv2.COLOR_YUV2BGR_UYVY)
    if enc == "uyvy":
        a = raw.reshape(h, step)[:, :w * 2].reshape(h, w, 2)
        return cv2.cvtColor(a, cv2.COLOR_YUV2BGR_UYVY)
    raise ValueError(f"unsupported image encoding: {img['encoding']}")


# =============================================================================
# Synchronization
# =============================================================================
def dedupe(items):
    """items: list of (stamp, recv, key) -> sorted by stamp, without duplicate stamps."""
    items = sorted(items, key=lambda x: (x[0], x[1]))
    out = []
    for it in items:
        if out and it[0] == out[-1][0]:
            continue
        out.append(it)
    return out


def nearest(sorted_stamps, t):
    i = bisect.bisect_left(sorted_stamps, t)
    best = None
    for j in (i - 1, i):
        if 0 <= j < len(sorted_stamps):
            d = abs(sorted_stamps[j] - t)
            if best is None or d < best[1]:
                best = (j, d)
    return best


def greedy_match(a_stamps, b_stamps, tol_ns):
    """1:1 matching between two sorted lists, starting from the smallest offsets. Returns {i_a: i_b}."""
    cands = []
    for ib, tb in enumerate(b_stamps):
        i = bisect.bisect_left(a_stamps, tb)
        for ia in (i - 1, i, i + 1):
            if 0 <= ia < len(a_stamps):
                d = abs(a_stamps[ia] - tb)
                if d <= tol_ns:
                    cands.append((d, ia, ib))
    cands.sort()
    used_a, used_b, out = set(), set(), {}
    for d, ia, ib in cands:
        if ia in used_a or ib in used_b:
            continue
        used_a.add(ia)
        used_b.add(ib)
        out[ia] = ib
    return out


def stats_ms(values_ns):
    if not values_ns:
        return "n/a"
    v = sorted(abs(x) / 1e6 for x in values_ns)
    p = lambda q: v[min(len(v) - 1, int(round(q * (len(v) - 1))))]
    return f"median {statistics.median(v):.2f} ms, p95 {p(0.95):.2f} ms, max {v[-1]:.2f} ms"


# =============================================================================
# Main
# =============================================================================
def parse_args():
    ap = argparse.ArgumentParser(
        description="Converts a RealSense ROS 2 bag into the cuVSLAM EDEX format (without ROS).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("bag", help="bag folder (containing the .db3/.mcap files) or a single file")
    ap.add_argument("-o", "--output", required=True, help="output folder")
    ap.add_argument("--cameras", nargs="+", default=None,
                    help="camera prefixes (default: detected from /<cam>/color/image_raw)")
    ap.add_argument("--rgb-topic", default="/{cam}/color/image_raw")
    ap.add_argument("--rgb-info-topic", default="/{cam}/color/camera_info")
    ap.add_argument("--ir1-topic", default="/{cam}/infra1/image_rect_raw")
    ap.add_argument("--ir1-info-topic", default="/{cam}/infra1/camera_info")
    ap.add_argument("--ir2-topic", default="/{cam}/infra2/image_rect_raw")
    ap.add_argument("--ir2-info-topic", default="/{cam}/infra2/camera_info")
    ap.add_argument("--imu-camera", default="cam2",
                    help="camera that owns the IMU ('none' to skip exporting it)")
    ap.add_argument("--imu-topic", default="/{cam}/imu")
    ap.add_argument("--time", choices=["header", "receive"], default="header",
                    help="time used for synchronization and written timestamps")
    ap.add_argument("--tolerance-ms", type=float, default=10.0,
                    help="maximum RGB <-> IR pair offset")
    ap.add_argument("--stereo-tolerance-ms", type=float, default=1.0,
                    help="maximum infra1 <-> infra2 offset")
    ap.add_argument("--start", type=float, default=None,
                    help="seconds from the start of the bag at which to start")
    ap.add_argument("--end", type=float, default=None,
                    help="seconds from the start of the bag at which to stop")
    ap.add_argument("--jpg-quality", type=int, default=95)
    ap.add_argument("--png-compression", type=int, default=6, choices=range(0, 10),
                    metavar="0-9", help="PNG compression level (lossless; 9 = smallest files)")
    ap.add_argument("--ir-format", choices=["png", "jpg"], default="png")
    ap.add_argument("--gyro-noise-density", type=float, default=0.000244)
    ap.add_argument("--gyro-random-walk", type=float, default=0.000019393)
    ap.add_argument("--accel-noise-density", type=float, default=0.001862)
    ap.add_argument("--accel-random-walk", type=float, default=0.003)
    ap.add_argument("--imu-frequency", type=float, default=None,
                    help="IMU frequency for the EDEX (default: estimated from the data)")
    ap.add_argument("--workers", type=int, default=max(2, min(8, os.cpu_count() or 4)),
                    help="threads for image encoding")
    ap.add_argument("--overwrite", action="store_true", help="overwrite the output folder")
    ap.add_argument("--stale-threshold", type=float, default=0.5,
                    help="drop messages whose stamp-receive offset deviates from the median "
                         "by more than N seconds (0 = disabled)")
    ap.add_argument("--reference", choices=["rgb", "ir", "auto"], default="rgb",
                    help="stream that defines the frames (camera 0): rgb, ir pair, or auto = the most frequent")
    ap.add_argument("--dry-run", action="store_true",
                    help="analyze and synchronize without writing images")
    return ap.parse_args()


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def main():
    args = parse_args()
    t_start = time.time()
    bag = Bag(args.bag)
    types = bag.topic_types()

    # --- cameras ------------------------------------------------------------
    if args.cameras:
        cameras = args.cameras
    else:
        rx = re.compile("^" + re.escape(args.rgb_topic).replace(re.escape("{cam}"), "([^/]+)") + "$")
        cameras = sorted(m.group(1) for t in types for m in [rx.match(t)] if m)
    if not cameras:
        sys.exit("No cameras found: use --cameras and/or --rgb-topic")
    imu_cam = None if args.imu_camera.lower() == "none" else args.imu_camera
    if imu_cam and imu_cam not in cameras:
        sys.exit(f"--imu-camera {imu_cam} is not among the cameras {cameras}")

    streams = {}  # (cam, role) -> topic
    for cam in cameras:
        for role, tmpl in (("rgb", args.rgb_topic), ("rgb_info", args.rgb_info_topic),
                           ("ir1", args.ir1_topic), ("ir1_info", args.ir1_info_topic),
                           ("ir2", args.ir2_topic), ("ir2_info", args.ir2_info_topic)):
            topic = tmpl.format(cam=cam)
            if topic in types:
                streams[(cam, role)] = topic
            elif role in ("rgb", "rgb_info"):
                sys.exit(f"Required topic missing: {topic}")
            else:
                log(f"WARNING: topic {topic} missing, camera {cam} will have no {role}")
    imu_topic = args.imu_topic.format(cam=imu_cam) if imu_cam else None
    if imu_topic and imu_topic not in types:
        sys.exit(f"IMU topic missing: {imu_topic}")

    out_root = os.path.expanduser(args.output)
    if os.path.exists(out_root) and os.listdir(out_root) and not args.dry_run:
        if not args.overwrite:
            sys.exit(f"Output folder {out_root} exists and is not empty (use --overwrite)")

    log(f"Bag: {', '.join(os.path.basename(f.path) for f in bag.files)}")
    log(f"Cameras: {', '.join(cameras)}   IMU: {imu_topic or 'none'}   time: {args.time}")

    # --- pass 1: metadata, timestamps, IMU, TF -------------------------------
    image_topics = {streams[k] for k in streams if not k[1].endswith("_info")}
    info_topics = {streams[k] for k in streams if k[1].endswith("_info")}
    small_topics = set(info_topics) | ({imu_topic} if imu_topic else set()) | \
        ({"/tf_static"} if "/tf_static" in types else set())

    log("Pass 1/2: reading timestamps and metadata ...")
    stamps = {t: [] for t in image_topics}
    infos, imu_samples, tf = {}, [], TfTree()
    imu_recv = []
    imu_frame = None
    bag_t0 = None
    bad_header = 0
    for topic, recv, key, data in bag.iterate(image_topics, prefix_only=True):
        bag_t0 = recv if bag_t0 is None else min(bag_t0, recv)
        st = header_stamp(data) if args.time == "header" else recv
        if st is None:
            bad_header += 1
            st = recv
        stamps[topic].append((st, recv, key))
    for topic, recv, key, data in bag.iterate(small_topics):
        bag_t0 = recv if bag_t0 is None else min(bag_t0, recv)
        if topic == "/tf_static":
            for parent, child, t, q in decode_tf(data):
                tf.add(parent, child, t, q)
        elif topic in info_topics:
            if topic not in infos:
                infos[topic] = decode_camera_info(data)
        elif topic == imu_topic:
            m = decode_imu(data)
            imu_frame = imu_frame or m["frame_id"]
            st = m["stamp"] if args.time == "header" and m["stamp"] > 0 else recv
            imu_samples.append((st, m["gyro"], m["acc"]))
            imu_recv.append((st, recv))
    if bad_header:
        log(f"WARNING: {bad_header} images without a valid stamp, using receive time")

    # "stale" messages: stamp far behind (or ahead of) the receive time, typically the first
    # frame left queued in the driver when recording started. Keeping them creates gaps of
    # tens of seconds at the start of the sequence.
    stale_dropped = {}
    if args.time == "header" and args.stale_threshold > 0:
        thr = int(args.stale_threshold * NS)
        for t, lst in stamps.items():
            if len(lst) < 3:
                continue
            med = statistics.median(st - rc for st, rc, _ in lst)
            keep = [x for x in lst if abs((x[0] - x[1]) - med) <= thr]
            if len(keep) != len(lst):
                stale_dropped[t] = len(lst) - len(keep)
                log(f"WARNING: {t}: dropped {len(lst) - len(keep)} messages with anomalous stamp "
                    f"(stamp-receive offset more than {args.stale_threshold} s from the median)")
                stamps[t] = keep
        if len(imu_recv) > 2:
            med = statistics.median(st - rc for st, rc in imu_recv)
            keep = [i for i, (st, rc) in enumerate(imu_recv) if abs((st - rc) - med) <= thr]
            if len(keep) != len(imu_samples):
                stale_dropped[imu_topic] = len(imu_samples) - len(keep)
                log(f"WARNING: {imu_topic}: dropped {len(imu_samples) - len(keep)} samples with anomalous stamp")
                imu_samples = [imu_samples[i] for i in keep]

    t_lo = bag_t0 + int(args.start * NS) if args.start is not None else None
    t_hi = bag_t0 + int(args.end * NS) if args.end is not None else None

    jobs = []           # (key, out_path, kind, cam, role)
    summary = []
    bag_report = {"bag": os.path.abspath(os.path.expanduser(args.bag)),
                  "files": [os.path.basename(f.path) for f in bag.files],
                  "time_base": args.time, "topics": {}, "cameras": {}, "stale_dropped": stale_dropped}

    # continuity statistics for each image topic and for the exported IMU
    def gap_stats(st_list):
        st = sorted(set(st_list))
        if len(st) < 3:
            return {"count": len(st)}
        dts = [b - a for a, b in zip(st, st[1:])]
        per = statistics.median(dts)
        gaps = [d for d in dts if d > 1.5 * per]
        lost = sum(max(1, int(round(d / per)) - 1) for d in gaps)
        return {"count": len(st), "duration_s": (st[-1] - st[0]) / NS, "rate_hz": NS / per,
                "period_ms": per / 1e6, "gaps": len(gaps), "max_gap_ms": max(dts) / 1e6,
                "est_lost": lost, "loss_pct": 100.0 * lost / (len(st) + lost)}
    for t, lst in stamps.items():
        bag_report["topics"][t] = gap_stats([x[0] for x in lst])
    if imu_samples:
        bag_report["topics"][imu_topic] = gap_stats([x[0] for x in imu_samples])
    for t in info_topics:
        bag_report["topics"].setdefault(t, {})["camera_info"] = t in infos

    def in_range(lst):
        if t_lo is not None:
            lst = [x for x in lst if x[1] >= t_lo]
        if t_hi is not None:
            lst = [x for x in lst if x[1] <= t_hi]
        return lst

    for cam in cameras:
        cam_dir = os.path.join(out_root, cam)
        rgb = in_range(dedupe(stamps[streams[(cam, "rgb")]]))
        ir1 = dedupe(stamps.get(streams.get((cam, "ir1")), []))
        ir2 = dedupe(stamps.get(streams.get((cam, "ir2")), []))

        # infra1/infra2 pairs
        pairs = []
        if ir1 and ir2:
            m12 = greedy_match([x[0] for x in ir1], [x[0] for x in ir2],
                               int(args.stereo_tolerance_ms * 1e6))
            pairs = sorted((ir1[i], ir2[j]) for i, j in m12.items())
            # --start/--end range on infra1 receive time
            if t_lo is not None:
                pairs = [p for p in pairs if p[0][1] >= t_lo]
            if t_hi is not None:
                pairs = [p for p in pairs if p[0][1] <= t_hi]
        if not rgb and not pairs:
            log(f"WARNING: no frames for {cam} in the requested range, skipping")
            continue

        # reference stream (camera 0)
        ref = args.reference
        if ref == "auto":
            ref = "ir" if len(pairs) > len(rgb) else "rgb"
        if ref == "ir" and not pairs:
            log(f"[{cam}] no IR pairs: using RGB as reference")
            ref = "rgb"
        if ref == "rgb" and not rgb:
            ref = "ir"

        rgb_stamps = [x[0] for x in rgb]
        pair_stamps = [p[0][0] for p in pairs]
        tol = int(args.tolerance_ms * 1e6)
        if ref == "rgb":
            ref_items = rgb
            m_assoc = greedy_match(rgb_stamps, pair_stamps, tol)       # rgb index -> pair index
            order = ["rgb"] + (["ir1", "ir2"] if pairs else [])
        else:
            ref_items = pairs
            m_assoc = greedy_match(pair_stamps, rgb_stamps, tol)       # pair index -> rgb index
            order = ["ir1", "ir2"] + (["rgb"] if rgb else [])
        ref_stamps = rgb_stamps if ref == "rgb" else pair_stamps
        assoc_stamps = pair_stamps if ref == "rgb" else rgb_stamps
        deltas = [assoc_stamps[j] - ref_stamps[i] for i, j in m_assoc.items()]
        raw_d = []
        for ts_ in assoc_stamps:
            nb = nearest(ref_stamps, ts_)
            if nb:
                raw_d.append(nb[1])

        # --- calibration --------------------------------------------------
        info_ref = infos.get(streams.get((cam, "rgb_info" if ref == "rgb" else "ir1_info")))
        if info_ref is None:
            sys.exit(f"No camera_info for the reference of {cam}")
        rig_frame = info_ref["frame_id"].lstrip("/")
        cams_edex, calib = [], {"rig_frame": rig_frame, "reference": ref, "cameras": [], "imu": None}
        for idx, role in enumerate(order):
            info_role = role + "_info"
            info = infos.get(streams.get((cam, info_role)))
            if info is None:
                sys.exit(f"No camera_info for {cam}/{role}")
            frame = info["frame_id"].lstrip("/")
            T = tf.lookup(rig_frame, frame)
            if T is None:
                sys.exit(f"Transform {rig_frame} <- {frame} not found in /tf_static.\n"
                         f"Available frames: {', '.join(tf.frames()) or 'none'}")
            note = None
            if role == "ir2":
                # ROS stereo convention (followed by realsense-ros): the right camera's camera_info
                # has the frame_id of the rectified LEFT camera and the baseline in
                # P[3] = -fx * B. In that case the right camera sits at +B along x of the left frame.
                ir1_info = infos.get(streams.get((cam, "ir1_info")))
                p = info["p"]
                same_frame = ir1_info is not None and \
                    ir1_info["frame_id"].lstrip("/") == frame
                if same_frame:
                    if abs(p[3]) < 1e-9 or p[0] == 0:
                        sys.exit(f"{cam}/infra2 has the same frame as infra1 but P[3] = 0: "
                                 f"cannot derive the stereo baseline")
                    baseline = -p[3] / p[0]
                    T_shift = np.eye(4)
                    T_shift[0, 3] = baseline
                    T = T @ T_shift
                    note = f"baseline {baseline:.5f} m from P[3] (frame_id = {frame})"
                    log(f"[{cam}] infra2: baseline {baseline * 1000:.2f} mm derived from P "
                        f"(frame_id = {frame})")
            k = info["k"]
            if role == "rgb":
                model = info["distortion_model"]
                if model not in ("plumb_bob", "rational_polynomial", ""):
                    log(f"WARNING: {cam} RGB has model '{model}', written as brown5k")
                intr = dict(distortion_model="brown5k", distortion_params=ros_d_to_brown5k(info["d"]),
                            focal=[k[0], k[4]], principal=[k[2], k[5]],
                            size=[info["width"], info["height"]])
            else:
                if any(abs(x) > 1e-9 for x in info["d"]):
                    log(f"WARNING: {cam}/{role} has non-zero distortion coefficients "
                        f"but is a rectified image: written as pinhole")
                intr = dict(distortion_model="pinhole", distortion_params=[],
                            focal=[k[0], k[4]], principal=[k[2], k[5]],
                            size=[info["width"], info["height"]])
            cams_edex.append({"intrinsics": intr, "transform": edex_camera_transform(T)})
            entry = {"id": idx, "role": role, "topic": streams[(cam, role)], "frame_id": frame,
                     "camera_info": {kk: info[kk] for kk in ("width", "height", "distortion_model",
                                                             "d", "k", "r", "p")},
                     "T_rig_from_camera_opencv": T.tolist()}
            if note:
                entry["note"] = note
            calib["cameras"].append(entry)

        header = {"version": "0.9", "frame_start": 0, "frame_end": len(ref_items) - 1,
                  "cameras": cams_edex}

        # --- IMU -------------------------------------------------------------
        imu_rows = []
        if imu_cam == cam and imu_samples:
            cand = [imu_frame, f"{cam}_imu_optical_frame", f"{cam}_gyro_optical_frame"]
            T_imu, used = None, None
            for fr in cand:
                if fr:
                    T_imu = tf.lookup(rig_frame, fr)
                    if T_imu is not None:
                        used = fr
                        break
            if T_imu is None:
                sys.exit(f"Transform {rig_frame} <- {imu_frame} (IMU) not found in /tf_static.\n"
                         f"Available frames: {', '.join(tf.frames())}")
            if used != imu_frame:
                log(f"WARNING: IMU frame '{imu_frame}' not in /tf_static, using '{used}'")
            lo = ref_stamps[0] - NS // 2
            hi = ref_stamps[-1] + NS // 2
            seen = set()
            for st, g, a in sorted(imu_samples, key=lambda x: x[0]):
                if lo <= st <= hi and st not in seen:
                    seen.add(st)
                    imu_rows.append((st, g, a))
            if len(imu_rows) > 1:
                dts = [b[0] - a[0] for a, b in zip(imu_rows, imu_rows[1:])]
                freq = args.imu_frequency or round(NS / statistics.median(dts))
            else:
                freq = args.imu_frequency or 200.0
            header["imu"] = {
                "measurements": "IMU.jsonl",
                "transform": edex_imu_transform(T_imu),
                "coordinate_system": "opencv",
                # keys read by the C++ code (gyro_/accel_) and by the Python tools (gyroscope_/accelerometer_)
                "gyro_noise_density": args.gyro_noise_density,
                "gyro_random_walk": args.gyro_random_walk,
                "accel_noise_density": args.accel_noise_density,
                "accel_random_walk": args.accel_random_walk,
                "gyroscope_noise_density": args.gyro_noise_density,
                "gyroscope_random_walk": args.gyro_random_walk,
                "accelerometer_noise_density": args.accel_noise_density,
                "accelerometer_random_walk": args.accel_random_walk,
                "frequency": float(freq),
            }
            calib["imu"] = {"topic": imu_topic, "frame_id": used, "T_rig_from_imu_opencv": T_imu.tolist(),
                            "frequency": float(freq)}

        ir_ext = args.ir_format
        ext = {"rgb": "jpg", "ir1": ir_ext, "ir2": ir_ext}
        sub = {"rgb": "rgb", "ir1": "infra1", "ir2": "infra2"}
        seq = [[f"images/{sub[r]}/000000.{ext[r]}"] for r in order]
        body = {"frame_metadata": "frame_metadata.jsonl", "sequence": seq}

        s_entry = dict(cam=cam, reference=ref, order=order, frames=len(ref_items), rgb=len(rgb),
                       ir_pairs=len(pairs), matched=len(m_assoc), ir1=len(ir1), ir2=len(ir2),
                       imu=len(imu_rows), raw=stats_ms(raw_d), used=stats_ms(deltas),
                       delta_abs_ms=[abs(d) / 1e6 for d in deltas],
                       raw_abs_ms=[abs(d) / 1e6 for d in raw_d])
        summary.append(s_entry)

        if args.dry_run:
            continue

        # --- write metadata -----------------------------------------------
        if os.path.isdir(cam_dir) and args.overwrite:
            shutil.rmtree(cam_dir)
        for r in order:
            os.makedirs(os.path.join(cam_dir, "images", sub[r]), exist_ok=True)
        with open(os.path.join(cam_dir, "stereo.edex"), "w") as f:
            json.dump([header, body], f, indent=2)
        with open(os.path.join(cam_dir, "calibration.json"), "w") as f:
            json.dump(calib, f, indent=2)

        def item_of(role, i):
            """(stamp, recv, key) of the role for reference frame i, or None."""
            if ref == "rgb":
                if role == "rgb":
                    return rgb[i]
                if i not in m_assoc:
                    return None
                return pairs[m_assoc[i]][0 if role == "ir1" else 1]
            if role in ("ir1", "ir2"):
                return pairs[i][0 if role == "ir1" else 1]
            return rgb[m_assoc[i]] if i in m_assoc else None

        with open(os.path.join(cam_dir, "frame_metadata.jsonl"), "w") as fm, \
                open(os.path.join(cam_dir, "sync.csv"), "w", newline="") as fs:
            w = csv.writer(fs)
            w.writerow(["frame_id", "reference", "rgb_stamp_ns", "infra1_stamp_ns", "infra2_stamp_ns",
                        "ref_recv_ns", "delta_rgb_ir_ms"])
            for i in range(len(ref_items)):
                name = f"{i:06d}"
                cams_meta, row_st = [], {}
                for idx, r in enumerate(order):
                    it = item_of(r, i)
                    if it is None:
                        continue
                    fn = f"images/{sub[r]}/{name}.{ext[r]}"
                    cams_meta.append({"id": idx, "filename": fn, "timestamp": int(it[0])})
                    jobs.append((it[2], os.path.join(cam_dir, fn), r, cam))
                    row_st[r] = it[0]
                ref_recv = item_of(order[0], i)[1]
                d = ""
                if "rgb" in row_st and "ir1" in row_st:
                    d = f"{(row_st['ir1'] - row_st['rgb']) / 1e6:.3f}"
                w.writerow([i, ref, row_st.get("rgb", ""), row_st.get("ir1", ""), row_st.get("ir2", ""),
                            ref_recv, d])
                fm.write(json.dumps({"frame_id": i, "cams": cams_meta}) + "\n")
        if imu_rows:
            with open(os.path.join(cam_dir, "IMU.jsonl"), "w") as fj, \
                    open(os.path.join(cam_dir, "imu.csv"), "w", newline="") as fc:
                w = csv.writer(fc)
                w.writerow(["timestamp_ns", "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z"])
                for st, g, a in imu_rows:
                    fj.write(json.dumps({
                        "AngularVelocityX": g[0], "AngularVelocityY": g[1], "AngularVelocityZ": g[2],
                        "LinearAccelerationX": a[0], "LinearAccelerationY": a[1],
                        "LinearAccelerationZ": a[2], "timestamp": int(st)}) + "\n")
                    w.writerow([st] + list(g) + list(a))

    # --- synchronization summary --------------------------------------------
    log("")
    for s in summary:
        log(f"[{s['cam']}] reference: {s['reference']} ({s['frames']} frames, cameras {s['order']})   "
            f"RGB: {s['rgb']}   infra1: {s['ir1']}   infra2: {s['ir2']}   IR pairs: {s['ir_pairs']}   "
            f"matches: {s['matched']}   IMU: {s['imu']}")
        log(f"        nearest RGB <-> IR offset: {s['raw']}")
        log(f"        offset in matches used (<= {args.tolerance_ms} ms): {s['used']}")
    log("Note: cuVSLAM uses the camera 0 timestamp for the whole frame; the documentation\n"
        "      asks for offsets <= 1 ms between 'nearly synchronized' cameras. Choose --tolerance-ms accordingly.")

    if args.dry_run:
        log("\n--dry-run: no files written.")
        return

    # --- pass 2: images ------------------------------------------------------
    log(f"\nPass 2/2: writing {len(jobs)} images with {args.workers} threads ...")
    by_key = {}
    for key, path, kind, cam_ in jobs:
        by_key.setdefault(key, []).append((path, kind, cam_))
    quality = {}  # (cam, role) -> list of (mean, saturated fraction, black fraction, Laplacian variance)
    mcap_topics = image_topics
    jpg_params = [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality]
    png_params = [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression]
    errors = []
    done = [0]
    lock = threading.Lock()
    slots = threading.BoundedSemaphore(args.workers * 4)

    def work(payload, targets):
        try:
            img = image_to_cv(decode_image(payload))
            gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            q = (float(gray.mean()), float((gray >= 250).mean()), float((gray <= 5).mean()),
                 float(cv2.Laplacian(gray, cv2.CV_64F).var()))
            with lock:
                for _p, kind_, cam_ in targets:
                    quality.setdefault((cam_, kind_), []).append(q)
            for path, kind, _cam in targets:
                if kind == "rgb" or args.ir_format == "jpg":
                    ok = cv2.imwrite(path, img, jpg_params)
                else:
                    ok = cv2.imwrite(path, img, png_params)
                if not ok:
                    raise IOError(f"write failed: {path}")
        except Exception as e:  # noqa: BLE001
            with lock:
                errors.append(f"{targets[0][0]}: {e}")
        finally:
            slots.release()
            with lock:
                done[0] += 1
                if done[0] % 200 == 0 or done[0] == len(by_key):
                    log(f"  {done[0]}/{len(by_key)} images")

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for f in bag.files:
            if isinstance(f, Db3File):
                mine = sorted(k for k in by_key if k[0] == f.path)
                for k in mine:
                    slots.acquire()
                    ex.submit(work, f.fetch(k[1]), by_key[k])
            else:
                for _topic, _ts, k, data in f.iterate(mcap_topics):
                    if k in by_key:
                        slots.acquire()
                        ex.submit(work, data, by_key[k])

    if errors:
        log(f"\n{len(errors)} errors while writing images, first ones:")
        for e in errors[:10]:
            log("  " + e)
        sys.exit(1)
    # --- analysis report ------------------------------------------------------
    def pct(v, q):
        v = sorted(v)
        return v[min(len(v) - 1, int(round(q * (len(v) - 1))))] if v else None
    for s_ in summary:
        c = s_["cam"]
        qd = {}
        for r in s_["order"]:
            vals = quality.get((c, r), [])
            if vals:
                cols = list(zip(*vals))
                qd[r] = {"n": len(vals),
                         "mean_median": pct(cols[0], 0.5),
                         "saturated_frac_median": pct(cols[1], 0.5),
                         "saturated_frac_p90": pct(cols[1], 0.9),
                         "frames_over_50pct_saturated": sum(x > 0.5 for x in cols[1]) / len(vals),
                         "black_frac_median": pct(cols[2], 0.5),
                         "frames_over_50pct_black": sum(x > 0.5 for x in cols[2]) / len(vals),
                         "laplacian_var_median": pct(cols[3], 0.5),
                         "laplacian_var_p10": pct(cols[3], 0.1)}
        bag_report["cameras"][c] = {k: v for k, v in s_.items()
                                    if k not in ("raw", "used", "delta_abs_ms", "raw_abs_ms")}
        bag_report["cameras"][c]["delta_ms"] = {"median": pct(s_["delta_abs_ms"], 0.5),
                                                "p95": pct(s_["delta_abs_ms"], 0.95),
                                                "max": max(s_["delta_abs_ms"]) if s_["delta_abs_ms"] else None}
        bag_report["cameras"][c]["nearest_ms"] = {"median": pct(s_["raw_abs_ms"], 0.5),
                                                  "p95": pct(s_["raw_abs_ms"], 0.95)}
        bag_report["cameras"][c]["image_quality"] = qd
        cal_path = os.path.join(out_root, c, "calibration.json")
        if os.path.exists(cal_path):
            with open(cal_path) as f:
                bag_report["cameras"][c]["calibration"] = json.load(f)
    with open(os.path.join(out_root, "bag_report.json"), "w") as f:
        json.dump(bag_report, f, indent=2)

    log(f"\nDone in {time.time() - t_start:.1f} s. Output: {out_root}")
    for s in summary:
        log(f"  {os.path.join(out_root, s['cam'], 'stereo.edex')}")


if __name__ == "__main__":
    main()
