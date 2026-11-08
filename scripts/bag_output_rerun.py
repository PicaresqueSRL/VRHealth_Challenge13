#!/usr/bin/env python3
"""
bag_output_rerun.py - Shows in Rerun a cuVSLAM run already done with Isaac ROS (isaac_ros_visual_slam),
from the bag that recorded its output topics, without ROS.

Input:
  * the output bag (e.g. isaac_ros_5/launch/slam_cam2.sh -> <run>/bag): /visual_slam/tracking/odometry,
    vo_path, slam_path, /visual_slam/status, /visual_slam/vis/* and /tf, /tf_static;
  * optionally the input bag played during the run (--input-bag), for the camera images and the
    camera calibration (camera_info, /tf_static).

What is shown (same layout and entity names as track_bag_rerun.py):
  world/odom_trajectory      odometry trajectory (line), from nav_msgs/Odometry
  world/rig                  current pose of the odometry child frame (e.g. cam2_link)
  world/rig/<cam>_<role>     camera frustum and image (with --input-bag)
  world/vo_path, world/slam_path   last nav_msgs/Path published (slam_path = trajectory after the
                             loop closure corrections)
  world/vis/<name>           point clouds and pose arrays of /visual_slam/vis/* (landmarks, observations,
                             loop closures, pose graph), transformed to the world frame with /tf
  stats/status/<field>       numeric fields of /visual_slam/status (vo_state, ...)
  stats/timing/<field>       execution times of /visual_slam/status
  stats/points/<name>        number of points of each point cloud topic

Every message is placed on the "time" timeline by its header stamp, relative to the first odometry
message, so the images of the input bag line up with the output of cuVSLAM.

Messages are decoded from the ROS 2 message definitions stored in the MCAP file (any message type,
including isaac_ros_visual_slam_interfaces); for .db3 bags built-in definitions of the common types are used.

Examples:
    R=~/workspaces/isaac_ros-dev/slam_runs/cam2_stereo_imu/loop_closure
    python3 bag_output_rerun.py $R/bag
    python3 bag_output_rerun.py $R/bag --input-bag ~/dotlumen/Captures/9_Loop_closure/loop_closure \\
        --streams cam2/ir1 cam2/ir2 --save $R/isaac_ros.rrd

Requirements: numpy, opencv-python-headless, rerun-sdk==0.33.1 (zstandard/lz4 for compressed MCAP).
Uses the bag readers of bag2edex.py (same folder).
"""

import argparse
import os
import re
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag2edex import (Bag, McapFile, TfTree, decode_camera_info, decode_image, decode_tf,  # noqa: E402
                            image_to_cv, quat_to_rot)

NS = 1_000_000_000
IMAGE_TOPICS = {"rgb": ("/{cam}/color/image_raw", "/{cam}/color/camera_info"),
                "ir1": ("/{cam}/infra1/image_rect_raw", "/{cam}/infra1/camera_info"),
                "ir2": ("/{cam}/infra2/image_rect_raw", "/{cam}/infra2/camera_info")}
TIMING_WORDS = ("time", "duration", "latency")


def log(msg):
    print(msg, file=sys.stderr, flush=True)


# =============================================================================
# Generic CDR decoding from ROS 2 message definitions (.msg text)
# =============================================================================
PRIMITIVES = {"bool": ("?", 1), "byte": ("B", 1), "char": ("B", 1), "int8": ("b", 1), "uint8": ("B", 1),
              "int16": ("h", 2), "uint16": ("H", 2), "int32": ("i", 4), "uint32": ("I", 4),
              "int64": ("q", 8), "uint64": ("Q", 8), "float32": ("f", 4), "float64": ("d", 8)}
NP_TYPES = {"int8": "i1", "uint8": "u1", "int16": "i2", "uint16": "u2", "int32": "i4", "uint32": "u4",
            "int64": "i8", "uint64": "u8", "float32": "f4", "float64": "f8", "bool": "?"}

# Definitions used when the bag does not store them (.db3). Field order as in ROS 2 Humble/Jazzy.
BUILTIN_DEFINITIONS = """
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/PoseStamped
std_msgs/Header header
Pose pose
================================================================================
MSG: geometry_msgs/PoseArray
std_msgs/Header header
Pose[] poses
================================================================================
MSG: geometry_msgs/PoseWithCovariance
Pose pose
float64[36] covariance
================================================================================
MSG: geometry_msgs/Twist
Vector3 linear
Vector3 angular
================================================================================
MSG: geometry_msgs/TwistWithCovariance
Twist twist
float64[36] covariance
================================================================================
MSG: geometry_msgs/Transform
Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
Transform transform
================================================================================
MSG: tf2_msgs/TFMessage
geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: nav_msgs/Odometry
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
================================================================================
MSG: nav_msgs/Path
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
================================================================================
MSG: sensor_msgs/PointField
string name
uint32 offset
uint8 datatype
uint32 count
================================================================================
MSG: sensor_msgs/PointCloud2
std_msgs/Header header
uint32 height
uint32 width
PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
"""


def short_type(name):
    """pkg/msg/Type or pkg/Type -> pkg/Type."""
    parts = name.strip().split("/")
    return f"{parts[0]}/{parts[-1]}" if len(parts) > 1 else parts[0]


class MsgDefs:
    """Parsed message definitions: type -> list of (base type, array kind, length, field name)."""

    FIELD = re.compile(r"^([\w/]+)(?:<=\d+)?(\[(<=)?(\d*)\])?$")

    def __init__(self):
        self.defs = {}
        self.add_text(None, BUILTIN_DEFINITIONS)

    def add_text(self, main_type, text):
        sections = re.split(r"^=+\s*$", text, flags=re.M)
        for k, sec in enumerate(sections):
            m = re.search(r"^MSG:\s*(\S+)\s*$", sec, flags=re.M)
            if m:
                tname, body = short_type(m.group(1)), sec[m.end():]
            elif k == 0 and main_type:
                tname, body = short_type(main_type), sec
            else:
                continue
            fields = self._fields(tname, body)
            if fields or tname not in self.defs:  # an empty definition never replaces a known one
                self.defs[tname] = fields

    def _fields(self, tname, body):
        pkg = tname.split("/")[0]
        out = []
        for line in body.splitlines():
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            toks = line.split()
            if len(toks) < 2 or "=" in "".join(toks[1:]):  # constant: TYPE NAME=VALUE or TYPE NAME = VALUE
                continue
            m = self.FIELD.match(toks[0])
            if not m:
                continue
            base = m.group(1)
            if base == "Header":
                base = "std_msgs/Header"
            if base not in PRIMITIVES and base not in ("string", "wstring"):
                base = short_type(base if "/" in base else f"{pkg}/{base}")
            if m.group(2) is None:
                kind, n = None, 0
            elif m.group(3) or m.group(4) == "":
                kind, n = "seq", 0
            else:
                kind, n = "fixed", int(m.group(4))
            out.append((base, kind, n, toks[1]))
        return out


class CdrDecoder:
    def __init__(self, defs, buf):
        self.d = defs.defs
        self.b = memoryview(buf)
        self.e = "<" if len(buf) > 1 and buf[1] == 1 else ">"
        self.p = 4

    def align(self, n):
        self.p += (-(self.p - 4)) % n

    def prim(self, t):
        fmt, n = PRIMITIVES[t]
        self.align(n)
        v, = struct.unpack_from(self.e + fmt, self.b, self.p)
        self.p += n
        return v

    def string(self):
        n = self.prim("uint32")
        s = bytes(self.b[self.p:self.p + max(n - 1, 0)]).decode("utf-8", "replace")
        self.p += n
        return s

    def value(self, t):
        if t in PRIMITIVES:
            return self.prim(t)
        if t in ("string", "wstring"):
            return self.string()
        if t not in self.d:
            raise KeyError(f"no definition for {t}")
        return {name: self.field(base, kind, n) for base, kind, n, name in self.d[t]}

    def field(self, base, kind, n):
        if kind is None:
            return self.value(base)
        count = n if kind == "fixed" else self.prim("uint32")
        if base in ("uint8", "byte", "char"):
            v = bytes(self.b[self.p:self.p + count])
            self.p += count
            return v
        if base in NP_TYPES:
            size = PRIMITIVES[base][1]
            self.align(size)
            v = np.frombuffer(self.b, dtype=np.dtype(NP_TYPES[base]).newbyteorder(self.e), count=count,
                              offset=self.p)
            self.p += size * count
            return v
        return [self.value(base) for _ in range(count)]


def read_mcap_schemas(path):
    """{schema name: definition text} from an MCAP file: summary section first, full scan otherwise."""
    out = {}

    def schema(body):
        _sid, = struct.unpack_from("<H", body, 0)
        name, off = McapFile._str(body, 2)
        enc, off = McapFile._str(body, off)
        n, = struct.unpack_from("<I", body, off)
        if enc in ("ros2msg", ""):
            out[name] = bytes(body[off + 4:off + 4 + n]).decode("utf-8", "replace")

    with open(path, "rb") as f:
        f.seek(0, 2)
        size = f.tell()
        if size > 37:
            f.seek(size - 37)
            op, length = struct.unpack("<BQ", f.read(9))
            if op == 0x02:
                summary_start, = struct.unpack("<Q", f.read(8))
                if summary_start:
                    f.seek(summary_start)
                    while f.tell() < size - 37:
                        head = f.read(9)
                        if len(head) < 9:
                            break
                        op, length = struct.unpack("<BQ", head)
                        body = f.read(length)
                        if op == 0x03:
                            schema(body)
        if out:
            return out
        f.seek(8)
        mc = McapFile(path)
        for op, body in mc._records(f):
            if op == 0x03:
                schema(body)
            elif op == 0x06:
                off = 16
                usize, = struct.unpack_from("<Q", body, off)
                off += 12
                comp, off = McapFile._str(body, off)
                rlen, = struct.unpack_from("<Q", body, off)
                off += 8
                import io
                inner = io.BytesIO(McapFile._decompress(comp, body[off:off + rlen], usize))
                for op2, body2 in mc._records(inner):
                    if op2 == 0x03:
                        schema(body2)
            elif op == 0x02:
                break
    return out


# =============================================================================
# Geometry
# =============================================================================
def pose_matrix(pose):
    T = np.eye(4)
    o, p = pose["orientation"], pose["position"]
    T[:3, :3] = quat_to_rot([o["x"], o["y"], o["z"], o["w"]])
    T[:3, 3] = [p["x"], p["y"], p["z"]]
    return T


def stamp_ns(header):
    return int(header["stamp"]["sec"]) * NS + int(header["stamp"]["nanosec"])


class DynamicTf:
    """Latest transform of every child frame (from /tf and /tf_static), for lookups in message order."""

    def __init__(self):
        self.parent = {}  # child -> (parent, T_parent_from_child)

    def add(self, parent, child, T):
        self.parent[child.lstrip("/")] = (parent.lstrip("/"), T)

    def _to_root(self, frame):
        T, chain, f = np.eye(4), [], frame
        while f in self.parent and f not in chain:
            chain.append(f)
            p, Tp = self.parent[f]
            T = Tp @ T
            f = p
        return f, T  # root, T_root_from_frame

    def lookup(self, target, source):
        target, source = target.lstrip("/"), source.lstrip("/")
        if target == source:
            return np.eye(4)
        r1, T1 = self._to_root(target)
        r2, T2 = self._to_root(source)
        return np.linalg.inv(T1) @ T2 if r1 == r2 else None


def cloud_xyz(msg):
    """sensor_msgs/PointCloud2 (decoded) -> N x 3 float array of x, y, z."""
    n = int(msg["width"]) * int(msg["height"])
    step = int(msg["point_step"])
    if n == 0 or step == 0:
        return np.zeros((0, 3), np.float32)
    off = {f["name"]: (int(f["offset"]), int(f["datatype"])) for f in msg["fields"]}
    if not all(k in off for k in "xyz"):
        return np.zeros((0, 3), np.float32)
    raw = np.frombuffer(msg["data"], dtype=np.uint8)[:n * step].reshape(n, step)
    end = ">" if msg["is_bigendian"] else "<"
    cols = []
    for k in "xyz":
        o, dt = off[k]
        dtype = np.dtype(end + ("f8" if dt == 8 else "f4"))
        cols.append(raw[:, o:o + dtype.itemsize].copy().view(dtype)[:, 0])
    pts = np.stack(cols, axis=1).astype(np.float32)
    return pts[np.isfinite(pts).all(axis=1)]


# =============================================================================
# Rerun layout
# =============================================================================
def make_blueprint(cameras):
    import rerun.blueprint as rrb

    views = [rrb.Spatial2DView(name=name, origin=ent) for name, ent in cameras]
    plots = rrb.Tabs(rrb.TimeSeriesView(name="vo_state", origin="stats/status"),
                     rrb.TimeSeriesView(name="timing", origin="stats/timing"),
                     rrb.TimeSeriesView(name="point clouds", origin="stats/points"), active_tab=0)
    world = rrb.Spatial3DView(name="world", origin="world")
    if views:
        images = rrb.Horizontal(*views) if len(views) <= 3 else rrb.Grid(*views, grid_columns=3)
        left = rrb.Vertical(images, plots, row_shares=[1, 1])
    else:
        left = plots
    return rrb.Blueprint(rrb.Horizontal(left, world, column_shares=[1, 1]))


# =============================================================================
# Main
# =============================================================================
def parse_args():
    ap = argparse.ArgumentParser(description="Show in Rerun a cuVSLAM run recorded from Isaac ROS",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("output_bag", help="bag with the cuVSLAM output topics (folder, .mcap or .db3)")
    ap.add_argument("--input-bag", default=None, help="bag played during the run, for images and calibration")
    ap.add_argument("--streams", nargs="+", default=None,
                    help="images to show, <cam>/<rgb|ir1|ir2> (default with --input-bag: <cam>/ir1, where "
                         "<cam> is the prefix of the odometry child frame, e.g. cam2_link -> cam2)")
    ap.add_argument("--odometry-topic", default="/visual_slam/tracking/odometry")
    ap.add_argument("--status-topic", default="/visual_slam/status")
    ap.add_argument("--world-frame", default=None,
                    help="frame of the 3D view (default: frame_id of the odometry, e.g. odom)")
    ap.add_argument("--image-every", type=int, default=1, help="log one image every N")
    ap.add_argument("--cloud-every", type=int, default=1, help="log one message every N for each point cloud topic")
    ap.add_argument("--save", default=None, help="save to a .rrd file instead of opening the viewer")
    ap.add_argument("--world", choices=["base", "native"], default="base",
                    help="3D world frame: 'base' = odometry child frame (e.g. cam2_link) at the first pose, the same "
                         "for every viewer (see view_frames.py); 'native' = world frame of Isaac ROS (e.g. odom)")
    ap.add_argument("--compare", nargs="+", default=None, metavar="LABEL=FOLDER",
                    help="overlay the trajectories saved in these folders (odometry.csv / slam_poses.csv of Isaac ROS "
                         "or of --out-dir of the PyCuVSLAM scripts); needs --world base")
    return ap.parse_args()


def main():
    args = parse_args()
    import rerun as rr

    out_bag = Bag(args.output_bag)
    types = out_bag.topic_types()
    defs = MsgDefs()
    for f in out_bag.files:
        if isinstance(f, McapFile):
            for name, text in read_mcap_schemas(f.path).items():
                defs.add_text(name, text)
    log(f"Output bag: {', '.join(os.path.basename(f.path) for f in out_bag.files)}")
    for t, ty in sorted(types.items()):
        known = "" if short_type(ty) in defs.defs else "   (no definition: skipped)"
        log(f"  {t}  [{ty}]{known}")
    if args.odometry_topic not in types:
        sys.exit(f"{args.odometry_topic} not in the output bag (use --odometry-topic)")

    # first odometry message: time origin, world frame and base frame
    t0 = world = base = None
    for _t, _r, _k, data in out_bag.iterate({args.odometry_topic}):
        m = CdrDecoder(defs, data).value(short_type(types[args.odometry_topic]))
        t0, world, base = stamp_ns(m["header"]), m["header"]["frame_id"].lstrip("/"), m["child_frame_id"].lstrip("/")
        pose0 = pose_matrix(m["pose"]["pose"])
        break
    if t0 is None:
        sys.exit(f"No messages on {args.odometry_topic}")
    world = args.world_frame or world
    log(f"World frame: {world}, rig (odometry child frame): {base}")

    # cameras of the input bag
    cameras = []
    if args.input_bag:
        in_bag = Bag(args.input_bag)
        in_types = in_bag.topic_types()
        cam_prefix = base.split("_")[0]
        names = args.streams or [f"{cam_prefix}/ir1"]
        static = TfTree()
        for _t, _r, _k, data in in_bag.iterate({"/tf_static"} & set(in_types)):
            for parent, child, t, q in decode_tf(data):
                static.add(parent, child, t, q)
        for name in names:
            cam, _, role = name.partition("/")
            if role not in IMAGE_TOPICS:
                sys.exit(f"Invalid stream {name} (<cam>/<rgb|ir1|ir2>)")
            topic, info_topic = (x.format(cam=cam) for x in IMAGE_TOPICS[role])
            if topic not in in_types or info_topic not in in_types:
                log(f"WARNING: {topic} or {info_topic} not in the input bag: {name} skipped")
                continue
            info = None
            for _t, _r, _k, data in in_bag.iterate({info_topic}):
                info = decode_camera_info(data)
                break
            frame = info["frame_id"].lstrip("/")
            T = static.lookup(base, frame)
            if T is None:
                log(f"WARNING: {base} <- {frame} not in /tf_static of the input bag: {name} shown at the rig origin")
                T = np.eye(4)
            if role == "ir2" and abs(info["p"][3]) > 1e-9 and info["p"][0]:
                shift = np.eye(4)
                shift[0, 3] = -info["p"][3] / info["p"][0]  # infra2 shares the infra1 frame id: baseline from P
                T = T @ shift
            cameras.append(dict(name=name, topic=topic, info=info, T=T, ent=f"world/rig/{cam}_{role}"))

    rr.init("cuVSLAM Isaac ROS", spawn=args.save is None)
    if args.save:
        path = os.path.abspath(os.path.expanduser(args.save))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        rr.save(path)
    rr.send_blueprint(make_blueprint([(c["name"], c["ent"]) for c in cameras]))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)  # ROS frames: x forward, z up
    # viewer world: odometry child frame at the first pose (--world base) or the Isaac ROS world frame
    from view_frames import log_compare, wait_for_viewer
    T_view = np.linalg.inv(pose0) if args.world == "base" and not args.world_frame else np.eye(4)
    if args.world == "base" and args.world_frame:
        log("NOTE: --world-frame given: 3D view in that frame (--world base ignored)")
    if args.compare:
        if args.world == "base" and not args.world_frame:
            log_compare(rr, args.compare, t0, log)
        else:
            log("WARNING: --compare needs --world base: the compared runs are not shown")
    for c in cameras:
        k, info = c["info"]["k"], c["info"]
        rr.log(c["ent"], rr.Transform3D(translation=c["T"][:3, 3], mat3x3=c["T"][:3, :3]), static=True)
        rr.log(c["ent"], rr.Pinhole(focal_length=[k[0], k[4]], principal_point=[k[2], k[5]],
                                    width=int(info["width"]), height=int(info["height"]),
                                    image_plane_distance=0.3), static=True)

    def set_time(ns):
        rr.set_time("time", duration=(ns - t0) / NS)

    # --- output topics ---------------------------------------------------------------
    tf = DynamicTf()
    traj, counts, warned = [], {}, set()
    t_last = t0
    lost = []
    topics = {t for t, ty in types.items() if short_type(ty) in defs.defs}
    for topic, recv, _k, data in out_bag.iterate(topics):
        ty = short_type(types[topic])
        try:
            m = CdrDecoder(defs, data).value(ty)
        except Exception as e:  # noqa: BLE001  (one bad message must not stop the viewer)
            if topic not in warned:
                log(f"WARNING: cannot decode {topic} [{ty}]: {e}")
                warned.add(topic)
            continue
        counts[topic] = counts.get(topic, 0) + 1
        name = topic.rstrip("/").split("/")[-1]
        hdr = m.get("header") if isinstance(m, dict) else None
        ts = stamp_ns(hdr) if hdr and (hdr["stamp"]["sec"] or hdr["stamp"]["nanosec"]) else recv

        if ty == "tf2_msgs/TFMessage":
            for tr in m["transforms"]:
                T = np.eye(4)
                q, t = tr["transform"]["rotation"], tr["transform"]["translation"]
                T[:3, :3] = quat_to_rot([q["x"], q["y"], q["z"], q["w"]])
                T[:3, 3] = [t["x"], t["y"], t["z"]]
                tf.add(tr["header"]["frame_id"], tr["child_frame_id"], T)
            continue

        def to_world(frame):
            T = tf.lookup(world, frame or world)
            if T is None and (topic, frame) not in warned:
                log(f"WARNING: {topic}: no /tf from {frame} to {world}, shown without transform")
                warned.add((topic, frame))
            return T_view @ (np.eye(4) if T is None else T)

        set_time(ts)
        t_last = max(t_last, ts)
        if topic == args.odometry_topic:
            T = to_world(hdr["frame_id"]) @ pose_matrix(m["pose"]["pose"])
            traj.append(T[:3, 3].tolist())
            rr.log("world/rig", rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]))
            rr.log("world/odom_trajectory", rr.LineStrips3D([traj], colors=[[42, 120, 214]], radii=0.01))
        elif ty == "nav_msgs/Path":
            Tw = to_world(hdr["frame_id"])
            pts = [(Tw @ pose_matrix(ps["pose"]))[:3, 3] for ps in m["poses"]]
            color = [235, 104, 52] if "slam" in name else [27, 175, 122]
            if pts:
                rr.log(f"world/{name}", rr.LineStrips3D([pts], colors=[color], radii=0.01))
        elif ty == "sensor_msgs/PointCloud2":
            pts = cloud_xyz(m)
            rr.log(f"stats/points/{name}", rr.Scalars(len(pts)))
            if counts[topic] % args.cloud_every == 0:
                Tw = to_world(hdr["frame_id"])
                pts = pts @ Tw[:3, :3].T + Tw[:3, 3] if len(pts) else pts
                rr.log(f"world/vis/{name}", rr.Points3D(pts, radii=0.01))
        elif ty == "geometry_msgs/PoseArray":
            Tw = to_world(hdr["frame_id"])
            pts = [(Tw @ pose_matrix(p))[:3, 3] for p in m["poses"]]
            rr.log(f"world/vis/{name}", rr.Points3D(pts, radii=0.02))
        elif ty == "geometry_msgs/PoseStamped":
            T = to_world(hdr["frame_id"]) @ pose_matrix(m["pose"])
            rr.log(f"world/vis/{name}", rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]))
        elif topic == args.status_topic:
            for k, v in m.items():
                if k == "header" or isinstance(v, (dict, list, bytes, str, np.ndarray)):
                    continue
                group = "timing" if any(w in k for w in TIMING_WORDS) else "status"
                rr.log(f"stats/{group}/{k}", rr.Scalars(float(v)))
                if k == "vo_state" and int(v) not in (1, 3):
                    lost.append(ts)

    log("Messages logged: " + ", ".join(f"{t} {n}" for t, n in sorted(counts.items())))
    if lost:
        log(f"Frames with vo_state not 'success': {len(lost)}")

    # --- images of the input bag --------------------------------------------------------
    if cameras:
        t_end = t_last + NS  # images outside the run (plus 1 s) are skipped
        by_topic = {c["topic"]: c for c in cameras}
        n_img = {t: 0 for t in by_topic}
        for topic, recv, _k, data in in_bag.iterate(set(by_topic)):
            n_img[topic] += 1
            if (n_img[topic] - 1) % args.image_every:
                continue
            img = decode_image(data)
            ts = img["stamp"] if img.get("stamp") else recv
            if ts < t0 - NS or ts > t_end:
                continue
            cv = image_to_cv(img)
            if cv.ndim == 3:
                cv = cv[:, :, ::-1]  # BGR -> RGB
            set_time(ts)
            rr.log(f"{by_topic[topic]['ent']}/image", rr.Image(np.ascontiguousarray(cv)).compress(jpeg_quality=75))
        log("Images logged: " + ", ".join(f"{t} {n}" for t, n in n_img.items()))

    if args.save:
        log(f"Saved {args.save}")
    else:
        wait_for_viewer(rr, log)


if __name__ == "__main__":
    main()
