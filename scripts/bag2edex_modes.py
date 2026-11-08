#!/usr/bin/env python3
"""
bag2edex_modes.py - Converts one ROS 2 capture (cam1 + cam2 D455) into one EDEX per cuVSLAM input
configuration. The images are written once and shared by all the configurations.

Output (<output>/):
    images/<cam>/{infra1,infra2,rgb,rgb_rect,depth_native,depth_aligned}/NNNNNN.{png,jpg}
    imu/IMU_<cam>.jsonl
    bag_report.json                    path of the source bag (used to read /tf_static)
    modes.json                         per configuration: frames, streams, sync statistics, flags
    <configuration>/
        stereo.edex  frame_metadata.jsonl  sync.csv  calibration.json
No links are used: the files of a configuration refer to ../images and ../imu, so a configuration
needs the whole capture folder.

Configurations (skipped, with the reason in modes.json, when a stream is missing):
    rgbd_ir_<cam>         infra1 + native depth (pixel-aligned with infra1)        RGBD
    rgbd_rgb_<cam>        RGB (brown5k) + depth aligned to RGB                     RGBD
    stereo_imu_<cam>      infra1 + infra2 + IMU of the same camera                 Inertial, stereo, Mono
    multicamera_ir        cam2 infra1/2, cam1 infra1/2                             Multicamera (experimental)
    multisensor_ir_depth  cam2/cam1 infra1 with depth, then infra2, IMU of cam2    Multisensor (experimental)
    multisensor_all       the above + cam2/cam1 RGB rectified to pinhole           Multisensor (experimental)

The two D455 are not hardware-synchronized. In the cam1+cam2 configurations the frames are defined by
cam2 infra1; cam1 images and the RGB images within --rig-tolerance-ms are attached and all the images of
a frame get the cam2 infra1 timestamp. The real stamps are kept in `true_timestamp` (frame_metadata.jsonl)
and in sync.csv. Such frames are accepted but not physically simultaneous.

Native depth is read from the original bags: the reduced MCAP bags do not contain it.

RGB rectification (rgb_rect): pinhole image with the same K and no rotation, sampled through the
forward Brown-Conrady model of camera_info (see build_rectification_map).
"""

import argparse
import csv
import json
import os
import shutil
import statistics
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag2edex import (NS, Bag, Db3File, TfTree, align_depth_to_color, decode_camera_info, decode_image,  # noqa: E402
                      decode_imu, decode_tf, dedupe, depth_to_u16, distort_normalized, edex_camera_transform,
                      edex_imu_transform, greedy_match, header_stamp, image_to_cv, quat_to_rot, ros_d_to_brown5k)
from merge_rig_edex import frustum_ratio  # noqa: E402

TOPICS = {"rgb": "/{cam}/color/image_raw", "ir1": "/{cam}/infra1/image_rect_raw",
          "ir2": "/{cam}/infra2/image_rect_raw", "depth": "/{cam}/depth/image_rect_raw",
          "aligned": "/{cam}/aligned_depth_to_color/image_raw"}
INFOS = {"rgb": "/{cam}/color/camera_info", "ir1": "/{cam}/infra1/camera_info",
         "ir2": "/{cam}/infra2/camera_info", "depth": "/{cam}/depth/camera_info"}
FOLDER = {"rgb": "rgb", "rgb_rect": "rgb_rect", "ir1": "infra1", "ir2": "infra2",
          "depth": "depth_native", "aligned": "depth_aligned"}
EXT = {"rgb": "jpg", "rgb_rect": "jpg", "ir1": "png", "ir2": "png", "depth": "png", "aligned": "png"}
MODES = ["rgbd_ir", "rgbd_rgb", "stereo_imu", "multicamera_ir", "multisensor_ir_depth", "multisensor_all"]
# cam1_link -> cam2_link from the mount CAD, as in merge_rig_edex.py
CAD_EXTRINSIC = [-0.040, 0.0, 0.0509, 0.0, -0.42273, 0.0, 0.90626]


def log(msg):
    print(msg, flush=True)


def parse_args():
    ap = argparse.ArgumentParser(description="One EDEX per cuVSLAM input configuration, with shared images",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("bag", help="capture folder (original .db3/.mcap, or converted/ copy)")
    ap.add_argument("-o", "--output", required=True, help="output folder of this capture")
    ap.add_argument("--modes", nargs="+", choices=MODES, default=MODES, help="configurations to write")
    ap.add_argument("--stereo-tolerance-ms", type=float, default=1.0, help="maximum infra1 <-> infra2 offset")
    ap.add_argument("--depth-tolerance-ms", type=float, default=1.0,
                    help="maximum image <-> depth offset (infra1 + native depth, RGB + recorded aligned depth)")
    ap.add_argument("--computed-depth-tolerance-ms", type=float, default=8.0,
                    help="maximum RGB <-> native depth offset when the aligned depth has to be computed")
    ap.add_argument("--rig-tolerance-ms", type=float, default=16.0,
                    help="maximum offset of cam1 and RGB images from the cam2 infra1 reference frame")
    ap.add_argument("--reference-cam", default="cam2", help="camera defining the frames of the cam1+cam2 rigs")
    ap.add_argument("--extrinsic", type=float, nargs=7, metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                    default=CAD_EXTRINSIC, help="T_cam1_link_from_cam2_link (default: CAD of the mount)")
    ap.add_argument("--depth-scale-factor", type=float, default=1000.0, help="depth units per meter")
    ap.add_argument("--depth-color-model", choices=["plumb_bob", "inverse"], default="plumb_bob",
                    help="RGB distortion model when computing the aligned depth")
    ap.add_argument("--rgb-distortion-scale", type=float, default=1.0,
                    help="factor applied to the camera_info distortion coefficients when rectifying the RGB")
    ap.add_argument("--stale-threshold", type=float, default=0.5,
                    help="drop messages whose stamp-receive offset is more than N s from the median (0 = off)")
    ap.add_argument("--gyro-noise-density", type=float, default=0.000244)
    ap.add_argument("--gyro-random-walk", type=float, default=1.9393e-05)
    ap.add_argument("--accel-noise-density", type=float, default=0.001862)
    ap.add_argument("--accel-random-walk", type=float, default=0.003)
    ap.add_argument("--jpg-quality", type=int, default=95)
    ap.add_argument("--png-compression", type=int, default=6, choices=range(10))
    ap.add_argument("--workers", type=int, default=max(2, min(8, os.cpu_count() or 4)))
    ap.add_argument("--overwrite", action="store_true", help="replace the output folder")
    ap.add_argument("--dry-run", action="store_true", help="build the configurations without writing anything")
    return ap.parse_args()


# -----------------------------------------------------------------------------
# Reading
# -----------------------------------------------------------------------------
def read_capture(args):
    bag = Bag(args.bag)
    types = bag.topic_types()
    cams = sorted({t.split("/")[1] for t in types if t.endswith("/infra1/image_rect_raw")})
    if not cams:
        sys.exit("No cameras with infra1 in this bag")
    images = {TOPICS[r].format(cam=c): (c, r) for c in cams for r in TOPICS if TOPICS[r].format(cam=c) in types}
    infos = {INFOS[r].format(cam=c): (c, r) for c in cams for r in INFOS if INFOS[r].format(cam=c) in types}
    imu_topics = {f"/{c}/imu": c for c in cams if f"/{c}/imu" in types}

    stamps = {k: [] for k in images.values()}
    for topic, recv, key, data in bag.iterate(set(images), prefix_only=True):
        st = header_stamp(data)
        stamps[images[topic]].append((st if st else recv, recv, key))

    calib, imu, tf = {}, {c: {"frame": None, "rows": []} for c in imu_topics.values()}, TfTree()
    small = set(infos) | set(imu_topics) | ({"/tf_static"} & set(types))
    for topic, recv, _key, data in bag.iterate(small):
        if topic == "/tf_static":
            for parent, child, t, q in decode_tf(data):
                tf.add(parent, child, t, q)
        elif topic in infos:
            calib.setdefault(infos[topic], decode_camera_info(data))
        else:
            m = decode_imu(data)
            c = imu_topics[topic]
            imu[c]["frame"] = imu[c]["frame"] or m["frame_id"]
            imu[c]["rows"].append((m["stamp"] if m["stamp"] > 0 else recv, recv, m["gyro"], m["acc"]))

    dropped = {}
    thr = int(args.stale_threshold * NS)
    if thr > 0:
        for k, lst in stamps.items():
            if len(lst) > 2:
                med = statistics.median(s - r for s, r, _ in lst)
                keep = [x for x in lst if abs((x[0] - x[1]) - med) <= thr]
                if len(keep) != len(lst):
                    dropped["/".join(k)] = len(lst) - len(keep)
                    stamps[k] = keep
        for c, d in imu.items():
            if len(d["rows"]) > 2:
                med = statistics.median(r[0] - r[1] for r in d["rows"])
                d["rows"] = [r for r in d["rows"] if abs((r[0] - r[1]) - med) <= thr]
    return bag, cams, {k: dedupe(v) for k, v in stamps.items()}, calib, imu, tf, dropped, set(images)


def build_rectification_map(info, scale=1.0):
    """Maps of cv2.remap producing a pinhole image with the same K from a Brown-Conrady image.

    For a pixel (u', v') of the rectified image:
        x = (u' - cx) / fx,   y = (v' - cy) / fy                          ideal normalized coordinates
        r^2 = x^2 + y^2
        x_d = x (1 + k1 r^2 + k2 r^4 + k3 r^6) + 2 p1 x y + p2 (r^2 + 2 x^2)
        y_d = y (1 + k1 r^2 + k2 r^4 + k3 r^6) + p1 (r^2 + 2 y^2) + 2 p2 x y
        u = fx x_d + cx,      v = fy y_d + cy                             source pixel in the raw image
    No rotation is applied (R = I), so the camera frame and the extrinsics do not change. The image
    is sampled at (u, v): every destination pixel gets exactly one interpolated source value.
    `scale` multiplies all the coefficients (1 = camera_info as it is)."""
    w, h = int(info["width"]), int(info["height"])
    k = info["k"]
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
    xd, yd = distort_normalized((uu - cx) / fx, (vv - cy) / fy, [scale * c for c in info["d"]], "plumb_bob")
    map_x, map_y = (fx * xd + cx).astype(np.float32), (fy * yd + cy).astype(np.float32)
    out = (map_x < 0) | (map_x > w - 1) | (map_y < 0) | (map_y > h - 1)
    shift = np.hypot(map_x - uu, map_y - vv)
    return map_x, map_y, {"distortion_scale": scale, "model": "plumb_bob forward, K unchanged, R = I",
                          "interpolation": "cubic", "border": "replicate",
                          "pixels_outside_source": int(out.sum()),
                          "max_shift_px": float(shift.max()), "median_shift_px": float(np.median(shift))}


# -----------------------------------------------------------------------------
# Per-camera bookkeeping
# -----------------------------------------------------------------------------
class Camera:
    """Streams of one D455: stamps, stereo pairs, depth associations, geometry."""

    def __init__(self, name, stamps, calib, imu, tf, args):
        self.name, self.calib, self.tf, self.args = name, calib, tf, args
        self.link = f"{name}_link"
        s = {r: stamps.get((name, r), []) for r in TOPICS}
        self.items = s
        self.index = {r: {it[2]: n for n, it in enumerate(v)} for r, v in s.items()}
        self.st = {r: [it[0] for it in v] for r, v in s.items()}
        tol = int(args.stereo_tolerance_ms * 1e6)
        self.has_ir2 = bool(s["ir2"]) and (name, "ir2") in calib
        m = greedy_match(self.st["ir1"], self.st["ir2"], tol) if self.has_ir2 else {}
        self.pairs = [(s["ir1"][i], s["ir2"][m[i]]) for i in sorted(m)]
        self.ir1_only = [(it, None) for it in s["ir1"]]
        tol = int(args.depth_tolerance_ms * 1e6)
        m = greedy_match(self.st["ir1"], self.st["depth"], tol)
        self.depth_of_ir1 = {self.st["ir1"][i]: s["depth"][j] for i, j in m.items()}
        self.aligned_source = None
        self.depth_of_rgb, self.aligned_stream = {}, None
        if s["aligned"]:
            m = greedy_match(self.st["rgb"], self.st["aligned"], tol)
            self.aligned_source, self.aligned_stream = "recorded", "aligned"
            self.depth_of_rgb = {self.st["rgb"][i]: s["aligned"][j] for i, j in m.items()}
        elif s["depth"] and s["rgb"]:
            m = greedy_match(self.st["rgb"], self.st["depth"], int(args.computed_depth_tolerance_ms * 1e6))
            self.aligned_source, self.aligned_stream = "computed", "depth"
            self.depth_of_rgb = {self.st["rgb"][i]: s["depth"][j] for i, j in m.items()}
        self.imu_frame, self.imu_rows = imu["frame"], imu["rows"]
        self.T_link_cam = {r: self._T_link_from(r) for r in ("rgb", "ir1", "ir2") if (name, r) in calib}

    def _T_link_from(self, role):
        info = self.calib[(self.name, role)]
        frame = info["frame_id"].lstrip("/")
        T = self.tf.lookup(self.link, frame)
        if T is None:
            sys.exit(f"Transform {self.link} <- {frame} not found in /tf_static")
        if role == "ir2":
            ir1 = self.calib[(self.name, "ir1")]
            if ir1["frame_id"].lstrip("/") == frame:   # realsense-ros: infra2 uses the rectified left frame
                shift = np.eye(4)
                shift[0, 3] = -info["p"][3] / info["p"][0]
                T = T @ shift
        return T

    def intrinsics(self, stream):
        role = "rgb" if stream == "rgb_rect" else stream
        info = self.calib[(self.name, role)]
        k = info["k"]
        base = dict(focal=[k[0], k[4]], principal=[k[2], k[5]], size=[info["width"], info["height"]])
        if stream == "rgb":
            return dict(base, distortion_model="brown5k", distortion_params=ros_d_to_brown5k(info["d"]))
        return dict(base, distortion_model="pinhole", distortion_params=[])

    def T_link_imu(self):
        for fr in (self.imu_frame, f"{self.name}_imu_optical_frame", f"{self.name}_gyro_optical_frame"):
            T = self.tf.lookup(self.link, fr) if fr else None
            if T is not None:
                return T, fr
        sys.exit(f"{self.name}: IMU frame not found in /tf_static")

    def file(self, stream, item):
        role = {"rgb_rect": "rgb"}.get(stream, stream)
        ref = self.aligned_stream if stream == "aligned" else role
        return f"images/{self.name}/{FOLDER[stream]}/{self.index[ref][item[2]]:06d}.{EXT[stream]}"


class Pool:
    """Images to write: source message key -> list of (relative path, kind, camera)."""

    def __init__(self):
        self.jobs = {}

    def add(self, cam, stream, item):
        path = cam.file(stream, item)
        if stream == "aligned" and cam.aligned_source == "computed":
            kind = "aligned_computed"
        else:
            kind = {"ir1": "ir", "ir2": "ir", "depth": "depth", "aligned": "depth"}.get(stream, stream)
        lst = self.jobs.setdefault(item[2], [])
        if not any(p == path for p, _k, _c in lst):
            lst.append((path, kind, cam.name))
        return path


def edex_imu_section(args, cam, T_rig_from_imu, rows):
    dts = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
    freq = round(NS / statistics.median(dts)) if dts else 200.0
    return {"measurements": f"../imu/IMU_{cam.name}.jsonl", "transform": edex_imu_transform(T_rig_from_imu),
            "coordinate_system": "opencv",
            "gyro_noise_density": args.gyro_noise_density, "gyro_random_walk": args.gyro_random_walk,
            "accel_noise_density": args.accel_noise_density, "accel_random_walk": args.accel_random_walk,
            "gyroscope_noise_density": args.gyro_noise_density, "gyroscope_random_walk": args.gyro_random_walk,
            "accelerometer_noise_density": args.accel_noise_density,
            "accelerometer_random_walk": args.accel_random_walk, "frequency": float(freq)}


# -----------------------------------------------------------------------------
# Writing one configuration
# -----------------------------------------------------------------------------
class Mode:
    """One configuration: slots = cameras of the rig, frames = list of per-frame dictionaries."""

    def __init__(self, name, slots, imu_cam=None, experimental=False):
        self.name, self.slots, self.imu_cam, self.experimental = name, slots, imu_cam, experimental
        self.frames = []

    def add_frame(self, ref_ts, images, depths=(), forced=False):
        """images: [(slot index, filename, real stamp)]; depths: [(depth id, filename, real stamp)]."""
        def entry(tag, f, st):
            d = {tag: f[0], "filename": "../" + f[1], "timestamp": int(ref_ts if forced else st)}
            if forced:
                d["true_timestamp"] = int(st)
            return d
        self.frames.append({"ts": ref_ts, "cams": [entry("id", (i, fn), st) for i, fn, st in images],
                            "depth": [entry("id", (i, fn), st) for i, fn, st in depths]})

    def write(self, out, ctx, args, dry):
        """Writes the configuration folder (not in dry-run) and returns its summary."""
        counts = [0] * len(self.slots)
        offsets = [[] for _ in self.slots]
        depth_counts = {}
        rows = []
        for f in self.frames:
            row = {}
            for c in f["cams"]:
                counts[c["id"]] += 1
                true = c.get("true_timestamp", c["timestamp"])
                offsets[c["id"]].append(abs(true - f["ts"]) / 1e6)
                row[c["id"]] = true
            for d in f["depth"]:
                depth_counts[d["id"]] = depth_counts.get(d["id"], 0) + 1
            rows.append((f, row))
        summary = {"status": "ok", "frames": len(self.frames), "experimental": self.experimental,
                   "imu": self.imu_cam,
                   "cameras": [{"id": k, "cam": s["cam"].name, "stream": s["stream"], "frames": counts[k],
                                "median_offset_ms": float(np.median(offsets[k])) if offsets[k] else None,
                                "max_offset_ms": float(np.max(offsets[k])) if offsets[k] else None,
                                "depth_id": s.get("depth_id")} for k, s in enumerate(self.slots)],
                   "depth_frames": {str(k): v for k, v in sorted(depth_counts.items())},
                   "complete_frames": sum(len(f["cams"]) == len(self.slots) for f in self.frames)}
        pairs = []
        for i in range(len(self.slots)):
            for j in range(i + 1, len(self.slots)):
                a, b = self.slots[i], self.slots[j]
                r = frustum_ratio(a["cam"].intrinsics(a["stream"]), b["cam"].intrinsics(b["stream"]),
                                  np.linalg.inv(b["T"]) @ a["T"])
                if r > 0.5:
                    pairs.append([i, j, round(r, 3)])
        summary["predicted_stereo_pairs"] = pairs
        if dry:
            return summary

        d = os.path.join(out, self.name)
        os.makedirs(d)
        cameras, calib = [], {"rig_frame": self.slots[0]["frame"], "base_frame": f"{self.slots[0]['cam'].name}_link",
                              "T_rig_from_base_opencv": np.linalg.inv(self.slots[0]["T_link"]).tolist(),
                              "cameras": [], "imu": None,
                              "experimental_forced_sync": self.experimental, "extrinsic_cam1_link_from_cam2_link":
                              args.extrinsic if self.experimental else None}
        for k, s in enumerate(self.slots):
            e = {"intrinsics": s["cam"].intrinsics(s["stream"]), "transform": edex_camera_transform(s["T"])}
            if s.get("depth_id") is not None:
                e["depth_id"] = s["depth_id"]
                e["depth_scale_factor"] = args.depth_scale_factor
            cameras.append(e)
            calib["cameras"].append({"id": k, "cam": s["cam"].name, "stream": s["stream"], "frame_id": s["frame"],
                                     "T_rig_from_camera_opencv": s["T"].tolist(), "depth_id": s.get("depth_id"),
                                     "depth_source": s.get("depth_source"), "rectification": s.get("rect")})
        header = {"version": "0.9", "frame_start": 0, "frame_end": len(self.frames) - 1, "cameras": cameras}
        if self.imu_cam:
            cam = ctx["cams"][self.imu_cam]
            T_li, frame = cam.T_link_imu()
            T = np.linalg.inv(ctx["T_link_frame"](self.slots[0], cam.name)) @ T_li
            header["imu"] = edex_imu_section(args, cam, T, ctx["imu_rows"][self.imu_cam])
            calib["imu"] = {"cam": cam.name, "frame_id": frame, "T_rig_from_imu_opencv": T.tolist()}
        # one entry per frame ("" = missing), so that the file numbers need not start at the frame number:
        # the images come from a pool shared by all the configurations
        seq = [[next((c["filename"] for c in f["cams"] if c["id"] == k), "") for f in self.frames]
               for k in range(len(self.slots))]
        body = {"frame_metadata": "frame_metadata.jsonl", "sequence": seq}
        depth_ids = sorted({x["id"] for f in self.frames for x in f["depth"]})
        if depth_ids:
            body["depth_sequence"] = [[next((x["filename"] for x in f["depth"] if x["id"] == k), "")
                                       for f in self.frames] for k in depth_ids]
        with open(os.path.join(d, "stereo.edex"), "w") as fp:
            json.dump([header, body], fp, indent=2)
        with open(os.path.join(d, "calibration.json"), "w") as fp:
            json.dump(calib, fp, indent=2)
        names = [f"{s['cam'].name}_{s['stream']}" for s in self.slots]
        with open(os.path.join(d, "frame_metadata.jsonl"), "w") as fm, \
                open(os.path.join(d, "sync.csv"), "w", newline="") as fs:
            w = csv.writer(fs)
            w.writerow(["frame_id", "frame_stamp_ns"] + [f"{n}_stamp_ns" for n in names] +
                       [f"{n}_offset_ms" for n in names])
            for i, (f, row) in enumerate(rows):
                meta = {"frame_id": i, "cams": f["cams"]}
                if f["depth"]:
                    meta["depth"] = f["depth"]
                fm.write(json.dumps(meta) + "\n")
                w.writerow([i, f["ts"]] + [row.get(k, "") for k in range(len(names))] +
                           [f"{(row[k] - f['ts']) / 1e6:.3f}" if k in row else "" for k in range(len(names))])
        return summary


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    t0 = time.time()
    out = os.path.abspath(os.path.expanduser(args.output))
    if os.path.exists(out) and os.listdir(out) and not args.dry_run:
        if not args.overwrite:
            sys.exit(f"{out} exists and is not empty (use --overwrite)")
        for entry in os.listdir(out):       # convert_all.py keeps its log in this folder
            full = os.path.join(out, entry)
            if entry != "convert.log":
                shutil.rmtree(full) if os.path.isdir(full) else os.remove(full)

    bag, cam_names, stamps, calib, imu, tf, dropped, image_topics = read_capture(args)
    cams = {c: Camera(c, stamps, calib, imu.get(c, {"frame": None, "rows": []}), tf, args) for c in cam_names}
    log(f"Capture {args.bag}: cameras {cam_names}, dropped stale {dropped or 'none'}")

    E = np.eye(4)
    E[:3, :3] = quat_to_rot(args.extrinsic[3:])
    E[:3, 3] = args.extrinsic[:3]

    def T_link_frame(slot0, cam_name):
        """T_<cam_name>_link_from_rig for a rig whose first slot is slot0."""
        ref = slot0["cam"].name
        T_ref_link_from_rig = slot0["T_link"]
        if ref == cam_name:
            return T_ref_link_from_rig
        if (ref, cam_name) == ("cam1", "cam2"):
            return np.linalg.inv(E) @ T_ref_link_from_rig
        return E @ T_ref_link_from_rig

    pool, rect = Pool(), {}
    for c, cam in cams.items():
        if (c, "rgb") in calib:
            mx, my, info = build_rectification_map(calib[(c, "rgb")], args.rgb_distortion_scale)
            rect[c] = (mx, my, info)
            log(f"[{c}] RGB rectification: max shift {info['max_shift_px']:.2f} px, "
                f"median {info['median_shift_px']:.2f} px, pixels outside source {info['pixels_outside_source']}")

    def slot(cam, stream, ref_T_link, ref_cam_name, **extra):
        """Rig slot: geometry of (cam, stream) in the rig whose camera 0 is given by its T_link and camera."""
        role = "rgb" if stream == "rgb_rect" else stream
        T_lc = cam.T_link_cam[role]
        if cam.name == ref_cam_name:
            T = np.linalg.inv(ref_T_link) @ T_lc
        elif (ref_cam_name, cam.name) == ("cam1", "cam2"):
            T = np.linalg.inv(ref_T_link) @ E @ T_lc
        else:
            T = np.linalg.inv(ref_T_link) @ np.linalg.inv(E) @ T_lc
        s = dict(cam=cam, stream=stream, T=T, T_link=ref_T_link,
                 frame=cam.calib[(cam.name, role)]["frame_id"].lstrip("/"))
        s.update(extra)
        return s

    modes, skipped = [], {}

    def want(name):
        return any(name == m or name.startswith(m + "_") for m in args.modes)

    def single(c):
        cam = cams[c]
        # RGB-D on infra1 + native depth (same stamps, K and viewpoint)
        name = f"rgbd_ir_{c}"
        if want(name):
            if not cam.depth_of_ir1:
                skipped[name] = "no native depth matched to infra1"
            else:
                ki, kd = calib[(c, "ir1")]["k"], calib.get((c, "depth"), {}).get("k")
                if kd is None or not np.allclose(ki, kd, atol=1e-3):
                    log(f"WARNING: {c}: depth intrinsics differ from infra1")
                T = cam.T_link_cam["ir1"]
                m = Mode(name, [slot(cam, "ir1", T, c, depth_id=0, depth_source="native")])
                for it in cam.items["ir1"]:
                    d = cam.depth_of_ir1.get(it[0])
                    if d is not None:
                        m.add_frame(it[0], [(0, pool.add(cam, "ir1", it), it[0])],
                                    [(0, pool.add(cam, "depth", d), d[0])])
                modes.append(m)
        # RGB-D on RGB + depth aligned to the RGB
        name = f"rgbd_rgb_{c}"
        if want(name):
            if not cam.depth_of_rgb or (c, "rgb") not in calib:
                skipped[name] = "no depth aligned to RGB"
            else:
                T = cam.T_link_cam["rgb"]
                m = Mode(name, [slot(cam, "rgb", T, c, depth_id=0, depth_source=cam.aligned_source)])
                for it in cam.items["rgb"]:
                    d = cam.depth_of_rgb.get(it[0])
                    if d is not None:
                        m.add_frame(it[0], [(0, pool.add(cam, "rgb", it), it[0])],
                                    [(0, pool.add(cam, "aligned", d), d[0])])
                modes.append(m)
        # Stereo IR + IMU
        name = f"stereo_imu_{c}"
        if want(name):
            if not cam.pairs:
                skipped[name] = "no infra2 in this capture"
            elif not cam.imu_rows:
                skipped[name] = "no IMU"
            else:
                T = cam.T_link_cam["ir1"]
                m = Mode(name, [slot(cam, "ir1", T, c), slot(cam, "ir2", T, c)], imu_cam=c)
                for a, b in cam.pairs:
                    m.add_frame(a[0], [(0, pool.add(cam, "ir1", a), a[0]), (1, pool.add(cam, "ir2", b), b[0])])
                modes.append(m)

    for c in cam_names:
        single(c)

    ref_name = args.reference_cam
    other = next((c for c in cam_names if c != ref_name), None)
    rig_modes = [m for m in ("multicamera_ir", "multisensor_ir_depth", "multisensor_all") if want(m)]
    if rig_modes and (other is None or ref_name not in cams):
        skipped.update({m: "needs both cameras" for m in rig_modes})
        rig_modes = []
    if rig_modes:
        rc, oc = cams[ref_name], cams[other]
        stereo = bool(rc.pairs and oc.pairs)
        ref = rc.pairs if stereo else rc.ir1_only
        oth = oc.pairs if stereo else oc.ir1_only
        ref_st = [a[0] for a, _ in ref]
        tol = int(args.rig_tolerance_ms * 1e6)
        m_o = greedy_match(ref_st, [a[0] for a, _ in oth], tol)
        rgb_match = {c.name: greedy_match(ref_st, c.st["rgb"], tol) for c in (rc, oc)}
        T = rc.T_link_cam["ir1"]

        def rig_frame(mode, i, opts):
            """opts: which optional sensors the configuration uses."""
            a, b = ref[i]
            ims, deps, frame_ts = [], [], a[0]
            pos = opts["pos"]
            ims.append((pos[(ref_name, "ir1")], pool.add(rc, "ir1", a), a[0]))
            if opts["depth"]:
                d = rc.depth_of_ir1.get(a[0])
                if d is not None:
                    deps.append((0, pool.add(rc, "depth", d), d[0]))
            if b is not None:
                ims.append((pos[(ref_name, "ir2")], pool.add(rc, "ir2", b), b[0]))
            if i in m_o:
                a2, b2 = oth[m_o[i]]
                ims.append((pos[(other, "ir1")], pool.add(oc, "ir1", a2), a2[0]))
                if opts["depth"]:
                    d = oc.depth_of_ir1.get(a2[0])
                    if d is not None:
                        deps.append((1, pool.add(oc, "depth", d), d[0]))
                if b2 is not None:
                    ims.append((pos[(other, "ir2")], pool.add(oc, "ir2", b2), b2[0]))
            if opts["rgb"]:
                for c in (rc, oc):
                    if i in rgb_match[c.name]:
                        it = c.items["rgb"][rgb_match[c.name][i]]
                        ims.append((pos[(c.name, "rgb")], pool.add(c, "rgb_rect", it), it[0]))
            mode.add_frame(frame_ts, ims, deps, forced=True)

        for name in rig_modes:
            if name == "multicamera_ir":
                if not stereo:
                    skipped[name] = "no infra2 in this capture"
                    continue
                order = [(rc, "ir1"), (rc, "ir2"), (oc, "ir1"), (oc, "ir2")]
                opts = {"depth": False, "rgb": False}
                imu_cam = None
            else:
                if not (rc.depth_of_ir1 and oc.depth_of_ir1):
                    skipped[name] = "no native depth"
                    continue
                order = [(rc, "ir1"), (oc, "ir1")]
                if stereo:
                    order += [(rc, "ir2"), (oc, "ir2")]
                opts = {"depth": True, "rgb": name == "multisensor_all"}
                imu_cam = ref_name if rc.imu_rows else None
                if opts["rgb"]:
                    order += [(rc, "rgb"), (oc, "rgb")]
            slots = []
            for c, st in order:
                role_stream = "rgb_rect" if st == "rgb" else st
                extra = {}
                if name != "multicamera_ir" and st == "ir1":
                    extra = {"depth_id": 0 if c is rc else 1, "depth_source": "native"}
                if st == "rgb":
                    extra["rect"] = rect[c.name][2]
                slots.append(slot(c, role_stream, T, ref_name, **extra))
            opts["pos"] = {(c.name, st): k for k, (c, st) in enumerate(order)}
            mode = Mode(name, slots, imu_cam=imu_cam, experimental=True)
            for i in range(len(ref)):
                rig_frame(mode, i, opts)
            modes.append(mode)

    # IMU rows actually written: from the first to the last frame +/- 0.5 s
    imu_rows = {}
    all_ts = [f["ts"] for m in modes for f in m.frames]
    if all_ts:
        lo, hi = min(all_ts) - NS // 2, max(all_ts) + NS // 2
        for c, cam in cams.items():
            seen, rows = set(), []
            for st, _r, g, a in sorted(cam.imu_rows, key=lambda x: x[0]):
                if lo <= st <= hi and st not in seen:
                    seen.add(st)
                    rows.append((st, g, a))
            imu_rows[c] = rows

    ctx = {"cams": cams, "T_link_frame": T_link_frame, "imu_rows": imu_rows}
    if not args.dry_run:
        os.makedirs(out, exist_ok=True)
        os.makedirs(os.path.join(out, "imu"), exist_ok=True)
        with open(os.path.join(out, "bag_report.json"), "w") as f:
            json.dump({"bag": os.path.abspath(os.path.expanduser(args.bag))}, f, indent=2)
        for c, rows in imu_rows.items():
            with open(os.path.join(out, "imu", f"IMU_{c}.jsonl"), "w") as f:
                for st, g, a in rows:
                    f.write(json.dumps({"AngularVelocityX": g[0], "AngularVelocityY": g[1], "AngularVelocityZ": g[2],
                                        "LinearAccelerationX": a[0], "LinearAccelerationY": a[1],
                                        "LinearAccelerationZ": a[2], "timestamp": int(st)}) + "\n")

    report = {"bag": os.path.abspath(os.path.expanduser(args.bag)), "rig_reference": ref_name,
              "rig_tolerance_ms": args.rig_tolerance_ms, "dropped_stale": dropped,
              "rgb_rectification": {c: v[2] for c, v in rect.items()},
              "aligned_depth_source": {c: cams[c].aligned_source for c in cam_names}, "modes": {}}
    for m in modes:
        report["modes"][m.name] = m.write(out, ctx, args, args.dry_run)
        r = report["modes"][m.name]
        cam_txt = ", ".join(f"{c['cam']}/{c['stream']} {c['frames']}" for c in r["cameras"])
        log(f"  {m.name}: {r['frames']} frames ({r['complete_frames']} complete), {cam_txt}, "
            f"pairs {r['predicted_stereo_pairs']}")
    for k, v in skipped.items():
        report["modes"][k] = {"status": "skipped", "reason": v}
        log(f"  {k}: skipped ({v})")
    if args.dry_run:
        log("--dry-run: nothing written")
        return

    # --- images --------------------------------------------------------------
    jobs = pool.jobs
    log(f"Writing {sum(len(v) for v in jobs.values())} images from {len(jobs)} messages ...")
    errors, done, lock = [], [0], threading.Lock()
    slots_sem = threading.BoundedSemaphore(args.workers * 4)
    for d in {os.path.dirname(p) for lst in jobs.values() for p, _k, _c in lst}:
        os.makedirs(os.path.join(out, d), exist_ok=True)
    jpg = [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality]
    png = [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression]

    def color_info(c):
        return calib[(c, "rgb")]

    def align_params(c):
        cam = cams[c]
        dinfo, cinfo = calib[(c, "depth")], color_info(c)
        T = cam.tf.lookup(cinfo["frame_id"].lstrip("/"), dinfo["frame_id"].lstrip("/"))
        return dinfo, cinfo, T

    aparams = {c: align_params(c) for c in cam_names
               if cams[c].aligned_source == "computed" and (c, "depth") in calib and (c, "rgb") in calib}

    def work(payload, targets):
        try:
            img = image_to_cv(decode_image(payload)) if targets[0][1] in ("ir", "rgb", "rgb_rect") \
                else depth_to_u16(decode_image(payload))
            for path, kind, cam in targets:
                full = os.path.join(out, path)
                if kind == "rgb_rect":
                    mx, my, _ = rect[cam]
                    res = cv2.remap(img, mx, my, cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
                    ok = cv2.imwrite(full, res, jpg)
                elif kind == "rgb":
                    ok = cv2.imwrite(full, img, jpg)
                elif kind == "ir":
                    ok = cv2.imwrite(full, img, png)
                elif kind == "aligned_computed":
                    di, ci, T = aparams[cam]
                    res = align_depth_to_color(img, di, ci, T, 1.0 / args.depth_scale_factor, args.depth_color_model)
                    ok = cv2.imwrite(full, res, png)
                else:
                    ok = cv2.imwrite(full, img, png)
                if not ok:
                    raise IOError(f"write failed: {full}")
        except Exception as e:  # noqa: BLE001
            with lock:
                errors.append(f"{targets[0][0]}: {e}")
        finally:
            slots_sem.release()
            with lock:
                done[0] += 1
                if done[0] % 1000 == 0 or done[0] == len(jobs):
                    log(f"  {done[0]}/{len(jobs)} messages")

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for f in bag.files:
            if isinstance(f, Db3File):
                for k in sorted(k for k in jobs if k[0] == f.path):
                    slots_sem.acquire()
                    ex.submit(work, f.fetch(k[1]), jobs[k])
            else:
                for _topic, _ts, k, data in f.iterate(image_topics):
                    if k in jobs:
                        slots_sem.acquire()
                        ex.submit(work, data, jobs[k])
    if errors:
        log(f"{len(errors)} errors while writing images, first ones:")
        for e in errors[:10]:
            log("  " + e)
        sys.exit(1)
    with open(os.path.join(out, "modes.json"), "w") as f:
        json.dump(report, f, indent=2)
    log(f"Done in {time.time() - t0:.1f} s: {out}")


if __name__ == "__main__":
    main()
