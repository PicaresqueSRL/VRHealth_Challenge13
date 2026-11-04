#!/usr/bin/env python3
"""
merge_rig_edex.py - Merges the per-camera EDEX outputs (cam1/, cam2/) produced by bag_to_cuvslam.py
into a single "rig" EDEX, using the extrinsic between the two D455s (default: values from the CAD of
the 20260330_D455_HOLDER mount).

    <capture>/                       output folder of bag_to_cuvslam.py
    ├── cam1/ cam2/                  per-camera EDEX (left unmodified)
    └── rig_<streams>/               single EDEX created by this script
        ├── stereo.edex
        ├── frame_metadata.jsonl
        ├── IMU.jsonl -> ../cam2/IMU.jsonl
        ├── images/cam1 -> ../../cam1/images, images/cam2 -> ../../cam2/images
        ├── sync.csv                 per-frame timestamp of each camera and offset from the reference
        └── rig_calibration.json     extrinsic used and transforms of each camera

Streams (--streams):
  ir   (default) cam1 infra1, cam1 infra2, cam2 infra1, cam2 infra2  -> 2 stereo pairs
  rgb            cam1 rgb, cam2 rgb   (~15-20% overlap: cuVSLAM does not accept it as a pair)
  all            the 4 IR + the 2 RGB

The reference (camera 0, the one that defines the frames) is the first stream of the first camera.
The other streams are matched by timestamp within --tolerance-ms (cuVSLAM requires <= 1 ms);
frames without a match remain partial.

The extrinsic is T_parent_from_child between the <cam>_link frames of the two cameras, in the ROS
convention (x forward, y left, z up), as in `ros2 run tf2_ros static_transform_publisher`.
The <cam>_link -> optical frame transforms are read from the bag's /tf_static.

Example:
    python3 merge_rig_edex.py ~/dotlumen/Captures/edex/9_Loop_closure/loop_closure --streams ir
"""

import argparse
import csv
import json
import os
import shutil
import statistics
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag_to_cuvslam import (Bag, TfTree, decode_tf, edex_camera_transform,  # noqa: E402
                            edex_imu_transform, quat_to_rot)

NS = 1_000_000_000
# cam1_link -> cam2_link from the mount CAD (accuracy of a few mm; to be replaced with Kalibr)
CAD_EXTRINSIC = [-0.040, 0.0, 0.0509, 0.0, -0.42273, 0.0, 0.90626]
STREAMS = {"ir": ["ir1", "ir2"], "rgb": ["rgb"], "all": ["ir1", "ir2", "rgb"]}
SUB = {"rgb": "rgb", "ir1": "infra1", "ir2": "infra2"}


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def load_cam(cap, cam):
    d = os.path.join(cap, cam)
    with open(os.path.join(d, "stereo.edex")) as f:
        header, _body = json.load(f)
    with open(os.path.join(d, "calibration.json")) as f:
        calib = json.load(f)
    roles = {}
    for c in calib["cameras"]:
        roles[c["role"]] = {"id": c["id"], "frame_id": c["frame_id"],
                            "T": np.array(c["T_rig_from_camera_opencv"]),
                            "intrinsics": header["cameras"][c["id"]]["intrinsics"]}
    # frames per role: sorted list of (timestamp, filename)
    id2role = {v["id"]: r for r, v in roles.items()}
    frames = {r: [] for r in roles}
    with open(os.path.join(d, "frame_metadata.jsonl")) as f:
        for line in f:
            for c in json.loads(line)["cams"]:
                r = id2role.get(c["id"])
                if r:
                    frames[r].append((int(c["timestamp"]), c["filename"]))
    for r in frames:
        frames[r] = sorted(set(frames[r]))
    return {"dir": d, "header": header, "calib": calib, "roles": roles, "frames": frames}


def read_tf(bag_path):
    tf = TfTree()
    bag = Bag(bag_path)
    if "/tf_static" not in bag.topic_types():
        sys.exit(f"/tf_static missing from bag {bag_path}")
    for _t, _r, _k, data in bag.iterate({"/tf_static"}):
        for parent, child, t, q in decode_tf(data):
            tf.add(parent, child, t, q)
    return tf


def frustum_ratio(intr_i, intr_j, T_j_from_i, d_min=2.0, d_max=4.0):
    """Same test as cuVSLAM (libs/camera/frustum_intersection_graph.cpp), with a pinhole model
    (distortion is ignored): fraction of a 31x31 grid of pixels of i that, at 2 and 4 m,
    falls inside the image of j. cuVSLAM forms a stereo pair if > 0.5."""
    (fxi, fyi), (cxi, cyi), (wi, hi) = intr_i["focal"], intr_i["principal"], intr_i["size"]
    (fxj, fyj), (cxj, cyj), (wj, hj) = intr_j["focal"], intr_j["principal"], intr_j["size"]
    n = 31
    R, t = T_j_from_i[:3, :3], T_j_from_i[:3, 3]
    hit = 0
    for a in range(n):
        for b in range(n):
            u = (a + 1) * (wi // (n + 2))
            v = (b + 1) * (hi // (n + 2))
            x, y = (u - cxi) / fxi, (v - cyi) / fyi
            ok = True
            for dd in (d_min, d_max):
                p = R @ np.array([x * dd, y * dd, dd]) + t
                if p[2] <= 1e-6:
                    ok = False
                    break
                uu, vv = fxj * p[0] / p[2] + cxj, fyj * p[1] / p[2] + cyj
                if not (0 < uu < wj and 0 < vv < hj):
                    ok = False
            hit += ok
    return hit / 1000.0


def greedy_match(ref, other, tol):
    """ref, other: sorted lists of timestamps. 1:1 matching starting from the smallest offsets."""
    import bisect
    cands = []
    for jb, tb in enumerate(other):
        i = bisect.bisect_left(ref, tb)
        for ia in (i - 1, i, i + 1):
            if 0 <= ia < len(ref) and abs(ref[ia] - tb) <= tol:
                cands.append((abs(ref[ia] - tb), ia, jb))
    cands.sort()
    ua, ub, out = set(), set(), {}
    for _d, ia, jb in cands:
        if ia not in ua and jb not in ub:
            ua.add(ia)
            ub.add(jb)
            out[ia] = jb
    return out


def main():
    ap = argparse.ArgumentParser(description="Merges the EDEX of two cameras into a single rig",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("capture", help="folder containing cam1/ and cam2/ (output of bag_to_cuvslam.py)")
    ap.add_argument("-o", "--output", default=None, help="output folder (default <capture>/rig_<streams>)")
    ap.add_argument("--cams", nargs=2, default=["cam1", "cam2"],
                    help="the two cameras; the first one is the reference")
    ap.add_argument("--streams", choices=list(STREAMS), default="ir")
    ap.add_argument("--extrinsic", nargs=7, type=float, default=CAD_EXTRINSIC,
                    metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                    help="T_<cam1>_link_from_<cam2>_link in meters and quaternion (default: CAD)")
    ap.add_argument("--link-frame", default="{cam}_link", help="name of each camera's base frame")
    ap.add_argument("--bag", default=None, help="bag to read /tf_static from (default: from bag_report.json)")
    ap.add_argument("--tolerance-ms", type=float, default=1.0,
                    help="maximum offset between the reference and the other streams")
    ap.add_argument("--copy", action="store_true", help="copy images instead of using symbolic links")
    ap.add_argument("--overwrite", action="store_true")
    args = ap.parse_args()

    cap = os.path.abspath(os.path.expanduser(args.capture))
    ca, cb = args.cams
    out = os.path.abspath(os.path.expanduser(args.output or os.path.join(cap, f"rig_{args.streams}")))
    if os.path.exists(out) and os.listdir(out):
        if not args.overwrite:
            sys.exit(f"{out} exists and is not empty (use --overwrite)")
        shutil.rmtree(out)

    cams = {c: load_cam(cap, c) for c in (ca, cb)}
    roles = STREAMS[args.streams]
    for c in cams:
        miss = [r for r in roles if r not in cams[c]["roles"]]
        if miss:
            sys.exit(f"{c}: missing streams {miss} (available: {list(cams[c]['roles'])})")

    # --- geometry -------------------------------------------------------------
    bag_path = args.bag
    if bag_path is None:
        rep = os.path.join(cap, "bag_report.json")
        if not os.path.exists(rep):
            sys.exit("bag_report.json missing: specify the bag with --bag")
        with open(rep) as f:
            bag_path = json.load(f)["bag"]
    log(f"Reading /tf_static from {bag_path}")
    tf = read_tf(bag_path)

    link = {c: args.link_frame.format(cam=c) for c in cams}
    T_link_from_rig = {}
    for c in cams:
        rf = cams[c]["calib"]["rig_frame"]
        T = tf.lookup(link[c], rf)
        if T is None:
            sys.exit(f"Transform {link[c]} <- {rf} not found. Frames: {', '.join(tf.frames())}")
        T_link_from_rig[c] = T
    E = np.eye(4)
    E[:3, :3] = quat_to_rot(args.extrinsic[3:])
    E[:3, 3] = args.extrinsic[:3]
    # T_<ca>rig_from_<c>rig
    T_arig = {ca: np.eye(4),
              cb: np.linalg.inv(T_link_from_rig[ca]) @ E @ T_link_from_rig[cb]}

    streams = [(c, r) for c in (ca, cb) for r in roles]          # camera order in the rig
    if args.streams == "all":                                     # IR first, then RGB
        streams = [(c, r) for c in (ca, cb) for r in ("ir1", "ir2")] + [(c, "rgb") for c in (ca, cb)]
    T_aref = {s: T_arig[s[0]] @ cams[s[0]]["roles"][s[1]]["T"] for s in streams}
    T_new = {s: np.linalg.inv(T_aref[streams[0]]) @ T_aref[s] for s in streams}

    # --- stereo pairs expected by cuVSLAM -------------------------------------
    log("\ncuVSLAM overlap test (pair if > 0.50, pinhole model):")
    pairs = []
    for i in range(len(streams)):
        for j in range(i + 1, len(streams)):
            si, sj = streams[i], streams[j]
            Ii = cams[si[0]]["roles"][si[1]]["intrinsics"]
            Ij = cams[sj[0]]["roles"][sj[1]]["intrinsics"]
            r = frustum_ratio(Ii, Ij, np.linalg.inv(T_new[sj]) @ T_new[si])
            tag = "PAIR" if r > 0.5 else ""
            if r > 0.5:
                pairs.append((i, j))
            log(f"  {i}:{si[0]}/{si[1]:4s} - {j}:{sj[0]}/{sj[1]:4s}  {r:.2f}  {tag}")
    if not pairs:
        log("WARNING: no stereo pair. cuVSLAM will reject the rig in Multicamera/Inertial "
            "('Bad calibration ... at least one stereo pair'); with these streams only Mono mode is left.")

    # --- frames -----------------------------------------------------------------
    tol = int(args.tolerance_ms * 1e6)
    ref_list = cams[ca]["frames"][streams[0][1]]
    ref_ts = [x[0] for x in ref_list]
    matches = {}
    for s in streams[1:]:
        lst = cams[s[0]]["frames"][s[1]]
        m = greedy_match(ref_ts, [x[0] for x in lst], tol)
        matches[s] = (m, lst)

    os.makedirs(os.path.join(out, "images"), exist_ok=True)
    for c in cams:
        src = os.path.join(cams[c]["dir"], "images")
        dst = os.path.join(out, "images", c)
        if args.copy:
            shutil.copytree(src, dst)
        else:
            os.symlink(os.path.relpath(src, os.path.dirname(dst)), dst)

    def fn(c, f):
        return f"images/{c}/" + f[len("images/"):]

    counts = {s: 0 for s in streams}
    deltas = {s: [] for s in streams[1:]}
    n_full = 0
    with open(os.path.join(out, "frame_metadata.jsonl"), "w") as fm, \
            open(os.path.join(out, "sync.csv"), "w", newline="") as fs:
        w = csv.writer(fs)
        w.writerow(["frame_id"] + [f"{c}_{r}_stamp_ns" for c, r in streams] +
                   [f"{c}_{r}_delta_ms" for c, r in streams[1:]])
        for i, (t0, f0) in enumerate(ref_list):
            meta = [{"id": 0, "filename": fn(ca, f0), "timestamp": t0}]
            row_t, row_d = [t0], []
            counts[streams[0]] += 1
            for k, s in enumerate(streams[1:], 1):
                m, lst = matches[s]
                if i in m:
                    t, f = lst[m[i]]
                    meta.append({"id": k, "filename": fn(s[0], f), "timestamp": t})
                    counts[s] += 1
                    deltas[s].append(abs(t - t0) / 1e6)
                    row_t.append(t)
                    row_d.append(f"{(t - t0) / 1e6:.3f}")
                else:
                    row_t.append("")
                    row_d.append("")
            n_full += len(meta) == len(streams)
            fm.write(json.dumps({"frame_id": i, "cams": meta}) + "\n")
            w.writerow([i] + row_t + row_d)

    # --- header -----------------------------------------------------------------
    header = {"version": "0.9", "frame_start": 0, "frame_end": len(ref_list) - 1,
              "cameras": [{"intrinsics": cams[c]["roles"][r]["intrinsics"],
                           "transform": edex_camera_transform(T_new[(c, r)])} for c, r in streams]}
    imu_cam = next((c for c in (ca, cb) if cams[c]["calib"].get("imu")), None)
    rig_cal = {"rig_frame": cams[ca]["roles"][streams[0][1]]["frame_id"],
               "extrinsic": {"parent": link[ca], "child": link[cb], "xyz": args.extrinsic[:3],
                             "quat_xyzw": args.extrinsic[3:]},
               "bag": bag_path, "tolerance_ms": args.tolerance_ms, "predicted_stereo_pairs": pairs,
               "cameras": [{"id": k, "cam": c, "role": r, "frame_id": cams[c]["roles"][r]["frame_id"],
                            "T_rig_from_camera_opencv": T_new[(c, r)].tolist()}
                           for k, (c, r) in enumerate(streams)], "imu": None}
    if imu_cam:
        imu_h = dict(cams[imu_cam]["header"]["imu"])
        T_imu = np.linalg.inv(T_aref[streams[0]]) @ T_arig[imu_cam] @ \
            np.array(cams[imu_cam]["calib"]["imu"]["T_rig_from_imu_opencv"])
        imu_h["transform"] = edex_imu_transform(T_imu)
        imu_h["measurements"] = "IMU.jsonl"
        header["imu"] = imu_h
        src = os.path.join(cams[imu_cam]["dir"], "IMU.jsonl")
        dst = os.path.join(out, "IMU.jsonl")
        if args.copy:
            shutil.copy2(src, dst)
        else:
            os.symlink(os.path.relpath(src, out), dst)
        rig_cal["imu"] = {"cam": imu_cam, "T_rig_from_imu_opencv": T_imu.tolist()}
    body = {"frame_metadata": "frame_metadata.jsonl",
            "sequence": [[fn(c, cams[c]["frames"][r][0][1])] for c, r in streams]}
    with open(os.path.join(out, "stereo.edex"), "w") as f:
        json.dump([header, body], f, indent=2)
    with open(os.path.join(out, "rig_calibration.json"), "w") as f:
        json.dump(rig_cal, f, indent=2)

    # --- summary ------------------------------------------------------------------
    log(f"\nRig: {len(streams)} cameras, reference {ca}/{streams[0][1]}, {len(ref_list)} frames, "
        f"complete {n_full} ({100 * n_full / max(1, len(ref_list)):.0f}%)")
    for k, s in enumerate(streams):
        extra = ""
        if s in deltas and deltas[s]:
            extra = f"   median offset {statistics.median(deltas[s]):.2f} ms, max {max(deltas[s]):.2f} ms"
        log(f"  camera {k}: {s[0]}/{s[1]:4s} present in {counts[s]} frames "
            f"({100 * counts[s] / max(1, len(ref_list)):.0f}%){extra}")
    if imu_cam and len(streams) > 2:
        log("Note: with more than 2 cameras cuVSLAM disables IMU fusion (Inertial mode not available).")
    log(f"\nWrote {os.path.join(out, 'stereo.edex')}")


if __name__ == "__main__":
    main()
