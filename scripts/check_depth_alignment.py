#!/usr/bin/env python3
"""
check_depth_alignment.py - Checks the depth alignment computed by bag2edex.py against the aligned depth
recorded by the realsense node (/<cam>/aligned_depth_to_color/image_raw), on bags that have both topics.

For a sample of frames where the recorded aligned depth and the native depth (/<cam>/depth/image_rect_raw)
have the same stamp, it computes the aligned depth from the native one with bag2edex.align_depth_to_color,
for both RGB distortion models (plumb_bob = forward Brown-Conrady, inverse = librealsense inverse
Brown-Conrady), and compares it pixel by pixel with the recorded one:

    both valid          pixels with depth in both images
    exact               share of those with the same value
    within 1% / 2 cm    share with |difference| <= 1% of the depth or <= 20 units
    median |diff|       median absolute difference (depth units, mm for RealSense)
    only recorded       pixels with depth only in the recorded image (holes of the computed one)
    only computed       pixels with depth only in the computed image

Usage:
    python3 check_depth_alignment.py ~/dotlumen/Captures/original/9_Loop_closure/loop_closure --cam cam2 --frames 20
    python3 check_depth_alignment.py <bag> --cam cam1 --save-dir /tmp/align_check   # also writes PNGs
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag2edex import (Bag, Db3File, TfTree, align_depth_to_color, decode_camera_info,  # noqa: E402
                      decode_image, decode_tf, depth_to_u16, header_stamp)


def compare(rec, comp):
    r, c = rec.astype(np.int64), comp.astype(np.int64)
    both = (r > 0) & (c > 0)
    d = np.abs(r - c)[both]
    n = int(both.sum())
    return {"both_valid": n,
            "exact": float((d == 0).mean()) if n else None,
            "within_1pct_or_20": float(((d <= 20) | (d <= 0.01 * r[both])).mean()) if n else None,
            "median_abs_diff": float(np.median(d)) if n else None,
            "only_recorded": int(((r > 0) & (c == 0)).sum()),
            "only_computed": int(((r == 0) & (c > 0)).sum()),
            "recorded_valid": int((r > 0).sum())}


def main():
    ap = argparse.ArgumentParser(description="Compare computed and recorded aligned depth")
    ap.add_argument("bag")
    ap.add_argument("--cam", default="cam2")
    ap.add_argument("--frames", type=int, default=20, help="number of frames sampled over the bag")
    ap.add_argument("--depth-scale-factor", type=float, default=1000.0)
    ap.add_argument("--save-dir", default=None, help="write recorded / computed / difference images here")
    ap.add_argument("-o", "--output", default=None, help="JSON file with the results")
    args = ap.parse_args()

    cam = args.cam
    t_al, t_dn = f"/{cam}/aligned_depth_to_color/image_raw", f"/{cam}/depth/image_rect_raw"
    t_di, t_ci = f"/{cam}/depth/camera_info", f"/{cam}/color/camera_info"
    bag = Bag(args.bag)
    types = bag.topic_types()
    for t in (t_al, t_dn, t_di, t_ci, "/tf_static"):
        if t not in types:
            sys.exit(f"Topic missing: {t}")

    stamps = {t_al: {}, t_dn: {}}
    for topic, recv, key, data in bag.iterate({t_al, t_dn}, prefix_only=True):
        stamps[topic].setdefault(header_stamp(data) or recv, key)
    common = sorted(set(stamps[t_al]) & set(stamps[t_dn]))
    if not common:
        sys.exit("No frames with the same stamp on the aligned and the native depth")
    pick = [common[int(i)] for i in np.linspace(0, len(common) - 1, min(args.frames, len(common)))]
    print(f"{len(common)} frames with both depths; comparing {len(pick)}")

    infos, tf = {}, TfTree()
    for topic, _r, _k, data in bag.iterate({t_di, t_ci, "/tf_static"}):
        if topic == "/tf_static":
            for parent, child, t, q in decode_tf(data):
                tf.add(parent, child, t, q)
        elif topic not in infos:
            infos[topic] = decode_camera_info(data)
    dinfo, cinfo = infos[t_di], infos[t_ci]
    T = tf.lookup(cinfo["frame_id"].lstrip("/"), dinfo["frame_id"].lstrip("/"))
    if T is None:
        sys.exit("depth -> color transform not found in /tf_static")
    print(f"depth -> color translation {np.round(T[:3, 3] * 1000, 2).tolist()} mm, RGB distortion "
          f"{cinfo['distortion_model']} {np.round(cinfo['d'], 5).tolist()}")

    want = {stamps[t_al][s]: ("al", s) for s in pick}
    want.update({stamps[t_dn][s]: ("dn", s) for s in pick})
    got = {}
    for f in bag.files:
        if isinstance(f, Db3File):
            for k in [k for k in want if k[0] == f.path]:
                got[want[k]] = depth_to_u16(decode_image(f.fetch(k[1])))
        else:
            for _t, _r, k, data in f.iterate({t_al, t_dn}):
                if k in want:
                    got[want[k]] = depth_to_u16(decode_image(data))
                    if len(got) == len(want):
                        break

    results = {"bag": os.path.abspath(args.bag), "cam": cam, "frames": len(pick), "models": {}}
    for model in ("plumb_bob", "inverse"):
        per = []
        for s in pick:
            if ("al", s) not in got or ("dn", s) not in got:
                continue
            comp = align_depth_to_color(got[("dn", s)], dinfo, cinfo, T, 1.0 / args.depth_scale_factor, model)
            per.append(compare(got[("al", s)], comp))
            if args.save_dir and model == "plumb_bob":
                import cv2
                os.makedirs(args.save_dir, exist_ok=True)
                cv2.imwrite(os.path.join(args.save_dir, f"{s}_recorded.png"), got[("al", s)])
                cv2.imwrite(os.path.join(args.save_dir, f"{s}_computed.png"), comp)
                diff = np.abs(got[("al", s)].astype(np.int32) - comp.astype(np.int32))
                cv2.imwrite(os.path.join(args.save_dir, f"{s}_absdiff.png"), np.clip(diff, 0, 65535).astype(np.uint16))
        agg = {k: float(np.median([p[k] for p in per if p[k] is not None])) for k in per[0]} if per else {}
        results["models"][model] = {"median_over_frames": agg, "frames": per}
        if agg:
            rv = agg["recorded_valid"] or 1
            print(f"{model:9s}: exact {100 * agg['exact']:5.1f}%   within 1%/2cm {100 * agg['within_1pct_or_20']:5.1f}%   "
                  f"median |diff| {agg['median_abs_diff']:.1f}   only recorded {100 * agg['only_recorded'] / rv:4.1f}%   "
                  f"only computed {100 * agg['only_computed'] / rv:4.1f}%   (medians over frames)")
    if args.output:
        with open(os.path.expanduser(args.output), "w") as f:
            json.dump(results, f, indent=1)
        print(f"Saved {args.output}")


if __name__ == "__main__":
    main()
