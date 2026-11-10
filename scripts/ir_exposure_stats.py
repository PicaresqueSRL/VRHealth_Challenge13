#!/usr/bin/env python3
"""Per-frame exposure statistics of the IR images of an EDEX capture.

Reads every image in images/cam{1,2}/infra{1,2} of an EDEX capture folder
(written by bag2edex.py) and writes one CSV row per image with intensity
statistics, to check for over/under-exposure.

File names are frame indices, not timestamps: the timestamp is looked up in
the frame_metadata.jsonl files of the mode subfolders (empty if not found).

Usage:
    ir_exposure_stats.py <edex_capture_dir> [-o out.csv] [-j N]
"""
import argparse
import csv
import glob
import json
import os
import sys
from multiprocessing import Pool

import cv2
import numpy as np

STREAMS = ["cam1/infra1", "cam1/infra2", "cam2/infra1", "cam2/infra2"]
FIELDS = ["capture", "camera", "file", "timestamp_ns", "bit_depth", "median", "mean",
          "p5", "p95", "frac_sat_ge250", "frac_dark_le5"]


def timestamp_map(capture_dir):
    """Map 'cam1/infra1/000123.png' -> timestamp (ns) from all frame_metadata.jsonl files."""
    stamps = {}
    for meta in glob.glob(os.path.join(capture_dir, "*", "frame_metadata.jsonl")):
        with open(meta) as f:
            for line in f:
                for cam in json.loads(line).get("cams", []):
                    fn = cam.get("filename", "")
                    key = fn.split("images/", 1)[-1]
                    if "infra" in key:
                        stamps.setdefault(key, cam.get("timestamp"))
    return stamps


def image_stats(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        return None
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    bits = 8
    if img.dtype == np.uint16:
        # 16-bit IR: scale to 0-255 so the thresholds below stay comparable
        bits = 16
        img = (img.astype(np.float64) / 257.0)
    v = img.ravel().astype(np.float64)
    p5, med, p95 = np.percentile(v, [5, 50, 95])
    return dict(bit_depth=bits, median=med, mean=v.mean(), p5=p5, p95=p95,
                frac_sat_ge250=float(np.mean(v >= 250)), frac_dark_le5=float(np.mean(v <= 5)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture_dir")
    ap.add_argument("-o", "--out", help="output CSV (default: <capture>_ir_stats.csv in cwd)")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    cap_dir = os.path.abspath(args.capture_dir.rstrip("/"))
    capture = os.path.basename(cap_dir)
    out = args.out or f"{capture}_ir_stats.csv"
    stamps = timestamp_map(cap_dir)

    jobs = []
    for s in STREAMS:
        files = sorted(glob.glob(os.path.join(cap_dir, "images", s, "*.png"))
                       + glob.glob(os.path.join(cap_dir, "images", s, "*.jpg")))
        if not files:
            print(f"WARNING: no images in {s}", file=sys.stderr)
        jobs += [(s, f) for f in files]

    with Pool(args.jobs) as pool:
        results = pool.map(image_stats, [f for _, f in jobs], chunksize=16)

    n16 = 0
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for (s, path), r in zip(jobs, results):
            if r is None:
                print(f"WARNING: cannot read {path}", file=sys.stderr)
                continue
            n16 += r["bit_depth"] == 16
            name = os.path.basename(path)
            row = dict(capture=capture, camera=s.replace("/", "_"), file=name,
                       timestamp_ns=stamps.get(f"{s}/{name}", ""), **r)
            for k in ("median", "mean", "p5", "p95"):
                row[k] = f"{row[k]:.2f}"
            for k in ("frac_sat_ge250", "frac_dark_le5"):
                row[k] = f"{row[k]:.6f}"
            w.writerow(row)
    if n16:
        print(f"NOTE: {n16} images were 16-bit and scaled to 0-255", file=sys.stderr)
    print(f"{len(jobs)} images -> {out}")


if __name__ == "__main__":
    main()
