#!/usr/bin/env python3
"""
convert_all.py - Converts every bag in a captures folder to EDEX, preserving the folder structure.

Rules:
  * a "capture" is a folder that directly contains .db3 or .mcap files;
  * if a capture has a converted/ subfolder containing a bag, that copy is used
    (the output still goes to the capture path, without "converted");
  * vslam* folders, converted (as a capture on its own) and the output folder are skipped;
  * bags without camera topics (e.g. SLAM output only) are reported and skipped.

For each capture it runs bag_to_cuvslam.py with --reference auto and saves:
  <output>/<relative path>/<cam>/...         EDEX per camera
  <output>/<relative path>/bag_report.json   statistics for the report
  <output>/<relative path>/convert.log       conversion log
and finally <output>/conversion_summary.json.

Usage:
    python3 convert_all.py ~/dotlumen/Captures -o ~/dotlumen/Captures/edex
    python3 convert_all.py ~/dotlumen/Captures -o ~/dotlumen/Captures/edex --list   # list only
"""

import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SKIP_PARTS = ("vslam",)


def bag_files(d):
    return sorted(glob.glob(os.path.join(d, "*.db3")) + glob.glob(os.path.join(d, "*.mcap")))


def find_captures(root, out_root):
    root = os.path.abspath(root)
    out_root = os.path.abspath(out_root)
    caps = []
    for dirpath, dirnames, _ in os.walk(root):
        ap = os.path.abspath(dirpath)
        if ap == out_root or ap.startswith(out_root + os.sep):
            dirnames[:] = []
            continue
        name = os.path.basename(ap)
        if name.startswith(SKIP_PARTS) or name == "converted":
            dirnames[:] = []
            continue
        conv = os.path.join(ap, "converted")
        if os.path.isdir(conv) and bag_files(conv):
            caps.append((os.path.relpath(ap, root), conv, "converted"))
        elif bag_files(ap):
            caps.append((os.path.relpath(ap, root), ap, "original"))
        dirnames.sort()
    return caps


def main():
    ap = argparse.ArgumentParser(description="Convert every capture in a folder to EDEX")
    ap.add_argument("root", help="captures folder (e.g. ~/dotlumen/Captures)")
    ap.add_argument("-o", "--output", required=True, help="output folder (e.g. ~/dotlumen/Captures/edex)")
    ap.add_argument("--list", action="store_true", help="list the captures found and exit")
    ap.add_argument("--skip-existing", action="store_true",
                    help="skip captures that already have bag_report.json in the output")
    ap.add_argument("--only", nargs="+", default=None, help="convert only these relative paths")
    ap.add_argument("--workers", type=int, default=max(2, min(8, os.cpu_count() or 4)))
    ap.add_argument("--extra", default="", help="extra arguments for bag_to_cuvslam.py")
    args = ap.parse_args()

    root = os.path.expanduser(args.root)
    out_root = os.path.expanduser(args.output)
    caps = find_captures(root, out_root)
    if args.only:
        caps = [c for c in caps if c[0] in args.only]

    print(f"Captures found: {len(caps)}")
    for rel, src, kind in caps:
        size = sum(os.path.getsize(f) for f in bag_files(src)) / 1e9
        print(f"  {rel:45s} {kind:9s} {size:6.1f} GB  {', '.join(os.path.basename(f) for f in bag_files(src))}")
    if args.list:
        return

    os.makedirs(out_root, exist_ok=True)
    free = shutil.disk_usage(out_root).free / 1e9
    print(f"Free space at destination: {free:.1f} GB")

    summary_path = os.path.join(out_root, "conversion_summary.json")
    summary = {}
    if os.path.exists(summary_path):
        with open(summary_path) as f:
            summary = json.load(f)

    for n, (rel, src, kind) in enumerate(caps, 1):
        dst = os.path.join(out_root, rel)
        if args.skip_existing and os.path.exists(os.path.join(dst, "bag_report.json")):
            print(f"[{n}/{len(caps)}] {rel}: already converted, skipping")
            continue
        os.makedirs(dst, exist_ok=True)
        log_path = os.path.join(dst, "convert.log")
        cmd = [sys.executable, os.path.join(HERE, "bag_to_cuvslam.py"), src, "-o", dst,
               "--reference", "auto", "--overwrite", "--workers", str(args.workers)] + args.extra.split()
        print(f"[{n}/{len(caps)}] {rel} ({kind}) ...", flush=True)
        t0 = time.time()
        with open(log_path, "w") as lf:
            lf.write(" ".join(cmd) + "\n\n")
            lf.flush()
            rc = subprocess.call(cmd, stdout=lf, stderr=subprocess.STDOUT)
        dt = time.time() - t0
        with open(log_path) as lf:
            tail = lf.read().strip().splitlines()[-3:]
        status = "ok" if rc == 0 else "error"
        if rc != 0 and any("No cameras found" in t for t in tail):
            status = "no cameras"
        summary[rel] = {"source": src, "kind": kind, "status": status, "returncode": rc,
                        "seconds": round(dt, 1), "log": log_path, "tail": tail}
        print(f"    {status} in {dt:.0f} s" + ("" if rc == 0 else f" -> {tail[-1] if tail else ''}"), flush=True)
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)

    ok = sum(v["status"] == "ok" for v in summary.values())
    print(f"\nDone: {ok}/{len(summary)} captures converted. Summary: {summary_path}")


if __name__ == "__main__":
    main()
