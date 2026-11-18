#!/usr/bin/env python3
"""
check_depth_sync.py - Measures the timestamps of the depth topics against the RGB and IR images,
for every capture of a folder, before exporting depth to EDEX.

For each capture and camera (cam1, cam2) it reads only the message headers (fast, no image decoding) of:
    /<cam>/color/image_raw                       RGB
    /<cam>/infra1/image_rect_raw                 IR (same clock as the native depth)
    /<cam>/depth/image_rect_raw                  native depth (infra1 viewpoint)
    /<cam>/aligned_depth_to_color/image_raw      depth aligned to the RGB by the realsense node
and reports, for each pair, the offset of every message of the first topic from the nearest message
of the second topic: share of identical stamps, median, 95th percentile, maximum, and share within
1, 2, 5, 8 and 10 ms.

Pairs:
    aligned -> rgb      which stamp the node puts on the aligned depth (0 ms = the RGB stamp)
    aligned -> depth    (0 ms = the stamp of the native depth frame)
    rgb -> depth        what the depth computed from the native depth would get, per RGB frame
    rgb -> aligned      what the recorded aligned depth gives, per RGB frame
    depth -> ir1        native depth vs infra1 (expected 0 ms)

Usage:
    python3 check_depth_sync.py ~/dotlumen/Captures/original -o depth_sync.json
    python3 check_depth_sync.py ~/dotlumen/Captures/original --only 9_Loop_closure/loop_closure
"""

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag2edex import Bag, header_stamp  # noqa: E402
from convert_all import find_captures  # noqa: E402

TOPICS = {"rgb": "/{cam}/color/image_raw", "ir1": "/{cam}/infra1/image_rect_raw",
          "depth": "/{cam}/depth/image_rect_raw", "aligned": "/{cam}/aligned_depth_to_color/image_raw"}
PAIRS = [("aligned", "rgb"), ("aligned", "depth"), ("rgb", "depth"), ("rgb", "aligned"), ("depth", "ir1")]
WITHIN_MS = (1, 2, 5, 8, 10)


def offsets_ms(a, b):
    """For each stamp of a, absolute offset (ms) from the nearest stamp of b (both sorted, ns)."""
    if not len(a) or not len(b):
        return None
    j = np.searchsorted(b, a)
    lo = np.abs(a - b[np.clip(j - 1, 0, len(b) - 1)])
    hi = np.abs(b[np.clip(j, 0, len(b) - 1)] - a)
    return np.minimum(lo, hi) / 1e6


def stats(d):
    if d is None:
        return None
    out = {"n": int(len(d)), "identical": float(np.mean(d == 0)),
           "median_ms": float(np.median(d)), "p95_ms": float(np.percentile(d, 95)), "max_ms": float(d.max())}
    for w in WITHIN_MS:
        out[f"within_{w}ms"] = float(np.mean(d <= w))
    return out


def analyse(src):
    bag = Bag(src)
    types = bag.topic_types()
    cams = sorted({t.split("/")[1] for t in types if t.endswith("/color/image_raw")})
    wanted = {TOPICS[k].format(cam=c) for c in cams for k in TOPICS} & set(types)
    stamps = {t: [] for t in wanted}
    for topic, recv, _key, data in bag.iterate(wanted, prefix_only=True):
        st = header_stamp(data)
        stamps[topic].append(st if st else recv)
    res = {}
    for c in cams:
        s = {k: np.array(sorted(set(stamps.get(TOPICS[k].format(cam=c), []))), dtype=np.int64) for k in TOPICS}
        dur = {k: (v[-1] - v[0]) / 1e9 for k, v in s.items() if len(v) > 1}
        res[c] = {"messages": {k: int(len(v)) for k, v in s.items()},
                  "rate_hz": {k: round(float((len(s[k]) - 1) / dur[k]), 2) for k in dur if dur[k] > 0},
                  "pairs": {f"{a}->{b}": stats(offsets_ms(s[a], s[b])) for a, b in PAIRS}}
    return res


def fmt(p):
    if not p:
        return "-"
    return (f"same {100 * p['identical']:5.1f}%  med {p['median_ms']:6.2f}  p95 {p['p95_ms']:6.2f}  "
            f"<=1ms {100 * p['within_1ms']:5.1f}%  <=8ms {100 * p['within_8ms']:5.1f}%")


def main():
    ap = argparse.ArgumentParser(description="Timestamps of depth vs RGB and IR in every capture")
    ap.add_argument("root", help="captures folder (e.g. ~/dotlumen/Captures/original)")
    ap.add_argument("-o", "--output", default="depth_sync.json", help="JSON file with all the statistics")
    ap.add_argument("--only", nargs="+", default=None, help="only these relative capture paths")
    args = ap.parse_args()

    root = os.path.expanduser(args.root)
    caps = find_captures(root, os.path.join(root, "edex"))
    caps = [c for c in caps if not args.only or c[0] in args.only]
    caps = [c for c in caps if not c[0].split(os.sep)[0] in ("mcap", "edex", "edex_ir")]
    out = {}
    for n, (rel, src, kind) in enumerate(caps, 1):
        print(f"[{n}/{len(caps)}] {rel} ({kind})", flush=True)
        try:
            out[rel] = {"source": src, "cameras": analyse(src)}
        except Exception as e:  # noqa: BLE001  (report and continue)
            out[rel] = {"source": src, "error": str(e)}
            print(f"    error: {e}")
            continue
        for cam, r in out[rel]["cameras"].items():
            m = r["messages"]
            print(f"    {cam}: rgb {m['rgb']} ir1 {m['ir1']} depth {m['depth']} aligned {m['aligned']}  "
                  f"rates {r['rate_hz']}")
            for k, p in r["pairs"].items():
                if p:
                    print(f"      {k:16s} {fmt(p)}")
        with open(os.path.expanduser(args.output), "w") as f:
            json.dump(out, f, indent=1)
    print(f"\nSaved {args.output}")


if __name__ == "__main__":
    main()
