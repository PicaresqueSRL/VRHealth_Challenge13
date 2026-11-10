#!/usr/bin/env python3
"""
compare_runs.py - Compares cuVSLAM runs saved in the Isaac ROS CSV format
(extract_paths.py for Isaac ROS, --out-dir of track_edex_rerun.py / track_bag_rerun.py for PyCuVSLAM).

Each run is a folder with odometry.csv and/or slam_poses.csv (pose columns
timestamp,x,y,z,qx,qy,qz,qw,...) and optionally status.csv, slam_metrics.csv, run_info.json.

What it computes, for each trajectory type (odometry, slam_poses):
  * per run: poses, duration, path length measured in two ways (sum over all consecutive poses,
    which includes frame-to-frame jitter, and over one pose per second), start-end distance
    (the drift of a closed loop), height range, speed (median and 95th percentile),
    frames lost, frames with loop closure, tracking time;
  * per pair of runs: poses matched by timestamp, position difference without alignment, and
    after a rigid alignment (Umeyama, rotation + translation): RMSE, maximum and rotation difference;
  * against the ground truth (--gt paths3d.csv): IcpAlign.py rigid (--no-scale) and with scale,
    as in results/ground_truth/RunBatchPipeline.py.

Output (--out):
  report.md        tables with all the metrics
  metrics.json     the same metrics, machine readable
  pairs.csv        per-pair metrics
  plots/*.png      top view, height over time, pairwise difference over time, tracked points,
                   trajectories aligned to the ground truth
  icp/             IcpAlign.py aligned CSV files and logs

Usage:
    R=~/dotlumen/VRHealth_Challenge13/results
    python3 compare_runs.py \\
        ros=$R/slam_csv/cam2_stereo_imu/loop_closure \\
        bag=$R/pycuvslam/bag/cam2_stereo_imu/loop_closure \\
        edex=$R/pycuvslam/edex/cam2_stereo_imu/loop_closure \\
        --gt $R/ground_truth/loop_closure/paths3d.csv \\
        --out $R/comparisons/cam2_stereo_imu/loop_closure

A run is given as label=folder (or just folder: the label is the folder name).
The first run is the reference for the plots (the others are aligned to it).
Requirements: numpy, matplotlib (IcpAlign.py also needs numpy).
"""

import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ICP = os.path.normpath(os.path.join(HERE, "..", "results", "ground_truth", "IcpAlign.py"))
TRAJECTORIES = ("odometry", "slam_poses")
# categorical palette (fixed order, never cycled) and a neutral color for the ground truth
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
GT_COLOR = "#5f5e5a"
INK, MUTED, GRID = "#2b2b29", "#6b6a64", "#e4e3de"


# -----------------------------------------------------------------------------
# Reading
# -----------------------------------------------------------------------------
def read_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def load_poses(path):
    """-> array N x 8 (t, x, y, z, qx, qy, qz, qw), sorted by time."""
    header, rows = read_csv(path)
    idx = [header.index(k) for k in ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")]
    a = np.array([[float(r[i]) for i in idx] for r in rows if r], dtype=float)
    return a[np.argsort(a[:, 0], kind="stable")] if len(a) else np.zeros((0, 8))


def load_run(label, folder):
    run = {"label": label, "folder": os.path.abspath(folder), "traj": {}, "info": {}}
    for t in TRAJECTORIES:
        p = os.path.join(folder, f"{t}.csv")
        if os.path.exists(p):
            run["traj"][t] = load_poses(p)
    if not run["traj"]:
        sys.exit(f"{folder}: no odometry.csv or slam_poses.csv")
    p = os.path.join(folder, "run_info.json")
    if os.path.exists(p):
        with open(p) as f:
            run["info"] = json.load(f)
    p = os.path.join(folder, "status.csv")
    if os.path.exists(p):
        header, rows = read_csv(p)
        run["status_header"], run["status"] = header, rows
    p = os.path.join(folder, "slam_metrics.csv")
    if os.path.exists(p):
        run["slam_metrics"] = read_csv(p)
    return run


# -----------------------------------------------------------------------------
# Geometry
# -----------------------------------------------------------------------------
def quat_to_rot(q):
    x, y, z, w = q / (np.linalg.norm(q) or 1.0)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) if len(p) > 1 else 0.0


def sample_every(a, period_s=1.0):
    """One pose per period (the first pose at or after each tick, plus the last pose)."""
    ticks = np.arange(a[0, 0], a[-1, 0], period_s)
    idx = np.unique(np.append(np.searchsorted(a[:, 0], ticks), len(a) - 1))
    return a[idx]


def umeyama(src, dst):
    """Rigid alignment (R, t) minimizing |R src + t - dst|."""
    ms, md = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, md - R @ ms


def associate(ta, tb, tol_s):
    """Nearest-timestamp 1:1 association within tol_s -> (indices in a, indices in b)."""
    j = np.searchsorted(tb, ta)
    ia, ib, used = [], [], set()
    for i, jj in enumerate(j):
        best = None
        for c in (jj - 1, jj):
            if 0 <= c < len(tb) and abs(tb[c] - ta[i]) <= tol_s and c not in used:
                if best is None or abs(tb[c] - ta[i]) < abs(tb[best] - ta[i]):
                    best = c
        if best is not None:
            used.add(best)
            ia.append(i)
            ib.append(best)
    return np.array(ia, dtype=int), np.array(ib, dtype=int)


def rotation_diff_deg(qa, qb, R_align):
    """Angle between R_align * Ra and Rb for each matched pose (degrees)."""
    out = []
    for a, b in zip(qa, qb):
        Rd = (R_align @ quat_to_rot(a)).T @ quat_to_rot(b)
        out.append(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rd) - 1) / 2)))))
    return np.array(out)


# -----------------------------------------------------------------------------
# Metrics
# -----------------------------------------------------------------------------
def run_metrics(run, traj):
    a = run["traj"].get(traj)
    if a is None or len(a) < 2:
        return None
    p = a[:, 1:4]
    s = sample_every(a)
    v = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(np.diff(a[:, 0]), 1e-6)
    m = {"poses": len(a), "duration_s": float(a[-1, 0] - a[0, 0]), "first_timestamp": float(a[0, 0]),
         "path_length_all_poses_m": path_length(p),
         "path_length_1s_m": path_length(s[:, 1:4]),
         "start_end_distance_m": float(np.linalg.norm(p[-1] - p[0])),
         "z_min_m": float(p[:, 2].min()), "z_max_m": float(p[:, 2].max()),
         "speed_median_mps": float(np.median(v)), "speed_p95_mps": float(np.percentile(v, 95))}
    if "status" in run:
        h = run["status_header"]
        if "vo_state" in h:
            k = h.index("vo_state")
            m["frames_lost"] = sum(r[k] not in ("1", "3") for r in run["status"] if r)
        if "track_time_ms" in h:
            k = h.index("track_time_ms")
            m["track_time_ms_median"] = float(np.median([float(r[k]) for r in run["status"] if r]))
    if "slam_metrics" in run:
        h, rows = run["slam_metrics"]
        if "lc_status" in h:
            k = h.index("lc_status")
            m["frames_with_loop_closure"] = sum(r[k].lower() in ("true", "1") for r in rows if r)
    return m


def pair_metrics(a, b, tol_s):
    ia, ib = associate(a[:, 0], b[:, 0], tol_s)
    if len(ia) < 3:
        return {"matched": int(len(ia))}, None
    A, B = a[ia, 1:4], b[ib, 1:4]
    raw = np.linalg.norm(A - B, axis=1)
    R, t = umeyama(A, B)
    err = np.linalg.norm(A @ R.T + t - B, axis=1)
    rot = rotation_diff_deg(a[ia, 4:8], b[ib, 4:8], R)
    m = {"matched": int(len(ia)), "unmatched_a": int(len(a) - len(ia)), "unmatched_b": int(len(b) - len(ib)),
         "raw_mean_m": float(raw.mean()), "raw_max_m": float(raw.max()),
         "aligned_rmse_m": float(np.sqrt((err ** 2).mean())), "aligned_max_m": float(err.max()),
         "aligned_rot_mean_deg": float(rot.mean()), "aligned_rot_max_deg": float(rot.max())}
    return m, (a[ia, 0], err)


def run_icp(icp_script, src_csv, gt_csv, out_csv, log, scale):
    cmd = [sys.executable, icp_script, src_csv, gt_csv, out_csv, log] + ([] if scale else ["--no-scale"])
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(log):
        return {"error": (r.stderr or r.stdout).strip().splitlines()[-1:] or ["IcpAlign.py failed"]}
    with open(log) as f:
        text = f.read()
    out = {}
    for key, name in (("scale", r"Applied Scale Factor"), ("rmse_m", r"Point-to-Target RMSE"),
                      ("symmetric_rmse_m", r"Symmetric Chamfer RMSE"), ("mae_m", r"Mean Euclidean Error")):
        m = re.search(name + r"\s*:\s*([-0-9.eE+]+)", text)
        if m:
            out[key] = float(m.group(1))
    out["aligned_csv"] = out_csv
    return out


def load_xyz(path):
    header, rows = read_csv(path)
    up = [h.strip().upper() for h in header]
    ix, iy, iz = up.index("X"), up.index("Y"), up.index("Z")
    sub = up.index("SUBPATH") if "SUBPATH" in up else None
    pts = np.array([[float(r[ix]), float(r[iy]), float(r[iz])] for r in rows if r])
    subs = [r[sub] for r in rows if r] if sub is not None else ["0"] * len(pts)
    return pts, subs


# -----------------------------------------------------------------------------
# Plots
# -----------------------------------------------------------------------------
def style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=11, color=INK)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=9)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=9)
    ax.tick_params(colors=MUTED, labelsize=8, length=0)
    ax.grid(True, color=GRID, linewidth=0.8)
    for s in ax.spines.values():
        s.set_visible(False)


def legend(ax):
    ax.legend(frameon=False, fontsize=8, labelcolor=INK, loc="best")


def make_plots(runs, traj, pairs_series, gt, icp_res, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(out_dir, exist_ok=True)
    files = []
    present = [r for r in runs if traj in r["traj"] and len(r["traj"][traj]) > 2]
    if not present:
        return files
    ref = present[0]["traj"][traj]
    t0 = min(r["traj"][traj][0, 0] for r in present)

    # top view and height, every run rigidly aligned to the first run
    aligned = {}
    for r in present:
        a = r["traj"][traj]
        if r is present[0]:
            aligned[r["label"]] = a[:, 1:4]
            continue
        ia, ib = associate(a[:, 0], ref[:, 0], 0.02)
        if len(ia) >= 3:
            R, t = umeyama(a[ia, 1:4], ref[ib, 1:4])
            aligned[r["label"]] = a[:, 1:4] @ R.T + t
        else:
            aligned[r["label"]] = a[:, 1:4]

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    for k, r in enumerate(present):
        p = aligned[r["label"]]
        ax.plot(p[:, 0], p[:, 1], color=COLORS[k % len(COLORS)], linewidth=2, label=r["label"])
        ax.plot(p[0, 0], p[0, 1], "o", color=COLORS[k % len(COLORS)], markersize=8,
                markeredgecolor="white", markeredgewidth=2)
        ax.plot(p[-1, 0], p[-1, 1], "s", color=COLORS[k % len(COLORS)], markersize=8,
                markeredgecolor="white", markeredgewidth=2)
    ax.set_aspect("equal", adjustable="datalim")
    style(ax, f"{traj}: top view (aligned to {present[0]['label']}; circle = start, square = end)",
          "x [m]", "y [m]")
    legend(ax)
    f = os.path.join(out_dir, f"{traj}_top_view.png")
    fig.tight_layout()
    fig.savefig(f, dpi=130)
    plt.close(fig)
    files.append(f)

    # z as saved (base frame at the first pose): the rigid alignment would tilt it, so it is not used here.
    # The base frame is not gravity-aligned: a camera pitched at the start turns horizontal motion into z.
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for k, r in enumerate(present):
        a = r["traj"][traj]
        ax.plot(a[:, 0] - t0, a[:, 3], color=COLORS[k % len(COLORS)], linewidth=2, label=r["label"])
    style(ax, f"{traj}: z over time (base frame at the first pose, not gravity-aligned)", "time [s]", "z [m]")
    legend(ax)
    f = os.path.join(out_dir, f"{traj}_height.png")
    fig.tight_layout()
    fig.savefig(f, dpi=130)
    plt.close(fig)
    files.append(f)

    # pairwise difference after alignment over time
    if pairs_series:
        fig, ax = plt.subplots(figsize=(9, 3.8))
        for k, (name, (ts, err)) in enumerate(pairs_series.items()):
            ax.plot(ts - t0, err, color=COLORS[k % len(COLORS)], linewidth=2, label=name)
        style(ax, f"{traj}: position difference between runs after rigid alignment", "time [s]",
              "difference [m]")
        legend(ax)
        f = os.path.join(out_dir, f"{traj}_pair_difference.png")
        fig.tight_layout()
        fig.savefig(f, dpi=130)
        plt.close(fig)
        files.append(f)

    # trajectories aligned to the ground truth by IcpAlign.py: one plot for the rigid alignment
    # (<traj>_vs_ground_truth.png) and one for the alignment with scale (<traj>_vs_ground_truth_scaled.png)
    if gt is not None:
        pts, subs = gt
        for mode, suffix, title in (("rigid", "", "rigid ICP alignment"),
                                    ("scaled", "_scaled", "ICP alignment with scale")):
            fig, ax = plt.subplots(figsize=(7.5, 6.5))
            for s in dict.fromkeys(subs):
                sel = np.array([x == s for x in subs])
                ax.plot(pts[sel, 0], pts[sel, 1], color=GT_COLOR, linewidth=3, alpha=0.6,
                        label="ground truth" if s == subs[0] else None)
            drawn = 0
            for k, r in enumerate(runs):
                res = icp_res.get((r["label"], traj, mode))
                if not res or "aligned_csv" not in res or not os.path.exists(res["aligned_csv"]):
                    continue
                p, _ = load_xyz(res["aligned_csv"])
                label = f"{r['label']} (RMSE {res.get('rmse_m', float('nan')):.2f} m"
                if mode == "scaled":
                    label += f", scale {res.get('scale', float('nan')):.3f}"
                ax.plot(p[:, 0], p[:, 1], color=COLORS[k % len(COLORS)], linewidth=2, label=label + ")")
                drawn += 1
            if not drawn:
                plt.close(fig)
                continue
            ax.set_aspect("equal", adjustable="datalim")
            style(ax, f"{traj}: {title} to the ground truth", "X [m]", "Y [m]")
            legend(ax)
            f = os.path.join(out_dir, f"{traj}_vs_ground_truth{suffix}.png")
            fig.tight_layout()
            fig.savefig(f, dpi=130)
            plt.close(fig)
            files.append(f)
    return files


def plot_tracking(runs, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    have = [r for r in runs if "status" in r and any(h.startswith("observations_cam") for h in r["status_header"])]
    if not have:
        return []
    t0 = min(float(r["status"][0][0]) for r in have)
    fig, ax = plt.subplots(figsize=(9, 3.8))
    for k, r in enumerate(runs):
        if r not in have:
            continue
        h = r["status_header"]
        cols = [i for i, x in enumerate(h) if x.startswith("observations_cam")]
        t = np.array([float(x[0]) for x in r["status"]])
        n = np.array([sum(float(x[i]) for i in cols) for x in r["status"]])
        ax.plot(t - t0, n, color=COLORS[k % len(COLORS)], linewidth=1.5, label=r["label"])
    style(ax, "Tracked points per frame (all cameras)", "time [s]", "observations")
    legend(ax)
    f = os.path.join(out_dir, "tracked_points.png")
    fig.tight_layout()
    fig.savefig(f, dpi=130)
    plt.close(fig)
    return [f]


# -----------------------------------------------------------------------------
# Report
# -----------------------------------------------------------------------------
def fmt(v, nd=2):
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.{nd}f}"
    return str(v)


def md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(fmt(x) for x in r) + " |" for r in rows]
    return "\n".join(out)


RUN_ROWS = [("poses", "Poses"), ("duration_s", "Duration [s]"),
            ("path_length_all_poses_m", "Path length, all poses [m]"),
            ("path_length_1s_m", "Path length, one pose per second [m]"),
            ("start_end_distance_m", "Start-end distance [m]"),
            ("z_min_m", "z min [m]"), ("z_max_m", "z max [m]"),
            ("speed_median_mps", "Speed, median [m/s]"), ("speed_p95_mps", "Speed, 95th percentile [m/s]"),
            ("frames_lost", "Frames lost"), ("frames_with_loop_closure", "Frames with loop closure"),
            ("track_time_ms_median", "Tracking time, median [ms]")]


def main():
    ap = argparse.ArgumentParser(description="Compare cuVSLAM runs saved in the Isaac ROS CSV format",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run folders, as label=folder or folder")
    ap.add_argument("--gt", default=None, help="ground-truth paths3d.csv (enables the IcpAlign.py comparison)")
    ap.add_argument("--icp-script", default=DEFAULT_ICP, help="IcpAlign.py")
    ap.add_argument("--out", required=True, help="output folder")
    ap.add_argument("--tolerance-ms", type=float, default=5.0,
                    help="maximum timestamp difference to match poses of two runs")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    runs = []
    for spec in args.runs:
        label, _, folder = spec.partition("=") if "=" in spec else (None, None, spec)
        folder = os.path.expanduser(folder)
        runs.append(load_run(label or os.path.basename(os.path.normpath(folder)), folder))
    labels = [r["label"] for r in runs]
    if len(set(labels)) != len(labels):
        sys.exit(f"Duplicate labels {labels}: use label=folder")
    out = os.path.abspath(os.path.expanduser(args.out))
    os.makedirs(out, exist_ok=True)

    gt = None
    if args.gt:
        if not os.path.exists(args.icp_script):
            sys.exit(f"IcpAlign.py not found: {args.icp_script} (use --icp-script)")
        gt = load_xyz(os.path.expanduser(args.gt))

    report = {"runs": {r["label"]: {"folder": r["folder"], "info": {k: v for k, v in r["info"].items()
                                                                 if k not in ("T_rig_from_base", "summary")}}
                       for r in runs},
              "metrics": {}, "pairs": {}, "ground_truth": {}}
    md = [f"# cuVSLAM run comparison", "",
          "Runs: " + ", ".join(f"`{r['label']}` = `{r['folder']}`" for r in runs), ""]
    if args.gt:
        gt_len = sum(path_length(gt[0][np.array([x == s for x in gt[1]])]) for s in dict.fromkeys(gt[1]))
        md += [f"Ground truth: `{os.path.abspath(os.path.expanduser(args.gt))}` (path length {gt_len:.2f} m)", ""]
        report["ground_truth_path_length_m"] = gt_len
    md += ["Path length is given in two ways: the sum over all consecutive poses includes the "
           "frame-to-frame jitter; one pose per second measures the distance actually walked.", ""]

    icp_res, plot_files, pair_rows = {}, [], []
    for traj in TRAJECTORIES:
        if not any(traj in r["traj"] for r in runs):
            continue
        md += [f"## {traj}", ""]
        mets = {r["label"]: run_metrics(r, traj) for r in runs}
        report["metrics"][traj] = mets
        rows = []
        for key, name in RUN_ROWS:
            vals = [mets[r["label"]].get(key) if mets[r["label"]] else None for r in runs]
            if any(v is not None for v in vals):
                rows.append([name] + vals)
        md += [md_table(["Metric"] + labels, rows), ""]

        # pairs
        series, prow = {}, []
        for i in range(len(runs)):
            for j in range(i + 1, len(runs)):
                a, b = runs[i]["traj"].get(traj), runs[j]["traj"].get(traj)
                if a is None or b is None:
                    continue
                name = f"{runs[i]['label']} vs {runs[j]['label']}"
                m, s = pair_metrics(a, b, args.tolerance_ms / 1000)
                report["pairs"].setdefault(traj, {})[name] = m
                if s is not None:
                    series[name] = s
                prow.append([name, m.get("matched"), m.get("raw_mean_m"), m.get("raw_max_m"),
                             m.get("aligned_rmse_m"), m.get("aligned_max_m"),
                             m.get("aligned_rot_mean_deg"), m.get("aligned_rot_max_deg")])
                pair_rows.append([traj] + prow[-1])
        if prow:
            md += ["Pairwise, poses matched by timestamp; 'aligned' = after a rigid alignment of the two runs:", "",
                   md_table(["Pair", "Matched", "Raw mean [m]", "Raw max [m]", "Aligned RMSE [m]",
                             "Aligned max [m]", "Rotation mean [deg]", "Rotation max [deg]"], prow), ""]

        # ground truth
        if gt is not None:
            grow = []
            for r in runs:
                if traj not in r["traj"]:
                    continue
                src = os.path.join(r["folder"], f"{traj}.csv")
                for mode in ("rigid", "scaled"):
                    d = os.path.join(out, "icp")
                    os.makedirs(d, exist_ok=True)
                    res = run_icp(args.icp_script, src, os.path.expanduser(args.gt),
                                  os.path.join(d, f"{r['label']}_{traj}_{mode}.csv"),
                                  os.path.join(d, f"{r['label']}_{traj}_{mode}.log"), mode == "scaled")
                    icp_res[(r["label"], traj, mode)] = res
                    report["ground_truth"].setdefault(traj, {}).setdefault(r["label"], {})[mode] = res
                rg, sc = icp_res[(r["label"], traj, "rigid")], icp_res[(r["label"], traj, "scaled")]
                grow.append([r["label"], rg.get("rmse_m"), rg.get("symmetric_rmse_m"), rg.get("mae_m"),
                             sc.get("scale"), sc.get("rmse_m")])
            md += ["Against the ground truth (IcpAlign.py):", "",
                   md_table(["Run", "Rigid RMSE [m]", "Rigid symmetric RMSE [m]", "Rigid MAE [m]",
                             "Scale factor", "Scaled RMSE [m]"], grow), ""]

        if not args.no_plots:
            plot_files += make_plots(runs, traj, series, gt, icp_res, os.path.join(out, "plots"))

    if not args.no_plots:
        plot_files += plot_tracking(runs, os.path.join(out, "plots"))
        if plot_files:
            md += ["## Plots", ""] + [f"![{os.path.basename(p)}](plots/{os.path.basename(p)})" for p in plot_files] + [""]

    with open(os.path.join(out, "report.md"), "w") as f:
        f.write("\n".join(md))
    with open(os.path.join(out, "metrics.json"), "w") as f:
        json.dump(report, f, indent=2, default=str)
    with open(os.path.join(out, "pairs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trajectory", "pair", "matched", "raw_mean_m", "raw_max_m", "aligned_rmse_m",
                    "aligned_max_m", "rotation_mean_deg", "rotation_max_deg"])
        w.writerows(pair_rows)
    print("\n".join(md[:]), flush=True)
    print(f"\nReport: {os.path.join(out, 'report.md')}")


if __name__ == "__main__":
    main()
