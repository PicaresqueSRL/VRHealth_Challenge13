#!/usr/bin/env python3
"""
isaac_localize_summary.py - Summary of the Isaac ROS relocalization queries of all captures, per input configuration,
and comparison between the configurations.

Configurations (--config, default all: every configuration with a folder in isaac_localize/):
  cam2_stereo_imu   cam2 IR stereo + IMU
  cam1_cam2_stereo  cam2 + cam1 IR stereo, multicamera, no IMU
For each configuration reads <root>/isaac_localize/<config>/<capture>/queries.csv (localize_query.py),
<root>/isaac_maps/<config>/<capture>/run_info.json and slam_poses.csv (map_cam2.sh) and the Rerun files
<root>/rerun/<config>/<capture>.rrd, and writes <root>/isaac_localize/<config>/summary.csv and summary.md.
With two or more configurations also writes <root>/isaac_localize/comparison.md and comparison.csv: per capture and
t_prev, and in total, both metrics side by side, plus the SLAM path length of each mapping run vs the Jetson slam_csv
run of the same configuration (--slam-csv-root).

Two success metrics per query (threshold --thr-t / --thr-r):
  localizer: position logged by visual_slam_node ("Successfully localized at {x, y, z}") within thr_t of the reference
             at the closest frame between the call and the log (translation only: the log has no rotation);
  published: pose published by Isaac ROS (map->odom * odometry) at the first settled frame after the localization,
             within thr_t and thr_r of the reference.
Success within N attempts: attempt_ok <= N (N = 1, 2, 4, 8, 16).
Segment after localization (settled frame -> t2, localized queries): post_rmse_t_m (published vs reference at the same
frame, no alignment) and post_rmsd_aligned_m (after the best rigid alignment: shape and drift only), median and maximum;
median number of points (post_frames) and duration (post_duration_s). "-" where the queries.csv has no such column.

Usage: python3 isaac_localize_summary.py [--root ~/dotlumen/localization] [--config all|cam2_stereo_imu|cam1_cam2_stereo]
       one configuration with explicit folders (as before the configurations):
       python3 isaac_localize_summary.py --config cam2_stereo_imu --localize-root <dir> --maps-root <dir> --rerun-root <dir>
"""
import argparse
import csv
import json
import math
import os
import statistics

NS_LIST = [1, 2, 4, 8, 16]
CONFIGS = ["cam2_stereo_imu", "cam1_cam2_stereo"]
CONFIG_TITLE = {"cam2_stereo_imu": "cam2 IR stereo + IMU", "cam1_cam2_stereo": "cam2 + cam1 IR stereo, no IMU"}
# captures whose results are kept but flagged as not reliable, per configuration (CAPTURE=REASON)
UNRELIABLE = {
    "cam2_stereo_imu": [
        "staircases_fix_1=the Isaac ROS stereo + IMU mapping run diverges at 20-22 s (SLAM path 130.7 m instead of "
        "~22 m, z down to -30.7 m), and so do 2 repeated desktop runs and the Jetson run; the bag is clean and the "
        "stereo-only run is plausible (31.4 m): failure of the cuVSLAM 15 visual-inertial fusion on this capture, so "
        "map and reference are wrong (report section 6b)"],
    "cam1_cam2_stereo": [],
}


def path_length(csv_path):
    """Length (m) of the trajectory of an Isaac ROS CSV (None if missing)."""
    if not os.path.exists(csv_path):
        return None
    pts = [tuple(float(r[k]) for k in "xyz") for r in csv.DictReader(open(csv_path))]
    return round(sum(math.dist(a, b) for a, b in zip(pts, pts[1:])), 1)


def fnum(r, k):
    v = r.get(k, "")
    return float(v) if v not in ("", None) else None


def med(rows, k):
    v = [fnum(r, k) for r in rows if fnum(r, k) is not None]
    return round(statistics.median(v), 3) if v else None


def vmax(rows, k):
    v = [fnum(r, k) for r in rows if fnum(r, k) is not None]
    return round(max(v), 3) if v else None


def counts(rows, n, key):
    loc = [r for r in rows if r["localized"] == "True" and r.get("attempt_ok") and int(r["attempt_ok"]) <= n]
    tp = [r for r in loc if r.get(key) == "True"]
    return {"localized": len(loc), "tp": len(tp), "fp": len(loc) - len(tp), "not_localized": len(rows) - len(loc)}


def group_stats(rows):
    loc = [r for r in rows if r["localized"] == "True"]
    d = {"queries": len(rows)}
    for n in NS_LIST:
        for key, name in (("localizer_tp", "loc"), ("published_tp", "pub")):
            c = counts(rows, n, key)
            d.update({f"N{n}_localized": c["localized"], f"N{n}_{name}_tp": c["tp"], f"N{n}_{name}_fp": c["fp"]})
        d[f"N{n}_not_localized"] = len(rows) - counts(rows, n, "published_tp")["localized"]
    d.update(med_prior_err_t_m=med(rows, "prior_err_t_m"), med_prior_err_r_deg=med(rows, "prior_err_r_deg"),
             med_log_err_t_m=med(loc, "loc_log_err_t_m"), med_pub_err_t_m=med(loc, "loc_err_t_m"),
             med_pub_err_r_deg=med(loc, "loc_err_r_deg"), med_pub_err_t_m_1s=med(loc, "pub_err_t_m_1s"),
             med_pub_err_t_m_2s=med(loc, "pub_err_t_m_2s"), med_final_err_t_m=med(loc, "post_final_err_t_m"),
             med_post_rmse_t_m=med(loc, "post_rmse_t_m"), max_post_rmse_t_m=vmax(loc, "post_rmse_t_m"),
             med_post_rmsd_aligned_m=med(loc, "post_rmsd_aligned_m"),
             max_post_rmsd_aligned_m=vmax(loc, "post_rmsd_aligned_m"),
             med_post_frames=med(loc, "post_frames"), med_post_duration_s=med(loc, "post_duration_s"),
             med_latency_ms=med(loc, "ok_attempt_latency_ms"),
             med_delay_s=med(loc, "loc_delay_s"), med_tf_settle_s=med(loc, "tf_update_delay_s"),
             med_attempts=med(rows, "attempts"))
    return d


def summarize(config, lroot, mroot, rroot, unreliable, slam_csv_root):
    """Summary of one configuration: writes lroot/summary.csv and summary.md; returns (csv_rows, all_rows, maps)."""

    captures = sorted(c for c in os.listdir(lroot) if os.path.isfile(os.path.join(lroot, c, "queries.csv"))
                      and os.path.isdir(os.path.join(mroot, c)))
    all_rows, csv_rows, maps = [], [], []
    for c in captures:
        rows = list(csv.DictReader(open(os.path.join(lroot, c, "queries.csv"))))
        for r in rows:
            r["capture"] = c
        all_rows += rows
        for tp in sorted({r["t_prev_s"] for r in rows}, key=float):
            csv_rows.append({"capture": c, "t_prev_s": tp, "reliable": c not in unreliable,
                             **group_stats([r for r in rows if r["t_prev_s"] == tp])})
        info = json.load(open(os.path.join(mroot, c, "run_info.json")))
        n_ref = sum(1 for _ in open(os.path.join(mroot, c, "slam_poses.csv"))) - 1
        rrd = os.path.join(rroot, f"{c}.rrd")
        maps.append({"capture": c, "poses": n_ref, "map_mb": info.get("map_size_mb"), "play_s": info.get("play_s"),
                     "total_s": info.get("total_s"), "save_map_s": info.get("save_map_s"),
                     "slam_path_m": path_length(os.path.join(mroot, c, "slam_poses.csv")),
                     "jetson_slam_path_m": path_length(os.path.join(slam_csv_root, config, c, "slam_poses.csv")),
                     "rrd_mb": round(os.path.getsize(rrd) / 1e6, 1) if os.path.exists(rrd) else None,
                     "queries": len(rows)})
    for tp in sorted({r["t_prev_s"] for r in all_rows}, key=float):
        csv_rows.append({"capture": "ALL", "t_prev_s": tp, "reliable": "",
                         **group_stats([r for r in all_rows if r["t_prev_s"] == tp])})
    csv_rows.append({"capture": "ALL", "t_prev_s": "all", "reliable": "", **group_stats(all_rows)})
    rel = [r for r in all_rows if r["capture"] not in unreliable]
    if unreliable:
        for tp in sorted({r["t_prev_s"] for r in rel}, key=float):
            csv_rows.append({"capture": "ALL_RELIABLE", "t_prev_s": tp, "reliable": True,
                             **group_stats([r for r in rel if r["t_prev_s"] == tp])})
        csv_rows.append({"capture": "ALL_RELIABLE", "t_prev_s": "all", "reliable": True, **group_stats(rel)})

    header = list(csv_rows[0].keys())
    with open(os.path.join(lroot, "summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(csv_rows)

    L = [f"# Isaac ROS relocalization - summary, {config} ({CONFIG_TITLE.get(config, config)})", "",
         "Captures: " + ", ".join(captures) + f". Queries: {len(all_rows)}.", "",
         "Localizer TP: logged position within 0.3 m of the reference (closest frame between call and log). "
         "Published TP: pose published by Isaac ROS at the first settled frame within 0.3 m and 10 deg. "
         "Errors are medians over the localized queries.", "",
         *([f"**Not reliable** (results kept and listed, flagged): " + "; ".join(
             f"**{c}**: {reason}" for c, reason in unreliable.items()), ""] if unreliable else []),
         "## Totals per t_prev (all captures)", "",
         "| t_prev | queries | localized (N=16) | localizer TP / FP | published TP / FP | not localized | "
         "localizer TP N=1/2/4/8/16 | published TP N=1/2/4/8/16 | prior err m / deg | log err m | published err m / deg "
         "| +1 s m | +2 s m | end m | segment RMSE m med / max | aligned RMSD m med / max | segment points / s | "
         "latency ms | delay from t1 s |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    def fmt(v):
        return "-" if v in (None, "") else v

    def seg(r):
        return (f"{fmt(r['med_post_rmse_t_m'])} / {fmt(r['max_post_rmse_t_m'])} | {fmt(r['med_post_rmsd_aligned_m'])} / "
                f"{fmt(r['max_post_rmsd_aligned_m'])} | {fmt(r['med_post_frames'])} / {fmt(r['med_post_duration_s'])}")

    def total_row(r):
        L.append(f"| {r['t_prev_s']} | {r['queries']} | {r['N16_localized']} | {r['N16_loc_tp']} / {r['N16_loc_fp']} | "
                 f"{r['N16_pub_tp']} / {r['N16_pub_fp']} | {r['N16_not_localized']} | "
                 + "/".join(str(r[f"N{n}_loc_tp"]) for n in NS_LIST) + " | "
                 + "/".join(str(r[f"N{n}_pub_tp"]) for n in NS_LIST)
                 + f" | {r['med_prior_err_t_m']} / {r['med_prior_err_r_deg']} | {r['med_log_err_t_m']} | "
                 f"{r['med_pub_err_t_m']} / {r['med_pub_err_r_deg']} | {r['med_pub_err_t_m_1s']} | "
                 f"{r['med_pub_err_t_m_2s']} | {r['med_final_err_t_m']} | {seg(r)} | {r['med_latency_ms']} | "
                 f"{r['med_delay_s']} |")
    total_header = L[-2:]
    for r in [x for x in csv_rows if x["capture"] == "ALL"]:
        total_row(r)
    if unreliable:
        L += ["", "## Totals per t_prev (reliable captures only: without " + ", ".join(unreliable) + ")", ""]
        L += total_header
        for r in [x for x in csv_rows if x["capture"] == "ALL_RELIABLE"]:
            total_row(r)
    L += ["", "## Per capture and t_prev (N=16)", "",
          "| capture | t_prev | queries | localized | localizer TP / FP | published TP / FP | not localized | "
          "prior err m | log err m | published err m / deg | +2 s m | end m | segment RMSE m med / max | "
          "aligned RMSD m med / max | segment points / s | latency ms |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in [x for x in csv_rows if x["capture"] not in ("ALL", "ALL_RELIABLE")]:
        L.append(f"| {r['capture']}{' (NOT RELIABLE)' if not r['reliable'] else ''} | {r['t_prev_s']} | {r['queries']} | {r['N16_localized']} | {r['N16_loc_tp']} / "
                 f"{r['N16_loc_fp']} | {r['N16_pub_tp']} / {r['N16_pub_fp']} | {r['N16_not_localized']} | "
                 f"{r['med_prior_err_t_m']} | {r['med_log_err_t_m']} | {r['med_pub_err_t_m']} / {r['med_pub_err_r_deg']}"
                 f" | {r['med_pub_err_t_m_2s']} | {r['med_final_err_t_m']} | {seg(r)} | {r['med_latency_ms']} |")
    L += ["", f"## Maps (map_cam2.sh CONFIG={config}, Isaac ROS 5.0, cuVSLAM 15.0.0+33da606)", "",
          "SLAM path: length of slam_poses.csv of the mapping run (desktop) and of the Jetson run of the same "
          "configuration (results/slam_csv).", "",
          "| capture | reference poses | SLAM path m | Jetson SLAM path m | map MB | play s | run s | save_map s | queries "
          "| rrd MB |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for m in maps:
        L.append(f"| {m['capture']} | {m['poses']} | {m['slam_path_m']} | {m['jetson_slam_path_m']} | {m['map_mb']} | "
                 f"{m['play_s']} | {m['total_s']} | {m['save_map_s']} | {m['queries']} | {m['rrd_mb']} |")
    L += ["", "## False positives (published)", "",
          "| capture | query | t1 s | t_prev s | prior err m / deg | log err m | published err m / deg | +2 s m | end m |",
          "|---|---|---|---|---|---|---|---|---|"]
    for r in all_rows:
        if r["localized"] == "True" and r.get("published_tp") != "True":
            L.append(f"| {r['capture']} | {r['query']} | {r['t1_s']} | {r['t_prev_s']} | {r.get('prior_err_t_m')} / "
                     f"{r.get('prior_err_r_deg')} | {r.get('loc_log_err_t_m')} | {r.get('loc_err_t_m')} / "
                     f"{r.get('loc_err_r_deg')} | {r.get('pub_err_t_m_2s')} | {r.get('post_final_err_t_m')} |")
    L += ["", "## Not localized", ""]
    nl = [r for r in all_rows if r["localized"] != "True"]
    L += [f"- {r['capture']} query {r['query']}: t1 {r['t1_s']} s, t_prev {r['t_prev_s']} s, {r['attempts']} attempts,"
          f" prior err {r.get('prior_err_t_m')} m / {r.get('prior_err_r_deg')} deg" for r in nl] or ["- none"]
    open(os.path.join(lroot, "summary.md"), "w").write("\n".join(L) + "\n")
    print(f"[{config}] {len(all_rows)} queries of {len(captures)} captures -> {lroot}/summary.md, summary.csv")
    return csv_rows, all_rows, maps


def fmt(v):
    return "-" if v in (None, "") else v


def compare(configs, results, unreliable, out_dir):
    """Comparison table between configurations: comparison.csv and comparison.md in out_dir."""
    A = {c: {(r["capture"], r["t_prev_s"]): r for r in results[c][0]} for c in configs}
    maps = {c: {m["capture"]: m for m in results[c][2]} for c in configs}
    caps = sorted({r["capture"] for c in configs for r in results[c][0]} - {"ALL", "ALL_RELIABLE"})
    flagged = {cap for c in configs for cap in unreliable[c]}
    both = [cap for cap in caps if all(cap in maps[c] for c in configs)]
    both_rel = [cap for cap in both if cap not in flagged]
    keys = ["queries", "N16_localized", "N16_loc_tp", "N16_loc_fp", "N16_pub_tp", "N16_pub_fp", "N16_not_localized",
            "N1_loc_tp", "N1_pub_tp", "med_prior_err_t_m", "med_log_err_t_m", "med_pub_err_t_m", "med_pub_err_r_deg",
            "med_pub_err_t_m_2s", "med_final_err_t_m", "med_post_rmse_t_m", "max_post_rmse_t_m",
            "med_post_rmsd_aligned_m", "max_post_rmsd_aligned_m", "med_post_frames", "med_post_duration_s",
            "med_latency_ms"]
    rows = []
    for cap in caps:
        tps = sorted({tp for c in configs for (k, tp) in A[c] if k == cap}, key=float)
        for tp in tps:
            row = {"capture": cap, "t_prev_s": tp}
            for c in configs:
                r = A[c].get((cap, tp), {})
                row[f"{c}_reliable"] = "" if not r else cap not in unreliable[c]
                row.update({f"{c}_{k}": r.get(k, "") for k in keys})
            rows.append(row)
    # totals over the captures present in all the configurations (same queries t1 / t_prev)
    for name, subset in (("ALL_COMMON", both), ("ALL_COMMON_RELIABLE", both_rel)):
        sub_rows = {c: [r for r in results[c][1] if r["capture"] in subset] for c in configs}
        tps = sorted({r["t_prev_s"] for c in configs for r in sub_rows[c]}, key=float)
        for tp in tps + ["all"]:
            row = {"capture": name, "t_prev_s": tp}
            for c in configs:
                sel = [r for r in sub_rows[c] if tp == "all" or r["t_prev_s"] == tp]
                row[f"{c}_reliable"] = name.endswith("RELIABLE")
                row.update({f"{c}_{k}": v for k, v in group_stats(sel).items() if k in keys} if sel else {})
            rows.append(row)
    header = ["capture", "t_prev_s"] + [f"{c}_{k}" for c in configs for k in ["reliable"] + keys]
    with open(os.path.join(out_dir, "comparison.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    names = " vs ".join(configs)
    L = [f"# Isaac ROS relocalization - comparison {names}", "",
         "Same captures, same query schedule (t1 every 5 s, t_prev 0/1/3/5 s, 16 attempts, N = 16), each configuration "
         "localized in its own map and measured against its own reference (slam_poses.csv of its mapping run): the "
         "errors are relative to a different reference per configuration.", "",
         "Localizer TP: logged position within 0.3 m of the reference. Published TP: published pose at the first "
         "settled frame within 0.3 m and 10 deg. Cells: " + " / ".join(configs) + ".", ""]
    if flagged:
        L += ["**Not reliable** (flagged, excluded from ALL_COMMON_RELIABLE): " + "; ".join(
            f"{cap} ({', '.join(c for c in configs if cap in unreliable[c])})" for cap in sorted(flagged)), ""]
    L += ["## Mapping runs: SLAM path length (m)", "",
          "| capture | " + " | ".join(f"{c} desktop | {c} Jetson" for c in configs) + " |",
          "|---|" + "---|---|" * len(configs)]
    for cap in caps:
        cells = []
        for c in configs:
            m = maps[c].get(cap, {})
            mark = " (NOT RELIABLE)" if cap in unreliable[c] else ""
            cells += [f"{fmt(m.get('slam_path_m'))}{mark}", f"{fmt(m.get('jetson_slam_path_m'))}"]
        L.append(f"| {cap} | " + " | ".join(cells) + " |")

    def two(row, k):
        return " / ".join(str(fmt(row.get(f"{c}_{k}"))) for c in configs)

    def table(title, sel):
        nonlocal L
        L += ["", title, "",
              "| capture | t_prev | queries | localized | localizer TP | localizer FP | published TP | published FP | "
              "not localized | published TP N=1 | prior err m | log err m | published err m | published err deg | "
              "+2 s m | end m | segment RMSE m med | segment RMSE m max | aligned RMSD m med | aligned RMSD m max | "
              "segment points | segment s |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for row in sel:
            flag = "".join(" (NOT RELIABLE: " + c + ")" for c in configs
                           if row["capture"] in unreliable[c])
            L.append(f"| {row['capture']}{flag} | {row['t_prev_s']} | {two(row, 'queries')} | {two(row, 'N16_localized')} | "
                     f"{two(row, 'N16_loc_tp')} | {two(row, 'N16_loc_fp')} | {two(row, 'N16_pub_tp')} | "
                     f"{two(row, 'N16_pub_fp')} | {two(row, 'N16_not_localized')} | {two(row, 'N1_pub_tp')} | "
                     f"{two(row, 'med_prior_err_t_m')} | {two(row, 'med_log_err_t_m')} | {two(row, 'med_pub_err_t_m')} | "
                     f"{two(row, 'med_pub_err_r_deg')} | {two(row, 'med_pub_err_t_m_2s')} | {two(row, 'med_final_err_t_m')} | "
                     f"{two(row, 'med_post_rmse_t_m')} | {two(row, 'max_post_rmse_t_m')} | "
                     f"{two(row, 'med_post_rmsd_aligned_m')} | {two(row, 'max_post_rmsd_aligned_m')} | "
                     f"{two(row, 'med_post_frames')} | {two(row, 'med_post_duration_s')} |")

    table(f"## Totals per t_prev, captures in all configurations ({len(both)}: {', '.join(both)})",
          [r for r in rows if r["capture"] == "ALL_COMMON"])
    if flagged:
        table(f"## Totals per t_prev, reliable in all configurations ({len(both_rel)}: {', '.join(both_rel)})",
              [r for r in rows if r["capture"] == "ALL_COMMON_RELIABLE"])
    # per capture, all t_prev together, then per t_prev
    per_cap = []
    for cap in caps:
        row = {"capture": cap, "t_prev_s": "all"}
        for c in configs:
            sel = [r for r in results[c][1] if r["capture"] == cap]
            row.update({f"{c}_{k}": v for k, v in group_stats(sel).items() if k in keys} if sel else {})
        per_cap.append(row)
    table("## Per capture (all t_prev)", per_cap)
    table("## Per capture and t_prev", [r for r in rows if not r["capture"].startswith("ALL")])
    open(os.path.join(out_dir, "comparison.md"), "w").write("\n".join(L) + "\n")
    print(f"comparison {names} -> {out_dir}/comparison.md, comparison.csv")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default="~/dotlumen/localization",
                    help="folder with isaac_localize/<config>, isaac_maps/<config>, rerun/<config>")
    ap.add_argument("--config", default="all", choices=["all"] + CONFIGS)
    ap.add_argument("--localize-root", default=None, help="only with one --config: isaac_localize folder of that config")
    ap.add_argument("--maps-root", default=None, help="only with one --config: isaac_maps folder of that config")
    ap.add_argument("--rerun-root", default=None, help="only with one --config: rerun folder of that config")
    ap.add_argument("--slam-csv-root", default="~/dotlumen/VRHealth_Challenge13/results/slam_csv",
                    help="Jetson runs (<root>/<config>/<capture>/slam_poses.csv), for the path length")
    ap.add_argument("--unreliable", nargs="*", default=None,
                    help="[CONFIG:]CAPTURE=REASON: captures whose results are kept but flagged as not reliable (without "
                         "CONFIG: all configurations); replaces the defaults (UNRELIABLE in this script)")
    args = ap.parse_args()
    root = os.path.expanduser(args.root)
    configs = CONFIGS if args.config == "all" else [args.config]
    configs = [c for c in configs if args.config != "all" or os.path.isdir(os.path.join(root, "isaac_localize", c))]
    if (args.localize_root or args.maps_root or args.rerun_root) and len(configs) != 1:
        ap.error("--localize-root / --maps-root / --rerun-root need a single --config")
    if args.unreliable is None:
        unreliable = {c: dict(u.split("=", 1) for u in UNRELIABLE.get(c, [])) for c in configs}
    else:
        unreliable = {c: {} for c in configs}
        for u in args.unreliable:
            key, reason = u.split("=", 1)
            cfg, _, cap = key.rpartition(":")
            for c in ([cfg] if cfg else configs):
                if c in unreliable:
                    unreliable[c][cap] = reason
    results = {}
    for c in configs:
        lroot = os.path.expanduser(args.localize_root or os.path.join(root, "isaac_localize", c))
        mroot = os.path.expanduser(args.maps_root or os.path.join(root, "isaac_maps", c))
        rroot = os.path.expanduser(args.rerun_root or os.path.join(root, "rerun", c))
        results[c] = summarize(c, lroot, mroot, rroot, unreliable[c], os.path.expanduser(args.slam_csv_root))
    if len(configs) > 1:
        compare(configs, results, unreliable, os.path.join(root, "isaac_localize"))


if __name__ == "__main__":
    main()
