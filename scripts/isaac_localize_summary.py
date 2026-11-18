#!/usr/bin/env python3
"""
isaac_localize_summary.py - Summary of the Isaac ROS relocalization queries of all captures.

Reads <localize_root>/<capture>/queries.csv (localize_query.py), <maps_root>/<capture>/run_info.json and
slam_poses.csv (map_cam2.sh) and the Rerun files <rerun_root>/<capture>.rrd, and writes
<localize_root>/summary.csv and <localize_root>/summary.md.

Two success metrics per query (threshold --thr-t / --thr-r):
  localizer: position logged by visual_slam_node ("Successfully localized at {x, y, z}") within thr_t of the reference
             at the closest frame between the call and the log (translation only: the log has no rotation);
  published: pose published by Isaac ROS (map->odom * odometry) at the first settled frame after the localization,
             within thr_t and thr_r of the reference.
Success within N attempts: attempt_ok <= N (N = 1, 2, 4, 8, 16).

Usage: python3 isaac_localize_summary.py [--localize-root ~/dotlumen/localization/isaac_localize]
                                         [--maps-root ~/dotlumen/localization/isaac_maps]
                                         [--rerun-root ~/dotlumen/localization/rerun]
"""
import argparse
import csv
import json
import os
import statistics

NS_LIST = [1, 2, 4, 8, 16]


def fnum(r, k):
    v = r.get(k, "")
    return float(v) if v not in ("", None) else None


def med(rows, k):
    v = [fnum(r, k) for r in rows if fnum(r, k) is not None]
    return round(statistics.median(v), 3) if v else None


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
             med_post_rmse_t_m=med(loc, "post_rmse_t_m"), med_latency_ms=med(loc, "ok_attempt_latency_ms"),
             med_delay_s=med(loc, "loc_delay_s"), med_tf_settle_s=med(loc, "tf_update_delay_s"),
             med_attempts=med(rows, "attempts"))
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--localize-root", default="~/dotlumen/localization/isaac_localize")
    ap.add_argument("--maps-root", default="~/dotlumen/localization/isaac_maps")
    ap.add_argument("--rerun-root", default="~/dotlumen/localization/rerun")
    ap.add_argument("--unreliable", nargs="*", default=[
        "staircases_fix_1=the Isaac ROS stereo + IMU mapping run diverges at 20-22 s (SLAM path 130.7 m instead of "
        "~22 m, z down to -30.7 m), and so do 2 repeated desktop runs and the Jetson run; the bag is clean and the "
        "stereo-only run is plausible (31.4 m): failure of the cuVSLAM 15 visual-inertial fusion on this capture, so "
        "map and reference are wrong (report section 6b)"],
                    help="CAPTURE=REASON: captures whose results are kept but flagged as not reliable")
    args = ap.parse_args()
    lroot, mroot, rroot = (os.path.expanduser(p) for p in (args.localize_root, args.maps_root, args.rerun_root))
    unreliable = dict(u.split("=", 1) for u in args.unreliable)

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

    L = ["# Isaac ROS relocalization - summary", "",
         "Captures: " + ", ".join(captures) + f". Queries: {len(all_rows)}.", "",
         "Localizer TP: logged position within 0.3 m of the reference (closest frame between call and log). "
         "Published TP: pose published by Isaac ROS at the first settled frame within 0.3 m and 10 deg. "
         "Errors are medians over the localized queries.", "",
         *([f"**Not reliable** (results kept and listed, flagged): " + "; ".join(
             f"**{c}**: {reason}" for c, reason in unreliable.items()), ""] if unreliable else []),
         "## Totals per t_prev (all captures)", "",
         "| t_prev | queries | localized (N=16) | localizer TP / FP | published TP / FP | not localized | "
         "localizer TP N=1/2/4/8/16 | published TP N=1/2/4/8/16 | prior err m / deg | log err m | published err m / deg "
         "| +1 s m | +2 s m | end m | latency ms | delay from t1 s |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    def total_row(r):
        L.append(f"| {r['t_prev_s']} | {r['queries']} | {r['N16_localized']} | {r['N16_loc_tp']} / {r['N16_loc_fp']} | "
                 f"{r['N16_pub_tp']} / {r['N16_pub_fp']} | {r['N16_not_localized']} | "
                 + "/".join(str(r[f"N{n}_loc_tp"]) for n in NS_LIST) + " | "
                 + "/".join(str(r[f"N{n}_pub_tp"]) for n in NS_LIST)
                 + f" | {r['med_prior_err_t_m']} / {r['med_prior_err_r_deg']} | {r['med_log_err_t_m']} | "
                 f"{r['med_pub_err_t_m']} / {r['med_pub_err_r_deg']} | {r['med_pub_err_t_m_1s']} | "
                 f"{r['med_pub_err_t_m_2s']} | {r['med_final_err_t_m']} | {r['med_latency_ms']} | {r['med_delay_s']} |")
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
          "prior err m | log err m | published err m / deg | +2 s m | end m | latency ms |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in [x for x in csv_rows if x["capture"] not in ("ALL", "ALL_RELIABLE")]:
        L.append(f"| {r['capture']}{' (NOT RELIABLE)' if not r['reliable'] else ''} | {r['t_prev_s']} | {r['queries']} | {r['N16_localized']} | {r['N16_loc_tp']} / "
                 f"{r['N16_loc_fp']} | {r['N16_pub_tp']} / {r['N16_pub_fp']} | {r['N16_not_localized']} | "
                 f"{r['med_prior_err_t_m']} | {r['med_log_err_t_m']} | {r['med_pub_err_t_m']} / {r['med_pub_err_r_deg']}"
                 f" | {r['med_pub_err_t_m_2s']} | {r['med_final_err_t_m']} | {r['med_latency_ms']} |")
    L += ["", "## Maps (map_cam2.sh, Isaac ROS 5.0, cuVSLAM 15.0.0+33da606)", "",
          "| capture | reference poses | map MB | play s | run s | save_map s | queries | rrd MB |",
          "|---|---|---|---|---|---|---|---|"]
    for m in maps:
        L.append(f"| {m['capture']} | {m['poses']} | {m['map_mb']} | {m['play_s']} | {m['total_s']} | {m['save_map_s']}"
                 f" | {m['queries']} | {m['rrd_mb']} |")
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
    print(f"{len(all_rows)} queries of {len(captures)} captures -> {lroot}/summary.md, summary.csv")


if __name__ == "__main__":
    main()
