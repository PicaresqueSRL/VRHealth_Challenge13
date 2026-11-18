#!/usr/bin/env python3
"""
isaac_localize_rerun.py - Rerun recording of the Isaac ROS relocalization queries of one capture.

Input: the output folder of isaac_ros_5/launch/localize_query.py (queries.csv, attempts.csv, q<NNN>/trace.csv,
q<NNN>/node_log.txt), the mapping run of map_cam2.sh (odometry.csv, slam_poses.csv) and the capture bag (IR images).
All poses are cam2_link poses in the map frame of the mapping run (ROS axes: x forward, y left, z up).

3D view ('world'):
  world/map/reference      slam_poses.csv of the mapping run (purple)       = reference
  world/map/odometry       odometry.csv of the mapping run (grey)           = source of the prior
  world/query/reference_cut      reference over the cut [t1, t2] (yellow)
  world/query/last_pose          last pose before the loss, T_last (orange arrow)
  world/query/track_before       new session before localization: T_last * O_first^-1 * O (orange)
  world/query/attempts/<k>/prior prior of attempt k (yellow arrow)
  world/query/log_position       position logged by the node ("Successfully localized at"):
                                 green = within --thr-t of the reference (localizer TP), red otherwise
  world/query/published          pose published by Isaac ROS after localization, map->odom * odometry:
                                 green = published TP at the settled frame, red = FP
  world/query/reference_now      reference pose of the current frame (purple point)
  world/query/localized          published localized pose of the query (arrow along cam2_link x, green/red, label with
                                 the errors), from the first settled frame to the end of the query
  world/overview/t_prev_<X>s/... static, all the queries of the capture with that t_prev: published (published
                                 localized poses, arrows green/red = published TP/FP), log_positions (logged positions,
                                 points green/red = localizer TP/FP), reference (reference poses at the same frames,
                                 light purple arrows), error_lines (published -> reference); labels "q<N>" = query
                                 number of queries.csv. Shown in the "overview" tab, hidden in the "map" tab.
  world/rig                      current pose (published after localization, prior before) with the IR image
Left: IR image (every --image-every frames), published error vs reference, event log (node log + query results).
Timelines: 'step' (queries one after the other), 'query', 'time' (capture time, s).

Usage (vrhealth-cuvslam image or venv with rerun-sdk 0.33):
  python3 isaac_localize_rerun.py --localize ~/dotlumen/localization/isaac_localize/loop_closure \
      --map-run ~/dotlumen/localization/isaac_maps/loop_closure \
      --bag ~/dotlumen/Captures/mcap/9_Loop_closure/loop_closure \
      --save ~/dotlumen/localization/rerun/loop_closure.rrd --image-every 5
"""
import argparse
import bisect
import csv
import math
import os
import re
import sys

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag2edex import Bag, decode_image, header_stamp, image_to_cv  # noqa: E402

NS = 1_000_000_000
IMAGE_TOPIC = "/cam2/infra1/image_rect_raw"
C_REF, C_ODOM, C_CUT, C_PRIOR, C_ATT = [190, 60, 210], [150, 150, 150], [240, 200, 40], [255, 140, 0], [240, 220, 60]
C_TP, C_FP = [40, 200, 80], [235, 50, 50]
C_REF_LIGHT = [210, 160, 225]


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat(t, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(*q)
    T[:3, 3] = t
    return T


class Trajectory:
    def __init__(self, path):
        rows = list(csv.DictReader(open(path)))
        self.ts = [int(round(float(r["timestamp"]) * NS)) for r in rows]
        self.T = [mat([float(r[k]) for k in "xyz"], [float(r[k]) for k in ("qx", "qy", "qz", "qw")]) for r in rows]

    def last_before(self, ts):
        i = bisect.bisect_right(self.ts, ts) - 1
        return self.T[i] if i >= 0 else None

    def at(self, ts, tol=2_000_000):
        i = bisect.bisect_left(self.ts, ts)
        c = [j for j in (i - 1, i) if 0 <= j < len(self.ts)]
        j = min(c, key=lambda j: abs(self.ts[j] - ts)) if c else None
        return self.T[j] if j is not None and abs(self.ts[j] - ts) <= tol else None


def read_trace(path):
    rows = []
    for r in csv.DictReader(open(path)):
        O = mat([float(r[k]) for k in ("odom_x", "odom_y", "odom_z")],
                [float(r[k]) for k in ("odom_qx", "odom_qy", "odom_qz", "odom_qw")])
        P = None
        if r["pub_x"] != "":
            P = mat([float(r[k]) for k in ("pub_x", "pub_y", "pub_z")],
                    [float(r[k]) for k in ("pub_qx", "pub_qy", "pub_qz", "pub_qw")])
        rows.append((int(r["timestamp_ns"]), int(r["recv_ns"]), O, P))
    return rows


def arrow(path, T, color, label=None, length=0.5):
    """Arrow along cam2_link x (forward)."""
    rr.log(path, rr.Arrows3D(origins=[T[:3, 3]], vectors=[T[:3, :3] @ np.array([length, 0.0, 0.0])], colors=[color],
                             radii=-3.0, labels=[label] if label else None))


def load_images(bag_path, stamps):
    """IR images (infra1) whose header stamp is in `stamps`, decoded to grayscale arrays."""
    need, out = set(stamps), {}
    for _, _, _, data in Bag(bag_path).iterate({IMAGE_TOPIC}):
        st = header_stamp(data[:12])
        if st in need:
            img = image_to_cv(decode_image(data))
            out[st] = np.ascontiguousarray(img if img.ndim == 2 else img[..., 0])
            if len(out) == len(need):
                break
    return out


def snap(rows, ts):
    """Timestamp of the trace frame closest to ts (the delays in queries.csv are rounded to 1 ms)."""
    return min((r[0] for r in rows), key=lambda t: abs(t - ts)) if rows else ts


def query_results(q, rows, ref, args):
    """Logged position, settled frame, published localized pose and references of one query."""
    out = {"P_log": None, "settled": None, "T_pub": None, "T_ref_pub": None, "T_ref_log": None}
    lp = os.path.join(args.localize, f"q{int(q['query']):03d}", "node_log.txt")
    if os.path.exists(lp):
        for line in open(lp):
            text = line.rstrip("\n").partition(" ")[2]
            m = re.search(r"\{([^}]*)\}", text)
            if text.startswith("Successfully localized") and m:
                out["P_log"] = np.array([float(v) for v in m.group(1).split(",")])
    if q.get("loc_ts_ns") and q.get("tf_update_delay_s"):
        loc_ts = int(q["loc_ts_ns"])
        settled = snap(rows, loc_ts + int(round(float(q["tf_update_delay_s"]) * NS)))
        out["settled"] = settled
        out["T_ref_log"] = ref.at(loc_ts)
        for ts, _, _, P in rows:
            if ts == settled and P is not None:
                out["T_pub"], out["T_ref_pub"] = P, ref.at(ts)
                break
    return out


def log_overview(queries, results):
    """Static overview of the localization results of all the queries, one entity group per t_prev:
    world/overview/t_prev_<X>s/{published, log_positions, reference, error_lines}."""
    groups = {}
    for q in queries:
        r = results.get(q["query"])
        if r:
            groups.setdefault(q["t_prev_s"], []).append((q, r))
    for tp, items in groups.items():
        base = f"world/overview/t_prev_{float(tp):g}s"
        pts, pcol, plab, ao, av, acol, alab, ro, rv, lines, lcol = [], [], [], [], [], [], [], [], [], [], []
        for q, r in items:
            if r["P_log"] is not None:
                pts.append(r["P_log"])
                pcol.append(C_TP if q.get("localizer_tp") == "True" else C_FP)
                plab.append(f"q{q['query']} log")
            if r["T_pub"] is not None:
                T = r["T_pub"]
                ao.append(T[:3, 3])
                av.append(T[:3, :3] @ np.array([0.5, 0.0, 0.0]))
                acol.append(C_TP if q.get("published_tp") == "True" else C_FP)
                alab.append(f"q{q['query']}")
                if r["T_ref_pub"] is not None:
                    R = r["T_ref_pub"]
                    ro.append(R[:3, 3])
                    rv.append(R[:3, :3] @ np.array([0.5, 0.0, 0.0]))
                    lines.append([T[:3, 3], R[:3, 3]])
                    lcol.append(acol[-1])
        if ao:
            rr.log(f"{base}/published", rr.Arrows3D(origins=ao, vectors=av, colors=acol, labels=alab, radii=-2.5),
                   static=True)
        if pts:
            rr.log(f"{base}/log_positions", rr.Points3D(pts, colors=pcol, labels=plab, radii=-6.0), static=True)
        if ro:
            rr.log(f"{base}/reference", rr.Arrows3D(origins=ro, vectors=rv, colors=[C_REF_LIGHT], radii=-1.5),
                   static=True)
            rr.log(f"{base}/error_lines", rr.LineStrips3D(lines, colors=lcol, radii=-1.0), static=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--localize", required=True, help="output folder of localize_query.py")
    ap.add_argument("--map-run", required=True, help="folder of map_cam2.sh (odometry.csv, slam_poses.csv)")
    ap.add_argument("--bag", required=True, help="capture bag (for the IR images)")
    ap.add_argument("--save", required=True, help="output .rrd")
    ap.add_argument("--image-every", type=int, default=5)
    ap.add_argument("--thr-t", type=float, default=0.3)
    ap.add_argument("--warning", default=None, help="warning shown in the events panel at the start (e.g. unreliable map)")
    args = ap.parse_args()

    ref = Trajectory(os.path.join(args.map_run, "slam_poses.csv"))
    odom_map = Trajectory(os.path.join(args.map_run, "odometry.csv"))
    queries = list(csv.DictReader(open(os.path.join(args.localize, "queries.csv"))))
    attempts = list(csv.DictReader(open(os.path.join(args.localize, "attempts.csv"))))
    traces = {}
    for q in queries:
        p = os.path.join(args.localize, f"q{int(q['query']):03d}", "trace.csv")
        if os.path.exists(p):
            traces[q["query"]] = read_trace(p)
    t0 = ref.ts[0]

    step_frames = {}
    step = 0
    want = []
    for q in queries:
        for k, row in enumerate(traces.get(q["query"], [])):
            step += 1
            if step % args.image_every == 0:
                want.append(row[0])
    images = load_images(args.bag, want)
    print(f"{len(queries)} queries, {len(want)} images", file=sys.stderr)

    os.makedirs(os.path.dirname(os.path.abspath(args.save)), exist_ok=True)
    rr.init("Isaac ROS relocalization")
    rr.save(args.save)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(rrb.Spatial2DView(name="cam2 infra1", origin="camera"),
                         rrb.Horizontal(rrb.TimeSeriesView(name="published error", origin="errors"),
                                        rrb.TextLogView(name="events", origin="events")), row_shares=[1, 1]),
            rrb.Tabs(rrb.Spatial3DView(name="map", origin="world", contents=["+ /world/**", "- /world/overview/**"]),
                     rrb.Spatial3DView(name="overview", origin="world",
                                       contents=["+ /world", "+ /world/map/**", "+ /world/overview/**"]),
                     active_tab=0),
            column_shares=[2, 3]),
        rrb.TimePanel(timeline="step", fps=14.0)))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/map/reference", rr.LineStrips3D([[T[:3, 3] for T in ref.T]], colors=[C_REF], radii=-1.5), static=True)
    rr.log("world/map/odometry", rr.LineStrips3D([[T[:3, 3] for T in odom_map.T]], colors=[C_ODOM], radii=-1.0),
           static=True)
    rr.log("errors/err_t_m", rr.SeriesLines(names=["published translation error (m)"], colors=[[255, 120, 60]]),
           static=True)
    rr.log("errors/err_r_deg", rr.SeriesLines(names=["published rotation error (deg)"], colors=[[90, 160, 255]]),
           static=True)

    results = {q["query"]: query_results(q, traces[q["query"]], ref, args) for q in queries if q["query"] in traces}
    log_overview(queries, results)

    step = 0
    if args.warning:
        rr.set_time("step", sequence=0)
        rr.log("events", rr.TextLog(f"WARNING: {args.warning}", level="WARN"))
    for q in queries:
        qid, rows = q["query"], traces.get(q["query"], [])
        if not rows:
            continue
        loc_ts = int(q["loc_ts_ns"]) if q.get("loc_ts_ns") else None
        settled = None
        if q.get("loc_ts_ns") and q.get("tf_update_delay_s"):
            settled = snap(rows, loc_ts + int(round(float(q["tf_update_delay_s"]) * NS)))
        pub_color = C_TP if q.get("published_tp") == "True" else C_FP
        log_color = C_TP if q.get("localizer_tp") == "True" else C_FP
        T_last = odom_map.last_before(int(q["prior_ts_ns"])) if q.get("prior_ts_ns") else None
        O_first = rows[0][2]
        q_att = {int(a["call_ts_ns"]): a for a in attempts if a["query"] == qid}
        logs = []
        lp = os.path.join(args.localize, f"q{int(qid):03d}", "node_log.txt")
        if os.path.exists(lp):
            for line in open(lp):
                recv, _, text = line.rstrip("\n").partition(" ")
                if "locali" in text.lower():
                    logs.append((int(recv), text))
        P_log = None
        for _, text in logs:
            m = re.search(r"\{([^}]*)\}", text)
            if text.startswith("Successfully localized") and m:
                P_log = np.array([float(v) for v in m.group(1).split(",")])

        step += 1
        rr.set_time("step", sequence=step)
        rr.set_time("query", sequence=int(qid))
        rr.set_time("time", duration=(rows[0][0] - t0) / NS)
        rr.log("world/query", rr.Clear(recursive=True))
        rr.log("errors", rr.Clear(recursive=True))
        cut = [T[:3, 3] for t, T in zip(ref.ts, ref.T) if rows[0][0] <= t <= rows[-1][0]]
        if cut:
            rr.log("world/query/reference_cut", rr.LineStrips3D([cut], colors=[C_CUT], radii=-3.0))
        if T_last is not None:
            arrow("world/query/last_pose", T_last, C_PRIOR, f"last pose (t1-{q['t_prev_s']}s)", 0.6)
        rr.log("events", rr.TextLog(
            f"query {qid}: t1={q['t1_s']} s t_prev={q['t_prev_s']} s, prior error {q.get('prior_err_t_m')} m / "
            f"{q.get('prior_err_r_deg')} deg", level="INFO"))

        before, after, li = [], [], 0
        for ts, recv, O, P in rows:
            step += 1
            rr.set_time("step", sequence=step)
            rr.set_time("time", duration=(ts - t0) / NS)
            while li < len(logs) and logs[li][0] <= recv:
                text = logs[li][1]
                level = "INFO" if text.startswith("Successfully") else ("WARN" if "Fail" in text else "DEBUG")
                rr.log("events", rr.TextLog(f"  node: {text}", level=level))
                if text.startswith("Successfully") and P_log is not None:
                    rr.log("world/query/log_position", rr.Points3D([P_log], colors=[log_color], radii=-8.0,
                                                                  labels=[f"log ({q.get('loc_log_err_t_m')} m)"]))
                li += 1
            T_prior = T_last @ np.linalg.inv(O_first) @ O if T_last is not None else None
            if ts in q_att and T_prior is not None:
                a = q_att[ts]
                arrow(f"world/query/attempts/{a['attempt']}/prior", T_prior, C_ATT, f"prior {a['attempt']}", 0.4)
            use_pub = settled is not None and ts >= settled and P is not None
            T_show = P if use_pub else T_prior
            if use_pub and ts == settled:   # published localized pose of the query, kept until the next query
                arrow("world/query/localized", P, pub_color,
                      f"q{qid} pub ({q.get('loc_err_t_m')} m, {q.get('loc_err_r_deg')} deg)", 0.7)
            if use_pub:
                after.append(P[:3, 3].tolist())
                rr.log("world/query/published", rr.LineStrips3D([after], colors=[pub_color], radii=-3.5))
                T_ref = ref.at(ts)
                if T_ref is not None:
                    d = P[:3, 3] - T_ref[:3, 3]
                    c = (np.trace(T_ref[:3, :3].T @ P[:3, :3]) - 1) / 2
                    rr.log("errors/err_t_m", rr.Scalars(float(np.linalg.norm(d))))
                    rr.log("errors/err_r_deg", rr.Scalars(math.degrees(math.acos(max(-1.0, min(1.0, c))))))
            elif T_prior is not None and (loc_ts is None or ts <= loc_ts):
                before.append(T_prior[:3, 3].tolist())
                rr.log("world/query/track_before", rr.LineStrips3D([before], colors=[C_PRIOR], radii=-3.0))
            if T_show is not None:
                rr.log("world/rig", rr.Transform3D(translation=T_show[:3, 3], mat3x3=T_show[:3, :3]))
                arrow("world/rig/heading", np.eye(4), [255, 255, 255], None, 0.3)
            T_ref = ref.at(ts)
            if T_ref is not None:
                rr.log("world/query/reference_now", rr.Points3D([T_ref[:3, 3]], colors=[C_REF], radii=-7.0))
            if ts in images:
                rr.log("camera", rr.Image(images[ts]).compress(jpeg_quality=75))

        if q["localized"] == "True":
            rr.log("events", rr.TextLog(
                f"  result: localized at attempt {q['attempt_ok']}; log position error {q.get('loc_log_err_t_m')} m "
                f"(localizer {'TP' if q.get('localizer_tp') == 'True' else 'FP'}); published {q.get('loc_err_t_m')} m"
                f" / {q.get('loc_err_r_deg')} deg ({'TP' if q.get('published_tp') == 'True' else 'FP'}); "
                f"+1 s {q.get('pub_err_t_m_1s')} m, +2 s {q.get('pub_err_t_m_2s')} m, end {q.get('post_final_err_t_m')} m",
                level="INFO" if q.get("published_tp") == "True" else "ERROR"))
        else:
            rr.log("events", rr.TextLog(f"  result: NOT localized ({q['attempts']} attempts)", level="WARN"))
    print(f"saved {args.save}", file=sys.stderr)


if __name__ == "__main__":
    main()
