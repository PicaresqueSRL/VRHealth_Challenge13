#!/usr/bin/env python3
"""
map_bag.py - cuVSLAM map building and relocalization tests on a ROS 2 bag (.mcap or .db3).

Two subcommands:

  build     runs cuVSLAM with SLAM on a capture (or on a time range of it), saves the SLAM map
            (LMDB database) and the reference trajectory in the native frame of the map.

  localize  simulates a tracking loss: a NEW cuVSLAM session is started on the cut [t1, t2] of a
            capture and tries to localize in a saved map. The prior is the reference pose at
            t1 - t_prev (last pose tracked before a loss of t_prev seconds). Up to N single-frame
            attempts are made on later frames of the new session; after a successful localization
            the session keeps tracking until t2 and its SLAM poses (in the map frame) are compared
            with the reference.

The bag is read with the same functions as track_bag_rerun.py (same streams, rig, synchronization
and IMU handling), so the frames are the same as in the step 3 runs. Existing scripts are not
modified.

Examples:
    B=~/dotlumen/Captures/mcap/9_Loop_closure/loop_closure
    M=~/dotlumen/VRHealth_Challenge13/results/maps/cam2_stereo_imu/loop_closure

    # build the map on the whole capture (cam2 IR stereo + IMU)
    python3 map_bag.py build $B --mode inertial --streams cam2/ir1 cam2/ir2 --rectified --out-dir $M

    # relocalization: t1 every 5 s, t_prev 0/1/3/5 s, 10 s segments, up to 16 attempts every 0.25 s
    python3 map_bag.py localize $B --map $M --t1-step 5 --t-prev 0 1 3 5 --duration 10 \\
        --attempts 16 --attempt-every 0.25 --out-dir $M/localize

Frames and conventions:
  - Map frame = native SLAM world frame of the build run (cuVSLAM rig = first stream camera,
    OpenCV axes x right, y down, z forward). All poses saved by this script are rig poses in this
    frame (world_from_rig), so no alignment is needed to compare them.
  - Timestamps: header stamps in nanoseconds (as cuVSLAM); t1/t2/--start/--end are seconds from
    the beginning of the bag (receive time), as --start/--end of track_bag_rerun.py.
  - Localization search (cuVSLAM Slam.LocalizationSettings): grid around the prior, horizontal
    radius along rig x and z, vertical radius along rig y, full rotation around rig y.

Requirements: as track_bag_rerun.py (cuvslam bindings built with LMDB, numpy, opencv-python,
rerun-sdk is imported but not used).
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag2edex import Bag, greedy_match  # noqa: E402
from merge_rig_edex import CAD_EXTRINSIC  # noqa: E402
from run_output import RunWriter, cuvslam_version, pose_to_matrix, rot_to_quat  # noqa: E402
from track_bag_rerun import (MULTICAM, TOPICS, build_rig, frame_images, imu_calibration,  # noqa: E402
                             read_metadata)
from track_edex_rerun import MODES, color_from_id, cuvslam, cv2, rr  # noqa: E402

NS = 1_000_000_000
POSE_COLS = ["tx", "ty", "tz", "qx", "qy", "qz", "qw"]
REF_TOL_NS = 2_000_000  # max offset between a query frame and its reference pose


def log(msg):
    print(msg, file=sys.stderr, flush=True)


# -----------------------------------------------------------------------------
# Small geometry helpers (4x4 matrices, cuvslam.Pose)
# -----------------------------------------------------------------------------
def to_mat(pose):
    return pose_to_matrix(pose)


def to_cupose(T):
    return cuvslam.Pose(rotation=rot_to_quat(T[:3, :3]), translation=T[:3, 3].tolist())


def mat_from_row(r):
    """[tx, ty, tz, qx, qy, qz, qw] -> 4x4."""
    return to_mat(cuvslam.Pose(rotation=list(r[3:7]), translation=list(r[0:3])))


def row_from_mat(T):
    return [*T[:3, 3].tolist(), *rot_to_quat(T[:3, :3])]


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def pose_error(T_est, T_ref, g_dir=None):
    """Translation error (m), rotation error (deg) and, if g_dir is given, vertical error (m, along gravity)."""
    d = T_est[:3, 3] - T_ref[:3, 3]
    e = {"err_t_m": float(np.linalg.norm(d)), "err_r_deg": rot_angle_deg(T_ref[:3, :3].T @ T_est[:3, :3])}
    if g_dir is not None:
        e["err_vert_m"] = float(abs(d @ g_dir))
    return e


def align_vectors(a, b):
    """Minimal rotation R with R @ a = b (unit vectors)."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(a @ b)
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        # opposite vectors: rotate by pi around any axis orthogonal to a
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


def gravity_vec(g):
    if g is None:
        return None
    try:
        v = np.asarray(list(g), dtype=float)
    except TypeError:
        v = np.asarray([g.x, g.y, g.z], dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-6 else None


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


# -----------------------------------------------------------------------------
# Bag, rig and tracker (same logic as track_bag_rerun.py)
# -----------------------------------------------------------------------------
def make_streams(names, types, mode, imu_camera):
    if names is None:
        if mode == "mono":
            names = [f"{imu_camera}/rgb"]
        elif mode == "inertial":
            names = [f"{imu_camera}/ir1", f"{imu_camera}/ir2"]
        else:
            cams = sorted({t.split("/")[1] for t in types if t.endswith("/color/image_raw")})
            names = [f"{c}/{r}" for c in cams for r in ("ir1", "ir2") if TOPICS[r][0].format(cam=c) in types]
    streams = []
    for n in names:
        cam, _, role = n.partition("/")
        if role not in TOPICS:
            sys.exit(f"Invalid stream: {n} (format <cam>/<rgb|ir1|ir2>)")
        streams.append(dict(name=n, cam=cam, role=role, topic=TOPICS[role][0].format(cam=cam),
                            info_topic=TOPICS[role][1].format(cam=cam)))
    return names, streams


class Session:
    """Bag metadata, rig and frame list, shared by build and localize."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.bag = Bag(cfg["bag"])
        types = self.bag.topic_types()
        self.names, self.streams = make_streams(cfg["streams"], types, cfg["mode"], cfg["imu_camera"])
        if cfg["mode"] == "mono" and len(self.streams) != 1:
            sys.exit("Mono uses a single stream (e.g. --streams cam2/rgb)")
        self.use_imu = cfg["mode"] == "inertial"
        imu_topic = f"/{cfg['imu_camera']}/imu" if self.use_imu else None
        log(f"Bag: {', '.join(os.path.basename(f.path) for f in self.bag.files)}")
        log("Reading timestamps, camera_info, /tf_static" + (", IMU" if self.use_imu else "") + " ...")
        (self.t_bag0, stamps, self.tf, infos, self.imu,
         imu_frame) = read_metadata(self.bag, self.streams, imu_topic, cfg["stale_threshold"])
        rig_args = argparse.Namespace(link_frame=cfg["link_frame"], extrinsic=cfg["extrinsic"])
        self.ref_frame = build_rig(self.streams, infos, self.tf, rig_args)

        # frames: one per message of the reference stream, other streams matched within the tolerance
        ref_list = stamps[self.streams[0]["topic"]]
        ref_ts = [x[0] for x in ref_list]
        frames = [{"ts": x[0], "recv": x[1], "keys": {0: x[2]}, "stamps": {0: x[0]}} for x in ref_list]
        tol = int(cfg["tolerance_ms"] * 1e6)
        for si, s in enumerate(self.streams[1:], 1):
            lst = stamps[s["topic"]]
            for i, j in greedy_match(ref_ts, [x[0] for x in lst], tol).items():
                frames[i]["keys"][si] = lst[j][2]
                frames[i]["stamps"][si] = lst[j][0]
        require_all = cfg["require_all_streams"]
        if require_all is None:
            require_all = cfg["mode"] == "inertial"
        cfg["require_all_streams"] = bool(require_all)
        n0 = len(frames)
        if require_all and len(self.streams) > 1:
            frames = [f for f in frames if len(f["keys"]) == len(self.streams)]
        self.frames_dropped = n0 - len(frames)
        self.frames = frames
        self.topics = {s["topic"] for s in self.streams}

        self.imu_cfg = None
        if self.use_imu:
            imu = self.imu
            rate = round(NS / statistics.median(b[0] - a[0] for a, b in zip(imu, imu[1:]))) if len(imu) > 1 else 200
            self.imu_cfg = imu_calibration(self.tf, self.ref_frame, imu_frame, cfg["imu_camera"], rate)
        self.rig = cuvslam.Rig(cameras=[s["camera"] for s in self.streams],
                               imus=[self.imu_cfg] if self.use_imu else [])
        log(f"Rig ({self.ref_frame} = origin), streams {self.names}, {len(frames)} frames "
            f"({self.frames_dropped} dropped without all the streams)")

    def frames_between(self, start_s=None, end_s=None):
        """Frames whose receive time is in [start_s, end_s] seconds from the beginning of the bag."""
        lo = self.t_bag0 + start_s * NS if start_s is not None else None
        hi = self.t_bag0 + end_s * NS if end_s is not None else None
        return [f for f in self.frames if (lo is None or f["recv"] >= lo) and (hi is None or f["recv"] <= hi)]

    def load_images(self, frames):
        """Decoded grayscale images of these frames, in order: list of (frame, [image per stream])."""
        out, last = [], frames[-1]["ts"] if frames else None
        for fr, imgs in frame_images(self.bag, frames, self.topics):
            images = []
            for si in range(len(self.streams)):
                img = imgs.get(si)
                if img is None:
                    images.append(np.empty(0, dtype=np.uint8))
                    continue
                if img.ndim == 3:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                images.append(np.ascontiguousarray(img))
            out.append((fr, images))
            if fr["ts"] == last:
                break  # stop reading the bag after the last needed frame
        return out

    def new_tracker(self, slam=True):
        cfg = self.cfg
        odom_cfg = cuvslam.Odometry.Config(
            odometry_mode=MODES[cfg["mode"]],
            multicam_mode=getattr(cuvslam.Odometry.MulticameraMode, MULTICAM[cfg["multicam_mode"]]),
            async_sba=False,
            rectified_stereo_camera=cfg["rectified"],
            enable_observations_export=True,
            enable_landmarks_export=True,
            enable_final_landmarks_export=True,
        )
        if not slam:
            return cuvslam.Tracker(self.rig, cuvslam.Tracker.Mode.OdometryOnlyOffline, odom_cfg)
        # offline: SLAM in the calling thread; max_map_size=0 keeps the whole pose graph
        slam_cfg = cuvslam.Slam.Config(sync_mode=True, enable_reading_internals=True,
                                       max_map_size=cfg["max_map_size"])
        return cuvslam.Tracker(self.rig, cuvslam.Tracker.Mode.OdometryWithSlamOffline, odom_cfg, slam_cfg)


class ImuFeeder:
    """Registers the IMU samples up to each frame timestamp (from the start of the session)."""

    def __init__(self, imu, t_start_ns):
        self.imu = [x for x in imu if x[0] >= t_start_ns]
        self.i = 0

    def feed(self, tracker, ts):
        while self.i < len(self.imu) and self.imu[self.i][0] <= ts:
            its, acc, gyr = self.imu[self.i]
            m = cuvslam.ImuMeasurement()
            m.timestamp_ns = its
            m.linear_accelerations = np.asarray(acc, dtype=np.float32)
            m.angular_velocities = np.asarray(gyr, dtype=np.float32)
            tracker.register_imu_measurement(0, m)
            self.i += 1


def session_config(args):
    return {"bag": os.path.abspath(os.path.expanduser(args.bag)), "mode": args.mode, "streams": args.streams,
            "imu_camera": args.imu_camera, "extrinsic": list(args.extrinsic), "link_frame": args.link_frame,
            "require_all_streams": args.require_all_streams, "tolerance_ms": args.tolerance_ms,
            "stale_threshold": args.stale_threshold, "multicam_mode": args.multicam_mode,
            "rectified": args.rectified, "max_map_size": args.max_map_size}


# -----------------------------------------------------------------------------
# build
# -----------------------------------------------------------------------------
def cmd_build(args):
    out = os.path.abspath(os.path.expanduser(args.out_dir))
    os.makedirs(out, exist_ok=True)
    cuvslam.set_verbosity(args.verbosity)
    cfg = session_config(args)
    ses = Session(cfg)
    frames = ses.frames_between(args.start, args.end)
    if args.max_frames:
        frames = frames[:args.max_frames]
    if not frames:
        sys.exit("No frames to process")

    # base frame for the standard run output (same CSVs as track_bag_rerun.py --out-dir)
    base = args.link_frame.format(cam=ses.streams[0]["cam"])
    T_rb = ses.tf.lookup(ses.ref_frame, base)
    if T_rb is None:
        T_rb, base = np.eye(4), "rig"
    run_dir = os.path.join(out, "run")
    writer = RunWriter(run_dir, T_rb, base, len(ses.streams), {
        "script": "map_bag.py build", "input": cfg["bag"], "mode": args.mode, "streams": ses.names,
        "slam": True, "imu": ses.use_imu, "rectified": args.rectified, "async_sba": False,
        "tolerance_ms": args.tolerance_ms, "start": args.start, "end": args.end, "max_frames": args.max_frames,
        "base_frame_source": "/tf_static of the bag", "cuvslam_version": cuvslam_version(cuvslam),
        "require_all_streams": cfg["require_all_streams"], "frames_in_input": len(frames),
        "max_map_size": args.max_map_size})

    tracker = ses.new_tracker(slam=True)
    feeder = ImuFeeder(ses.imu, frames[0]["ts"]) if ses.use_imu else None
    odom_rows, grav = [], {}
    lost, last_ts, t_wall = 0, None, time.time()
    log(f"Tracking {len(frames)} frames with SLAM (max_map_size={args.max_map_size}) ...")
    for n, (fr, images) in enumerate(ses.load_images(frames)):
        ts = fr["ts"]
        if last_ts is not None and ts <= last_ts:
            continue
        if feeder:
            feeder.feed(tracker, ts)
        t0 = time.perf_counter()
        est, slam_pose = tracker.track(ts, images)
        track_ms = (time.perf_counter() - t0) * 1000
        last_ts = ts
        obs_n = {si: len(tracker.odometry.get_last_observations(si) or [])
                 for si in range(len(ses.streams)) if images[si].size}
        writer.add_frame(ts, est, slam_pose, obs_n, len(tracker.odometry.get_last_landmarks() or []),
                         track_ms, tracker)
        if est.world_from_rig is None:
            lost += 1
        else:
            odom_rows.append([ts, *row_from_mat(to_mat(est.world_from_rig.pose))])
        if ses.use_imu:
            g = gravity_vec(tracker.odometry.get_last_gravity())
            if g is not None:
                grav[ts] = g
        # the PoseGraph data layer is filled only while it is read (first read enables it, it expires after
        # Slam.Config.retention_time_ms without reads): read it every frame so the final read is complete
        tracker.slam.get_pose_graph()
        if n % 10 == 0:
            tracker.slam.get_landmarks(cuvslam.Slam.DataLayer.Map)  # same for the Map layer (map landmarks)
        if (n + 1) % 100 == 0:
            log(f"  {n + 1}/{len(frames)} frames ({(n + 1) / (time.time() - t_wall):.1f} fps), lost {lost}")

    # reference trajectory: final SLAM poses (after loop closures), native map frame
    poses = sorted(tracker.slam.get_all_slam_poses(), key=lambda p: p.timestamp_ns)
    ref_rows = [[int(p.timestamp_ns), *row_from_mat(to_mat(p.pose))] for p in poses]
    write_csv(os.path.join(out, "reference_slam.csv"), ["timestamp_ns", *POSE_COLS], ref_rows)
    write_csv(os.path.join(out, "odometry_native.csv"), ["timestamp_ns", *POSE_COLS], odom_rows)

    # gravity direction in the map frame: gravity of each frame (rig frame) rotated by its final pose
    g_map = None
    if grav:
        ref_R = {r[0]: mat_from_row(r[1:])[:3, :3] for r in ref_rows}
        gs = [ref_R[ts] @ g for ts, g in grav.items() if ts in ref_R]
        if gs:
            m = np.mean(gs, axis=0)
            g_map = (m / np.linalg.norm(m)).tolist()
            spread = float(np.median([rot_angle_deg(align_vectors(np.asarray(g), np.asarray(g_map))) for g in gs]))
            log(f"Gravity in the map frame: {np.round(g_map, 4).tolist()} (median deviation {spread:.2f} deg)")

    # keyframes of the pose graph
    kf_rows = []
    try:
        pg = tracker.slam.get_pose_graph()
        kf_rows = [[int(nd.id), *row_from_mat(to_mat(nd.node_pose))] for nd in pg.nodes]
        write_csv(os.path.join(out, "keyframes.csv"), ["id", *POSE_COLS], kf_rows)
    except Exception as e:  # noqa: BLE001  (diagnostic only)
        log(f"WARNING: pose graph not available: {e}")

    # landmarks of the SLAM map (map frame), for the viewer of 'localize'
    lm_rows = []
    try:
        lms = tracker.slam.get_landmarks(cuvslam.Slam.DataLayer.Map).landmarks
        lm_rows = [[int(lm.id), *[float(c) for c in lm.coords], float(lm.weight)] for lm in lms]
        write_csv(os.path.join(out, "map_landmarks.csv"), ["id", "x", "y", "z", "weight"], lm_rows)
    except Exception as e:  # noqa: BLE001  (visualization only)
        log(f"WARNING: map landmarks not available: {e}")

    summary = writer.finish(tracker, True, len(tracker.odometry.get_final_landmarks() or {}))

    # save the map (sync mode: the callback runs before save_map returns)
    map_dir = os.path.join(out, "map")
    saved = {"ok": None}
    t0 = time.perf_counter()
    tracker.slam.save_map(map_dir, lambda ok: saved.update(ok=bool(ok)))
    save_ms = (time.perf_counter() - t0) * 1000
    if not saved["ok"]:
        sys.exit(f"Map NOT saved to {map_dir}")
    size = sum(os.path.getsize(os.path.join(map_dir, f)) for f in os.listdir(map_dir))
    log(f"Map saved to {map_dir} ({size / 1e6:.1f} MB, {save_ms:.0f} ms)")

    info = {"script": "map_bag.py build", "config": cfg, "streams": ses.names, "ref_frame": ses.ref_frame,
            "start": args.start, "end": args.end, "frames": len(frames), "frames_lost": lost,
            "first_ts_ns": frames[0]["ts"], "last_ts_ns": frames[-1]["ts"], "bag_t0_recv_ns": ses.t_bag0,
            "reference_poses": len(ref_rows), "keyframes": len(kf_rows), "map_landmarks": len(lm_rows),
            "gravity_map": g_map,
            "map_dir": "map", "map_size_mb": round(size / 1e6, 2), "save_map_ms": round(save_ms, 1),
            "cuvslam_version": cuvslam_version(cuvslam), "wall_time_s": round(time.time() - t_wall, 1),
            "run_summary": summary}
    with open(os.path.join(out, "map_info.json"), "w") as f:
        json.dump(info, f, indent=2, default=str)
    log(f"Done: {len(ref_rows)} reference poses, {len(kf_rows)} keyframes, {len(lm_rows)} map landmarks, "
        f"{lost} frames lost -> {out}")


# -----------------------------------------------------------------------------
# localize
# -----------------------------------------------------------------------------
class Reference:
    """Reference trajectory of a map (timestamp_ns -> 4x4 rig pose in the map frame)."""

    def __init__(self, path):
        rows = read_csv(path)
        self.ts = np.array([int(r["timestamp_ns"]) for r in rows], dtype=np.int64)
        self.T = [mat_from_row([float(r[c]) for c in POSE_COLS]) for r in rows]

    def at(self, ts, tol=REF_TOL_NS):
        i = int(np.searchsorted(self.ts, ts))
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.ts) and (best is None or abs(self.ts[j] - ts) < abs(self.ts[best] - ts)):
                best = j
        if best is None or abs(int(self.ts[best]) - ts) > tol:
            return None
        return self.T[best]

    def last_before(self, ts):
        """Last pose with timestamp <= ts (the last pose tracked before the loss)."""
        i = int(np.searchsorted(self.ts, ts, side="right")) - 1
        return (int(self.ts[i]), self.T[i]) if i >= 0 else (None, None)


def gravity_corrected(T_prior, g_rig, g_map):
    """Prior with roll/pitch from the current gravity estimate, heading and position unchanged."""
    if g_rig is None or g_map is None:
        return T_prior, False
    C = align_vectors(T_prior[:3, :3] @ g_rig, np.asarray(g_map))
    T = T_prior.copy()
    T[:3, :3] = C @ T_prior[:3, :3]
    return T, True


def probes_summary(tracker):
    try:
        lp = tracker.slam.get_localizer_probes()
        probes = list(lp.probes)
        solved = [p for p in probes if p.solved]
        return {"probes": len(probes), "probes_solved": len(solved),
                "best_weight": max((float(p.exact_result_weight) for p in solved), default=0.0)}
    except Exception:  # noqa: BLE001  (diagnostic only)
        return {"probes": None, "probes_solved": None, "best_weight": None}


class Viz:
    """Rerun view of the localization queries, in the native map frame (--viewer and/or --save).

    3D view: map landmarks (grey), reference trajectory of the map (purple) and keyframes (blue), static; for the
    current query the reference of the cut (yellow), the last pose before the loss (orange arrow), the pose of the new
    session before localization = last pose + new-session odometry (orange line), the search box and prior of the
    last attempt (yellow), the localization results (green = true positive, red = false positive), the SLAM pose
    after localization (green/red line), the reference pose of the current frame (purple point), the camera with its
    image and the landmarks tracked by the new session. Left: images, errors after localization, event log.
    Timelines: 'step' (all the queries one after the other), 'query', 'time' (capture time, s)."""

    C_REF, C_CUT, C_KF = [190, 60, 210], [240, 200, 40], [80, 140, 255]
    C_PRIOR, C_SEARCH, C_TP, C_FP, C_NOREF = [255, 140, 0], [240, 220, 60], [40, 210, 80], [235, 50, 50], [0, 200, 220]

    def __init__(self, args, ses, ref, map_root, g_map=None):
        self.args, self.ses, self.ref, self.g_map = args, ses, ref, g_map
        rr.init("cuVSLAM relocalization", spawn=args.viewer)
        if args.save:
            os.makedirs(os.path.dirname(os.path.abspath(args.save)), exist_ok=True)
            if args.viewer:  # both the live viewer and the file
                rr.set_sinks(rr.GrpcSink(), rr.FileSink(args.save))
            else:
                rr.save(args.save)
        self.ent = {si: f"world/rig/{s['cam']}_{s['role']}" for si, s in enumerate(ses.streams)}
        self._blueprint()
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)
        lm_path = os.path.join(map_root, "map_landmarks.csv")
        if os.path.exists(lm_path):
            pts = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in read_csv(lm_path)], dtype=np.float32)
            rr.log("world/map/landmarks", rr.Points3D(pts, colors=[[110, 110, 110, 80]], radii=-1.2), static=True)
        else:
            log(f"WARNING: {lm_path} not found (map built by an older map_bag.py): map landmarks not shown")
        rr.log("world/map/reference", rr.LineStrips3D([[T[:3, 3] for T in ref.T]], colors=[self.C_REF], radii=-1.5),
               static=True)
        kf_path = os.path.join(map_root, "keyframes.csv")
        if os.path.exists(kf_path):
            kf = [[float(r[c]) for c in ("tx", "ty", "tz")] for r in read_csv(kf_path)]
            if kf:
                rr.log("world/map/keyframes", rr.Points3D(kf, colors=[self.C_KF], radii=-3.0), static=True)
        for si, s in enumerate(ses.streams):
            T = s["T"]
            rr.log(self.ent[si], rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]), static=True)
            rr.log(self.ent[si], rr.Pinhole(focal_length=[float(x) for x in s["focal"]],
                                            principal_point=[float(x) for x in s["principal"]],
                                            width=int(s["size"][0]), height=int(s["size"][1]),
                                            image_plane_distance=0.3), static=True)
        rr.log("errors/err_t_m", rr.SeriesLines(names=["translation error (m)"], colors=[[255, 120, 60]]), static=True)
        rr.log("errors/err_r_deg", rr.SeriesLines(names=["rotation error (deg)"], colors=[[90, 160, 255]]), static=True)
        self.step = 0
        self.wall0 = None

    def _blueprint(self):
        import rerun.blueprint as rrb
        cams = [rrb.Spatial2DView(name=s["name"], origin=self.ent[si]) for si, s in enumerate(self.ses.streams)]
        left = rrb.Vertical(
            rrb.Horizontal(*cams),
            rrb.Horizontal(rrb.TimeSeriesView(name="error after localization", origin="errors"),
                           rrb.TextLogView(name="events", origin="events")),
            row_shares=[1, 1])
        # 'step' timeline by default: the cuts of different queries overlap in capture time ('time');
        # 14 steps/s is about the frame rate of the captures (1x = real time while a cut is played)
        # default 3D camera: top-down view of the map along gravity (inertial maps), else along -y (OpenCV up)
        P = np.array([T[:3, 3] for T in self.ref.T])
        center = (P.min(0) + P.max(0)) / 2
        up = -np.asarray(self.g_map, dtype=float) if self.g_map else np.array([0.0, -1.0, 0.0])
        fwd = np.array([0.0, 0.0, 1.0]) - up * up[2]
        eye = rrb.EyeControls3D(position=(center + up * max(8.0, 1.4 * float(np.ptp(P, axis=0).max()))).tolist(),
                                look_target=center.tolist(), eye_up=(fwd / np.linalg.norm(fwd)).tolist())
        world = rrb.Spatial3DView(name="map", origin="world", eye_controls=eye)
        rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(left, world, column_shares=[2, 3]),
                                        rrb.TimePanel(timeline="step", fps=14.0)))

    @staticmethod
    def _arrow(path, T, color, label=None, length=0.5):
        """Arrow along the rig z axis (camera forward) at pose T."""
        rr.log(path, rr.Arrows3D(origins=[T[:3, 3]], vectors=[T[:3, :3] @ np.array([0.0, 0.0, length])],
                                 colors=[color], radii=-3.0, labels=[label] if label else None))

    def _time(self, qid, ts):
        rr.set_time("step", sequence=self.step)
        rr.set_time("query", sequence=qid)
        rr.set_time("time", duration=(ts - self.ses.t_bag0) / NS)

    def event(self, text, level="INFO"):
        rr.log("events", rr.TextLog(text, level=level))

    def begin_query(self, qid, t1, t_prev, ts1, ts_end, T_last, T_ref1):
        self.step += 1
        self._time(qid, ts1)
        rr.log("world/query", rr.Clear(recursive=True))
        rr.log("errors", rr.Clear(recursive=True))
        cut = [T[:3, 3] for t, T in zip(self.ref.ts, self.ref.T) if ts1 <= t <= ts_end]
        if cut:
            rr.log("world/query/reference_cut", rr.LineStrips3D([cut], colors=[self.C_CUT], radii=-3.0))
        self._arrow("world/query/last_pose", T_last, self.C_PRIOR, f"last pose (t1-{t_prev:g}s)", 0.6)
        err = ""
        if T_ref1 is not None:
            e = pose_error(T_last, T_ref1)
            err = f", prior error {e['err_t_m']:.2f} m / {e['err_r_deg']:.1f} deg"
        self.event(f"query {qid}: t1={t1:.2f} s, t_prev={t_prev:g} s{err}")
        self.before, self.after, self.wall0, self.ts0 = [], [], None, ts1

    def frame(self, qid, ts, images, tracker, T_show, localized, color):
        self.step += 1
        self._time(qid, ts)
        if self.args.pace > 0:  # real-time pacing of the live viewer
            if self.wall0 is None:
                self.wall0 = time.perf_counter()
            wait = (ts - self.ts0) / NS / self.args.pace - (time.perf_counter() - self.wall0)
            if wait > 0:
                time.sleep(wait)
        if T_show is not None:
            rr.log("world/rig", rr.Transform3D(translation=T_show[:3, 3], mat3x3=T_show[:3, :3]))
            (self.after if localized else self.before).append(T_show[:3, 3].tolist())
            if self.before:
                rr.log("world/query/track_before", rr.LineStrips3D([self.before], colors=[self.C_PRIOR], radii=-3.5))
            if self.after:
                rr.log("world/query/track_after", rr.LineStrips3D([self.after], colors=[color], radii=-3.5))
            lms = tracker.odometry.get_last_landmarks()
            if lms:
                rr.log("world/rig/landmarks", rr.Points3D([list(lm.coords) for lm in lms],
                                                          colors=[color_from_id(lm.id) for lm in lms], radii=-2.0))
        T_ref = self.ref.at(ts)
        if T_ref is not None:
            rr.log("world/query/reference_now", rr.Points3D([T_ref[:3, 3]], colors=[self.C_REF], radii=-7.0))
            if localized and T_show is not None:
                e = pose_error(T_show, T_ref)
                rr.log("errors/err_t_m", rr.Scalars(e["err_t_m"]))
                rr.log("errors/err_r_deg", rr.Scalars(e["err_r_deg"]))
        if (self.step % self.args.image_every) == 0:
            for si, img in enumerate(images):
                if img.size == 0:
                    rr.log(f"{self.ent[si]}/image", rr.Clear(recursive=False))
                    continue
                rr.log(f"{self.ent[si]}/image", rr.Image(img).compress(jpeg_quality=75))
                obs = tracker.odometry.get_last_observations(si)
                if obs:
                    rr.log(f"{self.ent[si]}/image/observations",
                           rr.Points2D([[o.u, o.v] for o in obs], colors=[color_from_id(o.id) for o in obs], radii=3.0))

    def attempt(self, k, T_prior, T_loc, arow):
        a = self.args
        q = rot_to_quat(T_prior[:3, :3])
        rr.log("world/query/search", rr.Boxes3D(centers=[T_prior[:3, 3]], half_sizes=[[a.h_radius, a.v_radius, a.h_radius]],
                                                quaternions=[rr.Quaternion(xyzw=q)], colors=[self.C_SEARCH], radii=-1.5,
                                                fill_mode=rr.components.FillMode.MajorWireframe))
        self._arrow(f"world/query/attempts/{k}/prior", T_prior, self.C_SEARCH, f"prior {k}", 0.4)
        if T_loc is None:
            self.event(f"  attempt {k}: NOT localized ({arow['time_ms']:.0f} ms) {arow.get('message', '')}", "WARN")
            return None
        if "err_t_m" in arow:
            tp = arow["err_t_m"] <= a.thr_t and arow["err_r_deg"] <= a.thr_r
            color = self.C_TP if tp else self.C_FP
            txt = (f"  attempt {k}: localized, {'TRUE' if tp else 'FALSE'} positive, error {arow['err_t_m']:.3f} m / "
                   f"{arow['err_r_deg']:.2f} deg ({arow['time_ms']:.0f} ms)")
            self.event(txt, "INFO" if tp else "ERROR")
        else:
            color = self.C_NOREF
            self.event(f"  attempt {k}: localized, no reference pose ({arow['time_ms']:.0f} ms)")
        self._arrow(f"world/query/attempts/{k}/result", T_loc, color, f"result {k}", 0.6)
        return color

    def end_query(self, row):
        if row.get("post_frames"):
            self.event(f"  after localization: {row['post_frames']} frames, RMSE {row['post_rmse_t_m']:.3f} m, "
                       f"final error {row['post_final_err_t_m']:.3f} m / {row['post_final_err_r_deg']:.2f} deg")

    def close(self):
        if self.args.viewer:
            from view_frames import wait_for_viewer
            wait_for_viewer(rr, log)


def t1_values(args, ses, ref):
    if args.t1:
        return list(args.t1)
    # sampling: t1 - max(t_prev) must be inside the map, t2 inside the capture
    recv = {f["ts"]: f["recv"] for f in ses.frames}
    first = (recv.get(int(ref.ts[0]), ses.frames[0]["recv"]) - ses.t_bag0) / NS
    last_recv = (ses.frames[-1]["recv"] - ses.t_bag0) / NS
    lo = max(args.margin, first + max(args.t_prev) + args.margin)
    hi = last_recv - args.duration
    vals, t = [], lo
    while t <= hi + 1e-9:
        vals.append(round(t, 3))
        t += args.t1_step
    return vals


def cmd_localize(args):
    map_root = os.path.abspath(os.path.expanduser(args.map))
    with open(os.path.join(map_root, "map_info.json")) as f:
        minfo = json.load(f)
    map_dir = os.path.join(map_root, minfo.get("map_dir", "map"))
    ref = Reference(os.path.join(map_root, "reference_slam.csv"))
    g_map = minfo.get("gravity_map")
    g_dir = np.asarray(g_map) if g_map else None

    # same rig configuration as the map, query bag from the command line
    cfg = dict(minfo["config"])
    cfg["bag"] = os.path.abspath(os.path.expanduser(args.bag))
    same_capture = os.path.realpath(cfg["bag"]) == os.path.realpath(minfo["config"]["bag"])
    if not same_capture:
        sys.exit("Query capture different from the map capture: not supported yet (no reference poses)")
    out = os.path.abspath(os.path.expanduser(args.out_dir or os.path.join(map_root, "localize")))
    os.makedirs(out, exist_ok=True)
    cuvslam.set_verbosity(args.verbosity)
    ses = Session(cfg)

    settings = cuvslam.Slam.LocalizationSettings(
        horizontal_search_radius=args.h_radius, vertical_search_radius=args.v_radius,
        horizontal_step=args.h_step, vertical_step=args.v_step,
        angular_step_rads=math.radians(args.angle_step_deg))
    n_probes = ((int(2 * args.h_radius / args.h_step) + 1) ** 2 * (int(2 * args.v_radius / args.v_step) + 1)
                * int(2 * math.pi / math.radians(args.angle_step_deg)))
    t1s = t1_values(args, ses, ref)
    viz = Viz(args, ses, ref, map_root, g_map) if (args.viewer or args.save) else None
    log(f"Map {map_dir}: {len(ref.ts)} reference poses; {len(t1s)} values of t1 x {len(args.t_prev)} t_prev; "
        f"search grid {n_probes} probes max")

    q_rows, a_rows, p_rows = [], [], []
    qid = 0
    for t1 in t1s:
        frames = ses.frames_between(t1, t1 + args.duration)
        if len(frames) < 2:
            log(f"t1={t1}: not enough frames, skipped")
            continue
        loaded = ses.load_images(frames)
        ts1 = loaded[0][0]["ts"]
        T_ref1 = ref.at(ts1)
        for t_prev in args.t_prev:
            qid += 1
            ts_prior, T_last = ref.last_before(ts1 - int(round(t_prev * NS)))
            if T_last is None:
                log(f"query {qid}: no reference pose before t1 - t_prev, skipped")
                continue
            if viz:
                viz.begin_query(qid, t1, t_prev, ts1, loaded[-1][0]["ts"], T_last, T_ref1)
            res = run_query(args, ses, ref, loaded, T_last, map_dir, settings, g_map, g_dir, qid, a_rows, p_rows,
                            viz)
            row = {"query": qid, "t1_s": t1, "t_prev_s": t_prev, "t2_s": round(t1 + args.duration, 3),
                   "t1_ts_ns": ts1, "prior_ts_ns": ts_prior,
                   "prior_gap_s": round((ts1 - ts_prior) / NS, 3)}
            if T_ref1 is not None:
                pe = pose_error(T_last, T_ref1, g_dir)
                row.update(prior_err_t_m=round(pe["err_t_m"], 4), prior_err_r_deg=round(pe["err_r_deg"], 3),
                           prior_err_vert_m=round(pe.get("err_vert_m", float("nan")), 4))
            row.update(res)
            q_rows.append(row)
            if viz:
                viz.end_query(row)
            log(f"query {qid}: t1={t1:.2f} s t_prev={t_prev} s prior err "
                f"{row.get('prior_err_t_m', float('nan')):.2f} m / {row.get('prior_err_r_deg', float('nan')):.1f} deg"
                f" -> {'localized at attempt ' + str(res['attempt_ok']) if res['localized'] else 'NOT localized'}"
                + (f", err {res['loc_err_t_m']:.3f} m / {res['loc_err_r_deg']:.2f} deg" if res['localized'] else "")
                + f", localize time {res['localize_total_ms']:.0f} ms")

    # outputs
    q_header = ["query", "t1_s", "t_prev_s", "t2_s", "t1_ts_ns", "prior_ts_ns", "prior_gap_s",
                "prior_err_t_m", "prior_err_r_deg", "prior_err_vert_m", "localized", "attempt_ok", "attempts",
                "loc_ts_ns", "loc_delay_s", "loc_err_t_m", "loc_err_r_deg", "loc_err_vert_m", "within_threshold",
                "tracker_init_ms", "first_attempt_ms", "localize_total_ms", "localize_ok_attempt_ms",
                "post_frames", "post_rmse_t_m", "post_max_err_t_m", "post_final_err_t_m", "post_final_err_r_deg",
                "frames_lost"]
    write_csv(os.path.join(out, "queries.csv"), q_header, [[r.get(k, "") for k in q_header] for r in q_rows])
    a_header = ["query", "attempt", "ts_ns", "t_from_t1_s", "gravity_used", "prior_err_t_m", "prior_err_r_deg",
                "ok", "time_ms", "probes", "probes_solved", "best_weight", "err_t_m", "err_r_deg", "message"]
    write_csv(os.path.join(out, "attempts.csv"), a_header, [[r.get(k, "") for k in a_header] for r in a_rows])
    write_csv(os.path.join(out, "post_localization.csv"),
              ["query", "timestamp_ns", "t_from_loc_s", *POSE_COLS, "err_t_m", "err_r_deg"], p_rows)
    summary = summarize(q_rows, args)
    info = {"script": "map_bag.py localize", "map": map_root, "query_bag": cfg["bag"], "t1": t1s,
            "t_prev": args.t_prev, "duration_s": args.duration, "attempts": args.attempts,
            "attempt_every_s": args.attempt_every, "prior_gravity": args.prior_gravity,
            "search": {"h_radius": args.h_radius, "v_radius": args.v_radius, "h_step": args.h_step,
                       "v_step": args.v_step, "angle_step_deg": args.angle_step_deg, "max_probes": n_probes},
            "threshold": {"t_m": args.thr_t, "r_deg": args.thr_r}, "cuvslam_version": cuvslam_version(cuvslam),
            "summary": summary}
    with open(os.path.join(out, "localize_info.json"), "w") as f:
        json.dump(info, f, indent=2, default=str)
    log(json.dumps(summary, indent=2))
    log(f"Output: {out}")
    if viz:
        viz.close()


def run_query(args, ses, ref, loaded, T_last, map_dir, settings, g_map, g_dir, qid, a_rows, p_rows, viz=None):
    """One relocalization query on the frames already decoded (new tracker, up to N attempts)."""
    t0 = time.perf_counter()
    tracker = ses.new_tracker(slam=True)
    probes_summary(tracker)  # enables the LocalizerProbes data layer before the first attempt
    init_ms = (time.perf_counter() - t0) * 1000
    feeder = ImuFeeder(ses.imu, loaded[0][0]["ts"] - NS // 10) if ses.use_imu else None
    ts1 = loaded[0][0]["ts"]
    every = int(round(args.attempt_every * NS))

    res = {"localized": False, "attempt_ok": "", "attempts": 0, "tracker_init_ms": round(init_ms, 1),
           "first_attempt_ms": "", "localize_total_ms": 0.0, "frames_lost": 0}
    O_first = None
    next_attempt_ts = ts1
    loc_ts = None
    post = []
    last_ts = None
    loc_color = None
    for fr, images in loaded:
        ts = fr["ts"]
        if last_ts is not None and ts <= last_ts:
            continue
        if feeder:
            feeder.feed(tracker, ts)
        est, slam_pose = tracker.track(ts, images)
        last_ts = ts
        if est.world_from_rig is None:
            res["frames_lost"] += 1
            continue
        O = to_mat(est.world_from_rig.pose)
        if O_first is None:
            O_first = O
        if viz:
            # shown pose: SLAM pose (map frame) after localization, last pose + new-session odometry before
            T_show = (to_mat(slam_pose) if slam_pose is not None else None) if loc_ts is not None \
                else T_last @ np.linalg.inv(O_first) @ O
            viz.frame(qid, ts, images, tracker, T_show, loc_ts is not None, loc_color)

        if loc_ts is not None:
            # after localization: SLAM pose in the map frame vs reference
            T_ref = ref.at(ts)
            if slam_pose is not None and T_ref is not None:
                T = to_mat(slam_pose)
                e = pose_error(T, T_ref)
                post.append(e)
                p_rows.append([qid, ts, round((ts - loc_ts) / NS, 4), *row_from_mat(T),
                               round(e["err_t_m"], 4), round(e["err_r_deg"], 3)])
            continue

        if res["attempts"] >= args.attempts or ts < next_attempt_ts:
            if res["attempts"] >= args.attempts and not args.track_after_failure:
                break
            continue

        # attempt: prior = last tracked pose composed with the new session odometry since t1
        T_prior = T_last @ np.linalg.inv(O_first) @ O
        g_used = False
        if args.prior_gravity and ses.use_imu:
            T_prior, g_used = gravity_corrected(T_prior, gravity_vec(tracker.odometry.get_last_gravity()), g_map)
        res["attempts"] += 1
        k = res["attempts"]
        result = {}

        def finish_cb(pose, message, result=result):
            result["pose"], result["message"] = pose, message

        ta = time.perf_counter()
        tracker.slam.localize_in_map(map_dir, ts, to_cupose(T_prior), images, settings,
                                     lambda: None, finish_cb)
        dt = (time.perf_counter() - ta) * 1000
        res["localize_total_ms"] += dt
        if k == 1:
            res["first_attempt_ms"] = round(dt, 1)
        T_ref = ref.at(ts)
        arow = {"query": qid, "attempt": k, "ts_ns": ts, "t_from_t1_s": round((ts - ts1) / NS, 3),
                "gravity_used": g_used, "time_ms": round(dt, 1), "ok": result.get("pose") is not None,
                "message": (result.get("message") or "").replace("\n", " ")}
        if T_ref is not None:
            pe = pose_error(T_prior, T_ref)
            arow.update(prior_err_t_m=round(pe["err_t_m"], 4), prior_err_r_deg=round(pe["err_r_deg"], 3))
        arow.update(probes_summary(tracker))
        if result.get("pose") is not None:
            T_loc = to_mat(result["pose"])
            loc_ts = ts
            res.update(localized=True, attempt_ok=k, loc_ts_ns=ts, loc_delay_s=round((ts - ts1) / NS, 3),
                       localize_ok_attempt_ms=round(dt, 1))
            if T_ref is not None:
                e = pose_error(T_loc, T_ref, g_dir)
                arow.update(err_t_m=round(e["err_t_m"], 4), err_r_deg=round(e["err_r_deg"], 3))
                res.update(loc_err_t_m=round(e["err_t_m"], 4), loc_err_r_deg=round(e["err_r_deg"], 3),
                           loc_err_vert_m=round(e.get("err_vert_m", float("nan")), 4),
                           within_threshold=e["err_t_m"] <= args.thr_t and e["err_r_deg"] <= args.thr_r)
        a_rows.append(arow)
        if viz:
            loc_color = viz.attempt(k, T_prior, T_loc if result.get("pose") is not None else None, arow)
        next_attempt_ts = ts + every

    res["localize_total_ms"] = round(res["localize_total_ms"], 1)
    if post:
        et = np.array([e["err_t_m"] for e in post])
        res.update(post_frames=len(post), post_rmse_t_m=round(float(np.sqrt(np.mean(et ** 2))), 4),
                   post_max_err_t_m=round(float(et.max()), 4), post_final_err_t_m=round(float(et[-1]), 4),
                   post_final_err_r_deg=round(post[-1]["err_r_deg"], 3))
    return res


def summarize(rows, args):
    """Per t_prev: success rate within N attempts (N = 1, 2, 4, ...), errors and times."""
    out = {}
    ns, n = [], 1
    while n <= args.attempts:
        ns.append(n)
        n *= 2
    if ns[-1] != args.attempts:
        ns.append(args.attempts)
    for tp in args.t_prev:
        rs = [r for r in rows if r["t_prev_s"] == tp]
        if not rs:
            continue
        d = {"queries": len(rs)}
        for n in ns:
            ok = [r for r in rs if r["localized"] and r["attempt_ok"] <= n]
            good = [r for r in ok if r.get("within_threshold")]
            d[f"N{n}"] = {"localized": len(ok), "true_positive": len(good), "false_positive": len(ok) - len(good),
                          "not_localized": len(rs) - len(ok),
                          "tp_rate": round(len(good) / len(rs), 3)}
        loc = [r for r in rs if r["localized"]]
        if loc:
            d["loc_err_t_m_median"] = statistics.median(r["loc_err_t_m"] for r in loc if "loc_err_t_m" in r)
            d["loc_err_r_deg_median"] = statistics.median(r["loc_err_r_deg"] for r in loc if "loc_err_r_deg" in r)
            d["loc_delay_s_median"] = statistics.median(r["loc_delay_s"] for r in loc)
            d["ok_attempt_ms_median"] = statistics.median(r["localize_ok_attempt_ms"] for r in loc)
        d["localize_total_ms_median"] = statistics.median(r["localize_total_ms"] for r in rs)
        d["prior_err_t_m_median"] = statistics.median(r["prior_err_t_m"] for r in rs if "prior_err_t_m" in r) \
            if any("prior_err_t_m" in r for r in rs) else None
        out[f"t_prev_{tp}"] = d
    return out


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def add_bag_args(p):
    p.add_argument("bag", help="bag folder (.mcap/.db3) or single file")
    p.add_argument("--verbosity", type=int, default=0, help="cuvslam.set_verbosity (0-3)")


def parse_args():
    ap = argparse.ArgumentParser(description="cuVSLAM map building and relocalization tests on a ROS 2 bag",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    b = sub.add_parser("build", help="run SLAM on a capture and save the map + reference trajectory",
                       formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    add_bag_args(b)
    b.add_argument("--out-dir", required=True, help="output folder (map/, reference_slam.csv, map_info.json, run/)")
    b.add_argument("--mode", choices=list(MODES), default="inertial")
    b.add_argument("--streams", nargs="+", default=None,
                   help="streams <cam>/<rgb|ir1|ir2>; the first one is the rig origin (as track_bag_rerun.py)")
    b.add_argument("--imu-camera", default="cam2")
    b.add_argument("--extrinsic", nargs=7, type=float, default=CAD_EXTRINSIC,
                   metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"))
    b.add_argument("--link-frame", default="{cam}_link")
    b.add_argument("--require-all-streams", action=argparse.BooleanOptionalAction, default=None,
                   help="default: on in inertial mode, off otherwise")
    b.add_argument("--tolerance-ms", type=float, default=1.0)
    b.add_argument("--stale-threshold", type=float, default=0.5)
    b.add_argument("--multicam-mode", choices=list(MULTICAM), default="precision")
    b.add_argument("--rectified", action="store_true", help="rectified_stereo_camera=True (rectified IR pairs)")
    b.add_argument("--max-map-size", type=int, default=0,
                   help="Slam.Config.max_map_size (poses in the pose graph, 0 = unlimited; cuVSLAM default 300)")
    b.add_argument("--start", type=float, default=None, help="map range start, seconds from the beginning")
    b.add_argument("--end", type=float, default=None, help="map range end, seconds from the beginning")
    b.add_argument("--max-frames", type=int, default=None)

    q = sub.add_parser("localize", help="relocalization queries on [t1, t2] cuts of a capture",
                       formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    add_bag_args(q)
    q.add_argument("--map", required=True, help="folder written by 'build'")
    q.add_argument("--out-dir", default=None, help="output folder (default <map>/localize)")
    q.add_argument("--t1", nargs="+", type=float, default=None,
                   help="start times of the cuts (s from the beginning); default: sampled with --t1-step")
    q.add_argument("--t1-step", type=float, default=5.0, help="sampling step of t1 (s)")
    q.add_argument("--margin", type=float, default=2.0,
                   help="t1 - max(t_prev) at least this many seconds after the start of the map")
    q.add_argument("--duration", type=float, default=10.0, help="length of each cut t2 - t1 (s)")
    q.add_argument("--t-prev", nargs="+", type=float, default=[0.0, 1.0, 3.0, 5.0],
                   help="simulated tracking loss: prior = reference pose at t1 - t_prev")
    q.add_argument("--attempts", type=int, default=16, help="maximum single-frame localization attempts (N)")
    q.add_argument("--attempt-every", type=float, default=0.25, help="seconds between attempts")
    q.add_argument("--track-after-failure", action="store_true",
                   help="keep tracking until t2 when all the attempts failed (default: stop)")
    q.add_argument("--prior-gravity", action=argparse.BooleanOptionalAction, default=True,
                   help="take roll/pitch of the prior from the gravity estimated by the new session (inertial)")
    q.add_argument("--h-radius", type=float, default=4.0, help="horizontal search radius (m, rig x and z)")
    q.add_argument("--v-radius", type=float, default=1.0, help="vertical search radius (m, rig y)")
    q.add_argument("--h-step", type=float, default=0.5, help="horizontal step (m)")
    q.add_argument("--v-step", type=float, default=0.25, help="vertical step (m)")
    q.add_argument("--angle-step-deg", type=float, default=10.0, help="rotation step around rig y (deg)")
    q.add_argument("--thr-t", type=float, default=0.3, help="true positive threshold, translation (m)")
    q.add_argument("--thr-r", type=float, default=10.0, help="true positive threshold, rotation (deg)")
    q.add_argument("--viewer", action="store_true",
                   help="show the queries live in the Rerun viewer (map, reference, attempts, tracking)")
    q.add_argument("--save", default=None, help="save the Rerun recording to this .rrd file")
    q.add_argument("--pace", type=float, default=0.0,
                   help="with --viewer: play the cuts at this speed (1 = real time; 0 = as fast as possible)")
    q.add_argument("--image-every", type=int, default=1, help="send the images to Rerun every N frames")
    return ap.parse_args()


def main():
    args = parse_args()
    if args.cmd == "build":
        cmd_build(args)
    else:
        cmd_localize(args)


if __name__ == "__main__":
    main()
