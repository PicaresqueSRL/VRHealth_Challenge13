#!/usr/bin/env python3
"""
run_output.py - Saves the output of a PyCuVSLAM run in the same CSV format produced by
isaac_ros_5/launch/extract_paths.py for the Isaac ROS runs, so that the two can be compared
directly and fed to results/ground_truth/RunBatchPipeline.py / IcpAlign.py.

Used by track_edex_rerun.py and track_bag_rerun.py (option --out-dir). Files written:

    <out_dir>/
    ├── odometry.csv       one pose per tracked frame (visual-inertial odometry)
    ├── slam_online.csv    SLAM pose returned frame by frame during the run (only with --slam)
    ├── slam_poses.csv     final SLAM trajectory (get_all_slam_poses at the end of the run,
    │                      i.e. optimized with the loop closures; only with --slam)
    ├── status.csv         per-frame tracking status, processing time, observations, landmarks
    ├── slam_metrics.csv   per-frame loop closure / pose graph optimization status (only with --slam)
    ├── loop_closures.csv  last loop closure poses reported by SLAM (cuVSLAM keeps the last 10)
    └── run_info.json      input, parameters, frames and summary metrics of the run

Pose columns (same as the CSV files of the repository):
    timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg
Timestamps in seconds. Poses in meters, of the base frame (default <cam>_link, x forward,
y left, z up, as base_frame in the Isaac ROS launch files), relative to its first pose:
the first row is the identity, as in the Isaac ROS output.
"""

import csv
import json
import math
import os
import time

import numpy as np

POSE_HEADER = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "roll_deg", "pitch_deg", "yaw_deg"]


def quat_to_rot(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    """Rotation matrix -> [x, y, z, w] with w >= 0."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q) / np.linalg.norm(q)
    return (-q if q[3] < 0 else q).tolist()


def pose_to_matrix(pose):
    """cuvslam.Pose (rotation [x, y, z, w], translation) -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(list(pose.rotation))
    T[:3, 3] = list(pose.translation)
    return T


def rpy_deg(qx, qy, qz, qw):
    # same formula as extract_paths.py
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


def pose_row(ts_ns, T):
    q = rot_to_quat(T[:3, :3])
    return [ts_ns / 1e9, *T[:3, 3].tolist(), *q, *rpy_deg(*q)]


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def path_stats(rows):
    """Length of the path and distance between first and last position (meters)."""
    if len(rows) < 2:
        return {"poses": len(rows)}
    p = np.array([r[1:4] for r in rows])
    return {"poses": len(rows), "duration_s": rows[-1][0] - rows[0][0],
            "path_length_m": float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()),
            "start_end_distance_m": float(np.linalg.norm(p[-1] - p[0]))}


class RunWriter:
    """Collects the per-frame output of a Tracker and writes it to out_dir.

    T_rig_from_base: 4x4 transform from the base frame to the cuVSLAM rig frame (the first
    camera of the rig, OpenCV axes). The rig poses world_from_rig are converted to the base
    frame and made relative to the first pose: T = inv(W_b0) @ W_b, with W_b = world_from_rig @ T_rig_from_base.
    """

    def __init__(self, out_dir, T_rig_from_base, base_frame, n_cameras, info):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.T_rb = np.asarray(T_rig_from_base, dtype=float)
        self.base_frame = base_frame
        self.n_cameras = n_cameras
        self.info = dict(info)
        self.odom, self.slam_online, self.status, self.metrics = [], [], [], []
        self._odom0 = self._slam0 = None
        self.t_wall = time.time()

    def _to_base(self, pose, which):
        W = pose_to_matrix(pose) @ self.T_rb
        ref = getattr(self, which)
        if ref is None:
            ref = np.linalg.inv(W)
            setattr(self, which, ref)
        return ref @ W

    def add_frame(self, ts_ns, est, slam_pose, obs_counts, n_landmarks, track_ms, tracker=None):
        ok = est is not None and est.world_from_rig is not None
        if ok:
            self.odom.append(pose_row(ts_ns, self._to_base(est.world_from_rig.pose, "_odom0")))
        if slam_pose is not None:
            self.slam_online.append(pose_row(ts_ns, self._to_base(slam_pose, "_slam0")))
        # vo_state as in Isaac ROS (1 = success, 2 = failed)
        self.status.append([ts_ns / 1e9, 1 if ok else 2, round(track_ms, 3), n_landmarks]
                           + [obs_counts.get(k, 0) for k in range(self.n_cameras)])
        # reading tracker.slam raises RuntimeError when SLAM is not enabled
        slam = tracker.slam if tracker is not None and tracker.is_slam_enabled() else None
        if slam is not None and hasattr(slam, "get_slam_metrics"):
            try:
                m = slam.get_slam_metrics()
                self.metrics.append([ts_ns / 1e9, int(m.timestamp_ns), str(m.lc_status), str(m.pgo_status),
                                     m.lc_selected_landmarks_count, m.lc_tracked_landmarks_count,
                                     m.lc_pnp_landmarks_count, m.lc_good_landmarks_count])
            except Exception:  # noqa: BLE001  (metrics are optional: never stop the run)
                pass

    def finish(self, tracker=None, slam_enabled=False, n_final_landmarks=None):
        d = self.out_dir
        write_csv(os.path.join(d, "odometry.csv"), POSE_HEADER, self.odom)
        write_csv(os.path.join(d, "status.csv"),
                  ["timestamp", "vo_state", "track_time_ms", "landmarks"]
                  + [f"observations_cam{k}" for k in range(self.n_cameras)], self.status)
        summary = {"odometry": path_stats(self.odom),
                   "frames": len(self.status),
                   "frames_lost": sum(r[1] != 1 for r in self.status),
                   "track_time_ms_median": float(np.median([r[2] for r in self.status])) if self.status else None,
                   "wall_time_s": round(time.time() - self.t_wall, 2),
                   "final_landmarks": n_final_landmarks}
        files = ["odometry.csv", "status.csv"]

        slam = getattr(tracker, "slam", None) if (tracker is not None and slam_enabled) else None
        if slam_enabled:
            write_csv(os.path.join(d, "slam_online.csv"), POSE_HEADER, self.slam_online)
            summary["slam_online"] = path_stats(self.slam_online)
            files.append("slam_online.csv")
        if slam is not None and hasattr(slam, "get_all_slam_poses"):
            poses = sorted(slam.get_all_slam_poses(), key=lambda p: p.timestamp_ns)
            ref, rows = None, []
            for p in poses:
                W = pose_to_matrix(p.pose) @ self.T_rb
                ref = np.linalg.inv(W) if ref is None else ref
                rows.append(pose_row(int(p.timestamp_ns), ref @ W))
            write_csv(os.path.join(d, "slam_poses.csv"), POSE_HEADER, rows)
            summary["slam_final"] = path_stats(rows)
            files.append("slam_poses.csv")
        elif slam_enabled:
            summary["slam_final"] = "get_all_slam_poses not available in this cuvslam version"
        if slam is not None and hasattr(slam, "get_loop_closure_poses"):
            lc = sorted(slam.get_loop_closure_poses(), key=lambda p: p.timestamp_ns)
            # loop closure poses are in the SLAM world frame (rig, OpenCV axes): written as they are
            write_csv(os.path.join(d, "loop_closures.csv"), ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"],
                      [[p.timestamp_ns / 1e9, *list(p.pose.translation), *list(p.pose.rotation)] for p in lc])
            summary["loop_closure_poses_reported"] = len(lc)
            files.append("loop_closures.csv")
        if self.metrics:
            write_csv(os.path.join(d, "slam_metrics.csv"),
                      ["timestamp", "metrics_timestamp_ns", "lc_status", "pgo_status", "lc_selected_landmarks",
                       "lc_tracked_landmarks", "lc_pnp_landmarks", "lc_good_landmarks"], self.metrics)
            summary["frames_with_pgo"] = sum(r[3].lower() in ("true", "1") for r in self.metrics)
            summary["frames_with_lc"] = sum(r[2].lower() in ("true", "1") for r in self.metrics)
            files.append("slam_metrics.csv")

        self.info.update(base_frame=self.base_frame, T_rig_from_base=self.T_rb.tolist(),
                         summary=summary, files=files)
        with open(os.path.join(d, "run_info.json"), "w") as f:
            json.dump(self.info, f, indent=2, default=str)
        return summary


def cuvslam_version(cuvslam):
    try:
        v = cuvslam.get_version()
        return v if isinstance(v, str) else str(v)
    except Exception:  # noqa: BLE001
        return "unknown"
