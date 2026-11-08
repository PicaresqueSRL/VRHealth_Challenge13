#!/usr/bin/env python3
"""
view_frames.py - Common 3D frame for the Rerun viewers, so that runs of PyCuVSLAM
(track_edex_rerun.py, track_bag_rerun.py) and of Isaac ROS (bag_output_rerun.py) look the same and can be
overlaid.

World frame of the viewers (--world base, default): the base frame (e.g. cam2_link: x forward, y left, z up)
at the first pose of the run. It is the same convention as the saved CSV files (run_output.py, extract_paths.py):
the first pose is the identity, and every pose is relative to it.

    PyCuVSLAM:  world = rig (first camera, OpenCV axes) at the first frame
                -> new world = base frame at the first frame: W_b = world_from_rig @ T_rig_from_base,
                   pose = inv(W_b0) @ W_b
    Isaac ROS:  world = odom (z up, origin and heading set by the node)
                -> new world = base frame at the first odometry pose: pose = inv(T_odom_base0) @ T_odom_base

--world native keeps the original world frame of each tool.

--compare label=folder overlays the trajectories saved in folder (odometry.csv and slam_poses.csv, in the CSV
format of the Isaac ROS runs or of --out-dir). They are re-anchored at the pose closest to the first timestamp of the
current run, so that runs starting a few frames apart still start at the same point.
"""

import csv
import math
import os
import signal

import numpy as np

COMPARE_COLORS = [[235, 104, 52], [27, 175, 122], [237, 161, 0], [232, 123, 164], [0, 131, 0]]


def quat_to_rot(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix(rotation_xyzw, translation):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(list(rotation_xyzw))
    T[:3, 3] = list(translation)
    return T


class ViewFrame:
    """Converts the poses and points of a run to the world frame of the viewer.

    T_rig_from_base: transform from the base frame to the frame in which the tool gives its poses (the cuVSLAM rig for
    PyCuVSLAM; identity for Isaac ROS, whose odometry already gives the base frame). None = native world frame.
    """

    def __init__(self, T_rig_from_base=None):
        self.native = T_rig_from_base is None
        self.T_rb = np.eye(4) if self.native else np.asarray(T_rig_from_base, dtype=float)
        self.T_br = np.linalg.inv(self.T_rb)
        self.T_new_from_world = None if not self.native else np.eye(4)

    def view_coordinates(self, rr, native_coordinates):
        return native_coordinates if self.native else rr.ViewCoordinates.RIGHT_HAND_Z_UP

    def base_pose(self, world_from_rig):
        """4x4 pose (world of the tool, frame of the tool) -> pose of the base frame in the viewer world.
        The first call fixes the viewer world (base frame at that pose)."""
        W = np.asarray(world_from_rig) @ self.T_rb
        if self.T_new_from_world is None:
            self.T_new_from_world = np.linalg.inv(W)
        return self.T_new_from_world @ W

    def world_points(self, pts):
        """Points in the world frame of the tool -> viewer world."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        if self.T_new_from_world is None or not len(pts):
            return pts
        T = self.T_new_from_world
        return pts @ T[:3, :3].T + T[:3, 3]

    def rig_points(self, pts):
        """Points in the frame of the tool (rig) -> base frame (children of world/rig)."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        return pts @ self.T_br[:3, :3].T + self.T_br[:3, 3] if len(pts) else pts

    def rig_vector(self, v):
        return self.T_br[:3, :3] @ np.asarray(v, dtype=float)

    def base_from_camera(self, T_rig_from_cam):
        """Transform of a camera entity under world/rig."""
        return self.T_br @ np.asarray(T_rig_from_cam)


def wait_for_viewer(rr, log=print):
    """Keeps the container alive while the viewer is open. Under docker/run.sh the script is PID 1: when it exits
    the kernel kills every other process of the container, including the viewer. Outside a container: no effect."""
    rr.get_data_recording().flush()
    if os.getpid() != 1:
        return
    log("Rerun viewer still open: close its window (or press Ctrl+C) to exit")
    signal.signal(signal.SIGTERM, signal.default_int_handler)  # PID 1 ignores SIGTERM without a handler
    try:
        while True:
            os.waitpid(-1, 0)  # returns when a child exits; the viewer is a child of PID 1
    except (ChildProcessError, KeyboardInterrupt):
        pass


def read_pose_csv(path):
    """CSV with timestamp,x,y,z,qx,qy,qz,qw,... -> N x 8 array sorted by time."""
    with open(path) as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return np.zeros((0, 8))
    h = rows[0]
    idx = [h.index(k) for k in ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw")]
    a = np.array([[float(r[i]) for i in idx] for r in rows[1:] if r])
    return a[np.argsort(a[:, 0], kind="stable")]


def anchored_positions(a, t_first_s, max_gap_s=0.5):
    """Positions of the poses re-anchored at the pose closest to t_first_s (identity there)."""
    if not len(a):
        return a[:, 1:4]
    j = int(np.argmin(np.abs(a[:, 0] - t_first_s))) if t_first_s is not None else 0
    if t_first_s is not None and abs(a[j, 0] - t_first_s) > max_gap_s:
        j = 0
    ref = np.linalg.inv(matrix(a[j, 4:8], a[j, 1:4]))
    return np.array([(ref @ matrix(r[4:8], r[1:4]))[:3, 3] for r in a])


def parse_compare(specs):
    out = []
    for spec in specs or []:
        label, _, folder = spec.partition("=") if "=" in spec else (None, None, spec)
        folder = os.path.expanduser(folder)
        out.append((label or os.path.basename(os.path.normpath(folder)), folder))
    return out


def log_compare(rr, specs, t_first_ns, log=print):
    """Overlays the saved trajectories of other runs as static lines under world/compare/<label>/."""
    t_first = None if t_first_ns is None else t_first_ns / 1e9
    for k, (label, folder) in enumerate(parse_compare(specs)):
        color = COMPARE_COLORS[k % len(COMPARE_COLORS)]
        found = False
        for name, radius in (("odometry", 0.01), ("slam_poses", 0.006)):
            p = os.path.join(folder, f"{name}.csv")
            if not os.path.exists(p):
                continue
            a = read_pose_csv(p)
            if len(a) < 2:
                continue
            pts = anchored_positions(a, t_first)
            col = color if name == "odometry" else [int(0.6 * c + 0.4 * 255) for c in color]
            rr.log(f"world/compare/{label}/{name}", rr.LineStrips3D([pts], colors=[col], radii=radius), static=True)
            found = True
        if not found:
            log(f"WARNING: --compare {label}: no odometry.csv / slam_poses.csv in {folder}")
        else:
            log(f"Compared run '{label}' overlaid from {folder}")
