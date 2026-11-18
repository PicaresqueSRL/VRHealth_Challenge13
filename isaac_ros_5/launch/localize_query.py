#!/usr/bin/env python3
"""
localize_query.py - Relocalization queries with Isaac ROS cuVSLAM (visual_slam/localize_in_map) during bag playback.

Runs inside the Isaac ROS container (ROS 2 Lyrical, isaac_ros_cuvslam 5.0). For every query (t1, t_prev):
  1. starts a NEW visual_slam_node (cuvslam_stereo_imu_cam2_localize.launch.py): new session, no map;
  2. republishes /tf_static of the bag (latched) and plays the cut [t1 - 0.1 s, t1 + duration] at real-time speed;
  3. prior = last pose of the mapping run odometry.csv with timestamp <= t1 - t_prev (T_last), composed with the
     odometry of the new session since its first frame: prior_k = T_last * O_first^-1 * O_k (cam2_link poses);
  4. calls visual_slam/localize_in_map(map_folder_path, pose_hint=prior_k) on the first tracked frame; the node
     localizes on the next frame it tracks and logs the result on /rosout ("Successfully localized at ..." or
     "Failed to localize in map ..."); on failure a new attempt every --attempt-every s (bag time), up to --attempts;
  5. after success keeps playing until t2; the pose of every frame in the map frame is T(map<-odom) * O_k. The
     map->odom TF of the localization appears a few frames after the localized frame: the localized pose is taken at
     the first frame with the new map->odom (tf_update_delay_s); loc_log_err_t_m is the error of the position logged
     by the node at the localized frame;
  6. errors against slam_poses.csv of the mapping run (same map frame, cam2_link, no alignment).
Optional (--g-map, off by default): roll/pitch of each prior taken from the accelerometer (mean of the last
--imu-window s of /cam2/imu, rotated to cam2_link with /tf_static) aligned with the given gravity direction in the map
frame; heading and position unchanged. A query is stopped if no odometry arrives for --stall-timeout s (wall time).
Outputs (--out-dir): queries.csv, attempts.csv, post_localization.csv, localize_info.json, q<NNN>/ (bag + logs).

Usage (inside the container, ROS sourced):
  python3 localize_query.py --bag <bag> --map-run <isaac_maps/<capture>> --out-dir <dir> [--t1 20 35] [--t-prev 0 1 3 5]
"""
import argparse
import bisect
import csv
import json
import math
import os
import re
import signal
import subprocess
import time

import numpy as np
import rclpy
import rosbag2_py
import yaml
from geometry_msgs.msg import Pose
from isaac_ros_visual_slam_interfaces.srv import LocalizeInMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

NS = 1_000_000_000
HERE = os.path.dirname(os.path.abspath(__file__))
PLAY_TOPICS = ["/cam2/infra1/image_rect_raw", "/cam2/infra1/camera_info", "/cam2/infra2/image_rect_raw",
               "/cam2/infra2/camera_info", "/cam2/imu"]
RECORD_TOPICS = ["/visual_slam/tracking/odometry", "/visual_slam/tracking/slam_path", "/visual_slam/status",
                 "/tf", "/rosout"]


# ----------------------------------------------------------------------------- geometry
def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [x, y, z, w]


def mat(t, q):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(*q)
    T[:3, 3] = t
    return T


def from_pose_msg(p):
    return mat([p.position.x, p.position.y, p.position.z], [p.orientation.x, p.orientation.y, p.orientation.z,
                                                             p.orientation.w])


def to_pose_msg(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in rot_to_quat(T[:3, :3]))
    return p


def from_tf(tr):
    return mat([tr.translation.x, tr.translation.y, tr.translation.z],
               [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w])


def align_vectors(a, b):
    """Minimal rotation R with R @ a = b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


def pose_error(T, R, g=None):
    d = T[:3, 3] - R[:3, 3]
    c = (np.trace(R[:3, :3].T @ T[:3, :3]) - 1) / 2
    e = {"err_t_m": float(np.linalg.norm(d)), "err_r_deg": math.degrees(math.acos(max(-1.0, min(1.0, c))))}
    if g is not None:
        e["err_vert_m"] = float(abs(d @ g))
    return e


class Trajectory:
    """Isaac ROS CSV (timestamp s, x, y, z, qx, qy, qz, qw) of cam2_link poses."""

    def __init__(self, path):
        rows = list(csv.DictReader(open(path)))
        self.ts = [int(round(float(r["timestamp"]) * NS)) for r in rows]
        self.T = [mat([float(r[k]) for k in "xyz"], [float(r[k]) for k in ("qx", "qy", "qz", "qw")]) for r in rows]

    def last_before(self, ts):
        i = bisect.bisect_right(self.ts, ts) - 1
        return (self.ts[i], self.T[i]) if i >= 0 else (None, None)

    def at(self, ts, tol=2_000_000):
        i = bisect.bisect_left(self.ts, ts)
        best = min((j for j in (i - 1, i) if 0 <= j < len(self.ts)), key=lambda j: abs(self.ts[j] - ts), default=None)
        return self.T[best] if best is not None and abs(self.ts[best] - ts) <= tol else None


# ----------------------------------------------------------------------------- bag helpers
def bag_start_ns(bag):
    meta = yaml.safe_load(open(os.path.join(bag, "metadata.yaml")))["rosbag2_bagfile_information"]
    return int(meta["starting_time"]["nanoseconds_since_epoch"])


def header_offset_ns(bag, topic="/cam2/infra1/image_rect_raw", n=50):
    """Median (receive time - header stamp) of the first images: 0 for the original captures, days for the
    'converted' captures whose receive times were rewritten. t1/t2 are bag (receive) times, poses use header stamps."""
    import struct
    reader = rosbag2_py.SequentialReader()
    meta = yaml.safe_load(open(os.path.join(bag, "metadata.yaml")))["rosbag2_bagfile_information"]
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta["storage_identifier"]),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    d = []
    while reader.has_next() and len(d) < n:
        _, data, recv = reader.read_next()
        e = "<" if data[1] & 1 else ">"
        sec, nsec = struct.unpack_from(e + "iI", data, 4)
        d.append(recv - (sec * NS + nsec))
    return int(np.median(d)) if d else 0


def read_tf_static(bag):
    reader = rosbag2_py.SequentialReader()
    meta = yaml.safe_load(open(os.path.join(bag, "metadata.yaml")))["rosbag2_bagfile_information"]
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=meta["storage_identifier"]),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tf_static"]))
    transforms = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        transforms += deserialize_message(data, TFMessage).transforms
    return TFMessage(transforms=transforms)


def stop_group(proc, timeout=15):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue


# ----------------------------------------------------------------------------- one query
class QueryNode(Node):
    """Listens to the new session and drives the localization attempts of one query."""

    def __init__(self, args, T_last, map_dir, ts_start):
        super().__init__(f"localize_query_{os.getpid()}")
        self.args, self.T_last, self.map_dir, self.ts_start = args, T_last, map_dir, ts_start
        self.client = self.create_client(LocalizeInMap, "/visual_slam/localize_in_map")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self.on_odom, 50)
        self.g_map = np.asarray(args.g_map, dtype=float) / np.linalg.norm(args.g_map) if args.g_map else None
        self.imu = []             # (stamp_ns, accel 3-vector, frame_id), only with --g-map
        if self.g_map is not None:
            self.create_subscription(Imu, "/cam2/imu", self.on_imu, 400)
        self.last_odom_wall = None
        self.create_subscription(Log, "/rosout", self.on_log, QoSProfile(
            depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))
        self.every = int(round(args.attempt_every * NS))
        self.odom = []            # (stamp_ns, O) of the new session, in order
        self.O_first = None
        self.pending = None       # attempt in progress (dict)
        self.attempts = []        # finished attempts
        self.next_attempt_ns = None
        self.loc_ns = None        # first odometry stamp after the success log

    # odometry of the new session: one message per tracked frame
    def on_odom(self, msg):
        ts = Time.from_msg(msg.header.stamp).nanoseconds
        if ts < self.ts_start:
            return
        O = from_pose_msg(msg.pose.pose)
        self.odom.append((ts, O))
        self.last_odom_wall = time.time()
        if self.O_first is None:
            self.O_first = O
            self.next_attempt_ns = ts
        if self.pending is not None:
            return
        if self.loc_ns is not None:
            return
        if len(self.attempts) < self.args.attempts and ts >= self.next_attempt_ns:
            self.start_attempt(ts, O)

    def on_imu(self, msg):
        ts = Time.from_msg(msg.header.stamp).nanoseconds
        a = msg.linear_acceleration
        self.imu.append((ts, np.array([a.x, a.y, a.z]), msg.header.frame_id))
        if len(self.imu) > 2000:
            del self.imu[:1000]

    def gravity_base(self, ts):
        """Gravity direction in cam2_link from the mean accelerometer of the last --imu-window s (None if n/a)."""
        lo = ts - int(self.args.imu_window * NS)
        acc = [(a, f) for t, a, f in self.imu if lo <= t <= ts]
        if len(acc) < 5:
            return None
        try:
            tr = self.tf_buffer.lookup_transform("cam2_link", acc[-1][1], Time())
        except Exception:  # noqa: BLE001
            return None
        g = -from_tf(tr.transform)[:3, :3] @ np.mean([a for a, _ in acc], axis=0)  # accel = -gravity at rest
        n = np.linalg.norm(g)
        return g / n if n > 1e-6 else None

    def start_attempt(self, ts, O):
        prior = self.T_last @ np.linalg.inv(self.O_first) @ O
        g_used = False
        if self.g_map is not None:
            g = self.gravity_base(ts)
            if g is not None:
                prior = prior.copy()
                prior[:3, :3] = align_vectors(prior[:3, :3] @ g, self.g_map) @ prior[:3, :3]
                g_used = True
        req = LocalizeInMap.Request()
        req.map_folder_path = self.map_dir
        req.pose_hint = to_pose_msg(prior)
        self.pending = {"attempt": len(self.attempts) + 1, "call_ns": ts, "prior": prior,
                        "wall0": time.perf_counter(), "frame_ns": None, "gravity_used": g_used}
        self.client.call_async(req)  # returns at once ("started"); the result comes on /rosout

    # result of the attempt, from the log of visual_slam_node
    def on_log(self, msg):
        if self.pending is None or msg.name != "visual_slam_node":
            return
        text = msg.msg
        if text.startswith("Successfully localized"):
            ok = True
        elif text.startswith("Failed to localize in map") or text.startswith("Cannot localize"):
            ok = False
        else:
            return
        a = self.pending
        a.update(ok=ok, message=text, latency_ms=(time.perf_counter() - a["wall0"]) * 1000,
                 frame_ns=self.odom[-1][0] if self.odom else None)   # frame tracked when the result arrived
        self.attempts.append(a)
        self.pending = None
        if ok:
            self.loc_ns = a["frame_ns"]
        else:
            self.next_attempt_ns = a["call_ns"] + self.every

    def map_odom(self, ts):
        """T(map<-odom) at ts from the TF buffer (None if not available)."""
        try:
            return from_tf(self.tf_buffer.lookup_transform("map", "odom", Time(nanoseconds=ts)).transform)
        except Exception:  # noqa: BLE001
            return None

    def map_pose(self, ts, O):
        """Pose of cam2_link in the map frame at ts: T(map<-odom) * O (None if the transform is not available)."""
        try:
            tr = self.tf_buffer.lookup_transform("map", "odom", Time(nanoseconds=ts))
        except Exception:  # noqa: BLE001
            return None
        return from_tf(tr.transform) @ O


def read_query_bag(bag):
    """Odometry (stamp, recv, 4x4), map->odom TF by stamp and visual_slam_node log (recv, text) of a query bag."""
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    odom, tf, logs = [], {}, []
    while reader.has_next():
        topic, data, recv = reader.read_next()
        if topic == "/visual_slam/tracking/odometry":
            m = deserialize_message(data, types[topic])
            odom.append((Time.from_msg(m.header.stamp).nanoseconds, recv, from_pose_msg(m.pose.pose)))
        elif topic == "/tf":
            for x in deserialize_message(data, types[topic]).transforms:
                if x.header.frame_id == "map" and x.child_frame_id == "odom":
                    tf[Time.from_msg(x.header.stamp).nanoseconds] = from_tf(x.transform)
        elif topic == "/rosout":
            m = deserialize_message(data, types[topic])
            if m.name == "visual_slam_node":
                logs.append((recv, m.msg))
    return odom, tf, logs


def evaluate_bag(qdir, ref, args, qid):
    """Localization metrics of one query from its recorded bag.

    - localized frame: last odometry message received before the "Successfully localized" log;
      loc_delay_s = time of that frame from t1;
    - Isaac ROS publishes one transitional map->odom after the localized frame, then the settled one: the localized
      pose is T(map<-odom) * O at the first frame whose map->odom differs from the one at the localized frame and is
      equal to the one of the next frame (settled); errors there -> loc_err_*, published_tp; published errors at
      +1 s and +2 s from the localized frame;
    - localizer_tp: logged position within --thr-t of the reference at the closest frame between the call and the log;
    - post-localization errors from the settled frame to t2."""
    odom, tf, logs = read_query_bag(os.path.join(qdir, "bag"))
    out, post_rows = {}, []
    # per-frame trace of the query (for the Rerun view): odometry of the new session and published pose in the map
    with open(os.path.join(qdir, "trace.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_ns", "recv_ns", "odom_x", "odom_y", "odom_z", "odom_qx", "odom_qy", "odom_qz", "odom_qw",
                    "pub_x", "pub_y", "pub_z", "pub_qx", "pub_qy", "pub_qz", "pub_qw"])
        for ts, recv, O in odom:
            T = tf[ts] @ O if ts in tf else None
            w.writerow([ts, recv, *np.round(O[:3, 3], 5), *np.round(rot_to_quat(O[:3, :3]), 6),
                        *(list(np.round(T[:3, 3], 5)) + list(np.round(rot_to_quat(T[:3, :3]), 6)) if T is not None
                          else [""] * 7)])
    with open(os.path.join(qdir, "node_log.txt"), "w") as f:
        f.writelines(f"{recv} {text}\n" for recv, text in logs)
    ok_i = next((i for i, (r, t) in enumerate(logs) if t.startswith("Successfully localized")), None)
    if not odom or ok_i is None:
        return out, post_rows
    ok_log = logs[ok_i]
    call_recv = max((r for r, t in logs[:ok_i] if t.startswith("Starting async localization")), default=odom[0][1])
    ts1 = odom[0][0]
    before = [o for o in odom if o[1] <= ok_log[0]]
    loc = before[-1] if before else odom[0]     # last frame tracked before the success log
    out.update(loc_ts_ns=loc[0], loc_delay_s=round((loc[0] - ts1) / NS, 3))
    # localizer metric: logged position (cam2_link in map) vs reference at the closest frame among the frames tracked
    # between the call (2 frames earlier, to be safe) and the log (the localizer works on one of these frames; which
    # one is not published): optimistic by construction, at most ~0.5 s of motion
    m = re.search(r"\{\s*([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+)\s*\}", ok_log[1])
    if m:
        P = np.array([float(v) for v in m.groups()])
        i_call = next((i for i, o in enumerate(odom) if o[1] >= call_recv), 0)
        cand = [(float(np.linalg.norm(P - ref.at(o[0])[:3, 3])), i) for i, o in enumerate(odom)
                if max(0, i_call - 2) <= i and o[1] <= ok_log[0] and ref.at(o[0]) is not None]
        if cand:
            d, i_best = min(cand)
            out.update(loc_log_err_t_m=round(d, 4), log_frame_offset=odom.index(loc) - i_best,
                       localizer_tp=d <= args.thr_t)
    same = lambda A, B: pose_error(A, B)["err_t_m"] <= 0.01 and pose_error(A, B)["err_r_deg"] <= 0.5  # noqa: E731
    M0 = tf.get(loc[0])
    seq = [o for o in odom if o[0] > loc[0] and o[0] in tf]
    settled = None
    for i, o in enumerate(seq):
        if M0 is not None and same(tf[o[0]], M0):
            continue
        if i + 1 < len(seq) and same(tf[o[0]], tf[seq[i + 1][0]]):
            settled = o[0]
            break
    if settled is None:
        return out, post_rows
    out.update(tf_update_ts_ns=settled, tf_update_delay_s=round((settled - loc[0]) / NS, 3))
    post = []
    for ts, _, O in odom:
        if ts < settled or ts not in tf:
            continue
        T, T_ref = tf[ts] @ O, ref.at(ts)
        if T_ref is None:
            continue
        e = pose_error(T, T_ref)
        if not post:
            out.update(loc_err_t_m=round(e["err_t_m"], 4), loc_err_r_deg=round(e["err_r_deg"], 3),
                       published_tp=e["err_t_m"] <= args.thr_t and e["err_r_deg"] <= args.thr_r)
        for dt in (1, 2):   # published error 1 s and 2 s after the localized frame
            key = f"pub_err_t_m_{dt}s"
            if key not in out and ts >= loc[0] + dt * NS:
                out.update({key: round(e["err_t_m"], 4), f"pub_err_r_deg_{dt}s": round(e["err_r_deg"], 3)})
        post.append(e)
        post_rows.append([qid, ts, round((ts - loc[0]) / NS, 4), *T[:3, 3].round(5),
                          *np.round(rot_to_quat(T[:3, :3]), 6), round(e["err_t_m"], 4), round(e["err_r_deg"], 3)])
    if post:
        et = np.array([p["err_t_m"] for p in post])
        out.update(post_frames=len(post), post_rmse_t_m=round(float(np.sqrt((et ** 2).mean())), 4),
                   post_max_err_t_m=round(float(et.max()), 4), post_final_err_t_m=round(float(et[-1]), 4),
                   post_final_err_r_deg=round(post[-1]["err_r_deg"], 3))
    return out, post_rows


def run_query(args, qid, t1, t_prev, odom_map, ref, bag0, tf_static, g_dir, out_rows):
    qdir = os.path.join(args.out_dir, f"q{qid:03d}")
    os.makedirs(qdir, exist_ok=True)
    ts1_guess = bag0 + int(round(t1 * NS))
    ts_prior, T_last = odom_map.last_before(ts1_guess - int(round(t_prev * NS)))

    launch = subprocess.Popen(["ros2", "launch", args.launch], stdout=open(f"{qdir}/launch.log", "w"),
                              stderr=subprocess.STDOUT, start_new_session=True)
    node = QueryNode(args, T_last, os.path.abspath(args.map_run + "/map"), ts1_guess - NS // 10)
    play = record = None
    stalled = False
    t_wall = time.time()
    try:
        if not node.client.wait_for_service(timeout_sec=60):
            raise RuntimeError("visual_slam/localize_in_map not available (node not started?)")
        # /tf_static of the bag, latched (the cut starts after the static messages of the bag)
        static_pub = node.create_publisher(TFMessage, "/tf_static", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        static_pub.publish(tf_static)
        record = subprocess.Popen(["ros2", "bag", "record", "-s", "mcap", "-o", f"{qdir}/bag", "--topics",
                                   *RECORD_TOPICS], stdout=open(f"{qdir}/record.log", "w"),
                                  stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(2)
        play = subprocess.Popen(["ros2", "bag", "play", args.bag, "--topics", *PLAY_TOPICS,
                                 "--start-offset", f"{max(0.0, t1 - 0.1):.3f}",
                                 "--playback-duration", f"{args.duration + 0.1:.3f}"],
                                stdout=open(f"{qdir}/play.log", "w"), stderr=subprocess.STDOUT,
                                start_new_session=True)
        t_play = time.time()
        while play.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.01)
            idle = time.time() - (node.last_odom_wall or t_play)
            if idle > args.stall_timeout:   # no odometry from the node: stop the query
                stalled = True
                print(f"query {qid}: no odometry for {idle:.0f} s, stopped", flush=True)
                break
        end = time.time() + args.settle
        while time.time() < end:                       # last messages and pending result
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        stop_group(play)
        stop_group(record)
        stop_group(launch)

    # ---- results of the query
    ts1 = node.odom[0][0] if node.odom else None
    row = {"query": qid, "t1_s": t1, "t_prev_s": t_prev, "t2_s": round(t1 + args.duration, 3), "t1_ts_ns": ts1,
           "prior_ts_ns": ts_prior, "frames_tracked": len(node.odom), "attempts": len(node.attempts),
           "localized": node.loc_ns is not None, "query_wall_s": round(time.time() - t_wall, 1), "stalled": stalled}
    T_ref1 = ref.at(ts1) if ts1 else None
    if T_ref1 is not None:
        e = pose_error(T_last, T_ref1, g_dir)
        row.update(prior_gap_s=round((ts1 - ts_prior) / NS, 3), prior_err_t_m=round(e["err_t_m"], 4),
                   prior_err_r_deg=round(e["err_r_deg"], 3))
    for a in node.attempts:
        T_ref = ref.at(a["call_ns"])
        ar = {"query": qid, "attempt": a["attempt"], "call_ts_ns": a["call_ns"], "frame_ts_ns": a["frame_ns"],
              "t_from_t1_s": round((a["call_ns"] - ts1) / NS, 3) if ts1 else "", "ok": a["ok"],
              "latency_ms": round(a["latency_ms"], 1), "message": a["message"], "gravity_used": a["gravity_used"]}
        if T_ref is not None:
            e = pose_error(a["prior"], T_ref)
            ar.update(prior_err_t_m=round(e["err_t_m"], 4), prior_err_r_deg=round(e["err_r_deg"], 3))
        out_rows["attempts"].append(ar)
    if node.attempts:
        row["first_attempt_latency_ms"] = round(node.attempts[0]["latency_ms"], 1)
    if node.loc_ns is not None:
        ok = node.attempts[-1]
        row.update(attempt_ok=ok["attempt"], ok_attempt_latency_ms=round(ok["latency_ms"], 1))
        upd, post_rows = evaluate_bag(qdir, ref, args, qid)
        row.update(upd)
        out_rows["post"] += post_rows
    node.destroy_node()
    out_rows["queries"].append(row)
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--map-run", required=True, help="folder of map_cam2.sh: map/, odometry.csv, slam_poses.csv")
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--launch", default=os.path.join(HERE, "cuvslam_stereo_imu_cam2_localize.launch.py"))
    ap.add_argument("--t1", nargs="+", type=float, default=None)
    ap.add_argument("--t1-step", type=float, default=5.0)
    ap.add_argument("--margin", type=float, default=2.0)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--t-prev", nargs="+", type=float, default=[0.0, 1.0, 3.0, 5.0])
    ap.add_argument("--attempts", type=int, default=16)
    ap.add_argument("--attempt-every", type=float, default=0.25)
    ap.add_argument("--settle", type=float, default=3.0, help="seconds of spinning after the end of the cut")
    ap.add_argument("--thr-t", type=float, default=0.3)
    ap.add_argument("--thr-r", type=float, default=10.0)
    ap.add_argument("--g-map", nargs=3, type=float, default=None,
                    help="gravity direction in the map frame: enables the accelerometer roll/pitch prior correction")
    ap.add_argument("--imu-window", type=float, default=0.5, help="accelerometer averaging window (s)")
    ap.add_argument("--stall-timeout", type=float, default=10.0, help="stop a query after N s without odometry")
    ap.add_argument("--reevaluate", action="store_true",
                    help="recompute the localization metrics of an existing --out-dir from the recorded query bags")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    odom_map = Trajectory(os.path.join(args.map_run, "odometry.csv"))     # prior
    ref = Trajectory(os.path.join(args.map_run, "slam_poses.csv"))        # reference
    offset = header_offset_ns(args.bag)
    bag0 = bag_start_ns(args.bag) - offset      # bag start in the header-stamp time base
    if abs(offset) > NS:
        print(f"header stamps {offset / NS:.1f} s before the receive times: t1 kept in bag time", flush=True)
    tf_static = read_tf_static(args.bag)
    g_dir = None  # no gravity estimate from Isaac ROS: vertical error not computed
    if args.t1:
        t1s = list(args.t1)
    else:  # same sampling as map_bag.py: t1 - max(t_prev) inside the map, t2 inside the capture
        first = (ref.ts[0] - bag0) / NS
        last = (ref.ts[-1] - bag0) / NS
        t, t1s = max(args.margin, first + max(args.t_prev) + args.margin), []
        while t <= last - args.duration + 1e-9:
            t1s.append(round(t, 3))
            t += args.t1_step

    out = {"queries": [], "attempts": [], "post": []}
    if args.reevaluate:
        rows = list(csv.DictReader(open(os.path.join(args.out_dir, "queries.csv"))))
        out["attempts"] = list(csv.DictReader(open(os.path.join(args.out_dir, "attempts.csv"))))
        loc_keys = ["loc_ts_ns", "loc_delay_s", "loc_err_t_m", "loc_err_r_deg", "loc_err_vert_m", "loc_log_err_t_m",
                    "log_frame_offset", "localizer_tp", "published_tp", "within_threshold", "tf_update_delay_s",
                    "pub_err_t_m_1s", "pub_err_r_deg_1s", "pub_err_t_m_2s", "pub_err_r_deg_2s", "post_frames",
                    "post_rmse_t_m", "post_max_err_t_m", "post_final_err_t_m", "post_final_err_r_deg"]
        for r in rows:
            for k in loc_keys:
                r.pop(k, None)
            if r["localized"] == "True":
                upd, post_rows = evaluate_bag(os.path.join(args.out_dir, f"q{int(r['query']):03d}"), ref, args,
                                              int(r["query"]))
                r.update(upd)
                out["post"] += post_rows
            out["queries"].append(r)
        t1s = sorted({float(r["t1_s"]) for r in rows})
    rclpy.init()
    qid = 0
    for t1 in ([] if args.reevaluate else t1s):
        for t_prev in args.t_prev:
            qid += 1
            r = run_query(args, qid, t1, t_prev, odom_map, ref, bag0, tf_static, g_dir, out)
            print(f"query {qid}: t1={t1} t_prev={t_prev} -> "
                  + (f"localized at attempt {r['attempt_ok']}, log {r.get('loc_log_err_t_m')} m, published "
                     f"{r.get('loc_err_t_m')} m / {r.get('loc_err_r_deg')} deg" if r["localized"]
                     else f"NOT localized ({r['attempts']} attempts)"),
                  flush=True)
    rclpy.shutdown()

    q_header = ["query", "t1_s", "t_prev_s", "t2_s", "t1_ts_ns", "prior_ts_ns", "prior_gap_s", "prior_err_t_m",
                "prior_err_r_deg", "localized", "attempt_ok", "attempts", "loc_ts_ns", "loc_delay_s", "loc_err_t_m",
                "loc_err_r_deg", "loc_log_err_t_m", "log_frame_offset", "localizer_tp", "published_tp", "tf_update_delay_s",
                "pub_err_t_m_1s", "pub_err_r_deg_1s", "pub_err_t_m_2s", "pub_err_r_deg_2s", "first_attempt_latency_ms",
                "ok_attempt_latency_ms", "post_frames", "post_rmse_t_m", "post_max_err_t_m", "post_final_err_t_m",
                "post_final_err_r_deg", "frames_tracked", "query_wall_s", "stalled"]
    a_header = ["query", "attempt", "call_ts_ns", "frame_ts_ns", "t_from_t1_s", "prior_err_t_m", "prior_err_r_deg",
                "gravity_used", "ok", "latency_ms", "message"]
    for name, header, rows in (("queries.csv", q_header, [[r.get(k, "") for k in q_header] for r in out["queries"]]),
                               ("attempts.csv", a_header, [[r.get(k, "") for k in a_header] for r in out["attempts"]]),
                               ("post_localization.csv", ["query", "timestamp_ns", "t_from_loc_s", "x", "y", "z",
                                                          "qx", "qy", "qz", "qw", "err_t_m", "err_r_deg"], out["post"])):
        with open(os.path.join(args.out_dir, name), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(header)
            w.writerows(rows)
    json.dump({"script": "localize_query.py", "bag": args.bag, "map_run": args.map_run, "launch": args.launch,
               "t1": t1s, "t_prev": args.t_prev, "duration_s": args.duration, "attempts": args.attempts,
               "attempt_every_s": args.attempt_every, "threshold": {"t_m": args.thr_t, "r_deg": args.thr_r},
               "g_map": args.g_map, "imu_window_s": args.imu_window, "stall_timeout_s": args.stall_timeout},
              open(os.path.join(args.out_dir, "localize_info.json"), "w"), indent=2)


if __name__ == "__main__":
    main()
