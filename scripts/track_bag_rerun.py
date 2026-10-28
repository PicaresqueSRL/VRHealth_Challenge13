#!/usr/bin/env python3
"""
track_bag_rerun.py - Runs cuVSLAM (PyCuVSLAM) directly on a ROS 2 bag (.mcap or .db3)
and visualizes the result in Rerun, without going through an EDEX and without writing images to disk.

Does in memory what bag_to_cuvslam.py + merge_rig_edex.py + track_edex_rerun.py do:
  1. reads /tf_static, camera_info, image timestamps and IMU (pass 1);
  2. builds the rig from the streams chosen with --streams (also cam1 + cam2, using the mount
     extrinsic) and synchronizes the frames on the first stream;
  3. re-reads the images (pass 2) and feeds them to cuVSLAM as it goes, logging to Rerun
     trajectory, pose, frustums, images, tracked points, landmarks and IMU.

Rerun layout: on the left the images of each camera (with tracked points) and the plots below
(gyroscope; accelerometer and number of tracked points in tabs); on the right the 3D view.

Streams: <camera>/<role> with role rgb, ir1, ir2. The first stream is the reference:
it defines the frames, their timestamps and the rig origin (the trajectory is its own).

Examples:
    B=~/dotlumen/Captures/mcap/9_Loop_closure/loop_closure

    # stereo IR + IMU of cam2 (Inertial)
    python3 track_bag_rerun.py $B --mode inertial --streams cam2/ir1 cam2/ir2

    # 4-IR rig (cam1 + cam2, extrinsic from CAD), with SLAM, saving .rrd and TUM trajectory
    python3 track_bag_rerun.py $B --mode multicamera \\
        --streams cam1/ir1 cam1/ir2 cam2/ir1 cam2/ir2 --slam --save lc.rrd --tum lc.txt

    # Mono on the RGB of cam2
    python3 track_bag_rerun.py $B --mode mono --streams cam2/rgb

Requirements: cuvslam bindings, rerun-sdk==0.33.1, numpy, opencv-python, zstandard/lz4 for compressed MCAP.
Uses functions from bag_to_cuvslam.py, merge_rig_edex.py and track_edex_rerun.py (same folder).
"""

import argparse
import os
import statistics
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from bag_to_cuvslam import (Bag, TfTree, decode_camera_info, decode_image, decode_imu,  # noqa: E402
                            decode_tf, dedupe, greedy_match, header_stamp, image_to_cv,
                            quat_to_rot, ros_d_to_brown5k)
from merge_rig_edex import CAD_EXTRINSIC  # noqa: E402
from track_edex_rerun import MODES, color_from_id, cuvslam, cv2, rr, to_pose  # noqa: E402

NS = 1_000_000_000
TOPICS = {"rgb": ("/{cam}/color/image_raw", "/{cam}/color/camera_info"),
          "ir1": ("/{cam}/infra1/image_rect_raw", "/{cam}/infra1/camera_info"),
          "ir2": ("/{cam}/infra2/image_rect_raw", "/{cam}/infra2/camera_info")}
MULTICAM = {"precision": "Precision", "moderate": "Moderate", "performance": "Performance"}


def log(msg):
    print(msg, file=sys.stderr, flush=True)


def parse_args():
    ap = argparse.ArgumentParser(description="cuVSLAM on a ROS 2 bag with Rerun visualization",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("bag", help="bag folder (.mcap/.db3) or single file")
    ap.add_argument("--mode", choices=list(MODES), default="multicamera")
    ap.add_argument("--streams", nargs="+", default=None,
                    help="streams <cam>/<rgb|ir1|ir2>; the first one is the reference. Default: mono -> "
                         "<imu-camera>/rgb, inertial -> <imu-camera>/ir1 ir2, multicamera -> ir1 ir2 "
                         "of all cameras")
    ap.add_argument("--imu-camera", default="cam2", help="camera providing the IMU (inertial mode)")
    ap.add_argument("--extrinsic", nargs=7, type=float, default=CAD_EXTRINSIC,
                    metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
                    help="T_<cam1>_link_from_<cam2>_link (used only if /tf_static does not connect the cameras)")
    ap.add_argument("--link-frame", default="{cam}_link", help="base frame of each camera in /tf_static")
    ap.add_argument("--tolerance-ms", type=float, default=1.0,
                    help="maximum offset between the reference and the other streams (cuVSLAM requires <= 1 ms)")
    ap.add_argument("--stale-threshold", type=float, default=0.5,
                    help="discard messages whose stamp-receive offset is more than N s from the median")
    ap.add_argument("--multicam-mode", choices=list(MULTICAM), default="precision",
                    help="selection of primary cameras in Multicamera")
    ap.add_argument("--slam", action="store_true", help="also enable SLAM")
    ap.add_argument("--rectified", action="store_true",
                    help="rectified_stereo_camera=True (rectified IR pairs only)")
    ap.add_argument("--start", type=float, default=None, help="seconds from the beginning at which to start")
    ap.add_argument("--end", type=float, default=None, help="seconds from the beginning at which to stop")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--image-every", type=int, default=1, help="send images to Rerun every N frames")
    ap.add_argument("--save", default=None, help="save to an .rrd file instead of opening the viewer")
    ap.add_argument("--tum", default=None, help="save the odometry trajectory in TUM format")
    ap.add_argument("--slam-tum", default=None, help="save the SLAM trajectory in TUM format")
    ap.add_argument("--verbosity", type=int, default=1, help="cuvslam.set_verbosity (0-3)")
    return ap.parse_args()


# -----------------------------------------------------------------------------
# Pass 1: metadata, timestamps, IMU, TF
# -----------------------------------------------------------------------------
def read_metadata(bag, streams, imu_topic, stale_s):
    types = bag.topic_types()
    img_topics = {s["topic"] for s in streams}
    info_topics = {s["info_topic"] for s in streams}
    for t in sorted(img_topics | info_topics | ({imu_topic} if imu_topic else set())):
        if t not in types:
            sys.exit(f"Topic missing from the bag: {t}")
    if "/tf_static" not in types:
        sys.exit("/tf_static missing from the bag: camera extrinsics are required")

    stamps = {t: [] for t in img_topics}
    t0 = None
    for topic, recv, key, data in bag.iterate(img_topics, prefix_only=True):
        t0 = recv if t0 is None else min(t0, recv)
        st = header_stamp(data)
        stamps[topic].append((st if st is not None else recv, recv, key))

    tf, infos, imu, imu_recv, imu_frame = TfTree(), {}, [], [], None
    small = info_topics | {"/tf_static"} | ({imu_topic} if imu_topic else set())
    for topic, recv, _key, data in bag.iterate(small):
        if topic == "/tf_static":
            for parent, child, t, q in decode_tf(data):
                tf.add(parent, child, t, q)
        elif topic in info_topics:
            infos.setdefault(topic, decode_camera_info(data))
        elif topic == imu_topic:
            m = decode_imu(data)
            imu_frame = imu_frame or m["frame_id"]
            st = m["stamp"] if m["stamp"] > 0 else recv
            imu.append((st, m["acc"], m["gyro"]))
            imu_recv.append(st - recv)

    # anomalous stamps (frame left queued in the driver at startup, etc.)
    if stale_s > 0:
        thr = int(stale_s * NS)
        for t, lst in stamps.items():
            if len(lst) > 2:
                med = statistics.median(s - r for s, r, _ in lst)
                keep = [x for x in lst if abs((x[0] - x[1]) - med) <= thr]
                if len(keep) != len(lst):
                    log(f"WARNING: {t}: discarded {len(lst) - len(keep)} messages with anomalous stamp")
                stamps[t] = keep
        if len(imu) > 2:
            med = statistics.median(imu_recv)
            n0 = len(imu)
            imu = [x for x, d in zip(imu, imu_recv) if abs(d - med) <= thr]
            if len(imu) != n0:
                log(f"WARNING: {imu_topic}: discarded {n0 - len(imu)} samples with anomalous stamp")
    for t in stamps:
        stamps[t] = dedupe(stamps[t])
    imu.sort(key=lambda x: x[0])
    dedup_imu, last = [], None
    for x in imu:
        if x[0] != last:
            dedup_imu.append(x)
            last = x[0]
    return t0, stamps, tf, infos, dedup_imu, imu_frame


# -----------------------------------------------------------------------------
# Rig
# -----------------------------------------------------------------------------
def build_rig(streams, infos, tf, args):
    cams_used = sorted({s["cam"] for s in streams})
    ref = streams[0]
    ref_frame = infos[ref["info_topic"]]["frame_id"].lstrip("/")
    # connect the cameras with the extrinsic if /tf_static does not already do so
    if len(cams_used) > 1:
        links = [args.link_frame.format(cam=c) for c in cams_used]
        for lk in links:
            if lk not in tf.edges:
                sys.exit(f"Frame {lk} not found in /tf_static (use --link-frame). "
                         f"Frame: {', '.join(tf.frames())}")
        for lk in links[1:]:
            if tf.lookup(links[0], lk) is None:
                tf.add(links[0], lk, args.extrinsic[:3], args.extrinsic[3:])
                log(f"Extrinsic {links[0]} -> {lk}: t={args.extrinsic[:3]} q={args.extrinsic[3:]}")

    for s in streams:
        info = infos[s["info_topic"]]
        frame = info["frame_id"].lstrip("/")
        T = tf.lookup(ref_frame, frame)
        if T is None:
            sys.exit(f"Transform {ref_frame} <- {frame} not found in /tf_static")
        if s["role"] == "ir2":
            ir1 = infos.get(TOPICS["ir1"][1].format(cam=s["cam"]))
            if ir1 is not None and ir1["frame_id"].lstrip("/") == frame:
                # ROS stereo convention: P[3] = -fx * baseline, frame_id of the left camera
                p = info["p"]
                if abs(p[3]) < 1e-9 or p[0] == 0:
                    sys.exit(f"{s['name']}: same frame as infra1 but P[3] = 0, unknown baseline")
                shift = np.eye(4)
                shift[0, 3] = -p[3] / p[0]
                T = T @ shift
        k = info["k"]
        if s["role"] == "rgb":
            dist = cuvslam.Distortion(cuvslam.Distortion.Model.Brown, ros_d_to_brown5k(info["d"]))
        else:
            dist = cuvslam.Distortion(cuvslam.Distortion.Model.Pinhole, [])
        s.update(T=T, size=[info["width"], info["height"]], focal=[k[0], k[4]], principal=[k[2], k[5]])
        s["camera"] = cuvslam.Camera(size=s["size"], principal=s["principal"], focal=s["focal"],
                                     rig_from_camera=to_pose(T), distortion=dist)
    return ref_frame


def imu_calibration(tf, ref_frame, imu_frame, imu_cam, n_imu):
    T = None
    for fr in (imu_frame, f"{imu_cam}_imu_optical_frame", f"{imu_cam}_gyro_optical_frame"):
        if fr:
            T = tf.lookup(ref_frame, fr)
            if T is not None:
                if fr != imu_frame:
                    log(f"WARNING: IMU frame '{imu_frame}' not in /tf_static, using '{fr}'")
                break
    if T is None:
        sys.exit(f"Transform {ref_frame} <- {imu_frame} (IMU) not found in /tf_static")
    return cuvslam.ImuCalibration(rig_from_imu=to_pose(T), gyroscope_noise_density=0.000244,
                                  gyroscope_random_walk=0.000019393, accelerometer_noise_density=0.001862,
                                  accelerometer_random_walk=0.003, frequency=float(n_imu))


# -----------------------------------------------------------------------------
# Pass 2: images in frame order
# -----------------------------------------------------------------------------
def frame_images(bag, frames, topics):
    """Yield (frame, {stream index: image}) in frame order, reading the bag once.
    Images arrive in file order (~ receive order): they are buffered
    until the next frame has all of its streams."""
    need = {}
    for fi, fr in enumerate(frames):
        for si, key in fr["keys"].items():
            need[key] = (fi, si)
    buf = {}
    nxt = 0
    for _topic, _recv, key, data in bag.iterate(topics):
        if key not in need:
            continue
        fi, si = need.pop(key)
        buf.setdefault(fi, {})[si] = image_to_cv(decode_image(data))
        while nxt < len(frames) and len(buf.get(nxt, {})) == len(frames[nxt]["keys"]):
            yield frames[nxt], buf.pop(nxt)
            nxt += 1
    if nxt < len(frames):
        log(f"WARNING: {len(frames) - nxt} incomplete trailing frames not processed")


# -----------------------------------------------------------------------------
# Rerun layout
# -----------------------------------------------------------------------------
def make_blueprint(streams, ent, use_imu):
    """Layout: camera images (top) and plots (bottom) on the left,
    3D view with trajectory, rig and landmarks on the right."""
    import rerun.blueprint as rrb

    cams = [rrb.Spatial2DView(name=s["name"], origin=ent[si]) for si, s in enumerate(streams)]
    if len(cams) <= 3:
        images = rrb.Horizontal(*cams)
    else:
        images = rrb.Grid(*cams, grid_columns=min(3, (len(cams) + 1) // 2))
    tracked = rrb.TimeSeriesView(name="tracked points", origin="stats/tracked")
    if use_imu:
        plots = rrb.Horizontal(
            rrb.TimeSeriesView(name="imu/gyro", origin="imu/gyro"),
            rrb.Tabs(rrb.TimeSeriesView(name="imu/accel", origin="imu/accel"), tracked, active_tab=0))
    else:
        plots = rrb.Horizontal(tracked)
    world = rrb.Spatial3DView(name="world", origin="world")
    return rrb.Blueprint(
        rrb.Horizontal(rrb.Vertical(images, plots, row_shares=[1, 1]), world, column_shares=[1, 1]))


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
def main():
    args = parse_args()
    bag = Bag(args.bag)
    types = bag.topic_types()
    cams_in_bag = sorted({t.split("/")[1] for t in types if t.endswith("/color/image_raw")})

    names = args.streams
    if names is None:
        if args.mode == "mono":
            names = [f"{args.imu_camera}/rgb"]
        elif args.mode == "inertial":
            names = [f"{args.imu_camera}/ir1", f"{args.imu_camera}/ir2"]
        else:
            names = [f"{c}/{r}" for c in cams_in_bag for r in ("ir1", "ir2")
                     if TOPICS[r][0].format(cam=c) in types]
    streams = []
    for n in names:
        cam, _, role = n.partition("/")
        if role not in TOPICS:
            sys.exit(f"Invalid stream: {n} (format <cam>/<rgb|ir1|ir2>)")
        streams.append(dict(name=n, cam=cam, role=role, topic=TOPICS[role][0].format(cam=cam),
                            info_topic=TOPICS[role][1].format(cam=cam)))
    if args.mode == "mono" and len(streams) != 1:
        sys.exit("Mono uses a single stream (e.g. --streams cam2/rgb)")
    use_imu = args.mode == "inertial"
    imu_topic = f"/{args.imu_camera}/imu" if use_imu else None
    if use_imu and len(streams) > 2:
        log("WARNING: with more than 2 cameras cuVSLAM uses the IMU only for gravity (no fusion)")

    log(f"Bag: {', '.join(os.path.basename(f.path) for f in bag.files)}")
    log("Pass 1/2: timestamps, camera_info, /tf_static" + (", IMU" if use_imu else "") + " ...")
    t_bag0, stamps, tf, infos, imu, imu_frame = read_metadata(bag, streams, imu_topic, args.stale_threshold)
    ref_frame = build_rig(streams, infos, tf, args)

    # --- frames: one per message of the reference stream ------------------------
    ref_list = stamps[streams[0]["topic"]]
    if args.start is not None:
        ref_list = [x for x in ref_list if x[1] >= t_bag0 + args.start * NS]
    if args.end is not None:
        ref_list = [x for x in ref_list if x[1] <= t_bag0 + args.end * NS]
    ref_ts = [x[0] for x in ref_list]
    frames = [{"ts": x[0], "keys": {0: x[2]}, "stamps": {0: x[0]}} for x in ref_list]
    tol = int(args.tolerance_ms * 1e6)
    for si, s in enumerate(streams[1:], 1):
        lst = stamps[s["topic"]]
        for i, j in greedy_match(ref_ts, [x[0] for x in lst], tol).items():
            frames[i]["keys"][si] = lst[j][2]
            frames[i]["stamps"][si] = lst[j][0]
    if args.max_frames:
        frames = frames[:args.max_frames]
    if not frames:
        sys.exit("No frames to process")
    log(f"Rig ({ref_frame} = origin): {len(frames)} frames")
    for si, s in enumerate(streams):
        n = sum(si in f["keys"] for f in frames)
        d = [abs(f["stamps"][si] - f["ts"]) / 1e6 for f in frames if si in f["stamps"]]
        extra = f", median offset {statistics.median(d):.2f} ms" if si and d else ""
        log(f"  camera {si}: {s['name']:9s} {s['size'][0]}x{s['size'][1]} present in {n} frames "
            f"({100 * n / len(frames):.0f}%){extra}")

    # --- cuVSLAM -------------------------------------------------------------------
    cuvslam.set_verbosity(args.verbosity)
    imu_cfg = None
    if use_imu:
        rate = round(NS / statistics.median(b[0] - a[0] for a, b in zip(imu, imu[1:]))) if len(imu) > 1 else 200
        imu_cfg = imu_calibration(tf, ref_frame, imu_frame, args.imu_camera, rate)
    rig = cuvslam.Rig(cameras=[s["camera"] for s in streams], imus=[imu_cfg] if use_imu else [])
    odom_cfg = cuvslam.Odometry.Config(
        odometry_mode=MODES[args.mode],
        multicam_mode=getattr(cuvslam.Odometry.MulticameraMode, MULTICAM[args.multicam_mode]),
        async_sba=False,
        rectified_stereo_camera=args.rectified,
        enable_observations_export=True,
        enable_landmarks_export=True,
        enable_final_landmarks_export=True,
    )
    if args.slam:
        slam_cfg = cuvslam.Slam.Config()
        slam_cfg.sync_mode = True
        tracker = cuvslam.Tracker(rig, cuvslam.Tracker.Mode.OdometryWithSlamOffline, odom_cfg, slam_cfg)
    else:
        tracker = cuvslam.Tracker(rig, cuvslam.Tracker.Mode.OdometryOnlyOffline, odom_cfg)

    # --- Rerun ---------------------------------------------------------------------
    ent = {si: f"world/rig/{s['cam']}_{s['role']}" for si, s in enumerate(streams)}
    rr.init("cuVSLAM bag", spawn=args.save is None)
    if args.save:
        rr.save(args.save)
    rr.send_blueprint(make_blueprint(streams, ent, use_imu))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)  # OpenCV
    if use_imu:
        rr.log("imu/gyro", rr.SeriesLines(names=["gyro x", "gyro y", "gyro z"],
                                          colors=[[80, 120, 255], [60, 200, 200], [120, 220, 120]]), static=True)
        rr.log("imu/accel", rr.SeriesLines(names=["accel x", "accel y", "accel z"],
                                           colors=[[160, 90, 230], [60, 200, 200], [200, 210, 60]]), static=True)
    for si, s in enumerate(streams):
        rr.log(f"stats/tracked/{s['cam']}_{s['role']}", rr.SeriesLines(names=[s["name"]]), static=True)
    for si, s in enumerate(streams):
        T = s["T"]
        rr.log(ent[si], rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]), static=True)
        rr.log(ent[si], rr.Pinhole(focal_length=[float(x) for x in s["focal"]],
                                   principal_point=[float(x) for x in s["principal"]],
                                   width=int(s["size"][0]), height=int(s["size"][1]),
                                   image_plane_distance=0.3), static=True)

    log(f"Mode {args.mode}, streams {names}, IMU {'yes' if use_imu else 'no'}, "
        f"SLAM {'yes' if args.slam else 'no'}")
    log("Pass 2/2: tracking ...")
    t0 = frames[0]["ts"]
    imu = [x for x in imu if x[0] >= t0]
    imu_idx = 0
    traj, slam_traj, tum_rows, slam_rows = [], [], [], []
    lost, last_ts, t_wall = 0, None, time.time()
    topics = {s["topic"] for s in streams}
    for n, (fr, imgs) in enumerate(frame_images(bag, frames, topics)):
        ts = fr["ts"]
        if last_ts is not None and ts <= last_ts:
            continue
        while imu_idx < len(imu) and imu[imu_idx][0] <= ts:
            its, acc, gyr = imu[imu_idx]
            m = cuvslam.ImuMeasurement()
            m.timestamp_ns = its
            m.linear_accelerations = np.asarray(acc, dtype=np.float32)
            m.angular_velocities = np.asarray(gyr, dtype=np.float32)
            tracker.register_imu_measurement(0, m)
            imu_idx += 1
            if imu_idx % 4 == 0:
                rr.set_time("time", duration=(its - t0) / NS)
                rr.log("imu/accel", rr.Scalars(acc))
                rr.log("imu/gyro", rr.Scalars(gyr))

        images, shown = [], {}
        for si in range(len(streams)):
            img = imgs.get(si)
            if img is None:
                images.append(np.empty(0, dtype=np.uint8))
                continue
            if img.ndim == 3:
                shown[si] = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            images.append(np.ascontiguousarray(img))

        est, slam_pose = tracker.track(ts, images)
        last_ts = ts
        rr.set_time("frame", sequence=n)
        rr.set_time("time", duration=(ts - t0) / NS)
        if est.world_from_rig is None:
            lost += 1
            rr.log("status", rr.TextLog(f"tracking lost at frame {n}", level="WARN"))
            continue

        pose = est.world_from_rig.pose
        traj.append(list(pose.translation))
        tum_rows.append([ts / NS] + list(pose.translation) + list(pose.rotation))
        rr.log("world/odom_trajectory", rr.LineStrips3D([traj], colors=[[60, 140, 255]]))
        rr.log("world/rig", rr.Transform3D(translation=pose.translation, quaternion=pose.rotation))
        if args.slam and slam_pose is not None:
            slam_traj.append(list(slam_pose.translation))
            slam_rows.append([ts / NS] + list(slam_pose.translation) + list(slam_pose.rotation))
            rr.log("world/slam_trajectory", rr.LineStrips3D([slam_traj], colors=[[255, 255, 255]]))

        lms = tracker.odometry.get_last_landmarks()
        if lms:
            rr.log("world/rig/landmarks", rr.Points3D([list(l.coords) for l in lms],
                                                        colors=[color_from_id(l.id) for l in lms], radii=0.01))
        if n % args.image_every == 0:
            for si in range(len(streams)):
                if images[si].size == 0:
                    rr.log(f"{ent[si]}/image", rr.Clear(recursive=False))
                    continue
                rr.log(f"{ent[si]}/image", rr.Image(shown.get(si, images[si])).compress(jpeg_quality=75))
                obs = tracker.odometry.get_last_observations(si)
                rr.log(f"stats/tracked/{streams[si]['cam']}_{streams[si]['role']}", rr.Scalars(len(obs or [])))
                if obs:
                    rr.log(f"{ent[si]}/image/observations",
                           rr.Points2D([[o.u, o.v] for o in obs], colors=[color_from_id(o.id) for o in obs],
                                       radii=3.0))
        if use_imu:
            g = tracker.odometry.get_last_gravity()
            if g is not None:
                rr.log("world/rig/gravity", rr.Arrows3D(vectors=[np.asarray(g) / 9.81 * 0.5],
                                                        colors=[[255, 80, 80]]))
        if (n + 1) % 100 == 0:
            log(f"  {n + 1}/{len(frames)} frames ({(n + 1) / (time.time() - t_wall):.1f} fps), lost {lost}")

    final = tracker.odometry.get_final_landmarks()
    if final:
        pts = np.array([list(v) for v in final.values()], dtype=np.float32)
        rr.log("world/final_landmarks", rr.Points3D(pts, colors=[[180, 180, 180]], radii=0.01), static=True)

    for path, rows in ((args.tum, tum_rows), (args.slam_tum, slam_rows)):
        if path:
            with open(path, "w") as f:
                for r in rows:
                    f.write(" ".join(f"{v:.9f}" for v in r) + "\n")
    if traj:
        drift = np.linalg.norm(np.array(traj[-1]) - np.array(traj[0]))
        log(f"Distance between first and last pose (odometry): {drift:.3f} m")
    log(f"Done: {len(tum_rows)} poses, {lost} frames with tracking lost")


if __name__ == "__main__":
    main()
