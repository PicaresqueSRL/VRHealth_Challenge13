#!/usr/bin/env python3
"""
track_edex_modes.py - Runs cuVSLAM without a viewer on one EDEX written by bag2edex_modes.py, in any
odometry mode, and saves the output with run_output.py.

Depth (RGBD, Multisensor) is read from the `depth` entries of frame_metadata.jsonl. Each entry `id` is the
depth id of the camera whose `depth_id` matches; the depth image is passed at that camera's position.

Usage:
    python3 track_edex_modes.py <dir>/rgbd_ir_cam2/stereo.edex --mode rgbd --out-dir runs/rgbd_ir_cam2
    python3 track_edex_modes.py <dir>/stereo_imu_cam1/stereo.edex --mode inertial --rectified --slam --out-dir r
    python3 track_edex_modes.py <dir>/multisensor_all/stereo.edex --mode multisensor --out-dir r
"""

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cuvslam  # noqa: E402
from run_output import RunWriter, cuvslam_version  # noqa: E402
from track_edex_rerun import base_from_rig, edex_camera_to_opencv, load_edex, load_imu  # noqa: E402

ODOM = {"mono": cuvslam.Odometry.OdometryMode.Mono, "multicamera": cuvslam.Odometry.OdometryMode.Multicamera,
        "inertial": cuvslam.Odometry.OdometryMode.Inertial, "rgbd": cuvslam.Odometry.OdometryMode.RGBD,
        "multisensor": cuvslam.Odometry.OdometryMode.Multisensor}


def read_image(path):
    img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if img is None:
        sys.exit(f"Cannot read {path}")
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return np.ascontiguousarray(img)


def main():
    ap = argparse.ArgumentParser(description="cuVSLAM on an EDEX of bag2edex_modes.py, without viewer",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("edex", help="stereo.edex file")
    ap.add_argument("--mode", choices=list(ODOM), required=True)
    ap.add_argument("--cameras", type=int, nargs="+", default=None, help="EDEX cameras to use (default: all)")
    ap.add_argument("--no-imu", action="store_true", help="Multisensor: ignore the IMU of the EDEX")
    ap.add_argument("--rectified", action="store_true", help="rectified_stereo_camera=True")
    ap.add_argument("--slam", action="store_true")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--base-frame", default=None, help="frame of the saved poses (default: <cam>_link of camera 0)")
    ap.add_argument("--bag", default=None, help="source bag for /tf_static (default: from ../bag_report.json)")
    ap.add_argument("--verbosity", type=int, default=1)
    args = ap.parse_args()
    args.edex = os.path.abspath(args.edex)
    args.world = "base"

    cam_ids, cameras, _intr, imu_cfg, imu_path, frames = load_edex(args.edex, args.cameras)
    base = os.path.dirname(args.edex)
    with open(args.edex) as f:
        header, body = json.load(f)
    depth_cam = {header["cameras"][c]["depth_id"]: pos for pos, c in enumerate(cam_ids)
                 if "depth_id" in header["cameras"][c]}
    scale = next((float(header["cameras"][c]["depth_scale_factor"]) for c in cam_ids
                  if "depth_scale_factor" in header["cameras"][c]), 1.0)
    depth_of = {}
    with open(os.path.join(base, body["frame_metadata"])) as f:
        for line in f:
            m = json.loads(line)
            depth_of[m["frame_id"]] = m.get("depth", [])

    settings = {}
    if args.mode == "rgbd":
        if len(depth_cam) != 1:
            sys.exit(f"RGBD needs exactly one depth camera among the selected ones, found {len(depth_cam)}")
        settings["rgbd_settings"] = cuvslam.Odometry.RGBDSettings(
            depth_scale_factor=scale, depth_camera_id=next(iter(depth_cam.values())))
    elif args.mode == "multisensor":
        settings["multisensor_settings"] = cuvslam.Odometry.MultisensorSettings(
            depth_camera_ids=sorted(depth_cam.values()), depth_scale_factor=scale)

    use_imu = args.mode == "inertial" or (args.mode == "multisensor" and imu_cfg is not None and not args.no_imu)
    if use_imu and imu_cfg is None:
        sys.exit("This EDEX has no IMU")
    if args.mode == "mono":
        frames = [x for x in frames if x["files"][0] is not None]
    if args.mode == "rgbd":
        frames = [x for x in frames if depth_of.get(x["frame_id"])]
    if args.max_frames:
        frames = frames[:args.max_frames]
    if not frames:
        sys.exit("No frames to process")

    cuvslam.set_verbosity(args.verbosity)
    rig = cuvslam.Rig(cameras=cameras, imus=[imu_cfg] if use_imu else [])
    odom_cfg = cuvslam.Odometry.Config(odometry_mode=ODOM[args.mode], async_sba=False,
                                       rectified_stereo_camera=args.rectified, enable_observations_export=True,
                                       enable_landmarks_export=True, enable_final_landmarks_export=True, **settings)
    if args.slam:
        slam_cfg = cuvslam.Slam.Config()
        slam_cfg.sync_mode = True
        tracker = cuvslam.Tracker(rig, cuvslam.Tracker.Mode.OdometryWithSlamOffline, odom_cfg, slam_cfg)
    else:
        tracker = cuvslam.Tracker(rig, cuvslam.Tracker.Mode.OdometryOnlyOffline, odom_cfg)

    imu = load_imu(imu_path) if use_imu else []
    imu = [x for x in imu if x[0] >= frames[0]["ts"]]
    imu_idx = 0

    cal_path = os.path.join(base, "calibration.json")
    cal = {}
    if os.path.exists(cal_path):
        with open(cal_path) as f:
            cal = json.load(f)
    if "T_rig_from_base_opencv" in cal and args.base_frame in (None, cal.get("base_frame")):
        # the rig of the tracker is re-centered on the first selected camera
        T_rig_from_cam0 = edex_camera_to_opencv(header["cameras"][cam_ids[0]]["transform"])
        T_rb = np.linalg.inv(T_rig_from_cam0) @ np.array(cal["T_rig_from_base_opencv"])
        base_frame, src = cal["base_frame"], "calibration.json"
    else:
        T_rb, base_frame, src = base_from_rig(args, cam_ids[0])
    writer = RunWriter(args.out_dir, T_rb, base_frame, len(cam_ids), {
        "script": "track_edex_modes.py", "input": args.edex, "mode": args.mode, "cameras": cam_ids,
        "slam": args.slam, "imu": use_imu, "rectified": args.rectified, "async_sba": False,
        "base_frame_source": src, "cuvslam_version": cuvslam_version(cuvslam), "frames_in_input": len(frames)})

    lost, n_done, last_ts, t0 = 0, 0, None, time.time()
    for n, fr in enumerate(frames):
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
        images = [read_image(p) if p is not None else np.empty(0, dtype=np.uint8) for p in fr["files"]]
        if args.mode == "mono":
            images = images[:1]
        depths = None
        if args.mode in ("rgbd", "multisensor"):
            depths = [np.empty(0, dtype=np.uint16) for _ in cam_ids]
            for d in depth_of.get(fr["frame_id"], []):
                if d["id"] in depth_cam:
                    depths[depth_cam[d["id"]]] = cv2.imread(os.path.join(base, d["filename"]), cv2.IMREAD_UNCHANGED)
            if args.mode == "rgbd":
                depths = [depths[next(iter(depth_cam.values()))]] if depth_cam else depths
        t = time.perf_counter()
        est, slam_pose = tracker.track(ts, images, depths=depths) if depths is not None else tracker.track(ts, images)
        ms = (time.perf_counter() - t) * 1000
        last_ts = ts
        obs = {k: len(tracker.odometry.get_last_observations(k) or []) for k in range(len(images)) if images[k].size}
        writer.add_frame(ts, est, slam_pose if args.slam else None, obs,
                         len(tracker.odometry.get_last_landmarks() or []), ms, tracker)
        n_done += 1
        if est.world_from_rig is None:
            lost += 1
        if (n + 1) % 200 == 0:
            print(f"  {n + 1}/{len(frames)} frames ({(n + 1) / (time.time() - t0):.1f} fps), lost {lost}", file=sys.stderr)
    final = tracker.odometry.get_final_landmarks()
    summary = writer.finish(tracker, args.slam, len(final) if final else 0)
    print(json.dumps({"edex": args.edex, "mode": args.mode, "frames": n_done, "lost": lost, "summary": summary}))


if __name__ == "__main__":
    main()
