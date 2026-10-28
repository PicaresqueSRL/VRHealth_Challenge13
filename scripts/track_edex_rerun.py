#!/usr/bin/env python3
"""
track_edex_rerun.py - Runs cuVSLAM (PyCuVSLAM) on an EDEX and visualizes the result in Rerun.

Designed for the EDEX files produced by bag_to_cuvslam.py (camera 0 = RGB, 1 = infra1, 2 = infra2,
optional IMU), but reads any EDEX with frame_metadata.jsonl.

What it shows in Rerun:
  * odometry trajectory (and SLAM trajectory with --slam) and current rig pose;
  * for each camera used: frustum, image and tracked points (colored by id);
  * 3D landmarks of the last frame and, at the end of the run, the final point cloud;
  * with IMU: estimated gravity vector, accelerometer and gyroscope over time.

Examples:
    # Mono on the RGB of cam2
    python3 track_edex_rerun.py ~/claude/dotlumen/captures/staircases_102/edex/cam2/stereo.edex --mode mono

    # RGB + IR pair (Multicamera)
    python3 track_edex_rerun.py .../cam2/stereo.edex --mode multicamera

    # Stereo IR + IMU (Inertial); frames without IR are skipped
    python3 track_edex_rerun.py .../cam2/stereo.edex --mode inertial --cameras 1 2

    # save the recording instead of opening the viewer, and the trajectory in TUM format
    python3 track_edex_rerun.py .../cam2/stereo.edex --mode mono --save run.rrd --tum odom.txt

Requirements: cuvslam bindings installed, rerun-sdk==0.33.1, numpy, opencv-python.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("OpenCV is missing: pip install opencv-python")
try:
    import cuvslam
except ImportError:
    sys.exit("cuvslam bindings are missing: CUVSLAM_BUILD_DIR=$PWD/build pip install python/")
try:
    import rerun as rr
except ImportError:
    sys.exit("Rerun is missing: pip install rerun-sdk==0.33.1")

NS = 1_000_000_000
# EDEX files store extrinsics in the old cuVSLAM frame (y up, z backward);
# the Python API uses OpenCV (y down, z forward). See libs/common/coordinate_system.h.
K_FLIP = np.diag([1.0, -1.0, -1.0])


# -----------------------------------------------------------------------------
# Geometry
# -----------------------------------------------------------------------------
def to4x4(m34):
    T = np.eye(4)
    T[:3, :4] = np.asarray(m34, dtype=float)
    return T


def edex_camera_to_opencv(m34):
    T = to4x4(m34)
    out = np.eye(4)
    out[:3, :3] = K_FLIP @ T[:3, :3] @ K_FLIP
    out[:3, 3] = K_FLIP @ T[:3, 3]
    return out


def edex_imu_to_opencv(m34):
    # rig_from_imu: only the target frame (the rig) changes convention
    M = np.eye(4)
    M[:3, :3] = K_FLIP
    return M @ to4x4(m34)


def rot_to_quat_xyzw(R):
    """Rotation matrix -> quaternion [x, y, z, w]."""
    R = np.asarray(R, dtype=float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    return (q / np.linalg.norm(q)).tolist()


def to_pose(T):
    return cuvslam.Pose(rotation=rot_to_quat_xyzw(T[:3, :3]), translation=T[:3, 3].tolist())


def is_identity(T, tol=1e-6):
    return np.allclose(T, np.eye(4), atol=tol)


DISTORTION = {
    "pinhole": (cuvslam.Distortion.Model.Pinhole, 0),
    "brown5k": (cuvslam.Distortion.Model.Brown, 5),
    "fisheye": (cuvslam.Distortion.Model.Fisheye, 4),
    "fisheye4": (cuvslam.Distortion.Model.Fisheye, 4),
    "polynomial": (cuvslam.Distortion.Model.Polynomial, 8),
}


# -----------------------------------------------------------------------------
# EDEX reading
# -----------------------------------------------------------------------------
def load_edex(path, cam_ids):
    with open(path) as f:
        header, body = json.load(f)
    base = os.path.dirname(os.path.abspath(path))
    all_cams = header["cameras"]
    if cam_ids is None:
        cam_ids = list(range(len(all_cams)))
    for c in cam_ids:
        if not 0 <= c < len(all_cams):
            sys.exit(f"Camera {c} not present in the EDEX (it has {len(all_cams)})")

    # The rig is re-centered on the first selected camera (required for Mono,
    # convenient for the other modes: the trajectory is that camera's trajectory).
    T_edex = {c: edex_camera_to_opencv(all_cams[c]["transform"]) for c in cam_ids}
    T_new_from_old = np.linalg.inv(T_edex[cam_ids[0]])

    cameras, intr = [], []
    for c in cam_ids:
        i = all_cams[c]["intrinsics"]
        model_name = i.get("distortion_model", "pinhole").lower()
        if model_name not in DISTORTION:
            sys.exit(f"Unsupported distortion model: {model_name}")
        model, n = DISTORTION[model_name]
        params = [float(x) for x in i.get("distortion_params", [])]
        if len(params) != n:
            sys.exit(f"Camera {c}: {model_name} requires {n} parameters, found {len(params)}")
        T = T_new_from_old @ T_edex[c]
        cam = cuvslam.Camera(size=[int(i["size"][0]), int(i["size"][1])],
                             principal=[float(x) for x in i["principal"]],
                             focal=[float(x) for x in i["focal"]],
                             rig_from_camera=to_pose(T),
                             distortion=cuvslam.Distortion(model, params))
        cameras.append(cam)
        intr.append(dict(size=i["size"], focal=i["focal"], principal=i["principal"], T=T))

    imu_cfg, imu_path = None, None
    if "imu" in header:
        h = header["imu"]
        T_imu = T_new_from_old @ edex_imu_to_opencv(h["transform"])

        def g(*keys, default):
            for k in keys:
                if k in h:
                    return float(h[k])
            return default

        imu_cfg = cuvslam.ImuCalibration(
            rig_from_imu=to_pose(T_imu),
            gyroscope_noise_density=g("gyro_noise_density", "gyroscope_noise_density", default=0.00016968),
            gyroscope_random_walk=g("gyro_random_walk", "gyroscope_random_walk", default=0.000019393),
            accelerometer_noise_density=g("accel_noise_density", "accelerometer_noise_density", default=0.002),
            accelerometer_random_walk=g("accel_random_walk", "accelerometer_random_walk", default=0.003),
            frequency=g("frequency", default=200.0))
        if h.get("coordinate_system", "opencv").lower() != "opencv":
            print("WARNING: IMU coordinate_system is not 'opencv': measurements used as they are",
                  file=sys.stderr)
        imu_path = os.path.join(base, h["measurements"])

    if "frame_metadata" not in body:
        sys.exit("The EDEX has no frame_metadata: this script only supports EDEX files with per-frame timestamps")
    frames = []
    pos = {c: k for k, c in enumerate(cam_ids)}
    with open(os.path.join(base, body["frame_metadata"])) as f:
        for line in f:
            if not line.strip():
                continue
            m = json.loads(line)
            files, stamps = [None] * len(cam_ids), []
            for c in m["cams"]:
                if c["id"] in pos:
                    files[pos[c["id"]]] = os.path.join(base, c["filename"])
                    stamps.append(int(c["timestamp"]))
            if stamps:
                # same as the library: the frame time is that of the first camera present
                first = next(c for c in m["cams"] if c["id"] in pos and
                             pos[c["id"]] == min(pos[x["id"]] for x in m["cams"] if x["id"] in pos))
                frames.append(dict(frame_id=m["frame_id"], files=files, ts=int(first["timestamp"]),
                                   spread=max(stamps) - min(stamps)))
    frames.sort(key=lambda x: x["ts"])
    return cam_ids, cameras, intr, imu_cfg, imu_path, frames


def load_imu(path):
    out = []
    with open(path) as f:
        for line in f:
            if line.strip():
                m = json.loads(line)
                out.append((int(m["timestamp"]),
                            [m["LinearAccelerationX"], m["LinearAccelerationY"], m["LinearAccelerationZ"]],
                            [m["AngularVelocityX"], m["AngularVelocityY"], m["AngularVelocityZ"]]))
    out.sort(key=lambda x: x[0])
    return out


# -----------------------------------------------------------------------------
# Visualization
# -----------------------------------------------------------------------------
def color_from_id(i):
    i = int(i)
    return [(i * 97) % 200 + 55, (i * 57) % 200 + 55, (i * 23) % 200 + 55]


def setup_rerun(args, cam_ids, intr):
    rr.init("cuVSLAM EDEX", spawn=args.save is None)
    if args.save:
        rr.save(args.save)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_DOWN, static=True)  # OpenCV
    for k, c in enumerate(cam_ids):
        p = intr[k]
        T = p["T"]
        path = f"world/rig/cam{c}"
        rr.log(path, rr.Transform3D(translation=T[:3, 3], mat3x3=T[:3, :3]), static=True)
        rr.log(path, rr.Pinhole(focal_length=[float(x) for x in p["focal"]],
                                principal_point=[float(x) for x in p["principal"]],
                                width=int(p["size"][0]), height=int(p["size"][1]),
                                image_plane_distance=0.3), static=True)


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
MODES = {
    "multicamera": cuvslam.Odometry.OdometryMode.Multicamera,
    "inertial": cuvslam.Odometry.OdometryMode.Inertial,
    "mono": cuvslam.Odometry.OdometryMode.Mono,
}


def main():
    ap = argparse.ArgumentParser(description="cuVSLAM on an EDEX with Rerun visualization",
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("edex", help="stereo.edex file")
    ap.add_argument("--mode", choices=list(MODES), default="multicamera")
    ap.add_argument("--cameras", type=int, nargs="+", default=None,
                    help="indices of the EDEX cameras to use (default: mono -> 0, otherwise all)")
    ap.add_argument("--slam", action="store_true", help="also enable SLAM")
    ap.add_argument("--rectified", action="store_true",
                    help="rectified_stereo_camera=True (pinhole camera pairs only, e.g. --cameras 1 2)")
    ap.add_argument("--start", type=int, default=0, help="first frame to process (index)")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--log-input", action="store_true",
                    help="also log the exact array passed to cuVSLAM as a plain 2D image "
                         "(entity input/camN), to compare it with the frustum visualization")
    ap.add_argument("--image-every", type=int, default=1,
                    help="send images to Rerun every N frames (makes the recording lighter)")
    ap.add_argument("--save", default=None, help="save to an .rrd file instead of opening the viewer")
    ap.add_argument("--tum", default=None, help="save the odometry trajectory in TUM format")
    ap.add_argument("--verbosity", type=int, default=1, help="cuvslam.set_verbosity (0-3)")
    args = ap.parse_args()

    cam_ids = args.cameras if args.cameras is not None else ([0] if args.mode == "mono" else None)
    cam_ids, cameras, intr, imu_cfg, imu_path, frames = load_edex(args.edex, cam_ids)
    use_imu = args.mode == "inertial"
    if use_imu and imu_cfg is None:
        sys.exit("Inertial mode requested but the EDEX has no imu section")
    if args.mode == "mono" and len(cam_ids) != 1:
        print("WARNING: in Mono only the first selected camera is used", file=sys.stderr)

    frames = frames[args.start:]
    if args.max_frames:
        frames = frames[:args.max_frames]
    if not frames:
        sys.exit("No frames to process")

    # Mono needs camera 0 in every frame; the other modes just need one camera present
    if args.mode == "mono":
        frames = [f for f in frames if f["files"][0] is not None]
    spread_bad = sum(f["spread"] > 1_000_000 for f in frames)
    if spread_bad:
        print(f"WARNING: {spread_bad} frames with cameras offset by more than 1 ms (cuVSLAM "
              f"treats them as synchronized)", file=sys.stderr)

    cuvslam.set_verbosity(args.verbosity)
    rig = cuvslam.Rig(cameras=cameras, imus=[imu_cfg] if use_imu else [])
    odom_cfg = cuvslam.Odometry.Config(
        odometry_mode=MODES[args.mode],
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

    imu = load_imu(imu_path) if use_imu else []
    # samples before the first frame are discarded (as cuvslam_api_launcher does)
    imu = [x for x in imu if x[0] >= frames[0]["ts"]]
    imu_idx = 0

    setup_rerun(args, cam_ids, intr)
    print(f"Mode {args.mode}, cameras {cam_ids}, IMU {'yes' if use_imu else 'no'}, "
          f"SLAM {'yes' if args.slam else 'no'}, frames {len(frames)}", file=sys.stderr)

    traj, slam_traj, tum_rows = [], [], []
    lost = 0
    last_ts = None
    t0_wall = time.time()
    t0 = frames[0]["ts"]
    for n, fr in enumerate(frames):
        ts = fr["ts"]
        if last_ts is not None and ts <= last_ts:
            continue  # cuVSLAM requires strictly increasing timestamps
        # IMU up to the current frame (non-decreasing time order)
        while imu_idx < len(imu) and imu[imu_idx][0] <= ts:
            its, acc, gyr = imu[imu_idx]
            m = cuvslam.ImuMeasurement()
            m.timestamp_ns = its
            m.linear_accelerations = np.asarray(acc, dtype=np.float32)
            m.angular_velocities = np.asarray(gyr, dtype=np.float32)
            tracker.register_imu_measurement(0, m)
            imu_idx += 1
            if imu_idx % 4 == 0:  # fewer points in the plots
                rr.set_time("time", duration=(its - t0) / NS)
                rr.log("imu/accel", rr.Scalars(acc))
                rr.log("imu/gyro", rr.Scalars(gyr))

        images, color0 = [], None
        for k, path in enumerate(fr["files"]):
            if path is None:
                images.append(np.empty(0, dtype=np.uint8))  # camera missing in this frame
                continue
            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if img is None:
                sys.exit(f"Cannot read {path}")
            if img.ndim == 3:
                color0 = color0 if k else img
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            images.append(np.ascontiguousarray(img))

        if args.mode == "mono":
            images = images[:1]
        if n == 0:
            for k, c in enumerate(cam_ids[:len(images)]):
                if images[k].size:
                    print(f"  camera {c}: image passed to cuVSLAM {images[k].shape[1]}x{images[k].shape[0]}, "
                          f"size in the EDEX {intr[k]['size'][0]}x{intr[k]['size'][1]}", file=sys.stderr)
        est, slam_pose = tracker.track(ts, images)
        last_ts = ts

        rr.set_time("frame", sequence=fr["frame_id"])
        rr.set_time("time", duration=(ts - t0) / NS)
        if est.world_from_rig is None:
            lost += 1
            rr.log("status", rr.TextLog(f"tracking lost at frame {fr['frame_id']}", level="WARN"))
            continue

        pose = est.world_from_rig.pose
        traj.append(list(pose.translation))
        tum_rows.append([ts / NS] + list(pose.translation) + list(pose.rotation))
        rr.log("world/odom_trajectory", rr.LineStrips3D([traj], colors=[[60, 140, 255]]))
        rr.log("world/rig", rr.Transform3D(translation=pose.translation, quaternion=pose.rotation))

        if args.slam and slam_pose is not None:
            slam_traj.append(list(slam_pose.translation))
            rr.log("world/slam_trajectory", rr.LineStrips3D([slam_traj], colors=[[255, 255, 255]]))

        lms = tracker.odometry.get_last_landmarks()
        if lms:
            rr.log("world/rig/landmarks", rr.Points3D([list(l.coords) for l in lms],
                                                        colors=[color_from_id(l.id) for l in lms], radii=0.01))

        if n % args.image_every == 0:
            for k, c in enumerate(cam_ids[:len(images)]):
                if images[k].size == 0:
                    rr.log(f"world/rig/cam{c}/image", rr.Clear(recursive=False))
                    continue
                obs = tracker.odometry.get_last_observations(k)
                shown = cv2.cvtColor(color0, cv2.COLOR_BGR2RGB) if (k == 0 and color0 is not None) else images[k]
                rr.log(f"world/rig/cam{c}/image", rr.Image(shown).compress(jpeg_quality=75))
                if args.log_input:
                    # outside the Pinhole hierarchy: Rerun shows it pixel by pixel, without projections
                    rr.log(f"input/cam{c}", rr.Image(images[k]))
                    if obs:
                        rr.log(f"input/cam{c}/observations",
                               rr.Points2D([[o.u, o.v] for o in obs],
                                           colors=[color_from_id(o.id) for o in obs], radii=3.0))
                if obs:
                    rr.log(f"world/rig/cam{c}/image/observations",
                           rr.Points2D([[o.u, o.v] for o in obs], colors=[color_from_id(o.id) for o in obs],
                                       radii=3.0))

        if use_imu:
            gvec = tracker.odometry.get_last_gravity()
            if gvec is not None:
                rr.log("world/rig/gravity", rr.Arrows3D(vectors=[np.asarray(gvec) / 9.81 * 0.5],
                                                        colors=[[255, 80, 80]]))

        if (n + 1) % 100 == 0:
            fps = (n + 1) / (time.time() - t0_wall)
            print(f"  {n + 1}/{len(frames)} frames ({fps:.1f} fps), lost {lost}", file=sys.stderr)

    final = tracker.odometry.get_final_landmarks()
    if final:
        pts = np.array([list(v) for v in final.values()], dtype=np.float32)
        rr.log("world/final_landmarks", rr.Points3D(pts, colors=[[180, 180, 180]], radii=0.01), static=True)

    if args.tum:
        with open(args.tum, "w") as f:
            for r in tum_rows:
                f.write(" ".join(f"{v:.9f}" for v in r) + "\n")
    print(f"Done: {len(tum_rows)} poses, {lost} frames with tracking lost"
          + (f", trajectory in {args.tum}" if args.tum else ""), file=sys.stderr)


if __name__ == "__main__":
    main()
