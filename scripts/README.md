# scripts

ROS-free tools to convert the RealSense D455 captures to EDEX, run cuVSLAM (PyCuVSLAM) on EDEX
files or directly on ROS 2 bags, visualize the runs in Rerun and save their output in the same
format as the Isaac ROS runs (`isaac_ros_5/launch/slam_cam2.sh` + `extract_paths.py`).

| Script | Purpose |
|---|---|
| `bag2edex.py` | Converts a ROS 2 bag (`.db3` / `.mcap`) to one EDEX per camera (RGB, infra1, infra2, IMU); with `--depth` also the depth aligned to the RGB |
| `check_depth_sync.py` | Measures the stamps of native and aligned depth against RGB and IR in every capture |
| `check_depth_alignment.py` | Compares the aligned depth computed by `bag2edex.py` with the one recorded by the realsense node |
| `bag2edex_modes.py` | Converts a capture into one EDEX per cuVSLAM input configuration (RGB-D, stereo + IMU, Multicamera, Multisensor), with shared images |
| `convert_all.py` | Runs `bag2edex.py` (or `--converter <script>`) on every capture of a folder, preserving the folder structure |
| `merge_rig_edex.py` | Merges the cam1 and cam2 EDEX files into a single rig EDEX (extrinsic from the CAD of the mount) |
| `track_edex_rerun.py` | Runs cuVSLAM on an EDEX, with Rerun visualization |
| `track_bag_rerun.py` | Runs cuVSLAM directly on a ROS 2 bag, with Rerun visualization |
| `track_edex_modes.py` | Runs cuVSLAM without viewer on an EDEX of `bag2edex_modes.py`, in any odometry mode (also RGBD and Multisensor) |
| `bag_output_rerun.py` | Shows in Rerun a cuVSLAM run already done, from the bag of its output topics or from a folder with its saved trajectories (optionally with the images of the input bag) |
| `view_frames.py` | Module used by the three Rerun viewers: common 3D frame (base frame at the first pose, `--world`) and overlay of saved runs (`--compare`) |
| `run_output.py` | Module used by the two tracking scripts to save the cuVSLAM output (`--out-dir`) |
| `compare_runs.py` | Compares saved runs (Isaac ROS and PyCuVSLAM) with each other and with the ground truth |
| `db3_to_mcap.sh` | Reduces a bag to the topics needed by cuVSLAM and converts it to compressed MCAP |
| `all_db3_to_mcap.sh` | Runs `db3_to_mcap.sh` on every capture of a folder |

Setup: `python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt`.
The tracking scripts also need the cuVSLAM Python bindings and `rerun-sdk==0.33.1`.

Docker: `docker build -f docker/Dockerfile -t vrhealth-cuvslam .` builds an image with cuVSLAM, its Python
bindings and every dependency; `docker/run.sh <script> [args]` runs a script in it (GPU, display for Rerun,
`~/dotlumen` mounted at the same path). Full documentation: `docs/scripts_documentation.md` (and `.pdf`).

## Saved output (`--out-dir`)

`track_edex_rerun.py` and `track_bag_rerun.py` write, with `--out-dir <folder>`:

| File | Content |
|---|---|
| `odometry.csv` | one pose per tracked frame (visual-inertial odometry) |
| `slam_poses.csv` | final SLAM trajectory (`get_all_slam_poses` at the end of the run, optimized with the loop closures); with `--slam` |
| `slam_online.csv` | SLAM pose returned frame by frame during the run; with `--slam` |
| `status.csv` | per frame: `vo_state` (1 = tracked, 2 = lost), tracking time, landmarks, observations per camera |
| `slam_metrics.csv` | per frame: loop closure and pose graph optimization status; with `--slam` |
| `loop_closures.csv` | last loop closure poses reported by SLAM (cuVSLAM keeps the last 10), in the SLAM world frame |
| `run_info.json` | input, parameters, cuVSLAM version, number of frames and summary metrics |

The pose files use the columns of the Isaac ROS CSV files
(`timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg`): timestamps in seconds, poses in meters
of the base frame (default `<cam>_link`, as `base_frame` in the Isaac ROS launch files), relative to
the first pose. They can be compared directly with `results/slam_csv/...` and aligned to the ground
truth with `results/ground_truth/RunBatchPipeline.py`.

`--base-frame rig` keeps the poses of the rig (first camera, OpenCV axes) instead.
For an EDEX, the transform to the base frame is read from `/tf_static` of the source bag
(path taken from `bag_report.json`, or `--bag`).

## Stereo IR + IMU on loop_closure: EDEX vs bag

The same input for both runs: all IR pairs of cam2, IMU of cam2, rectified images (as `rectified_images: True`
in `cuvslam_stereo_imu_cam2.launch.py`).

```bash
B=~/dotlumen/Captures/original/9_Loop_closure/loop_closure        # the same bag played by slam_cam2.sh
E=~/dotlumen/Captures/edex/9_Loop_closure/loop_closure
R=~/dotlumen/VRHealth_Challenge13/results/pycuvslam

# 1. One EDEX per input configuration; stereo_imu_cam2 has cameras 0 = infra1, 1 = infra2
python3 bag2edex_modes.py $B -o $E

# 2. cuVSLAM on the EDEX
python3 track_edex_rerun.py $E/stereo_imu_cam2/stereo.edex --mode inertial --cameras 0 1 --rectified --slam \
    --out-dir $R/edex/cam2_stereo_imu/loop_closure --save $R/edex/cam2_stereo_imu/loop_closure/run.rrd

# 3. cuVSLAM directly on the bag (same library, no EDEX)
python3 track_bag_rerun.py $B --mode inertial --streams cam2/ir1 cam2/ir2 --rectified --slam \
    --out-dir $R/bag/cam2_stereo_imu/loop_closure --save $R/bag/cam2_stereo_imu/loop_closure/run.rrd
```

The Isaac ROS run of the same bag is in `results/slam_csv/cam2_stereo_imu/loop_closure`.
Steps 2 and 3 use the same cuVSLAM library and the same frames, so any difference between them comes
from the input format; differences from the Isaac ROS run also include the cuVSLAM version of the
Isaac ROS node, real-time playback and its parameters (`--async-sba` reproduces the asynchronous
bundle adjustment of a real-time run).

## Comparing the runs

```bash
python3 compare_runs.py \
    ros=$R/../slam_csv/cam2_stereo_imu/loop_closure \
    bag=$R/bag/cam2_stereo_imu/loop_closure \
    edex=$R/edex/cam2_stereo_imu/loop_closure \
    --gt ~/dotlumen/VRHealth_Challenge13/results/ground_truth/loop_closure/paths3d.csv \
    --out ~/dotlumen/VRHealth_Challenge13/results/comparisons/cam2_stereo_imu/loop_closure
```

It writes `report.md` (tables), `metrics.json`, `pairs.csv`, the plots in `plots/` and the
`IcpAlign.py` results in `icp/`. Path length is reported both as the sum over all consecutive
poses (includes frame-to-frame jitter) and over one pose per second. The z coordinate is in the base
frame of the first pose, which is not gravity-aligned.
