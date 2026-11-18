# VRHealth Challenge 13 - Isaac ROS relocalization tools

Reference documentation of the tools that measure **relocalization with Isaac ROS 5.0** (ROS 2 Lyrical,
`isaac_ros_cuvslam` 5.0.0, cuVSLAM 15.0.0+33da606) on the recorded D455 captures. Configuration: cam2 IR stereo + IMU
(`tracking_mode 1`, rectified images, `base_frame cam2_link`), the same as the `slam_csv` runs.

The tools simulate a **tracking loss**: a new Isaac ROS session is started on a cut of a capture and asked to localize
itself in a map saved earlier from the same capture, with a prior taken from the last pose known before the loss.

There are two steps:

1. one **mapping run** per capture (`map_cam2.sh`), which plays the bag in real time in Isaac ROS and saves the cuVSLAM
   map, the odometry (source of the prior) and the final SLAM trajectory (reference);
2. **relocalization queries** (`localize_query.py`), each in a new Isaac ROS session, on cuts of the same capture.

The results are then shown in Rerun (`isaac_localize_rerun.py`) and summarized over all the captures
(`isaac_localize_summary.py`).

The map, the prior (`odometry.csv`) and the reference (`slam_poses.csv`) all come from **the same mapping run**:
Isaac ROS in real time is not repeatable (two desktop runs of loop_closure differ by 2.7 m RMSE without alignment), so
mixing runs would add an error that has nothing to do with the localization.

The ROS node of `localize_query.py` is described in detail in `localize_query_node.md` / `localize_query_node.pdf`
(same folder). Results and analysis are in `~/dotlumen/localization/report_isaac_localization.md`.

## Contents

1. Overview
2. Containers and dependencies
3. Typical workflow
4. Script reference
   - map_cam2.sh
   - Launch files (map, map without IMU, localize)
   - localize_query.py
   - isaac_localize_rerun.py
   - isaac_localize_summary.py
5. File formats
6. Limits and notes

## Overview

| File | Folder | Type | What it does |
|---|---|---|---|
| `map_cam2.sh` | `isaac_ros_5/launch` | Bash | Mapping run: plays one or more captures at real-time speed in Isaac ROS, saves the map (`visual_slam/save_map`), records the output topics and extracts the trajectories |
| `cuvslam_stereo_imu_cam2_map.launch.py` | `isaac_ros_5/launch` | ROS 2 launch | Node parameters of the mapping run (stereo + IMU) |
| `cuvslam_stereo_cam2_map_noimu.launch.py` | `isaac_ros_5/launch` | ROS 2 launch | Mapping run without IMU (`tracking_mode 0`), for diagnostics |
| `cuvslam_stereo_imu_cam2_localize.launch.py` | `isaac_ros_5/launch` | ROS 2 launch | Node parameters of the relocalization queries (adds the localizer search grid) |
| `localize_query.py` | `isaac_ros_5/launch` | Python (rclpy) | Relocalization queries with `visual_slam/localize_in_map`, one new Isaac ROS session per query; metrics from the recorded query bags |
| `isaac_localize_rerun.py` | `scripts` | Python | Rerun recording of the queries of one capture (map, prior, attempts, logged and published poses, overview of all the results) |
| `isaac_localize_summary.py` | `scripts` | Python | `summary.md` / `summary.csv` of the queries of all the captures |

![Workflow of the Isaac ROS relocalization tools](figures/workflow.svg)

Existing files used but **not modified**: `extract_paths.py` (called by `map_cam2.sh`), `slam_cam2.sh` (model of
`map_cam2.sh`), `cuvslam_stereo_imu_cam2.launch.py` (model of the new launch files) and, in `scripts/`, `bag2edex.py`
(imported by `isaac_localize_rerun.py`) and `docker/run.sh`.

## Containers and dependencies

| Tool | Where it runs | Needs |
|---|---|---|
| `map_cam2.sh` | host (one throwaway container per capture) or inside an Isaac ROS container (`IN_CONTAINER=1`) | Docker with the NVIDIA runtime, image `cached_isaac_run_dev_image_local` (ROS 2 Lyrical, `isaac_ros_cuvslam` 5.0.0); `extract_paths.py` in the same folder |
| launch files | Isaac ROS container | `isaac_ros_cuvslam`, `rclcpp_components` |
| `localize_query.py` | Isaac ROS container, ROS sourced | `rclpy`, `rosbag2_py`, `tf2_ros`, `isaac_ros_visual_slam_interfaces`, `numpy`, `yaml` (all in the image); the localize launch file in the same folder |
| `isaac_localize_rerun.py` | host venv or the `vrhealth-cuvslam` image (`scripts/docker/run.sh`) | `rerun-sdk` 0.33, `numpy`, `bag2edex.py` (same folder, with its dependencies `zstandard`/`lz4` for compressed bags) |
| `isaac_localize_summary.py` | anywhere | Python standard library only |

Folder conventions:

- the data folder `~/dotlumen` is mounted at `/workspaces/isaac_ros-dev` in the Isaac ROS container; paths given to
  `map_cam2.sh` and to `localize_query.py` (when run with `-w /workspaces/isaac_ros-dev`) are relative to it;
- maps: `~/dotlumen/localization/isaac_maps/<capture>/`;
- queries: `~/dotlumen/localization/isaac_localize/<capture>/`;
- Rerun recordings: `~/dotlumen/localization/rerun/<capture>.rrd`.

Use the MCAP bags of `Captures/mcap`: the db3 copies in `lyrical/workspaces/isaac_ros-dev/bags` have a
`metadata.yaml` that names a file that does not exist (`loop_closure_0.db3` instead of `loop_closure_0-001.db3`).

## Typical workflow

```bash
# 1. maps (host): one container per capture, ~1 min per capture
cd ~/dotlumen/VRHealth_Challenge13/isaac_ros_5/launch
./map_cam2.sh loop_closure=Captures/mcap/9_Loop_closure/loop_closure

# 2. queries of one capture (host): 36 queries x ~17 s on loop_closure
docker run --rm --gpus all --ipc=host --user $(id -u):$(id -g) -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog \
  -v ~/dotlumen:/workspaces/isaac_ros-dev -w /workspaces/isaac_ros-dev cached_isaac_run_dev_image_local \
  bash -c 'source /opt/ros/lyrical/setup.bash; python3 VRHealth_Challenge13/isaac_ros_5/launch/localize_query.py \
   --bag Captures/mcap/9_Loop_closure/loop_closure --map-run localization/isaac_maps/loop_closure \
   --out-dir localization/isaac_localize/loop_closure'        # outdoor_b_2000: add --duration 5

# 3. Rerun recording of the queries (scripts folder, vrhealth-cuvslam image)
cd ~/dotlumen/VRHealth_Challenge13/scripts
NO_X11=1 docker/run.sh python3 isaac_localize_rerun.py --localize ~/dotlumen/localization/isaac_localize/loop_closure \
  --map-run ~/dotlumen/localization/isaac_maps/loop_closure --bag ~/dotlumen/Captures/mcap/9_Loop_closure/loop_closure \
  --save ~/dotlumen/localization/rerun/loop_closure.rrd --image-every 5
rerun ~/dotlumen/localization/rerun/loop_closure.rrd

# 4. summary of all the captures
python3 isaac_localize_summary.py
```

Wall times on the desktop (RTX 4070 Ti): maps about 1 minute per capture; queries about 16.5 s each (192 queries of
11 captures in about 55 minutes); Rerun recordings about 1 minute in total; summary under a second.

# Script reference

## map_cam2.sh

Runs Isaac ROS cuVSLAM (cam2 IR stereo + IMU) on one or more bags at real-time speed, records the output topics and
**saves the cuVSLAM map** of each run with the `visual_slam/save_map` service. It is modeled on `slam_cam2.sh`, which is
not modified.

**How it works**

The script has two modes, selected by `IN_CONTAINER`:

- **host mode** (default): for each `<capture>=<bag>` argument it starts a throwaway container
  (`docker run --rm --gpus all --ipc=host`, host user, `HOME=/tmp`, `ROS_LOG_DIR=/tmp/roslog`, data folder mounted at
  `/workspaces/isaac_ros-dev`) that runs the same script with `IN_CONTAINER=1` on that single capture. Each container
  gets a random `ROS_DOMAIN_ID` between 100 and 199, so that runs do not see each other's topics. The script must be
  inside the data folder (it is called through its path in the container).
- **container mode** (`IN_CONTAINER=1`): sources `/opt/ros/*/setup.bash` and processes the captures one after the other.

For each capture, in container mode:

1. If `<OUT_ROOT>/<capture>/done` exists and `FORCE` is not 1, the capture is skipped. Otherwise the output folder is
   deleted and recreated.
2. `ros2 launch $LAUNCH` (default `cuvslam_stereo_imu_cam2_map.launch.py`) in its own process group; waits up to 60 s
   for `visual_slam_node` in `ros2 node list`, then 3 s more.
3. `ros2 bag record -s mcap` of `/visual_slam/tracking/{odometry,vo_path,slam_path}`, `/visual_slam/status`, `/tf`,
   `/tf_static`; 3 s wait and check that the recorder is alive.
4. `ros2 bag play` of the whole capture at rate 1.0, with `/cam2/infra{1,2}/image_rect_raw`,
   `/cam2/infra{1,2}/camera_info`, `/cam2/imu` and `/tf_static`.
5. After `SETTLE_S` seconds: `ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath
   "{file_path: '<out>/map'}"`. The call is considered successful if its answer contains `success=True`.
6. Stops the recorder and the node (SIGINT, SIGTERM after 5 s, SIGKILL after 30 s, on the whole process group).
7. `extract_paths.py --bag <out>/bag <out>` → `odometry.csv`, `vo_path.csv`, `slam_poses.csv`, `status.csv` (same
   format as `slam_csv`).
8. Writes `run_info.json` (versions read from the image: `ros2 pkg xml isaac_ros_cuvslam --tag version` and the
   cuVSLAM version string inside `libcuvslam.so`).
9. Writes the `done` marker only if `save_map` succeeded, `map/data.mdb` exists and `slam_poses.csv` was extracted;
   otherwise prints an error and goes on with the next capture.

**Usage**

```bash
cd ~/dotlumen/VRHealth_Challenge13/isaac_ros_5/launch
./map_cam2.sh loop_closure=Captures/mcap/9_Loop_closure/loop_closure
./map_cam2.sh staircases_up_109=Captures/mcap/3_Staircases/staircases_up_109 low_detail=Captures/mcap/10_Low_feature/low_detail
# redo a capture already done
FORCE=1 ./map_cam2.sh loop_closure=Captures/mcap/9_Loop_closure/loop_closure
# another launch file (path as seen inside the container), e.g. without IMU, under another output name
LAUNCH=/workspaces/isaac_ros-dev/VRHealth_Challenge13/isaac_ros_5/launch/cuvslam_stereo_cam2_map_noimu.launch.py \
    ./map_cam2.sh staircases_fix_1_noimu=Captures/mcap/3_Staircases/staircases_fix_1
# inside a container that already has ROS 2 + isaac_ros_cuvslam
IN_CONTAINER=1 ./map_cam2.sh loop_closure=Captures/mcap/9_Loop_closure/loop_closure
```

**Arguments and environment variables**

| Argument / variable | Value | Default | Description |
|---|---|---|---|
| `<capture>=<bag>` (positional, one or more) | text | **required** | output name and bag path, relative to the data folder |
| `IMAGE` | text | `cached_isaac_run_dev_image_local` | Docker image (host mode) |
| `DATA` | path | `$HOME/dotlumen` | data folder, mounted at `/workspaces/isaac_ros-dev`; the script must be inside it |
| `OUT_ROOT` | path | `localization/isaac_maps` | output folder, relative to the data folder |
| `SETTLE_S` | number | `5` | seconds after the end of the playback before saving the map |
| `FORCE` | `0` \| `1` | `0` | `1` = redo the captures already done (`done` file) |
| `LAUNCH` | path | `cuvslam_stereo_imu_cam2_map.launch.py` (next to the script) | launch file, as seen inside the container |
| `IN_CONTAINER` | `0` \| `1` | `0` | `1` = run directly in the current container (ROS 2 and `isaac_ros_cuvslam` needed) |

The exit code in host mode is the code of the last container that failed (0 if all succeeded). Errors of single
captures are printed and do not stop the loop.

**Output** (`<OUT_ROOT>/<capture>/`)

| File | Content |
|---|---|
| `map/data.mdb` | cuVSLAM map saved by Isaac ROS (LMDB), input of `localize_query.py` |
| `bag/` | MCAP recording of the output topics |
| `odometry.csv`, `vo_path.csv`, `slam_poses.csv`, `status.csv` | trajectories of the run, extracted with `extract_paths.py`: `timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg`, poses of `cam2_link`, header stamps in seconds. `odometry.csv` is the source of the prior; `slam_poses.csv` (last `slam_path`, after loop closures) is the reference of the queries |
| `run_info.json` | script, capture, bag, launch, image, ROS distro, `isaac_ros_cuvslam` and cuVSLAM versions, played and recorded topics, play return code and duration, settle time, `save_map` return code / success / duration, map folder and size (MB), extraction summary, total time |
| `launch.log`, `record.log`, `play.log`, `save_map.log`, `extract.log` | logs of the steps |
| `done` | marker of a complete run |

On the desktop: play = length of the capture (17–59 s), `save_map` about 1 s, total about 1 minute per capture; maps of
16–103 MB (loop_closure: 799 poses, 102.6 MB, 81 s in total).

**Limits**

- One real-time run: the map and the trajectories change from run to run (see the introduction). Keep map, prior and
  reference of the same run together.
- The map contains whatever the run produced: on `staircases_fix_1` every stereo + IMU run of Isaac ROS 5.0 diverges at
  20–22 s, so map and reference of that capture are wrong (flagged as not reliable in the summary; the stereo-only map
  `staircases_fix_1_noimu` was made with `LAUNCH=..._map_noimu.launch.py` for this diagnosis).
- `save_map` is called only once, at the end; if the node crashes during the playback there is no map (the capture is
  not marked `done`).

## Launch files

Three new launch files, all copies of `cuvslam_stereo_imu_cam2.launch.py` (the launch of the `slam_csv` runs, not
modified). They start `visual_slam_node` (`isaac_ros_cuvslam`, plugin `nvidia::isaac_ros::visual_slam::VisualSlamNode`)
in a `component_container_mt`, with the same remappings (`/cam2/infra1` = left = `image_0`, `/cam2/infra2` = right =
`image_1`, `/cam2/imu`) and the same parameters, except for the differences below.

| Parameter | slam_csv launch | `..._imu_cam2_map` | `..._cam2_map_noimu` | `..._imu_cam2_localize` |
|---|---|---|---|---|
| `tracking_mode` | 1 (stereo + IMU) | 1 | **0** (stereo only) | 1 |
| `enable_slam_visualization`, `enable_landmarks_view`, `enable_observations_view` | True | **False** | **False** | **False** |
| `slam_max_map_size` | default (300 poses) | **100000** | **100000** | **100000** |
| `verbosity` | default | default | default | **1** |
| `localizer_horizontal_radius` | default (1.5 m) | default | default | **4.0** m |
| `localizer_vertical_radius` | default (0.5 m) | default | default | **1.0** m |
| `localizer_horizontal_step` | default (0.5 m) | default | default | **0.5** m |
| `localizer_vertical_step` | default (0.25 m) | default | default | **0.25** m |
| `localizer_angular_step` | default (0.1745 rad) | default | default | **0.1745** rad (10 deg) |

Common parameters: `rectified_images: True`, `base_frame: cam2_link`, `imu_frame: cam2_gyro_optical_frame`, IMU noise
`gyro_noise_density 0.000244`, `gyro_random_walk 0.000019393`, `accel_noise_density 0.001862`,
`accel_random_walk 0.003`, `calibration_frequency 200`, `image_jitter_threshold_ms 150`, ground constraints off,
`enable_localization_n_mapping: True` (required by `save_map` and `localize_in_map`), `enable_image_denoising: False`,
`num_cameras 2`, `min_num_images 2`.

| File | Used by | Purpose |
|---|---|---|
| `cuvslam_stereo_imu_cam2_map.launch.py` | `map_cam2.sh` (default) | mapping run: the whole capture stays in the saved map (`slam_max_map_size` large; with the default of 300 poses the map would keep only the last 300 poses), visualization off to keep the load low during the real-time run |
| `cuvslam_stereo_cam2_map_noimu.launch.py` | `map_cam2.sh` with `LAUNCH=...` | same as the map launch without IMU fusion; diagnostic of the diverging stereo + IMU runs of `staircases_fix_1` |
| `cuvslam_stereo_imu_cam2_localize.launch.py` | `localize_query.py` (default `--launch`) | relocalization queries: the localizer search grid of `map_bag.py localize` (radius 4 m horizontal, 1 m vertical, steps 0.5 / 0.25 m, 10 deg) |

Notes:

- The localizer grid parameters are read by the node at start-up and used for every `localize_in_map` request. The grid
  is centered on the prior: horizontal on the rig x/z plane of cuVSLAM, vertical along rig y, with a full 360 deg
  rotation around the vertical axis; candidates are tried starting from the closest one to the prior. Maximum number of
  candidates with these values: 17² × 9 × 36 = 93636.
- The docstring of the localize launch file says "verbosity 3", but the file sets `verbosity: 1`. Verbosity 3 only adds
  "Lost IMU msgs" warnings, because `libcuvslam.so` in the image is built without the SLAM debug output.
- The map launch files do not set `save_map_folder_path`: the map is saved explicitly by `map_cam2.sh` with the service.

## localize_query.py

Runs relocalization queries with Isaac ROS: for every pair (t1, t_prev) it starts a **new** Isaac ROS session, plays the
cut [t1, t1 + duration] of the capture at real-time speed and calls the `visual_slam/localize_in_map` service with a
prior computed from the last pose known before a simulated loss of `t_prev` seconds. It runs inside the Isaac ROS
container. The design of the node (interfaces, state machine, timing, evaluation) is described in detail in
`localize_query_node.md`.

**How it works** (for each query)

1. Starts `ros2 launch <--launch>`: a new `visual_slam_node`, with no map loaded, in its own process group.
2. Creates the rclpy node `QueryNode`, waits up to 60 s for the `localize_in_map` service, republishes the `/tf_static`
   of the bag (latched, `TRANSIENT_LOCAL`: the cut starts after the static messages of the bag).
3. Starts `ros2 bag record -s mcap` of `/visual_slam/tracking/odometry`, `/visual_slam/tracking/slam_path`,
   `/visual_slam/status`, `/tf`, `/rosout` into `q<NNN>/bag`; after 2 s starts `ros2 bag play` of the cam2 IR images,
   camera infos and IMU with `--start-offset t1 − 0.1` and `--playback-duration duration + 0.1`.
4. Prior of attempt k: `prior_k = T_last · O_first⁻¹ · O_k`, with `T_last` the last pose of the mapping
   `odometry.csv` with timestamp ≤ t1 − t_prev, `O_first` the first odometry pose of the new session, `O_k` the odometry
   pose of the frame of the attempt.
5. The first attempt is made at the first odometry message; the service answers immediately ("started"), the result is
   read from the log of `visual_slam_node` on `/rosout` (`Successfully localized at {x, y, z}` or
   `Failed to localize in map ...` / `Cannot localize ...`). On failure, a new attempt at the first frame at least
   `--attempt-every` s (bag time) after the previous call, up to `--attempts`.
6. The playback continues until t2 in every case; then `--settle` s of spinning, and the three processes are stopped.
7. The metrics are computed from the recorded query bag (`evaluate_bag`): localized frame, settled `map->odom`,
   logged-position error (localizer metric), published-pose error (published metric), errors at +1 s / +2 s, errors
   from the settled frame to t2. All errors are against `slam_poses.csv` of the mapping run, in the map frame,
   `cam2_link` poses, without alignment.

**Sampling of the queries**

When `--t1` is not given, t1 goes from `max(margin, start of the reference + max(t_prev) + margin)` to
`end of the reference − duration`, in steps of `--t1-step` (same rule as `map_bag.py localize`), in seconds from the
start of the bag. Each t1 is combined with every `--t-prev`: on loop_closure, 9 values of t1 (6.97 … 46.97 s) × 4 t_prev
= 36 queries. Queries are numbered from 1 in that order (t1 outer loop, t_prev inner loop).

**Usage** (inside the Isaac ROS container, ROS sourced; paths relative to `/workspaces/isaac_ros-dev`)

```bash
# all the default queries of a capture
python3 VRHealth_Challenge13/isaac_ros_5/launch/localize_query.py \
    --bag Captures/mcap/9_Loop_closure/loop_closure --map-run localization/isaac_maps/loop_closure \
    --out-dir localization/isaac_localize/loop_closure
# short test: 2 cuts x 2 t_prev = 4 queries
python3 VRHealth_Challenge13/isaac_ros_5/launch/localize_query.py --bag Captures/mcap/9_Loop_closure/loop_closure \
    --map-run localization/isaac_maps/loop_closure --out-dir localization/isaac_localize/loop_closure_test \
    --t1 20 35 --t-prev 0 3
# recompute the metrics of an existing run from its recorded bags (no new query)
python3 VRHealth_Challenge13/isaac_ros_5/launch/localize_query.py --bag Captures/mcap/9_Loop_closure/loop_closure \
    --map-run localization/isaac_maps/loop_closure --out-dir localization/isaac_localize/loop_closure --reevaluate
```

From the host, wrap the command in `docker run ... cached_isaac_run_dev_image_local bash -c 'source
/opt/ros/lyrical/setup.bash; python3 ...'` as in the typical workflow above.

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `--bag` | path | **required** | capture bag (MCAP or db3 folder with `metadata.yaml`); must be the capture of the map |
| `--map-run` | path | **required** | folder of `map_cam2.sh`: `map/`, `odometry.csv` (prior), `slam_poses.csv` (reference) |
| `--out-dir` | path | **required** | output folder (created) |
| `--launch` | path | `cuvslam_stereo_imu_cam2_localize.launch.py` next to the script | launch file of the new session |
| `--t1` | one or more numbers | sampled with `--t1-step` | start times of the cuts (s from the start of the bag) |
| `--t1-step` | number | `5.0` | sampling step of t1 (s) |
| `--margin` | number | `2.0` | t1 − max(t_prev) at least this many seconds after the start of the reference (sampling only) |
| `--duration` | number | `10.0` | length of each cut t2 − t1 (s); 5 s was used for outdoor_b_2000 (17 s capture) |
| `--t-prev` | one or more numbers | `0 1 3 5` | simulated loss: `T_last` = last mapping odometry pose at t1 − t_prev |
| `--attempts` | integer | `16` | maximum number of `localize_in_map` calls per query (N) |
| `--attempt-every` | number | `0.25` | minimum bag time between two calls (s); a new call waits for the result of the previous one |
| `--settle` | number | `3.0` | seconds of spinning after the end of the cut (last messages, pending result) |
| `--thr-t` | number | `0.3` | true positive threshold, translation (m), both metrics |
| `--thr-r` | number | `10.0` | true positive threshold, rotation (deg), published metric only |
| `--g-map` | `X` `Y` `Z` | off | gravity direction in the map frame: enables the accelerometer roll/pitch correction of the prior (not used in the results) |
| `--imu-window` | number | `0.5` | accelerometer averaging window for `--g-map` (s) |
| `--stall-timeout` | number | `10.0` | stop the query after N s of wall time without odometry |
| `--reevaluate` | flag | off | do not run queries: recompute the localization metrics of an existing `--out-dir` from its `q<NNN>/bag` |

**Output** (`--out-dir`)

| File | Content |
|---|---|
| `queries.csv` | one row per query: t1/t_prev/t2, first frame and prior timestamps, prior gap and error, `localized`, `attempt_ok`, `attempts`, localized frame and delay, published error at the settled frame, logged-position error, `localizer_tp`, `published_tp`, `map->odom` settling delay, published errors at +1 s / +2 s, latencies, post-localization statistics, frames tracked, wall time, `stalled` (columns in the File formats section) |
| `attempts.csv` | one row per attempt: call frame, frame at the result, time from t1, prior error, `gravity_used`, `ok`, latency (ms), node message |
| `post_localization.csv` | one row per frame from the settled frame to t2: published pose in the map frame and its error vs the reference |
| `localize_info.json` | bag, map run, launch, t1 values, t_prev, duration, attempts, attempt spacing, thresholds, `g_map`, IMU window, stall timeout |
| `q<NNN>/bag/` | MCAP recording of the query: odometry, `slam_path`, status, `/tf`, `/rosout` |
| `q<NNN>/trace.csv` | one row per tracked frame: odometry of the new session and published pose (`map->odom · O`, empty when no `map->odom` with that stamp) |
| `q<NNN>/node_log.txt` | `/rosout` messages of `visual_slam_node` (receive time in ns + text) |
| `q<NNN>/launch.log`, `record.log`, `play.log` | logs of the three processes |

The standard output prints one line per query (`query 4: t1=6.971 t_prev=5.0 -> localized at attempt 1, log 0.0166 m,
published 0.0515 m / 0.91 deg`).

**Limits**

- Each query is a single real-time run: the published result of the same query can change between runs (on
  loop_closure, t1 = 26.97 s / t_prev = 1 s gave 2.44, 1.01 and 0.05 m in three runs).
- Query capture = map capture: the images that built the map are used to localize, so the results are an optimistic
  upper bound of a new visit.
- The localizer metric uses the frame closest to the logged position among those tracked between the call and the log
  (the node does not publish which frame it used): optimistic by at most the motion in that window (~0.5 s).
- The reference is the SLAM trajectory of the mapping run, not ground truth; in the elevators it is probably
  unreliable; on staircases_fix_1 it is wrong (diverging mapping run).
- A failed attempt takes 1.4–1.6 s to be reported, so in a 10 s cut at most about 6 failed attempts fit, whatever
  `--attempts` is.
- Times are desktop times (RTX 4070 Ti), not Jetson times.

## isaac_localize_rerun.py

Writes a Rerun recording (`.rrd`) of all the queries of one capture, from the outputs of `localize_query.py` and
`map_cam2.sh` and from the IR images of the capture bag. It does not need ROS: the bag is read with `bag2edex.py`.
All poses are `cam2_link` poses in the map frame of the mapping run (ROS axes: x forward, y left, z up).

**How it works**

1. Reads `slam_poses.csv` (reference) and `odometry.csv` (prior source) of the mapping run, `queries.csv`,
   `attempts.csv`, and `q<NNN>/trace.csv` / `q<NNN>/node_log.txt` of each query.
2. Numbers the frames of all the queries one after the other (timeline `step`) and decodes the infra1 images of every
   `--image-every`-th frame from the bag (matched by header stamp).
3. Logs the static map (reference and mapping odometry) and a static **overview** of the results of all the queries, one
   group per t_prev.
4. For each query, frame by frame: the prior track before localization (`T_last · O_first⁻¹ · O`), the prior of each
   attempt, the node messages, the logged position when the success log arrives, and from the settled frame on the
   published pose (`map->odom · O` from `trace.csv`) with its error against the reference; the rig is drawn at the
   published pose after localization and at the prior before.
5. One summary line per query in the event log (ERROR level for published false positives, WARN for not localized).

**Usage** (vrhealth-cuvslam image or venv with `rerun-sdk` 0.33)

```bash
cd ~/dotlumen/VRHealth_Challenge13/scripts
NO_X11=1 docker/run.sh python3 isaac_localize_rerun.py --localize ~/dotlumen/localization/isaac_localize/loop_closure \
    --map-run ~/dotlumen/localization/isaac_maps/loop_closure --bag ~/dotlumen/Captures/mcap/9_Loop_closure/loop_closure \
    --save ~/dotlumen/localization/rerun/loop_closure.rrd --image-every 5
# with a warning at the start of the event log (e.g. unreliable map)
python3 isaac_localize_rerun.py --localize ~/dotlumen/localization/isaac_localize/staircases_fix_1 \
    --map-run ~/dotlumen/localization/isaac_maps/staircases_fix_1 \
    --bag ~/dotlumen/Captures/mcap/3_Staircases/staircases_fix_1 \
    --save ~/dotlumen/localization/rerun/staircases_fix_1.rrd --warning "mapping run diverges at 20-22 s: map and reference wrong"
rerun ~/dotlumen/localization/rerun/loop_closure.rrd
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `--localize` | path | **required** | output folder of `localize_query.py` |
| `--map-run` | path | **required** | folder of `map_cam2.sh` (`odometry.csv`, `slam_poses.csv`) |
| `--bag` | path | **required** | capture bag (IR images, `/cam2/infra1/image_rect_raw`) |
| `--save` | path | **required** | output `.rrd` (folder created) |
| `--image-every` | integer | `5` | log one IR image every N frames (size of the recording) |
| `--thr-t` | number | `0.3` | accepted for symmetry with the other scripts; the TP/FP colors come from `localizer_tp` / `published_tp` in `queries.csv` |
| `--warning` | text | - | warning shown at the start of the event panel |

**Content of the recording**

| Entity | Color | Content |
|---|---|---|
| `world/map/reference` | purple | `slam_poses.csv` of the mapping run (reference) |
| `world/map/odometry` | grey | `odometry.csv` of the mapping run (source of the prior) |
| `world/query/reference_cut` | yellow | reference over the cut [t1, t2] |
| `world/query/last_pose` | orange arrow | `T_last`, the last pose before the loss |
| `world/query/track_before` | orange | new session before localization: `T_last · O_first⁻¹ · O` |
| `world/query/attempts/<k>/prior` | yellow arrow | prior of attempt k |
| `world/query/log_position` | green / red point | position logged by the node; green = localizer TP |
| `world/query/published` | green / red line | published pose after localization (`map->odom · O`); green = published TP |
| `world/query/localized` | green / red arrow | published pose at the settled frame, with the errors in the label |
| `world/query/reference_now` | purple point | reference pose of the current frame |
| `world/overview/t_prev_<X>s/...` | static | all the queries with that t_prev: published localized poses (arrows, green/red), logged positions (points, green/red), reference poses at the same frames (light purple arrows), error lines; labels `q<N>` = query number |
| `world/rig` | white arrow | current pose: published after localization, prior before |
| `camera` | - | IR image (infra1) every `--image-every` frames |
| `errors/err_t_m`, `errors/err_r_deg` | orange / blue | published translation / rotation error vs reference |
| `events` | - | query parameters, node messages, query result |

Layout: left the IR image, the error plot and the event log; right two 3D tabs, `map` (queries, overview hidden) and
`overview` (map and overview only). Arrows point along `cam2_link` x (forward). Timelines: `step` (default, queries one
after the other, played at 14 steps/s), `query`, `time` (capture time, s from the first reference pose; cuts of
different queries overlap).

Sizes: 4.6–195 MB per capture with `--image-every 5` (623 MB for the 11 captures).

**Limits**

- The rotation of the logged localization is not available (the log has only the position): the localizer result is a
  point, not an arrow.
- The settled frame is found again by matching `loc_ts_ns + tf_update_delay_s` (rounded to 1 ms in `queries.csv`) to the
  closest frame of `trace.csv`.

## isaac_localize_summary.py

Summary of the relocalization queries of all the captures: per capture and t_prev, totals, success within N attempts,
false positives, not-localized queries, maps. Python standard library only.

**How it works**

1. Captures = the subfolders of `--localize-root` that have a `queries.csv` **and** a folder with the same name in
   `--maps-root` (test folders such as `loop_closure_test` are therefore skipped).
2. For each capture and t_prev, and for all the captures together (`ALL`) and without the captures listed in
   `--unreliable` (`ALL_RELIABLE`), computes:
   - for N = 1, 2, 4, 8, 16: localized queries with `attempt_ok ≤ N`, true and false positives with both metrics,
     not localized;
   - medians: prior error, logged-position error, published error at the settled frame, at +1 s and +2 s, final error,
     post-localization RMSE, latency of the successful attempt, delay from t1, `map->odom` settling delay, attempts.
3. Reads `run_info.json` and `slam_poses.csv` of each map (poses, map size, play/run/save times) and the size of
   `<rerun-root>/<capture>.rrd`.

Two success metrics per query (thresholds of `localize_query.py`, 0.3 m and 10 deg):

- **localizer**: position logged by `visual_slam_node` within 0.3 m of the reference (translation only);
- **published**: pose published by Isaac ROS at the first settled frame within 0.3 m and 10 deg.

**Usage**

```bash
cd ~/dotlumen/VRHealth_Challenge13/scripts
python3 isaac_localize_summary.py
python3 isaac_localize_summary.py --localize-root ~/dotlumen/localization/isaac_localize \
    --maps-root ~/dotlumen/localization/isaac_maps --rerun-root ~/dotlumen/localization/rerun
# no capture flagged as unreliable
python3 isaac_localize_summary.py --unreliable
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `--localize-root` | path | `~/dotlumen/localization/isaac_localize` | folder with one `localize_query.py` output per capture; `summary.md` / `summary.csv` are written here |
| `--maps-root` | path | `~/dotlumen/localization/isaac_maps` | folder with one `map_cam2.sh` output per capture |
| `--rerun-root` | path | `~/dotlumen/localization/rerun` | folder with the `<capture>.rrd` files (only their size is read) |
| `--unreliable` | zero or more `CAPTURE=REASON` | `staircases_fix_1=<reason>` | captures kept in the results but flagged as not reliable and excluded from `ALL_RELIABLE` |

**Output** (`--localize-root`)

| File | Content |
|---|---|
| `summary.csv` | one row per (capture, t_prev), plus `ALL` and `ALL_RELIABLE` rows per t_prev and for all t_prev: `queries`, `N<n>_localized`, `N<n>_loc_tp/fp`, `N<n>_pub_tp/fp`, `N<n>_not_localized`, the medians `med_*`, `reliable` |
| `summary.md` | totals per t_prev (all captures and reliable only), per capture and t_prev (N = 16), maps, list of published false positives, list of not-localized queries |

# File formats

## Pose CSV of the mapping run (map_cam2.sh, extract_paths.py)

`timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg`: timestamp = header stamp in seconds, pose of `cam2_link`
in `odom` (odometry) or `map` (SLAM), meters; the first pose is the identity. In a new session `map` and `odom`
coincide at start, so the frame of `slam_poses.csv` is the map frame used by the queries.

## queries.csv (localize_query.py)

| Column | Meaning |
|---|---|
| `query`, `t1_s`, `t_prev_s`, `t2_s` | query number, cut start (s from the bag start), simulated loss, cut end |
| `t1_ts_ns` | header stamp of the first frame tracked by the new session |
| `prior_ts_ns`, `prior_gap_s` | stamp of `T_last` in `odometry.csv` and its distance from the first frame |
| `prior_err_t_m`, `prior_err_r_deg` | error of `T_last` vs the reference at the first frame |
| `localized`, `attempt_ok`, `attempts` | success, number of the successful attempt, attempts made |
| `loc_ts_ns`, `loc_delay_s` | localized frame (last frame received before the success log) and its delay from the first frame |
| `loc_err_t_m`, `loc_err_r_deg`, `published_tp` | published pose at the settled frame vs reference; TP if ≤ `--thr-t` and ≤ `--thr-r` |
| `loc_log_err_t_m`, `log_frame_offset`, `localizer_tp` | logged position vs reference at the closest frame between call and log, offset (frames) of that frame before the localized frame, TP if ≤ `--thr-t` |
| `tf_update_delay_s` | settled frame − localized frame |
| `pub_err_t_m_1s`, `pub_err_r_deg_1s`, `pub_err_t_m_2s`, `pub_err_r_deg_2s` | published error at the first frames ≥ 1 s and ≥ 2 s after the localized frame |
| `first_attempt_latency_ms`, `ok_attempt_latency_ms` | wall time from the call to the result log, first and successful attempt |
| `post_frames`, `post_rmse_t_m`, `post_max_err_t_m`, `post_final_err_t_m`, `post_final_err_r_deg` | published errors from the settled frame to t2 |
| `frames_tracked`, `query_wall_s`, `stalled` | odometry messages received, wall time of the query, stopped for lack of odometry |

## attempts.csv, post_localization.csv, trace.csv

| File | Columns |
|---|---|
| `attempts.csv` | `query, attempt, call_ts_ns, frame_ts_ns, t_from_t1_s, prior_err_t_m, prior_err_r_deg, gravity_used, ok, latency_ms, message` |
| `post_localization.csv` | `query, timestamp_ns, t_from_loc_s, x, y, z, qx, qy, qz, qw, err_t_m, err_r_deg` |
| `q<NNN>/trace.csv` | `timestamp_ns, recv_ns, odom_x … odom_qw, pub_x … pub_qw` |
| `q<NNN>/node_log.txt` | `<receive time ns> <message>` per line |

# Limits and notes

- **Repeatability**: Isaac ROS in real time is not repeatable, both for the mapping runs and for the queries. Single
  queries should be repeated before drawing conclusions.
- **Optimistic setting**: query capture = map capture, and the reference is the SLAM trajectory of the same run.
- **Two results**: the logged localizer position and the published pose (after the merge of the session into the map)
  often disagree; both are measured (see `localize_query_node.md` and the report).
- **Time base**: t1 and t2 are bag (receive) times, poses use header stamps; for the "converted" captures, whose header
  stamps are days away from the receive times, `localize_query.py` measures the offset from the images.
- **Isolation**: every `map_cam2.sh` container uses its own `ROS_DOMAIN_ID`; `localize_query.py` runs its queries one
  after the other in a single container (default domain), so two `localize_query.py` runs must not share a container
  or a domain.
