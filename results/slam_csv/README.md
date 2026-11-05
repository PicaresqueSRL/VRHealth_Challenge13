# cuVSLAM results

Trajectories and performance metrics of cuVSLAM (Isaac ROS 5.0, cuVSLAM 15.0) on the
Jetson Orin Nano, for every D455 recording (bag) in `bench_list.txt`.

## Configurations

| Folder | Cameras | cuVSLAM mode | Launch (trajectories / metrics) |
|---|---|---|---|
| `cam2_stereo_imu/` | cam2 IR stereo + cam2 IMU | `tracking_mode: 1`, visual-inertial | `cuvslam_stereo_imu_cam2.launch.py` / `cuvslam_stereo_imu_cam2_bench.launch.py` |
| `cam1_cam2_stereo/` | cam2 + cam1 IR stereo, no IMU | `tracking_mode: 0`, multicamera, `sync_ms: 35` | `cuvslam_stereo_cam1_cam2.launch.py` / `cuvslam_stereo_cam1_cam2_bench.launch.py` |

cuVSLAM does not support the IMU with more than one stereo pair, so the multicamera
configuration runs without IMU.

## Layout

```
<config>/
  summary.csv            one row per bag, main metrics
  summary.md             summary.csv as a Markdown table
  summary_detailed.csv   summary.csv plus frame counts and checks
  <bag>/
    odometry.csv         visual(-inertial) odometry, one pose per frame
    slam_poses.csv       final SLAM trajectory
    frames.csv           per-frame processing time and CPU/GPU load
```

## How the files are produced

The files of one folder come from **two separate runs** of the same bag:

| Files | Run | Scripts |
|---|---|---|
| `odometry.csv`, `slam_poses.csv` | full run: cuVSLAM output recorded to MCAP, visualization on | `slam_cam2.sh` / `slam_cam1_cam2.sh`, then `extract_paths.py` |
| `frames.csv`, `summary*` | benchmark run: nothing recorded, visualization off, so the load is not inflated | `tools/run_benchmark_multi_cam.sh` (with `tools/bench_monitor.py`), then `tools/summarize.py` |

Both runs play the bag at real-time speed. The frames are the same, so timestamps
match, but tracking can differ slightly between the two runs.

## Pose files: `odometry.csv`, `slam_poses.csv`

| File | Source | Content |
|---|---|---|
| `odometry.csv` | every message of `/visual_slam/tracking/odometry` | odometry frame by frame; accumulates drift, no loop closure correction |
| `slam_poses.csv` | last message of `/visual_slam/tracking/slam_path` | final SLAM trajectory, optimized with loop closures; the one to compare with the floor plans |

When cuVSLAM makes no loop closure the two files are the same.

| Column | Meaning |
|---|---|
| `timestamp` | frame timestamp (s, epoch), the same as the images in the bag |
| `x`, `y`, `z` | position (m) |
| `qx`, `qy`, `qz`, `qw` | orientation as a quaternion |
| `roll_deg`, `pitch_deg`, `yaw_deg` | the same orientation as angles (degrees) |

Poses are in metres and need no scaling. They refer to `cam2_link` (ROS convention:
x forward, y left, z up) and are **relative to the starting pose**: the first pose is
(0, 0, 0) with no rotation. The start frame is not aligned with gravity (the camera is
held tilted), so before overlaying a trajectory on a floor plan it must be levelled,
then aligned in position and heading.

## `frames.csv`

One row per frame processed by cuVSLAM (one `/visual_slam/status` message per frame).

| Column | Meaning |
|---|---|
| `frame_timestamp_s` | frame timestamp (s, epoch) |
| `window_ms` | measurement window: time since the previous frame (monitor clock) |
| `tracking_state` | `ok` = tracking succeeded, `error` = tracking lost (`vo_state` of the status message) |
| `frame_processing_time_ms` | time cuVSLAM took to process the frame (`node_callback_execution_time`) |
| `jetson_cpu_%` | CPU of the **whole Jetson** in the window, all processes |
| `cuvslam_cpu_%` | CPU used **by the cuVSLAM process** in the window |
| `jetson_gpu_%` | GPU of the Jetson in the window |

### How the load is measured

At every frame the monitor reads two counters in nanoseconds, the difference from the
previous frame, divided by the window and by the number of cores, gives the load.

| Value | Source |
|---|---|
| Jetson CPU | `/proc/schedstat`: time each core spent running tasks |
| cuVSLAM CPU | CPU clock of the cuVSLAM process (all threads) |
| GPU | `/sys/devices/platform/bus@0/17000000.gpu/load`, sampled every 10 ms and averaged over the window |

CPU percentages refer to **all 6 cores** (100% = all cores busy), so `cuvslam_cpu_%`
and `jetson_cpu_%` are directly comparable, and the first is part of the second.

**Per-frame `jetson_cpu_%` can exceed 100%.** `/proc/schedstat` adds a thread's run
time to the core only when the thread leaves it: time accumulated over several frames
can land in a single window (observed up to 204%), and the previous windows read too
low. The mean over time is correct; the per-frame maximum is not, so the summary
reports the maximum over 1 s intervals instead (see below). The cuVSLAM process clock
does not have this problem.

## `summary.csv` and `summary.md`

One row per bag. `summary.md` contains the same table in Markdown.

| Column | Meaning | How it is computed |
|---|---|---|
| `bag` | recording name | |
| `bag_duration_s` | duration of the recording | first to last cam2 IR message, using the time the bag recorded them (header stamps can jump) |
| `camera_capture_fps` | fps at which the camera captured IR frames | (camera frames - 1) / `bag_duration_s` |
| `slam_processed_fps` | fps actually processed by cuVSLAM | `camera_capture_fps` × processed frames / camera frames |
| `frames_lost_in_recording_%` | camera frames without a complete cam2 stereo pair in the bag | 1 - stereo pairs / camera frames |
| `frames_tracking_lost_stereo_%` | processed frames where cuVSLAM lost tracking | frames with `tracking_state = error` / processed frames |
| `frame_processing_time_mean_ms` | mean time to process a frame | frames with tracking `ok` only (failed frames end earlier and would lower it) |
| `frame_processing_time_max_ms` | maximum time to process a frame | frames with tracking `ok` only |
| `jetson_cpu_mean_%` | mean CPU of the whole Jetson | mean over time, each frame weighted by `window_ms` |
| `jetson_cpu_max_1s_%` | maximum CPU of the whole Jetson | highest mean over consecutive 1 s intervals |
| `cuvslam_cpu_mean_%` | mean CPU of cuVSLAM | mean over time, weighted by `window_ms` |
| `cuvslam_cpu_max_%` | maximum CPU of cuVSLAM | highest per-frame value |
| `jetson_gpu_mean_%` | mean GPU of the Jetson | mean over time, weighted by `window_ms` |
| `jetson_gpu_max_1s_%` | maximum GPU of the Jetson | highest mean over consecutive 1 s intervals |

Definitions:

- **Camera frame**: a frame exists if the bag contains at least one of its four cam2
  messages (`camera_info` and image, left and right) with timestamps within 5 ms. A
  message lost in recording therefore does not remove the frame from the count.
- **Stereo pair**: cam2 left and right images with timestamps within 5 ms.
- **1 s intervals**: the time axis is the sum of the frame windows, each frame goes to
  the interval where its window ends. Intervals covered by frames for less than 0.5 s
  (e.g. the last one) are ignored.
- The first frame is excluded from processing time and load: its window includes the
  start-up of cuVSLAM.

In `cam1_cam2_stereo` the recording metrics (camera frames, stereo pairs,
`frames_lost_in_recording_%`) are computed on **cam2 only**. Frames dropped because
cam1 is missing or out of sync (`sync_ms`) appear in `frames_lost_in_processing_%` of
`summary_detailed.csv`.

## `summary_detailed.csv`

All the columns of `summary.csv`, plus:

| Column | Meaning |
|---|---|
| `camera_frames` | frames produced by the camera (see *Camera frame*) |
| `recorded_stereo_pairs` | complete cam2 stereo pairs in the bag |
| `slam_processed_frames` | frames processed by cuVSLAM (rows of `frames.csv`) |
| `frames_lost_in_processing_%` | stereo pairs in the bag that cuVSLAM did not process: 1 - processed frames / stereo pairs |
| `max_gap_between_processed_frames_ms` | longest interval between two processed frames |
| `header_timestamp_jumps` | jumps of more than 1 s in the image header timestamps (the reason the duration uses the bag recording time) |