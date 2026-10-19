# cuVSLAM Benchmark: cam2 stereo IR + IMU

Performance of cuVSLAM (Isaac ROS 5.0, cuVSLAM 15.0) on the Jetson Orin Nano, run on the
D455 recordings (bags), and extraction of the estimated trajectories.

## What is measured

For each recording:

| Item | Content |
|---|---|
| Speed | processing time of each frame (mean and maximum) |
| Load | CPU of the whole Jetson, CPU used by cuVSLAM and GPU (mean and maximum) |
| Lost frames | lost in recording and lost by tracking |
| Trajectory | position and rotation for each frame, as CSV |

## Configuration

| Item | Value |
|---|---|
| cuVSLAM mode | `tracking_mode: 1` (stereo + IMU) |
| Inputs | cam2 `infra1` + `infra2` + `/cam2/imu` |
| Launch | `launch/cuvslam_stereo_imu_cam2_bench.launch.py` |
| Results | `results/stereo_imu_cam2/` |

## Output files

```
results/stereo_imu_cam2/
  summary.csv               one row per bag, main metrics
  summary.md                the same, as a Markdown table
  summary_detailed.csv      main metrics plus counts and checks
  <bag>/
    frames.csv              one row per processed frame (time, CPU, GPU)
    odometry.csv            odometry trajectory, one pose per frame
    slam_odometry.csv       SLAM trajectory computed during processing
    slam_poses.csv          final SLAM trajectory, optimized after loop closures
    cuvslam.log, monitor.log, play.log
```

### `frames.csv` (one row per frame)

| Column | Meaning |
|---|---|
| `frame_timestamp_s` | frame timestamp (s, epoch), the same as the images in the bag |
| `window_ms` | length of the measurement window: time since the previous frame |
| `tracking_state` | `ok` = tracking succeeded, `error` = tracking lost |
| `frame_processing_time_ms` | time cuVSLAM took to process the frame |
| `jetson_cpu_%` | CPU of the **whole Jetson** in the window, all processes included |
| `cuvslam_cpu_%` | CPU used **by cuVSLAM** in the window |
| `jetson_gpu_%` | GPU of the Jetson in the window |

### Pose files (`odometry.csv`, `slam_odometry.csv`, `slam_poses.csv`)

| Column | Meaning |
|---|---|
| `timestamp` | frame timestamp (s, epoch) |
| `x`, `y`, `z` | position (m) |
| `qx`, `qy`, `qz`, `qw` | rotation as a quaternion |
| `roll_deg`, `pitch_deg`, `yaw_deg` | the same rotation as angles (degrees) |

Poses refer to the `cam2_link` frame (ROS convention: x forward, y left, z up) and are
**relative to the starting point**, which is (0, 0, 0) with no rotation.

| File | Content |
|---|---|
| `odometry.csv` | visual inertial odometry, frame by frame; accumulates drift |
| `slam_odometry.csv` | SLAM pose as it was when the frame was processed |
| `slam_poses.csv` | final SLAM pose, corrected by loop closures; the one to use for comparison with floor plans |

Poses exist only for frames where tracking succeeded (`tracking_state = ok`).

## Metrics (`summary.csv`)

### Frames

| Column | Meaning | How it is computed |
|---|---|---|
| `bag_duration_s` | duration of the recording | from the first to the last IR message, using the time the bag recorded them |
| `camera_capture_fps` | fps at which the camera captured IR frames | (camera frames - 1) / `bag_duration_s` |
| `slam_processed_fps` | fps actually processed by cuVSLAM | `camera_capture_fps` × processed frames / camera frames |
| `frames_lost_in_recording_%` | % of frames produced by the camera but without a complete stereo pair in the bag | 1 - (pairs in the bag / camera frames) |
| `frames_tracking_lost_stereo_%` | % of processed frames where cuVSLAM lost tracking | frames with `tracking_state = error` / processed frames |

**Camera frame**: a frame exists if the bag contains at least one of its four messages
(`camera_info` and image, left and right) with timestamps within 5 ms. This way a message
lost in recording does not remove the frame from the count.

**Stereo pair**: left and right image with timestamps within 5 ms, the same threshold
cuVSLAM uses (`sync_matching_threshold_ms`).

With the IMU, `frames_tracking_lost_stereo_%` reports the tracking lost by the visual
inertial odometry.

### Processing time

| Column | Meaning |
|---|---|
| `frame_processing_time_mean_ms` | mean time cuVSLAM took to process a frame |
| `frame_processing_time_max_ms` | maximum time cuVSLAM took to process a frame |

This is the time cuVSLAM spends on each frame: computing the new pose, updating the SLAM
map and publishing the results. It comes from cuVSLAM itself
(`node_callback_execution_time` of the `visual_slam/status` message, one per frame).
**Only frames where tracking succeeded** are used: when tracking fails cuVSLAM ends
earlier, and those frames would lower the mean.

### Load

| Column | Meaning |
|---|---|
| `jetson_cpu_mean_%`, `jetson_cpu_max_%` | CPU of the whole Jetson, all processes included |
| `cuvslam_cpu_mean_%`, `cuvslam_cpu_max_%` | CPU used by cuVSLAM for SLAM |
| `jetson_gpu_mean_%`, `jetson_gpu_max_%` | GPU of the Jetson |

CPU percentages refer to **all 6 cores** (100% = all cores busy), both for the Jetson and
for cuVSLAM: the two columns are directly comparable, and `cuvslam_cpu` is always a part
of `jetson_cpu`.

**How they are measured.** Every time cuVSLAM finishes a frame, the monitor reads two
Linux counters in **nanoseconds**:

| Counter | Source |
|---|---|
| Jetson | `/proc/schedstat`: time each core spent running tasks |
| cuVSLAM | CPU clock of its process (all threads) |

The difference from the previous reading, divided by the window length and by the number
of cores, gives the load in the **window** between the two frames. Values are continuous:
a running Jetson is never at 0%.

The GPU is read every 10 ms (`/sys/devices/platform/bus@0/17000000.gpu/load`), and for
each frame the samples falling in its window are averaged.

**Mean**: weighted by the window length, that is the average load over time.

**Maximum**: the highest value among the frames. When two frames arrive a few milliseconds
apart the window is very short and the percentage describes the load of that instant: the
maximum can therefore be much higher than the mean.

### Other columns in `summary_detailed.csv`

| Column | Meaning |
|---|---|
| `camera_frames` | frames produced by the camera (see above) |
| `recorded_stereo_pairs` | complete stereo pairs in the bag |
| `slam_processed_frames` | frames processed by cuVSLAM |
| `frames_lost_in_processing_%` | pairs present in the bag but not processed by cuVSLAM |
| `max_gap_between_processed_frames_ms` | longest interval between two processed frames |
| `header_timestamp_jumps` | jumps of more than 1 s in the image header timestamps; for this reason the duration uses the bag recording time |

## How to read the numbers

| Question | Where to look |
|---|---|
| Does cuVSLAM keep up with real time? | yes if `frame_processing_time_max_ms` stays below the interval between two frames, 1000 / `camera_capture_fps` (66 ms at 15 fps, 33 ms at 30 fps) |
| How much of the Jetson does SLAM use? | `cuvslam_cpu_mean_%` and `jetson_gpu_mean_%`: what is left is available for the rest of the pipeline |
| How robust is tracking? | a high `frames_tracking_lost_stereo_%` indicates difficult scenes (for example light exposure or low texture) |