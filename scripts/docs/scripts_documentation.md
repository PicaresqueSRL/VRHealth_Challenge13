# VRHealth Challenge 13 - scripts

Reference documentation of the tools in `VRHealth_Challenge13/scripts`. They convert the Intel RealSense
D455 captures (ROS 2 bags recorded with two cameras, `cam1` and `cam2`) for NVIDIA cuVSLAM, run
cuVSLAM without ROS, visualize the runs in Rerun, and compare the results with the Isaac ROS runs and
with the ground truth.

None of the tools needs ROS: the bags (`.db3` sqlite3 or `.mcap`) are read directly and the ROS 2
messages are decoded by `bag_to_cuvslam.py`.

## Contents

1. Overview
2. Setup
3. Docker
4. Typical workflows
5. Script reference
   - bag_to_cuvslam.py
   - convert_all.py
   - merge_rig_edex.py
   - track_edex_rerun.py
   - track_bag_rerun.py
   - run_output.py
   - compare_runs.py
   - db3_to_mcap.sh
   - all_db3_to_mcap.sh
6. File formats
7. Conventions and notes

## Overview

| Script | Type | What it does |
|---|---|---|
| `bag_to_cuvslam.py` | Python | Converts a ROS 2 bag into one EDEX dataset per camera (RGB, infra1, infra2, IMU), with synchronization, calibration and a quality report of the bag |
| `convert_all.py` | Python | Runs `bag_to_cuvslam.py` on every capture of a folder, preserving the folder structure and using the `converted/` copies when present |
| `merge_rig_edex.py` | Python | Merges the cam1 and cam2 EDEX datasets into a single multi-camera rig, using the extrinsic between the two cameras (default: from the CAD of the mount) |
| `track_edex_rerun.py` | Python | Runs cuVSLAM (PyCuVSLAM) on an EDEX dataset, shows the run in Rerun and optionally saves the output |
| `track_bag_rerun.py` | Python | Runs cuVSLAM directly on a ROS 2 bag, without creating an EDEX and without writing images to disk; shows the run in Rerun and optionally saves the output |
| `run_output.py` | Python module | Saves the output of a PyCuVSLAM run in the CSV format of the Isaac ROS runs; used by the two tracking scripts (`--out-dir`) |
| `compare_runs.py` | Python | Compares saved runs with each other and with the ground truth: metrics, tables, plots |
| `db3_to_mcap.sh` | Bash | Reduces a bag to the topics needed by cuVSLAM and converts it to compressed MCAP |
| `all_db3_to_mcap.sh` | Bash | Runs `db3_to_mcap.sh` on every capture of a folder, preserving the folder structure |
| `docker/Dockerfile` | Docker | Image with cuVSLAM, its Python bindings and all the dependencies of the scripts |
| `docker/run.sh` | Bash | Starts the container with GPU, display, host user and the data folder mounted |

Dependencies between the scripts (they must stay in the same folder):

- `track_bag_rerun.py` imports from `bag_to_cuvslam.py`, `merge_rig_edex.py`, `track_edex_rerun.py` and `run_output.py`;
- `track_edex_rerun.py` imports `run_output.py` (and `bag_to_cuvslam.py` to read `/tf_static` when `--out-dir` is used);
- `merge_rig_edex.py` imports from `bag_to_cuvslam.py`;
- `convert_all.py` runs `bag_to_cuvslam.py`; `all_db3_to_mcap.sh` runs `db3_to_mcap.sh`, which runs `bag_to_cuvslam.py --dry-run` as a check.

## Setup

```bash
cd ~/dotlumen/VRHealth_Challenge13/scripts
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt          # numpy, opencv-python-headless, zstandard, lz4
```

Additional requirements:

| Needed by | Requirement |
|---|---|
| `track_edex_rerun.py`, `track_bag_rerun.py` | cuVSLAM Python bindings (built from the cuVSLAM repository) and `rerun-sdk==0.33.1` |
| `compare_runs.py` | `matplotlib`; `IcpAlign.py` from `results/ground_truth` for the ground-truth comparison |
| `db3_to_mcap.sh`, `all_db3_to_mcap.sh` | `rosbags` (`pip install rosbags`), which provides `rosbags-convert` |

## Docker

`docker/Dockerfile` builds an image that runs every script: it compiles cuVSLAM (default: commit `2a0882c`,
version 17.0.0, with cuNLS and LMDB) and its Python bindings on CUDA 12.6 / Ubuntu 24.04, and installs the Python
dependencies, `rerun-sdk==0.33.1`, `matplotlib` and `rosbags`. The scripts are copied to `/opt/vrhealth/scripts`,
which is on the `PATH`.

Host requirements: NVIDIA driver supporting CUDA 12.6 (version 560 or later), Docker and the NVIDIA Container
Toolkit (`--gpus all`).

```bash
cd ~/dotlumen/VRHealth_Challenge13/scripts
docker build -f docker/Dockerfile -t vrhealth-cuvslam .
# faster: compile only for your GPU (compute capability, e.g. 86 = RTX 30xx, 89 = RTX 40xx)
docker build -f docker/Dockerfile --build-arg CUDA_ARCHITECTURES=89 -t vrhealth-cuvslam .
```

| Build argument | Default | Description |
|---|---|---|
| `CUVSLAM_REF` | `2a0882c` | cuVSLAM branch, tag or commit to build |
| `CUVSLAM_REPO` | `https://github.com/nvidia-isaac/cuVSLAM.git` | cuVSLAM repository |
| `CUDA_ARCHITECTURES` | `all` | CUDA architectures to compile for; `all` works on any GPU but is slow to build |
| `CUDA_VERSION`, `UBUNTU_VERSION` | `12.6.3`, `24.04` | base images `nvidia/cuda:<cuda>-{devel,runtime}-ubuntu<ubuntu>` |

`docker/run.sh` starts the container with the GPU, the host display for the Rerun viewer (X11), the host user id
(files written by the scripts belong to you), the data folder mounted at the same absolute path as on the host (so the
paths saved in `bag_report.json` and `run_info.json` stay valid) and the scripts folder mounted over
`/opt/vrhealth/scripts` (the current version of the scripts is used without rebuilding).

```bash
docker/run.sh                                             # interactive shell
docker/run.sh track_bag_rerun.py ~/dotlumen/Captures/9_Loop_closure/loop_closure \
    --mode inertial --streams cam2/ir1 cam2/ir2 --rectified --slam --out-dir ~/dotlumen/runs/lc --no-viewer
DATA=/mnt/data NO_X11=1 docker/run.sh convert_all.py /mnt/data/Captures -o /mnt/data/edex
```

| Variable | Default | Description |
|---|---|---|
| `IMAGE` | `vrhealth-cuvslam` | image name |
| `DATA` | `$HOME/dotlumen` | data folder mounted at the same path |
| `NO_X11` | `0` | `1` = do not share the display (use `--save` or `--no-viewer` in the tracking scripts) |
| `EXTRA_ARGS` | - | additional `docker run` arguments, e.g. `"-v /mnt/disk:/mnt/disk"` |

If the Rerun viewer cannot open a window, save the recording with `--save run.rrd` and open it on the host
with `rerun run.rrd`.

## Typical workflows

**Convert all captures to EDEX**

```bash
python3 convert_all.py ~/dotlumen/Captures -o ~/dotlumen/Captures/edex
```

**Reduce the bags (keep only what cuVSLAM needs, compressed MCAP)**

```bash
./all_db3_to_mcap.sh -o ~/dotlumen/Captures/mcap ~/dotlumen/Captures
```

**Stereo IR + IMU on one capture, from the EDEX and from the bag, with saved output**

```bash
B=~/dotlumen/Captures/9_Loop_closure/loop_closure
E=~/dotlumen/Captures/edex_ir/9_Loop_closure/loop_closure
R=~/dotlumen/VRHealth_Challenge13/results/pycuvslam

python3 bag_to_cuvslam.py $B -o $E --reference ir --cameras cam2
python3 track_edex_rerun.py $E/cam2/stereo.edex --mode inertial --cameras 0 1 --rectified --slam \
    --out-dir $R/edex/cam2_stereo_imu/loop_closure --save $R/edex/cam2_stereo_imu/loop_closure/run.rrd
python3 track_bag_rerun.py $B --mode inertial --streams cam2/ir1 cam2/ir2 --rectified --slam \
    --out-dir $R/bag/cam2_stereo_imu/loop_closure --save $R/bag/cam2_stereo_imu/loop_closure/run.rrd
```

**Compare with the Isaac ROS run and the ground truth**

```bash
G=~/dotlumen/VRHealth_Challenge13/results
python3 compare_runs.py ros=$G/slam_csv/cam2_stereo_imu/loop_closure \
    bag=$R/bag/cam2_stereo_imu/loop_closure edex=$R/edex/cam2_stereo_imu/loop_closure \
    --gt $G/ground_truth/loop_closure/paths3d.csv --out $G/comparisons/cam2_stereo_imu/loop_closure
```

**4-IR rig with both cameras**

```bash
python3 merge_rig_edex.py ~/dotlumen/Captures/edex/9_Loop_closure/loop_closure --streams ir
python3 track_edex_rerun.py ~/dotlumen/Captures/edex/9_Loop_closure/loop_closure/rig_ir/stereo.edex --mode multicamera
# or directly from the bag
python3 track_bag_rerun.py $B --mode multicamera --streams cam1/ir1 cam1/ir2 cam2/ir1 cam2/ir2
```

# Script reference

## bag_to_cuvslam.py

Converts a ROS 2 bag recorded with RealSense cameras into the EDEX format read by cuVSLAM, one dataset
per camera. It reads the bag directly (sqlite3 `.db3` or MCAP, also zstd/lz4 compressed) and decodes the
`Image`, `CameraInfo`, `Imu` and `TFMessage` messages itself.

**What it does**

1. Reads the image timestamps, the `camera_info` messages, `/tf_static` and the IMU of the chosen camera.
2. Drops messages whose header stamp is anomalous compared to the bag receive time (`--stale-threshold`),
   e.g. a frame left queued in the driver when the recording started.
3. Pairs infra1 and infra2 (`--stereo-tolerance-ms`) and matches the IR pairs to the RGB frames (`--tolerance-ms`,
   1:1, smallest offsets first).
4. Chooses the reference stream, which defines the EDEX frames and becomes camera 0 (`--reference`):
   with `rgb` every RGB image is a frame and IR pairs are attached when synchronized; with `ir` every IR pair
   is a frame and RGB images are attached; `auto` picks the more frequent stream.
5. Writes the calibration: RGB as `brown5k` (ROS `plumb_bob` coefficients reordered), IR as `pinhole`
   (rectified images). The infra2 baseline is taken from `P[3] = -fx * baseline` of its `camera_info` when it uses
   the infra1 frame id (realsense-ros convention). Extrinsics come from `/tf_static`; the IMU frame falls back to
   `<cam>_gyro_optical_frame` when the IMU frame id is not in `/tf_static`.
6. Writes the images (in parallel) and a quality report (brightness, saturated and black pixels, sharpness).

**Camera order in the EDEX**: `rgb, ir1, ir2` with `--reference rgb`; `ir1, ir2, rgb` with `--reference ir`.
Use these indices in `track_edex_rerun.py --cameras`.

**Output**

```
<output>/
├── bag_report.json           topic statistics, gaps, image quality, calibration, dropped messages
└── <cam>/
    ├── stereo.edex           EDEX header and body (cameras, extrinsics, IMU section)
    ├── frame_metadata.jsonl  one line per frame: image file and timestamp of each camera present
    ├── calibration.json      original camera_info, transforms used, IMU frame
    ├── sync.csv              per frame: stamps of RGB / infra1 / infra2, receive time, RGB-IR offset
    ├── IMU.jsonl, imu.csv    IMU samples (only for --imu-camera)
    └── images/{rgb,infra1,infra2}/000000.{jpg,png}
```

**Usage**

```bash
python3 bag_to_cuvslam.py <bag> -o <output> [options]
python3 bag_to_cuvslam.py ~/dotlumen/Captures/9_Loop_closure/loop_closure -o /tmp/lc --dry-run   # analysis only
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `bag` (positional) | text | **required** | bag folder (containing the .db3/.mcap files) or a single file |
| `-o`, `--output` | text | **required** | output folder |
| `--cameras` | one or more `CAMERAS` | - | camera prefixes (default: detected from /&lt;cam&gt;/color/image_raw) |
| `--rgb-topic` | text | `/{cam}/color/image_raw` | RGB image topic; `{cam}` is replaced by the camera prefix |
| `--rgb-info-topic` | text | `/{cam}/color/camera_info` | RGB camera_info topic |
| `--ir1-topic` | text | `/{cam}/infra1/image_rect_raw` | infra1 image topic |
| `--ir1-info-topic` | text | `/{cam}/infra1/camera_info` | infra1 camera_info topic |
| `--ir2-topic` | text | `/{cam}/infra2/image_rect_raw` | infra2 image topic |
| `--ir2-info-topic` | text | `/{cam}/infra2/camera_info` | infra2 camera_info topic (its `P[3]` gives the stereo baseline) |
| `--imu-camera` | text | `cam2` | camera that owns the IMU ('none' to skip exporting it) |
| `--imu-topic` | text | `/{cam}/imu` | IMU topic (`sensor_msgs/Imu`, gyroscope and accelerometer combined) |
| `--time` | `header` \| `receive` | `header` | time used for synchronization and written timestamps |
| `--tolerance-ms` | number | `10.0` | maximum RGB &lt;-&gt; IR pair offset |
| `--stereo-tolerance-ms` | number | `1.0` | maximum infra1 &lt;-&gt; infra2 offset |
| `--start` | number | - | seconds from the start of the bag at which to start |
| `--end` | number | - | seconds from the start of the bag at which to stop |
| `--jpg-quality` | integer | `95` | JPEG quality of the RGB images (0-100) |
| `--png-compression` | `0` \| `1` \| `2` \| `3` \| `4` \| `5` \| `6` \| `7` \| `8` \| `9` | `6` | PNG compression level (lossless; 9 = smallest files) |
| `--ir-format` | `png` \| `jpg` | `png` | image format of infra1/infra2 (`png` is lossless) |
| `--gyro-noise-density` | number | `0.000244` | gyroscope noise density written to the EDEX IMU section [rad/s/sqrt(Hz)] |
| `--gyro-random-walk` | number | `1.9393e-05` | gyroscope random walk written to the EDEX [rad/s^2/sqrt(Hz)] |
| `--accel-noise-density` | number | `0.001862` | accelerometer noise density written to the EDEX [m/s^2/sqrt(Hz)] |
| `--accel-random-walk` | number | `0.003` | accelerometer random walk written to the EDEX [m/s^3/sqrt(Hz)] |
| `--imu-frequency` | number | - | IMU frequency for the EDEX (default: estimated from the data) |
| `--workers` | integer | number of CPUs, between 2 and 8 | threads for image encoding |
| `--overwrite` | flag | off | overwrite the output folder |
| `--stale-threshold` | number | `0.5` | drop messages whose stamp-receive offset deviates from the median by more than N seconds (0 = disabled) |
| `--reference` | `rgb` \| `ir` \| `auto` | `rgb` | stream that defines the frames (camera 0): rgb, ir pair, or auto = the most frequent |
| `--dry-run` | flag | off | analyze and synchronize without writing images |

## convert_all.py

Converts every capture of a folder to EDEX by running `bag_to_cuvslam.py --reference auto --overwrite` on
each one, and mirrors the folder structure in the output folder.

**Rules**

- A capture is a folder that directly contains `.db3` or `.mcap` files.
- If a capture has a `converted/` subfolder with a bag, that copy is used; the output still goes to the
  capture path (without `converted`).
- Folders whose name starts with `vslam`, `converted` folders and the output folder are skipped.
- Bags without camera topics (e.g. SLAM output only) are reported with status `no cameras`.

**Output**: for each capture, the `bag_to_cuvslam.py` output and `convert.log`; at the end
`<output>/conversion_summary.json` (source, status, return code, duration, last log lines of each capture).

**Usage**

```bash
python3 convert_all.py ~/dotlumen/Captures -o ~/dotlumen/Captures/edex --list     # only list the captures
python3 convert_all.py ~/dotlumen/Captures -o ~/dotlumen/Captures/edex
python3 convert_all.py ~/dotlumen/Captures -o ~/dotlumen/Captures/edex --only 9_Loop_closure/loop_closure \
    --extra "--stale-threshold 1.0"
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `root` (positional) | text | **required** | captures folder (e.g. ~/dotlumen/Captures) |
| `-o`, `--output` | text | **required** | output folder (e.g. ~/dotlumen/Captures/edex) |
| `--list` | flag | off | list the captures found and exit |
| `--skip-existing` | flag | off | skip captures that already have bag_report.json in the output |
| `--only` | one or more `ONLY` | - | convert only these relative paths |
| `--workers` | integer | number of CPUs, between 2 and 8 | threads for image encoding, passed to `bag_to_cuvslam.py` |
| `--extra` | text | `` | extra arguments for bag_to_cuvslam.py |

## merge_rig_edex.py

Merges the per-camera EDEX datasets of a capture (`cam1/` and `cam2/`, produced by `bag_to_cuvslam.py`) into a
single EDEX rig, so that cuVSLAM can use both cameras together. The per-camera folders are not modified.

**What it does**

1. Reads `stereo.edex`, `calibration.json` and `frame_metadata.jsonl` of both cameras.
2. Reads `/tf_static` of the source bag (path from `bag_report.json`, or `--bag`) to get `<cam>_link -> optical frame`
   of each camera, and connects the two cameras with `--extrinsic` (`T_<cam1>_link_from_<cam2>_link`). The default is
   the transform measured on the CAD of the mount: 50 degrees of relative pitch, cam2 about 5 cm above and 4 cm behind cam1.
3. Builds the frames on the first stream of the first camera and matches the other streams by timestamp
   (`--tolerance-ms`, cuVSLAM requires <= 1 ms); frames without a match stay partial.
4. Runs the same frustum-overlap test used by cuVSLAM to form stereo pairs (pairs above 0.5) and warns when no pair
   is formed: the cam1/cam2 RGB pair overlaps about 20% and is not accepted by cuVSLAM.

**Output** (`<capture>/rig_<streams>/` by default): `stereo.edex`, `frame_metadata.jsonl`, `sync.csv`,
`rig_calibration.json`, `IMU.jsonl` (link to the IMU of the camera that has it) and `images/<cam>` (links, or copies
with `--copy`).

**Notes**: with more than two cameras cuVSLAM uses the IMU only to estimate gravity (no visual-inertial fusion).
The CAD extrinsic is accurate to a few millimetres; a Kalibr calibration is recommended.

**Usage**

```bash
python3 merge_rig_edex.py ~/dotlumen/Captures/edex/9_Loop_closure/loop_closure --streams ir
python3 merge_rig_edex.py <capture> --streams all --extrinsic -0.040 0 0.051 0 -0.4227 0 0.9063 --tolerance-ms 3
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `capture` (positional) | text | **required** | folder containing cam1/ and cam2/ (output of bag_to_cuvslam.py) |
| `-o`, `--output` | text | - | output folder (default &lt;capture&gt;/rig_&lt;streams&gt;) |
| `--cams` | 2 values | `cam1 cam2` | the two cameras; the first one is the reference |
| `--streams` | `ir` \| `rgb` \| `all` | `ir` | streams of each camera put in the rig: `ir` (infra1 + infra2), `rgb`, `all` (IR first, then RGB) |
| `--extrinsic` | `X` `Y` `Z` `QX` `QY` `QZ` `QW` | `-0.04 0.0 0.0509 0.0 -0.42273 0.0 0.90626` | T_&lt;cam1&gt;_link_from_&lt;cam2&gt;_link in meters and quaternion (default: CAD) |
| `--link-frame` | text | `{cam}_link` | name of each camera's base frame |
| `--bag` | text | - | bag to read /tf_static from (default: from bag_report.json) |
| `--tolerance-ms` | number | `1.0` | maximum offset between the reference and the other streams |
| `--copy` | flag | off | copy images instead of using symbolic links |
| `--overwrite` | flag | off | replace the output folder if it exists |

## track_edex_rerun.py

Runs cuVSLAM through its Python bindings (PyCuVSLAM) on an EDEX dataset with `frame_metadata.jsonl`, and
visualizes the run in Rerun.

**What it does**

1. Loads the cameras selected with `--cameras` and re-centres the rig on the first of them (required by Mono,
   and the trajectory is that camera's trajectory). Converts the EDEX extrinsics (legacy cuVSLAM axes) to OpenCV axes.
2. In Inertial mode, feeds the IMU samples up to each frame; samples before the first frame are discarded.
3. Tracks every frame that contains at least one selected camera (Mono: camera 0 must be present).
   Frames whose cameras are more than 1 ms apart are reported.
4. Logs to Rerun: odometry trajectory, SLAM trajectory (`--slam`), rig pose, frustums, images with the tracked
   points, landmarks, final point cloud, and IMU plots and gravity vector in Inertial mode.
5. With `--out-dir`, saves the output through `run_output.py` (see File formats). The poses are
   converted from the rig to the base frame (`--base-frame`, default `<cam>_link`) using `calibration.json` and
   `/tf_static` of the source bag.

**Usage**

```bash
python3 track_edex_rerun.py <stereo.edex> --mode inertial --cameras 0 1 --rectified --slam \
    --out-dir runs/edex/loop_closure --no-viewer
python3 track_edex_rerun.py <stereo.edex> --mode mono --cameras 0 --save run.rrd --tum odom.txt
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `edex` (positional) | text | **required** | stereo.edex file |
| `--mode` | `multicamera` \| `inertial` \| `mono` | `multicamera` | cuVSLAM odometry mode |
| `--cameras` | one or more `CAMERAS` | - | indices of the EDEX cameras to use (default: mono -&gt; 0, otherwise all) |
| `--slam` | flag | off | also enable SLAM |
| `--rectified` | flag | off | rectified_stereo_camera=True (pinhole camera pairs only, e.g. --cameras 1 2) |
| `--start` | integer | `0` | first frame to process (index) |
| `--max-frames` | integer | - | maximum number of frames to process |
| `--log-input` | flag | off | also log the exact array passed to cuVSLAM as a plain 2D image (entity input/camN), to compare it with the frustum visualization |
| `--image-every` | integer | `1` | send images to Rerun every N frames (makes the recording lighter) |
| `--save` | text | - | save to an .rrd file instead of opening the viewer |
| `--tum` | text | - | save the odometry trajectory in TUM format |
| `--verbosity` | integer | `1` | cuvslam.set_verbosity (0-3) |
| `--out-dir` | text | - | save the cuVSLAM output (CSV in the Isaac ROS format + run_info.json) to this folder |
| `--base-frame` | text | - | frame of the saved poses (default: &lt;cam&gt;_link of the EDEX rig; 'rig' = no conversion) |
| `--bag` | text | - | source bag for /tf_static (default: from ../bag_report.json) |
| `--async-sba` | flag | off | async_sba=True (as in real time; default False: deterministic offline run) |
| `--no-viewer` | flag | off | do not open the Rerun viewer |

## track_bag_rerun.py

Runs cuVSLAM directly on a ROS 2 bag (`.mcap` or `.db3`) and visualizes the run in Rerun, doing in memory what
`bag_to_cuvslam.py` + `merge_rig_edex.py` + `track_edex_rerun.py` do, without writing images to disk.

**What it does**

1. Pass 1: reads image timestamps, `camera_info`, `/tf_static` and the IMU; drops stale messages as `bag_to_cuvslam.py`.
2. Builds the rig from `--streams` (`<cam>/<rgb|ir1|ir2>`). The first stream is the reference: it defines the
   frames, their timestamps and the rig origin. Streams of two cameras are connected with `--extrinsic` when
   `/tf_static` does not connect them. The other streams are matched to the reference within `--tolerance-ms`.
3. Pass 2: reads the images in file order and feeds the frames to cuVSLAM in timestamp order.
4. Rerun layout: the image of each camera with its tracked points (top left), gyroscope, accelerometer and
   number of tracked points (bottom left), 3D view with trajectory, rig and landmarks (right).
5. With `--out-dir`, saves the output through `run_output.py`, with the poses of `<cam>_link` of the reference stream.

**Default streams**: Mono -> `<imu-camera>/rgb`; Inertial -> `<imu-camera>/ir1 <imu-camera>/ir2`;
Multicamera -> `ir1 ir2` of every camera in the bag.

**Note**: frames where only some streams are within the tolerance are passed to cuVSLAM as partial frames
(e.g. infra1 without infra2), whereas the EDEX created with `--reference ir` and the Isaac ROS node
(`min_num_images: 2`) contain only complete IR pairs.

**Usage**

```bash
B=~/dotlumen/Captures/mcap/9_Loop_closure/loop_closure
python3 track_bag_rerun.py $B --mode inertial --streams cam2/ir1 cam2/ir2 --rectified --slam --out-dir runs/bag/lc
python3 track_bag_rerun.py $B --mode multicamera --streams cam1/ir1 cam1/ir2 cam2/ir1 cam2/ir2 --save lc.rrd
python3 track_bag_rerun.py $B --mode mono --streams cam2/rgb --start 10 --end 40
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `bag` (positional) | text | **required** | bag folder (.mcap/.db3) or single file |
| `--mode` | `multicamera` \| `inertial` \| `mono` | `multicamera` | cuVSLAM odometry mode |
| `--streams` | one or more `STREAMS` | - | streams &lt;cam&gt;/&lt;rgb\|ir1\|ir2&gt;; the first one is the reference. Default: mono -&gt; &lt;imu-camera&gt;/rgb, inertial -&gt; &lt;imu-camera&gt;/ir1 ir2, multicamera -&gt; ir1 ir2 of all cameras |
| `--imu-camera` | text | `cam2` | camera providing the IMU (inertial mode) |
| `--extrinsic` | `X` `Y` `Z` `QX` `QY` `QZ` `QW` | `-0.04 0.0 0.0509 0.0 -0.42273 0.0 0.90626` | T_&lt;cam1&gt;_link_from_&lt;cam2&gt;_link (used only if /tf_static does not connect the cameras) |
| `--link-frame` | text | `{cam}_link` | base frame of each camera in /tf_static |
| `--tolerance-ms` | number | `1.0` | maximum offset between the reference and the other streams (cuVSLAM requires &lt;= 1 ms) |
| `--stale-threshold` | number | `0.5` | discard messages whose stamp-receive offset is more than N s from the median |
| `--multicam-mode` | `precision` \| `moderate` \| `performance` | `precision` | selection of primary cameras in Multicamera |
| `--slam` | flag | off | also enable SLAM |
| `--rectified` | flag | off | rectified_stereo_camera=True (rectified IR pairs only) |
| `--start` | number | - | seconds from the beginning at which to start |
| `--end` | number | - | seconds from the beginning at which to stop |
| `--max-frames` | integer | - | maximum number of frames to process |
| `--image-every` | integer | `1` | send images to Rerun every N frames |
| `--save` | text | - | save to an .rrd file instead of opening the viewer |
| `--tum` | text | - | save the odometry trajectory in TUM format |
| `--slam-tum` | text | - | save the SLAM trajectory in TUM format |
| `--verbosity` | integer | `1` | cuvslam.set_verbosity (0-3) |
| `--out-dir` | text | - | save the cuVSLAM output (CSV in the Isaac ROS format + run_info.json) to this folder |
| `--base-frame` | text | - | frame of the saved poses (default: &lt;cam&gt;_link of the reference stream; 'rig' = no conversion) |
| `--async-sba` | flag | off | async_sba=True (as in real time; default False: deterministic offline run) |
| `--no-viewer` | flag | off | do not open the Rerun viewer |

## run_output.py

Python module (no command line) used by `track_edex_rerun.py` and `track_bag_rerun.py` when `--out-dir` is given.
It collects the per-frame output of the cuVSLAM `Tracker` and writes it in the same format as
`isaac_ros_5/launch/extract_paths.py`, so that PyCuVSLAM and Isaac ROS runs can be compared directly and aligned
to the ground truth with `results/ground_truth/RunBatchPipeline.py` / `IcpAlign.py`.

**Main elements**

| Element | Description |
|---|---|
| `RunWriter(out_dir, T_rig_from_base, base_frame, n_cameras, info)` | Collects the output; `T_rig_from_base` converts the rig poses to the base frame |
| `RunWriter.add_frame(ts_ns, est, slam_pose, obs_counts, n_landmarks, track_ms, tracker)` | Called after each `Tracker.track()`: odometry pose, online SLAM pose, status, SLAM metrics |
| `RunWriter.finish(tracker, slam_enabled, n_final_landmarks)` | Writes the files; reads the final SLAM trajectory (`get_all_slam_poses`) and the loop closure poses; returns the summary |
| `cuvslam_version(cuvslam)` | cuVSLAM version string for `run_info.json` |

The files it writes are described in File formats.

## compare_runs.py

Compares cuVSLAM runs saved in the Isaac ROS CSV format: Isaac ROS runs (`extract_paths.py`) and PyCuVSLAM runs
(`--out-dir`). Each run is a folder with `odometry.csv` and/or `slam_poses.csv`, and optionally `status.csv`,
`slam_metrics.csv` and `run_info.json`.

**Metrics** (for `odometry` and `slam_poses` separately)

| Group | Metrics |
|---|---|
| Per run | poses, duration, path length over all consecutive poses (includes frame-to-frame jitter) and over one pose per second (distance actually walked), start-end distance (drift of a closed loop), z range, speed (median, 95th percentile), frames lost, frames with loop closure, median tracking time |
| Per pair of runs | poses matched by timestamp (`--tolerance-ms`); position difference without alignment (mean, max); after a rigid alignment (Umeyama, rotation + translation): RMSE, maximum, mean and maximum rotation difference |
| Against the ground truth | `IcpAlign.py` rigid (`--no-scale`): RMSE, symmetric RMSE, mean error; with scale: scale factor and RMSE |

**Output** (`--out`): `report.md`, `metrics.json`, `pairs.csv`, `icp/` (aligned CSV files and logs of
`IcpAlign.py`) and `plots/`:

| Plot | Content |
|---|---|
| `<traj>_top_view.png` | top view of all runs, rigidly aligned to the first run; circle = start, square = end |
| `<traj>_height.png` | z over time in the base frame of the first pose (not gravity-aligned) |
| `<traj>_pair_difference.png` | position difference between each pair of runs over time, after rigid alignment |
| `<traj>_vs_ground_truth.png` | runs aligned to the ground truth by `IcpAlign.py` (rigid), with their RMSE |
| `tracked_points.png` | tracked points per frame (runs with `status.csv` from PyCuVSLAM) |

**Usage**

```bash
python3 compare_runs.py ros=<isaac ros folder> bag=<folder> edex=<folder> \
    --gt <ground_truth>/paths3d.csv --out <comparison folder>
```

A run is given as `label=folder`, or as `folder` (the label is then the folder name). The first run is the
reference of the top-view and height plots.

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `runs` (positional) | one or more `RUNS` | **required** | run folders, as label=folder or folder |
| `--gt` | text | - | ground-truth paths3d.csv (enables the IcpAlign.py comparison) |
| `--icp-script` | text | `../results/ground_truth/IcpAlign.py` | IcpAlign.py |
| `--out` | text | **required** | output folder |
| `--tolerance-ms` | number | `5.0` | maximum timestamp difference to match poses of two runs |
| `--no-plots` | flag | off | do not create the plots |

## db3_to_mcap.sh

Reduces a ROS 2 bag to the topics needed by cuVSLAM and converts it to MCAP with internal zstd compression,
using `rosbags-convert`. Then checks that the reduced bag can still be converted to EDEX
(`bag_to_cuvslam.py --dry-run`) and prints the size before and after.

**Topics kept** (for `cam1` and `cam2`): `color/image_raw`, `color/camera_info`, `infra1/image_rect_raw`,
`infra1/camera_info`, `infra2/image_rect_raw`, `infra2/camera_info`, `aligned_depth_to_color/image_raw`,
`aligned_depth_to_color/camera_info`, `imu`; and `/tf_static`. Edit `CAMS`, `INCLUDED_TOPICS` and `GLOBAL_TOPICS`
at the top of the script to change them.

**Details**

- `--compress zstd --compress-mode storage` is used: the MCAP chunks are compressed internally, which
  `bag_to_cuvslam.py` and Rerun can read. The rosbags default (`file`) would produce a `.mcap.zstd` file, and
  `message` would compress every message: neither can be read by `bag_to_cuvslam.py`.
- Bags whose `metadata.yaml` points to a `.db3` with a different name than the file present (e.g. `low_detail_0.db3`
  instead of `low_detail_0-003.db3`) are opened through a temporary folder with a link from the expected name
  to the real file; the original bag is not modified.
- The destination must not exist (`rosbags-convert` does not overwrite).

**Usage**

```bash
./db3_to_mcap.sh <source bag folder> <destination folder>
./db3_to_mcap.sh ~/dotlumen/Captures/10_Low_feature/low_detail_3 ~/dotlumen/Captures/mcap/10_Low_feature/low_detail_3
```

**Arguments**

| Argument | Description |
|---|---|
| `<source bag folder>` | folder of the bag to reduce (`.db3` or `.mcap`, with `metadata.yaml`) |
| `<destination folder>` | output bag folder; must not exist |

## all_db3_to_mcap.sh

Runs `db3_to_mcap.sh` on every capture of a folder, with the same capture rules as `convert_all.py`
(`converted/` copies preferred; `vslam*`, `converted`, `edex`, `mcap` and the output folder skipped), and mirrors
the folder structure. A failed capture does not stop the others.

**Output**: one reduced bag per capture, `<capture>.log` next to it, and `conversion_summary.tsv` (capture,
source, kind, status, seconds, size before and after). Statuses: `ok`, `error`, `no cameras` (bag without camera
topics), `skipped`, `destination exists`.

**Usage**

```bash
./all_db3_to_mcap.sh [options] [captures folder]
./all_db3_to_mcap.sh -l ~/dotlumen/Captures                   # only list the captures
./all_db3_to_mcap.sh -k low_detail_3 ~/dotlumen/Captures      # one capture
./all_db3_to_mcap.sh -o /data/mcap -s ~/dotlumen/Captures     # other output folder, skip done ones
```

**Arguments and options**

| Argument | Value | Default | Description |
|---|---|---|---|
| `[captures folder]` (positional) | path | `~/dotlumen/Captures` | folder that contains the captures |
| `-o` | path | `<captures>/mcap` | output folder |
| `-l` | flag | off | only list the captures found |
| `-s` | flag | off | skip captures already converted (destination exists) |
| `-f` | flag | off | reconvert captures already converted (deletes the destination) |
| `-k` | pattern | - | convert only captures whose relative path contains the pattern; repeatable |
| `-h` | flag | - | show the usage |

`-s` and `-f` are mutually exclusive. Without either, a capture whose destination exists is reported as
`destination exists` and not converted.

# File formats

## EDEX (bag_to_cuvslam.py, merge_rig_edex.py)

| File | Content |
|---|---|
| `stereo.edex` | JSON list `[header, body]`. The header has `cameras` (intrinsics: `distortion_model` `brown5k` or `pinhole`, `distortion_params`, `focal`, `principal`, `size`; `transform`: 3x4 rig-from-camera in the legacy cuVSLAM axes, y up and z backward) and the optional `imu` section (`measurements`, `transform`, noise parameters, `frequency`). The body has `frame_metadata` and `sequence` |
| `frame_metadata.jsonl` | one JSON line per frame: `frame_id` and `cams` (`id`, `filename`, `timestamp` in ns) for each camera present; partial frames allowed |
| `IMU.jsonl` | one JSON line per sample: `AngularVelocityX/Y/Z` [rad/s], `LinearAccelerationX/Y/Z` [m/s^2], `timestamp` [ns], in the IMU frame |

## Saved run output (run_output.py, --out-dir)

| File | Content |
|---|---|
| `odometry.csv` | one pose per tracked frame (visual or visual-inertial odometry) |
| `slam_poses.csv` | final SLAM trajectory: `get_all_slam_poses()` at the end of the run, i.e. after the loop closure corrections computed by cuVSLAM (as the last `slam_path` of the Isaac ROS node); with `--slam` |
| `slam_online.csv` | SLAM pose returned frame by frame during the run, without later corrections; with `--slam` |
| `status.csv` | `timestamp, vo_state` (1 = tracked, 2 = lost, as Isaac ROS), `track_time_ms, landmarks, observations_cam<k>` |
| `slam_metrics.csv` | per frame: `lc_status`, `pgo_status` and landmark counts of the loop closure, as reported by cuVSLAM; with `--slam` |
| `loop_closures.csv` | last loop closure poses reported by cuVSLAM (it keeps the last 10), in the SLAM world frame |
| `run_info.json` | script, input, mode, cameras or streams, parameters, cuVSLAM version, base frame and its source, frames, summary metrics |

Pose files use the columns of the Isaac ROS CSV files: `timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg`.
Timestamps are in seconds; poses are in meters, of the base frame, relative to its first pose (the first row is the
identity, as in the Isaac ROS output).

## Other files

| File | Written by | Content |
|---|---|---|
| `bag_report.json` | `bag_to_cuvslam.py` | per topic: count, rate, gaps, estimated losses; per camera: reference, frames, matches, RGB-IR offsets, image quality, calibration; dropped stale messages |
| `sync.csv` | `bag_to_cuvslam.py`, `merge_rig_edex.py` | per frame: timestamp of each stream and offset from the reference |
| `rig_calibration.json` | `merge_rig_edex.py` | extrinsic used, rig frame, transform of each camera, predicted stereo pairs, IMU transform |
| `conversion_summary.json` | `convert_all.py` | per capture: source, status, return code, duration, log |
| `conversion_summary.tsv` | `all_db3_to_mcap.sh` | per capture: source, status, duration, size before and after |
| `report.md`, `metrics.json`, `pairs.csv` | `compare_runs.py` | comparison tables and metrics |

# Conventions and notes

- **Timestamps**: the header stamps of the messages are used by default (`--time receive` in `bag_to_cuvslam.py` uses
  the bag receive time). cuVSLAM uses the timestamp of camera 0 for the whole frame and expects the cameras of a frame
  to be within 1 ms.
- **Axes**: the Python API of cuVSLAM and the rig poses use OpenCV axes (x right, y down, z forward). EDEX files store
  extrinsics in the legacy cuVSLAM axes (y up, z backward); the scripts convert between the two. Saved poses use the
  ROS base frame `<cam>_link` (x forward, y left, z up). That frame is not gravity-aligned.
- **Stereo pairs**: cuVSLAM forms stereo pairs automatically from the frustum overlap (more than 50% of a 31x31 grid of
  pixels visible in the other camera between 2 and 4 m). Multicamera and Inertial modes need at least one pair.
- **IMU**: Inertial mode needs one stereo pair and the IMU; with more than two cameras the IMU is used only for gravity.
  The default noise parameters are those of the Isaac ROS launch files.
- **Rectified images**: the RealSense IR topics are rectified (`image_rect_raw`); `--rectified` sets
  `rectified_stereo_camera`, as `rectified_images: True` in the Isaac ROS launch files.
- **Determinism**: the tracking scripts run with `async_sba=False` by default (offline, repeatable); `--async-sba`
  reproduces the asynchronous bundle adjustment of a real-time run.
