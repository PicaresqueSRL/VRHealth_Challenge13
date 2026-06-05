# Quick Setup for D455 and Jetson Orin Nano 

Documentation for the configuration and usage of the [`isaac_ros_visual_slam`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) package with two Intel RealSense D455 cameras, running inside the Isaac ROS container on a Jetson Orin Nano.

---

## Prerequisites

Before launching the container, the following must be available on the Jetson:

- **JetPack** installed (compatible version with Isaac ROS, JetPack 5.x or higher recommended)
- **Docker** installed and configured
- **NVIDIA Container Toolkit** to allow Docker to access the GPU and peripherals

---

## Isaac ROS Common Repository

The starting point is the [`isaac_ros_common`](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common) repository. It is developed and optimized specifically for NVIDIA Jetson boards, with pre-built Docker images for the ARM architecture. The included container comes fully pre-configured: it ships with ROS 2 Humble, Intel RealSense drivers, all Isaac ROS dependencies, and the CUDA/cuDNN toolchain. No additional packages need to be installed and no code needs to be written once the repo is cloned, the container can be started and you are ready to go.

> **Branch note:** the `main` branch (version 4.x) removed `run_dev.sh` in favour of the Isaac ROS CLI. The setup described in this document uses the `release-3.2` branch, which is compatible with ROS 2 Humble and JetPack 5.x, and still includes `scripts/run_dev.sh`.

```bash
git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
```

---

## Hardware

| Component | Details |
|---|---|
| **SBC** | NVIDIA Jetson Orin Nano |
| **Cameras** | 2× Intel RealSense D455 |
| **cam1** serial | `260722305330` |
| **cam2** serial | `260722300093` |
| **ROS 2** | Humble |
| **Container** | `ros2_humble.realsense` (Isaac ROS Dev) |

---

## 1. Starting the Container

All development takes place inside the Isaac ROS container. To launch it:

```bash
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -i ros2_humble.realsense
```

The `-i ros2_humble.realsense` flag selects the image that includes the RealSense drivers.

### Environment setup (required in every terminal opened inside the container)

```bash
source /opt/ros/humble/setup.bash
```

> **Note:** every time a new terminal is opened inside the container, the ROS 2 environment must be sourced again.

---

## 2. Data Acquisition — Workflow

A recording session requires **three terminals**, all with the container running and the environment sourced.

```
Terminal 1           Terminal 2           Terminal 3
───────────────      ───────────────      ───────────────
ros2 launch cam1  +  ros2 launch cam2  +  ros2 bag record
```

### Terminal 1 — Launch cam1

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam1 \
  camera_namespace:=cam1 \
  serial_no:=_260722305330 \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_accel:=true \
  enable_gyro:=true \
  unite_imu_method:=2 \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 \
  gyro_fps:=200 \
  accel_fps:=100
```

### Terminal 2 — Launch cam2

```bash
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=cam2 \
  camera_namespace:=cam2 \
  serial_no:=_260722300093 \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=true \
  enable_infra2:=true \
  enable_accel:=true \
  enable_gyro:=true \
  unite_imu_method:=2 \
  pointcloud.enable:=true \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30 \
  gyro_fps:=200 \
  accel_fps:=100
```

### Key parameters

| Parameter | Value | Description |
|---|---|---|
| `depth_module.profile` | `640x480x30` | Resolution and FPS for the depth|
| `rgb_camera.profile` | `640x480x30` | Resolution and FPS for the RGB camera |
| `unite_imu_method` | `2` | Fuses accel + gyro into a single `/imu` topic (linear interpolation) |
| `align_depth.enable` | `true` | Aligns the depth frame to the RGB frame |
| `pointcloud.enable` | `true` | Publishes the 3D point cloud |
| `gyro_fps` | `200` | Gyroscope sampling rate |
| `accel_fps` | `100` | Accelerometer sampling rate |

### Terminal 3 — Bag recording

```bash
ros2 bag record \
  /cam1/color/image_raw \
  /cam1/color/camera_info \
  /cam1/depth/image_rect_raw \
  /cam1/depth/camera_info \
  /cam1/infra1/image_rect_raw \
  /cam1/infra1/camera_info \
  /cam1/infra2/image_rect_raw \
  /cam1/infra2/camera_info \
  /cam1/aligned_depth_to_color/image_raw \
  /cam1/aligned_depth_to_color/camera_info \
  /cam1/aligned_depth_to_infra1/image_raw \
  /cam1/aligned_depth_to_infra1/camera_info \
  /cam1/imu \
  /cam1/accel/sample \
  /cam1/gyro/sample \
  /cam1/depth/color/points \
  /cam2/color/image_raw \
  /cam2/color/camera_info \
  /cam2/depth/image_rect_raw \
  /cam2/depth/camera_info \
  /cam2/infra1/image_rect_raw \
  /cam2/infra1/camera_info \
  /cam2/infra2/image_rect_raw \
  /cam2/infra2/camera_info \
  /cam2/aligned_depth_to_color/image_raw \
  /cam2/aligned_depth_to_color/camera_info \
  /cam2/aligned_depth_to_infra1/image_raw \
  /cam2/aligned_depth_to_infra1/camera_info \
  /cam2/imu \
  /cam2/accel/sample \
  /cam2/gyro/sample \
  /cam2/depth/color/points \
  /tf \
  /tf_static \
  -o scenario
```

The bag is saved to the `scenario/` directory in the current folder.

#### Recorded topics per camera

| Category | Topic (cam1 / cam2) |
|---|---|
| RGB | `color/image_raw`, `color/camera_info` |
| Depth | `depth/image_rect_raw`, `depth/camera_info` |
| Infrared | `infra1/image_rect_raw`, `infra2/image_rect_raw` + camera_info |
| Depth aligned to RGB | `aligned_depth_to_color/image_raw` + camera_info |
| Depth aligned to infra1 | `aligned_depth_to_infra1/image_raw` + camera_info |
| IMU (fused) | `imu` |
| Raw accelerometer | `accel/sample` |
| Raw gyroscope | `gyro/sample` |
| Point cloud | `depth/color/points` |
| TF | `/tf`, `/tf_static` |

Once the recording is stopped Terminal 3, the cameras are stopped in the same way in Terminals 1 and 2.

---

## 3. Bag Replay and Inspection

### Playing back the bag

```bash
ros2 bag play scenario --loop
```

The `--loop` flag automatically restarts the bag when it reaches the end.

### Visualisation with RViz2

```bash
rviz2
```

Add the following displays in RViz2:

| Display | Topic | Notes |
|---|---|---|
| Image | `/cam1/color/image_raw` | RGB cam1 |
| Image | `/cam1/depth/image_rect_raw` | Depth cam1 |
| Image | `/cam1/infra1/image_rect_raw` | Left infrared cam1 |
| Image | `/cam2/color/image_raw` | RGB cam2 |
| PointCloud2 | `/cam1/depth/color/points` | 3D point cloud cam1 |
| PointCloud2 | `/cam2/depth/color/points` | 3D point cloud cam2 |

### IMU inspection from terminal

```bash
ros2 topic echo /cam1/imu
```

Displays real-time linear acceleration and angular velocity data from the fused IMU topic of cam1. Same for cam2:

```bash
ros2 topic echo /cam2/imu
```

To check the actual publish rate of a topic:

```bash
ros2 topic hz /cam1/imu
ros2 topic hz /cam1/color/image_raw
```