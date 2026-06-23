# Offline SLAM with RTAB-Map from ROS2 bags (RealSense D455)

Pipeline to compute odometry and SLAM (map + point cloud) from an existing
ROS2 recording (`.db3`) of a RealSense D455 camera, without needing RealSense
drivers or native RTAB-Map packages installed on the host.

## General idea

The raw camera data (RGB, depth, IMU) is already inside the bag. To compute
SLAM, the RTAB-Map nodes need to run while the bag is "replayed" as if it
were a live stream:

1. The RTAB-Map nodes (odometry + SLAM) run inside a **Docker container**.
2. The original bag is replayed on the **host** with `ros2 bag play`,
   publishing the RGB/depth/IMU topics.
3. RTAB-Map (inside the container) subscribes to those topics, computes
   odometry and the map, and publishes the results on new topics (`/odom`,
   `/map`, `/cloud_map`, `/mapPath`, `/tf`).
4. A second `ros2 bag record` on the host saves these new topics into an
   output bag.


## Requirements

- Docker installed and working
- ROS2 on the host (any recent distro) with `ros2 bag` and `rviz2` available
- The Docker image `introlab3it/rtabmap_ros:humble-latest` (downloaded 
manually with `docker pull introlab3it/rtabmap_ros:humble-latest`)
- A `.db3` recording containing at least: color image, depth aligned to
  color, camera_info, optionally IMU

## Files in this folder

| File | What it does |
|---|---|
| `rtabmap_cam2.launch.py` | ROS2 launch file (run **inside** the container) that defines the nodes: IMU filter, visual odometry (`rgbd_odometry`), SLAM (`rtabmap`). Edit this if topic/camera names change. |
| `1_start_rtabmap.sh` | Starts the Docker container and launches the launch file inside it. Keeps running, waiting for data. |
| `2_play_bag.sh` | Replays the source bag on the host, publishing only the needed topics, with simulated clock. |
| `3_record_odom.sh` | Records the RTAB-Map output (odometry, map, TF) into a new bag. |

## Execution order

Three separate terminals, **in this order**:

```bash
# Terminal 1 — start RTAB-Map in Docker (keeps running)
cd ~/slam_ws
./1_start_rtabmap.sh

# Terminal 2 — start recording the output
./3_record_odom.sh output_bag_name

# Terminal 3 — replay the source bag
./2_play_bag.sh ~/bags/source_bag_name
```

The order matters: if the bag (terminal 3) starts before the recording
(terminal 2) is active, the first data is lost or the recording stays empty.
If the bag finishes before SLAM has processed everything, some final frames
are lost.

The final result is saved in `~/bags/output_bag_name/`.

## How to use it with a different bag

1. **Check the topic names** of the new bag:
   ```bash
   ros2 bag info ~/bags/new_bag
   ```
   You need at least: color image topic, aligned depth topic, camera_info,
   and (optionally) IMU.

2. **If the topic names differ** from `/cam2/color/image_raw`,
   `/cam2/aligned_depth_to_color/image_raw`, `/cam2/color/camera_info`,
   `/cam2/imu`, update:
   - the `remappings` in `rtabmap_cam2.launch.py`
   - the `--topics` list in `2_play_bag.sh`

3. **Check the `frame_id`**: the `frame_id` parameter in
   `rtabmap_cam2.launch.py` must match the actual TF frame of the camera in
   the new bag.

4. **Launch the pipeline** with the new bag's path:
   ```bash
   ./2_play_bag.sh ~/bags/new_bag
   ```

5. **If the bag has no IMU**, set `use_imu: False` in the launch file and
   remove the `imu_filter_madgwick` node — visual odometry still works
   without it, with lower accuracy on roll/pitch.

6. **Playback speed**: `--rate 0.3` in `2_play_bag.sh` slows down playback to
   give RTAB-Map enough real compute time per frame. On simpler bags or more
   powerful hardware this can be increased (e.g. `--rate 1.0`); if RTAB-Map
   drops frames or tracking quality is poor, lower it further.

## Cleanup between runs

RTAB-Map keeps internal state (SLAM database) inside the container. To avoid
data from a previous run "polluting" the next one, restart the container
fresh before each new processing run:

```bash
docker rm -f $(docker ps -aq --filter ancestor=introlab3it/rtabmap_ros:humble-latest) 2>/dev/null
```

The `--delete_db_on_start` flag on the launch file's nodes forces a new
database on every start, but only works correctly if the container is
actually fresh.

## Visualizing the result in RViz2

```bash
# Terminal 1
rviz2
# Fixed Frame -> map
# Add displays: TF, Odometry (/odom), Map (/map),
#                PointCloud2 (/cloud_map), Path (/mapPath)
ros2 param set /rviz2 use_sim_time true

# Terminal 2 — loop playback of the result bag
ros2 bag play ~/bags/output_bag_name --loop --clock
```

## Topics generated in the output bag

| Topic | Type | Content |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Visual odometry (camera position/orientation) |
| `/map` | `nav_msgs/OccupancyGrid` | 2D occupancy map |
| `/cloud_map` | `sensor_msgs/PointCloud2` | Reconstructed 3D point cloud |
| `/mapPath` | `nav_msgs/Path` | Estimated camera trajectory |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Transforms between frames |


