# Executing isaac_ros_visual_slam on Captured Data

In order to execute the cuVSLAM implementation in the [`isaac_ros_visual_slam` repository](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
different passes should be performed. We provide a custom launch file to execute the mono RGBD evaluation of a recorded
Rosbag capture file.

## Prerequisites

A ROS environment should be installed on your machine (or ran in a docker container). For the installation steps, refer to
[our Jetson configuration](https://github.com/PicaresqueSRL/VRHealth_Challenge13/blob/main/configuration.md) or to the 
official [NVIDIA Isaac ROS quickstart](https://nvidia-isaac-ros.github.io/getting_started/index.html).

## Installing Isaac Ros Visual Slam

To install the Visual ROS cuVSLAM implementation we used for the experimentation, enter in your ROS container or environment
and type:

```
sudo apt install -y ros-${ROS_DISTRO}-isaac-ros-visual-slam
```

If the installation process fails, try to *update* your apt repositories first.

## Running cuVSLAM on Captured Data

To launch cuVSLAM on the recorded capture Rosbag files, you need to run 3 separate terminals:
the first to set up the cuVSLAM ROS node, the second to start the recording script and the third one to
play the recorded data. 

### 1. Start the cuVSLAM node

To enable the node capable of processing bag data using cuVSLAM, you can use the `mono_rgbd_launch.sh` script from the 
current folder:
```
./mono_rgbd_launch.sh
```
The script will copy the launch configuration stored in `mono_rgbd.launch.py` to the `isaac_ros_visual_slam` folder, and
then will launch the related ROS node.

If you want to use the second camera, you can launch the `mono_cam2_launch.sh` instead. If you want to experiment with IMU, you can use `mono_imu_launch.sh`. These scripts will copy the related _launch_ file directly in the ROS cuVSLAM shared folder.

### 2. Record

To record the cuVSLAM topics as long as they are published, you can use the `record_vslam.sh` script. From another
terminal in this folder, type:
```
./record_vslam.sh <output folder>
```
The script will automatically listen on all published `visual_slam` topics, along with `tf` and `tf_static` topics. The
argument specifies the folder to store the recorded cuVSLAM bag and the related metadata.

### 3. Play the Rosbag Capture file

To play the desired capture, open another terminal and type the command:
```
ros2 bag play <path to capture bag>
```
You can optionally specify the `-p` option, to make the playing starting in pause mode. If so, you can resume it by
pressing space in the terminal.

**Note**: using the loop `-l` option is discouraged as the resulting data could not
replicate the exact capture path.

### 4. Cleaning up

After the playing process is over, you can kill the cuVSLAM and recording processes their respective terminals by pressing
`CTRL+C` (`SIGINT`). Shutting down the recording process will commit the recorded data into the bag output folder 
specified when running the script.

## cuVSLAM Published Topics

The cuVSLAM recorded bag file will include the following topics:

| Topic Name                         | Description                                            |
|------------------------------------|--------------------------------------------------------|
| `/visual_slam/tracking/slam_path`  | Includes the evaluated poses for the capture.          |
| `/visual_slam/vis/landmarks_cloud` | Comprises the slam 3D landmarks evaluated for the map. |

Other topics are also published, and their complete description can be found in the 
[`isaac_ros_visual_slam` documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#ros-topics-published).

## Showing Landmarks and Path in RViz2

To visualize the process of reconstruction, it is possible to use the `rviz2` application. To install it, in your ROS environment type:

```
sudo apt install -y ros-$ROS_DISTRO-rviz2
```

then, activate the RViz2 environment and run the application with:

```
source /opt/ros/${ROS_DISTRO}/setup.bash
rviz2 -d ./cam2.cfg.rviz
```

The RViz2 GUI should appear with the configuration specified. Then, in another terminal, play the cuVSLAM Rosbag file (we suggest to start it in paused and loop mode):

```
ros2 bag play -p -l <path to vslam bag file>
```

Within the RViz2 interface, add the `/visual_slam/tracking/slam_path` and the `/visual_slam/vis/landmarks_cloud`
from the main menu. If needed, update the main frame to `cam1_color_frame`.

Eventually, you can start the playing by pressing `SPACE` in the bag playing terminal.
