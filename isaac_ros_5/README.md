# Isaac ROS 5.0 (ROS 2 Lyrical) on Jetson Orin Nano — JetPack 7.2

Setup for running **cuVSLAM** (`isaac_ros_cuvslam`) on the two D455 cameras and on the
recorded bags, using **Isaac ROS 5.0** in Docker mode.

> The previous setup (Isaac ROS 3.2, ROS 2 Humble, JetPack 6, `run_dev.sh`) is documented in
> [`../configuration.md`](../configuration.md) and [`../cuVSLAM`](../cuVSLAM). It is kept as is.

## What changes compared to Isaac ROS 3.2

| | 3.2 | 5.0 |
|---|---|---|
| JetPack | 6.x | **7.2** (Ubuntu 24.04, CUDA 13) |
| ROS 2 | Humble | **Lyrical** |
| Container | `run_dev.sh -i ros2_humble.realsense` | **`isaac-ros activate`** (Isaac ROS CLI) |
| Package | `isaac_ros_visual_slam` | **`isaac_ros_cuvslam`** (same plugin name) |
| IMU fusion | `enable_imu_fusion: True` | **`tracking_mode: 1`** |
| Camera frames param | `camera_optical_frames` | `camera_optical_frames` (unchanged) |
| D455 firmware | 5.15.x | **5.16.0.1** (required) |

## Contents

```
isaac_ros_5/
  README.md
  workspace/                       copied into $ISAAC_ROS_WS (step 8)
    docker/Dockerfile.cuvslam        adds cuVSLAM, RViz and DISPLAY to the image
    scripts/.isaac_ros_common-config tells the CLI where Dockerfile.cuvslam is
    .isaac-ros-cli/config.yaml       enables the realsense + cuvslam image layers
  launch/
    cuvslam_stereo_cam2.launch.py      stereo only (tracking_mode 0)
    cuvslam_stereo_imu_cam2.launch.py  stereo + IMU (tracking_mode 1)
```

---

## Host setup (once per Jetson)

All commands in this section run on the **host**, not in the container.

### 1. JetPack 7.2 and SDK components

Flash JetPack 7.2 with the ISO installer then:

```bash
cat /etc/nv_tegra_release          # must show R39 (release), REVISION: 2.x
sudo apt update && sudo apt install nvidia-jetpack
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
nvcc --version                     # CUDA 13.x
```

### 2. Power mode

```bash
grep -i "POWER_MODEL" /etc/nvpmodel.conf   # find the MAXN_SUPER id (2 on our board)
sudo nvpmodel -m 2
sudo jetson_clocks                          # not persistent: repeat after every reboot
```

### 3. Clock

Our board has no RTC battery: after a reboot the date may be wrong (e.g. 1970), and HTTPS
downloads then fail with *certificate not yet valid*.

```bash
timedatectl status
sudo timedatectl set-ntp true
# if it still does not synchronize (NTP blocked on the network):
sudo date -s "YYYY-MM-DD HH:MM:SS"
```

### 4. Workspace and Isaac ROS apt repository

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src
echo 'export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"' >> ~/.bashrc
source ~/.bashrc

sudo apt update && sudo apt install -y locales curl gnupg software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository -y universe

k="/usr/share/keyrings/nvidia-isaac-ros.gpg"
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor | sudo tee -a $k > /dev/null
f="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
sudo touch $f
s="deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-5.0 noble-jetpack main"
grep -qxF "$s" $f || echo "$s" | sudo tee -a $f
sudo apt-get update
```

### 5. Docker with the NVIDIA runtime as default

Docker and the NVIDIA Container Toolkit come with JetPack. On Jetson, `docker run --gpus all`
**does not work** (the toolkit runs in `csv` mode); the `nvidia` runtime must be the default:

```bash
sudo nvidia-ctk runtime configure --runtime=docker --set-as-default
sudo usermod -aG docker $USER
newgrp docker
sudo systemctl daemon-reload && sudo systemctl restart docker

docker info | grep -E "Runtimes|Default Runtime"   # Default Runtime: nvidia
docker run --rm ubuntu:24.04 bash -lc 'echo "NVIDIA runtime OK"'
```

### 6. Isaac ROS CLI

```bash
sudo apt-get install -y isaac-ros-cli
sudo isaac-ros init docker
```

### 7. RealSense udev rules

With the cameras **unplugged**:

```bash
wget https://raw.githubusercontent.com/realsenseai/librealsense/v2.56.3/config/99-realsense-libusb.rules && \
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/ && \
sudo udevadm control --reload-rules && sudo udevadm trigger && echo "Successfully added udev rules"
```

### 8. Copy this configuration into the workspace

From the root of this repository:

```bash
cp -r isaac_ros_5/workspace/. $ISAAC_ROS_WS/
mkdir -p $ISAAC_ROS_WS/launch
cp isaac_ros_5/launch/*.launch.py $ISAAC_ROS_WS/launch/
```

If `realsense` is already listed in `~/.config/isaac-ros-cli/config.yaml` (from the official
RealSense guide), remove it from there: it is now in the workspace config.

### 9. Display (monitor attached to the Jetson)

On our Jetson the monitor session is display **`:1`** (check with `ls /tmp/.X11-unix/`).
When the container is started from an SSH terminal, the host's empty `DISPLAY` overrides the
one in the image, so set a fallback on the host and allow local containers to open windows:

```bash
echo 'export DISPLAY=${DISPLAY:-:1}' >> ~/.bashrc
echo 'xhost +local: >/dev/null 2>&1' >> ~/.bashrc
source ~/.bashrc
```

### 10. Build the image

Plug the cameras into the USB3 ports, then:

```bash
isaac-ros activate --build-local
```

The first build takes a long time (librealsense is compiled from source). The final
`exporting layers` step can sit silently for several minutes: do not interrupt it.
Afterwards, `isaac-ros activate` (without `--build-local`) reuses the image.
Rebuild with `--build-local` only after changing `Dockerfile.cuvslam` or `config.yaml`.

Check inside the container:

```bash
echo $DISPLAY                                    # :1
ros2 pkg list | grep -iE "cuvslam|visual_slam"   # isaac_ros_cuvslam, isaac_ros_visual_slam_interfaces
rs-enumerate-devices | grep -E "Serial Number|Firmware Version|Usb Type"
```

### 11. D455 firmware 5.16.0.1

Isaac ROS 5.0 requires firmware **5.16.0.1** (our cameras shipped with 5.15.1.55).
Inside the container:

```bash
cd /workspaces/isaac_ros-dev
# get the real download link from the firmware page (the plain ?file= link returns HTML)
wget -O fw_page.html "https://dev.realsenseai.com/firmware-download/?file=41890"
grep -oE 'https://dev.realsenseai.com/download/41890/[^"]*' fw_page.html | head -1
wget --content-disposition "<link printed above>"
unzip Signed_Image_UVC_5_16_0_1.zip -d fw_5.16.0.1
rs-fw-update -l
rs-fw-update -s <serial> -f fw_5.16.0.1/Signed_Image_UVC_5_16_0_1.bin   # one camera at a time
rs-fw-update -l                                                          # check: 5.16.0.1
```

Alternatively, download *Version-5.16.0.1* from the
[D400 firmware page](https://dev.realsenseai.com/docs/firmware-releases-d400) with a browser.
Do not unplug the camera during the update.

---

## Daily routine

```bash
# host
sudo date -s "YYYY-MM-DD HH:MM:SS"   # only if the date is wrong
sudo jetson_clocks
isaac-ros activate                   # repeat in every new terminal to enter the same container
```

## Running cuVSLAM on a recorded bag

The bag must contain `/camX/infra1` **and** `/camX/infra2` (`image_rect_raw` + `camera_info`),
`/tf_static`, and `/camX/imu` for the IMU mode. Check with:

```bash
ros2 bag info <bag> | grep -E "infra[12]/(image|camera_info)|/imu |tf_static"
```

Bags recorded on Humble (`.db3`) play fine on Lyrical. Put them in `$ISAAC_ROS_WS/bags/`
(mounted in the container at `/workspaces/isaac_ros-dev/bags/`).

Three terminals, all inside the container, **in this order**:

```bash
# Terminal 1 - cuVSLAM (stereo; use cuvslam_stereo_imu_cam2.launch.py for stereo + IMU)
ros2 launch /workspaces/isaac_ros-dev/launch/cuvslam_stereo_cam2.launch.py
#   wait for: Loaded node '/visual_slam_node'

# Terminal 2 - RViz
rviz2 -d $(ros2 pkg prefix isaac_ros_cuvslam --share)/rviz/default.cfg.rviz

# Terminal 3 - bag
ros2 bag play /workspaces/isaac_ros-dev/bags/<bag>
```

Restart Terminal 1 before replaying a bag, so the trajectory starts from scratch.
In RViz, set **Fixed Frame** to `map` or `odom` if nothing is shown.

Expected log lines in Terminal 1: `Tracking mode: Multicamera` and `Enable IMU Fusion: false`
(stereo) or `true` (stereo + IMU).

### Notes

- `Delta between current and previous frame [...] is above threshold` means frames are
  missing in the recording (our bags run at ~14 fps with gaps of 150-200 ms). Tracking
  continues; raise `image_jitter_threshold_ms` to silence it.
- realsense-ros labels the infra2 `camera_info` with the infra1 frame. The launch files
  therefore set `camera_optical_frames` explicitly.
- The IMU noise parameters are NVIDIA's defaults, not calibrated for our D455s.
- cuVSLAM modes are exclusive: stereo (0), stereo + IMU (1), RGBD (2). There is no RGBD + IMU.
- With the IR emitter on, the projected dots degrade tracking: record with the emitter off.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `invoking the NVIDIA Container Runtime Hook directly (e.g. specifying the docker --gpus flag) is not supported` | Use the `nvidia` runtime (step 5), not `--gpus` |
| `cannot verify ... certificate ... not yet valid` | Wrong system date (step 3) |
| `timedatectl`: `System has not been booted with systemd` | You are inside the container: run it on the host |
| `xhost: unable to open display ":0"` | Our display is `:1` (step 9) |
| `echo $DISPLAY` empty in the container | Step 9 missing; exit and run `isaac-ros activate` again |
| `rviz2: error while loading shared libraries: libOgreMain...` | After installing rviz2 by hand: `source /opt/ros/lyrical/setup.bash` |
| `jtop` shows *JetPack NOT DETECTED* | Cosmetic: jetson-stats does not know 7.2 yet |
| Packages installed with `apt` in the container disappear | Add them to `Dockerfile.cuvslam` and rebuild with `--build-local` |