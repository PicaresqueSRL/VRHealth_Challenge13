BAG_DIR="${1:-$HOME/bags}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker run -it --rm \
  --network host \
  -v "$BAG_DIR":/bags \
  -v "$SCRIPT_DIR/rtabmap_cam2.launch.py":/launch/rtabmap_cam2.launch.py \
  -v "$SCRIPT_DIR/rtabmap_multicam.launch.py":/launch/rtabmap_multicam.launch.py \
  introlab3it/rtabmap_ros:humble-latest \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    ros2 launch /launch/rtabmap_cam2.launch.py
  "
