BAG_NAME="${1:-loop_closure}"
OUTPUT_DIR="$HOME/bags"
OUTPUT_NAME="${BAG_NAME}_slam"

source /opt/ros/lyrical/setup.bash

echo "Registrazione SLAM in: $OUTPUT_DIR/$OUTPUT_NAME"
ros2 bag record \
  -o "$OUTPUT_DIR/$OUTPUT_NAME" \
  -s sqlite3 \
  --use-sim-time \
  --topics \
    /odom \
    /odom_info \
    /global_pose \
    /map \
    /cloud_map \
    /mapGraph \
    /mapPath \
    /tf \
    /tf_static
