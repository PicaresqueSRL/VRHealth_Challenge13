BAG_PATH="${1:-$HOME/bags/loop_closure}"

if [ ! -d "$BAG_PATH" ]; then
  echo "Errore: cartella bag non trovata: $BAG_PATH"
  echo "Uso: $0 /percorso/al/bag"
  exit 1
fi

source /opt/ros/lyrical/setup.bash

echo "Avvio riproduzione: $BAG_PATH"
ros2 bag play "$BAG_PATH" \
  --clock \
  --rate 0.3 \
  --topics \
    /cam2/color/image_raw \
    /cam2/aligned_depth_to_color/image_raw \
    /cam2/color/camera_info \
    /cam2/aligned_depth_to_color/camera_info \
    /cam2/imu \
    /tf_static
