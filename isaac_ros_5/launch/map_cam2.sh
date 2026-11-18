#!/bin/bash
# map_cam2.sh - Runs Isaac ROS cuVSLAM (cam2 IR stereo + IMU) on one or more bags at real-time speed, records the
# output topics (as slam_cam2.sh) and SAVES THE cuVSLAM MAP of each run with the visual_slam/save_map service.
#
# Two ways to run it:
#   on the host (one throwaway container per bag, image with ROS 2 Lyrical + Isaac ROS 5.0):
#     ./map_cam2.sh <capture_name>=<bag_path> [...]
#   inside a container that already has ROS 2 + isaac_ros_cuvslam:
#     IN_CONTAINER=1 ./map_cam2.sh <capture_name>=<bag_path> [...]
#   <bag_path> is relative to the data folder (~/dotlumen on the host, /workspaces/isaac_ros-dev in the container),
#   e.g.  loop_closure=lyrical/workspaces/isaac_ros-dev/bags/loop_closure
#
# Environment variables:
#   IMAGE=cached_isaac_run_dev_image_local   Docker image (host mode)
#   DATA=$HOME/dotlumen                      data folder, mounted at /workspaces/isaac_ros-dev
#   OUT_ROOT=localization/isaac_maps         output folder (relative to the data folder)
#   SETTLE_S=5                               seconds to wait after the end of the playback before saving the map
#   FORCE=1                                  redo captures already done
#
# Output, for each capture (OUT_ROOT/<capture>/):
#   map/                 cuVSLAM map saved by Isaac ROS (data.mdb)
#   bag/                 MCAP recording of the cuVSLAM output topics
#   odometry.csv, vo_path.csv, slam_poses.csv, status.csv   extracted with extract_paths.py (same format as slam_csv)
#   run_info.json        parameters, versions, durations, map size, save time
#   launch.log, record.log, play.log, save_map.log, extract.log
#
# Uses cuvslam_stereo_imu_cam2_map.launch.py (same parameters as the slam_csv runs, plus slam_max_map_size large and
# visualization off). Does not modify slam_cam2.sh or the other launch files.
set -u

WS=/workspaces/isaac_ros-dev
SELF=$(readlink -f "$0")
LAUNCH_DIR=$(dirname "$SELF")

# ----------------------------------------------------------------------------- host mode: one container per bag
if [ "${IN_CONTAINER:-0}" != 1 ]; then
  IMAGE=${IMAGE:-cached_isaac_run_dev_image_local}
  DATA=$(realpath "${DATA:-$HOME/dotlumen}")
  case "$SELF" in "$DATA"/*) ;; *) echo "The script must be inside the data folder $DATA"; exit 1 ;; esac
  IN_SELF=$WS/${SELF#"$DATA"/}
  rc=0
  for spec in "$@"; do
    docker run --rm --gpus all --ipc=host --user "$(id -u):$(id -g)" \
      -e IN_CONTAINER=1 -e HOME=/tmp -e ROS_LOG_DIR=/tmp/roslog -e ROS_DOMAIN_ID=$((RANDOM % 100 + 100)) \
      -e LAUNCH="${LAUNCH:-}" -e OUT_ROOT="${OUT_ROOT:-localization/isaac_maps}" -e SETTLE_S="${SETTLE_S:-5}" -e FORCE="${FORCE:-0}" \
      -e IMAGE="$IMAGE" -v "$DATA:$WS" -w "$WS" "$IMAGE" bash "$IN_SELF" "$spec" || rc=$?
  done
  exit $rc
fi

# ----------------------------------------------------------------------------- container mode
set +u; source /opt/ros/*/setup.bash; set -u
LAUNCH=${LAUNCH:-$LAUNCH_DIR/cuvslam_stereo_imu_cam2_map.launch.py}
EXTRACT=$LAUNCH_DIR/extract_paths.py
OUT_ROOT=$WS/${OUT_ROOT:-localization/isaac_maps}
SETTLE_S=${SETTLE_S:-5}
FORCE=${FORCE:-0}

PLAY_TOPICS=(
  /cam2/infra1/image_rect_raw /cam2/infra1/camera_info
  /cam2/infra2/image_rect_raw /cam2/infra2/camera_info
  /cam2/imu /tf_static
)
RECORD_TOPICS=(
  /visual_slam/tracking/odometry
  /visual_slam/tracking/vo_path
  /visual_slam/tracking/slam_path
  /visual_slam/status
  /tf /tf_static
)

LAUNCH_PID=""
RECORD_PID=""

# stops a process group: SIGINT, then SIGTERM, then SIGKILL (as slam_cam2.sh)
stop_group() {
  local pid=$1
  [ -z "$pid" ] && return
  kill -0 "$pid" 2>/dev/null || return
  kill -INT -- -"$pid" 2>/dev/null
  for i in $(seq 60); do
    kill -0 "$pid" 2>/dev/null || return
    [ "$i" -eq 10 ] && kill -TERM -- -"$pid" 2>/dev/null
    sleep 0.5
  done
  echo "  WARNING: process $pid does not exit, SIGKILL"
  kill -KILL -- -"$pid" 2>/dev/null
}
cleanup() { stop_group "$RECORD_PID"; stop_group "$LAUNCH_PID"; }
trap 'echo; echo "Interrupted"; cleanup; exit 130' INT TERM

wait_for_node() {
  for _ in $(seq 60); do
    ros2 node list 2>/dev/null < /dev/null | grep -q visual_slam_node && return 0
    sleep 1
  done
  return 1
}

for spec in "$@"; do
  NAME=${spec%%=*}
  BAG=$WS/${spec#*=}
  DIR=$OUT_ROOT/$NAME
  if [ -f "$DIR/done" ] && [ "$FORCE" != 1 ]; then echo "[$NAME] already done, skipped"; continue; fi
  [ -e "$BAG" ] || { echo "[$NAME] ERROR: bag not found: $BAG"; continue; }
  rm -rf "$DIR"; mkdir -p "$DIR"
  T0=$(date +%s.%N)
  echo "[$NAME] cuVSLAM mapping run, bag $BAG"

  setsid ros2 launch "$LAUNCH" > "$DIR/launch.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
  if ! wait_for_node; then
    echo "[$NAME] ERROR: visual_slam_node not started, see $DIR/launch.log"; stop_group "$LAUNCH_PID"; continue
  fi
  sleep 3
  setsid ros2 bag record -s mcap -o "$DIR/bag" --topics "${RECORD_TOPICS[@]}" > "$DIR/record.log" 2>&1 < /dev/null &
  RECORD_PID=$!
  sleep 3
  if ! kill -0 "$RECORD_PID" 2>/dev/null; then
    echo "[$NAME] ERROR: ros2 bag record exited, see $DIR/record.log"; RECORD_PID=""; stop_group "$LAUNCH_PID"; continue
  fi

  echo "[$NAME] play (rate 1.0)"
  TP0=$(date +%s.%N)
  ros2 bag play "$BAG" --topics "${PLAY_TOPICS[@]}" > "$DIR/play.log" 2>&1 < /dev/null
  PLAY_RC=$?
  TP1=$(date +%s.%N)
  sleep "$SETTLE_S"

  echo "[$NAME] save_map -> $DIR/map"
  TS0=$(date +%s.%N)
  ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath \
    "{file_path: '$DIR/map'}" > "$DIR/save_map.log" 2>&1 < /dev/null
  SAVE_RC=$?
  TS1=$(date +%s.%N)
  SAVE_OK=false; grep -q "success=True" "$DIR/save_map.log" && SAVE_OK=true

  stop_group "$RECORD_PID"; RECORD_PID=""
  stop_group "$LAUNCH_PID"; LAUNCH_PID=""
  sleep 2

  EXTRACT_OUT=""
  if [ -f "$DIR/bag/metadata.yaml" ]; then
    EXTRACT_OUT=$(python3 "$EXTRACT" --bag "$DIR/bag" "$DIR" 2>&1 | tee "$DIR/extract.log" | tail -1)
  fi
  MAP_MB=$(du -sb "$DIR/map" 2>/dev/null | awk '{printf "%.2f", $1/1e6}')
  T1=$(date +%s.%N)

  python3 - "$DIR/run_info.json" <<EOF
import json, subprocess, sys
def pkgver(p):
    try:
        return subprocess.run(['ros2', 'pkg', 'xml', p, '--tag', 'version'], capture_output=True, text=True).stdout.strip()
    except Exception:
        return None
def cuvslam_version():
    import glob, re
    for f in glob.glob('/opt/ros/*/lib/libcuvslam.so'):
        m = re.search(rb'1[0-9]\.[0-9]+\.[0-9]+\+[0-9a-f]+', open(f, 'rb').read())
        if m:
            return m.group().decode()
info = {
    "script": "map_cam2.sh", "capture": "$NAME", "bag": "$BAG", "launch": "$LAUNCH",
    "image": "${IMAGE:-}", "ros_distro": "${ROS_DISTRO:-}",
    "isaac_ros_cuvslam_version": pkgver("isaac_ros_cuvslam"), "cuvslam_version": cuvslam_version(),
    "play_topics": "${PLAY_TOPICS[*]}".split(), "record_topics": "${RECORD_TOPICS[*]}".split(),
    "play_rc": $PLAY_RC, "play_s": round($TP1 - $TP0, 2), "settle_s": $SETTLE_S,
    "save_map_rc": $SAVE_RC, "save_map_ok": $( [ "$SAVE_OK" = true ] && echo True || echo False ),
    "save_map_s": round($TS1 - $TS0, 2), "map_dir": "map", "map_size_mb": float("${MAP_MB:-0}" or 0),
    "extract": "$EXTRACT_OUT", "total_s": round($T1 - $T0, 2),
}
json.dump(info, open(sys.argv[1], "w"), indent=2)
EOF

  if [ "$SAVE_OK" = true ] && [ -f "$DIR/map/data.mdb" ] && [ -f "$DIR/slam_poses.csv" ]; then
    touch "$DIR/done"
    echo "[$NAME] ok: map ${MAP_MB} MB, $EXTRACT_OUT"
  else
    echo "[$NAME] ERROR: play rc=$PLAY_RC, save_map rc=$SAVE_RC ok=$SAVE_OK, see the logs in $DIR"
  fi
done
