#!/bin/bash
# Fa girare cuVSLAM su ogni bag di bench_list.txt e registra l'output in MCAP.
#
# Configurazione: multi-camera cam1 + cam2 stereo IR, senza IMU (tracking_mode 0).
# cuVSLAM non supporta la IMU con piu' di una coppia stereo.
#
# Uso: ./slam_cam1_cam2.sh
#
# Opzioni (variabili d'ambiente):
#   BENCH_LIST=bench_list.txt   un path di bag per riga (righe vuote e # ignorate)
#   OUT_ROOT=slam_runs          cartella di output
#   FORCE=1                     rifà anche i bag già completati
#
# Output, per ogni bag:
#   $OUT_ROOT/<config>/<bag>/bag/        registrazione MCAP dell'output di cuVSLAM
#   $OUT_ROOT/<config>/<bag>/rerun/      MCAP da aprire in Rerun: point cloud, odometria, path
#   $OUT_ROOT/<config>/<bag>/*.log       log di launch, record, play e conversione
#
# Richiede to_rerun_mcap.py nella stessa cartella dello script (nessun pip: usa rosbag2_py).

set -u

WS=/workspaces/isaac_ros-dev
LAUNCH_DIR=$WS/VRHealth_Challenge13/isaac_ros_5/launch
BENCH_LIST=${BENCH_LIST:-$WS/bench_list.txt}
OUT_ROOT=${OUT_ROOT:-$WS/slam_runs}
FORCE=${FORCE:-0}
# script che converte la registrazione in un MCAP leggibile da Rerun (stessa cartella di questo script)
RERUN_SCRIPT=$(dirname "$(readlink -f "$0")")/to_rerun_mcap.py

CAM2_TOPICS=(
  /cam2/infra1/image_rect_raw /cam2/infra1/camera_info
  /cam2/infra2/image_rect_raw /cam2/infra2/camera_info
)
CAM1_TOPICS=(
  /cam1/infra1/image_rect_raw /cam1/infra1/camera_info
  /cam1/infra2/image_rect_raw /cam1/infra2/camera_info
)

CONFIG=cam1_cam2_stereo
LAUNCH=$LAUNCH_DIR/cuvslam_stereo_cam1_cam2.launch.py
LAUNCH_ARGS=(sync_ms:=35.0)
PLAY_TOPICS=("${CAM2_TOPICS[@]}" "${CAM1_TOPICS[@]}" /tf_static)

# topic di output di cuVSLAM da registrare
RECORD_TOPICS=(
  /visual_slam/tracking/odometry
  /visual_slam/tracking/vo_path
  /visual_slam/tracking/slam_path
  /visual_slam/status
  /tf /tf_static
)

[ -f "$LAUNCH" ] || { echo "Launch non trovato: $LAUNCH"; exit 1; }
[ -f "$BENCH_LIST" ] || { echo "Lista bag non trovata: $BENCH_LIST"; exit 1; }

LAUNCH_PID=""
RECORD_PID=""

# ferma un processo e tutti i suoi figli (gruppo): SIGINT, poi SIGTERM, poi SIGKILL.
# I processi lanciati in background da uno script ignorano SIGINT, per questo serve SIGTERM.
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
  echo "  ATTENZIONE: processo $pid non si chiude, SIGKILL"
  kill -KILL -- -"$pid" 2>/dev/null
}

cleanup() {
  stop_group "$RECORD_PID"
  stop_group "$LAUNCH_PID"
}
trap 'echo; echo "Interrotto"; cleanup; exit 130' INT TERM

wait_for_node() {
  for _ in $(seq 60); do
    ros2 node list 2>/dev/null < /dev/null | grep -q visual_slam_node && return 0
    sleep 1
  done
  return 1
}

while read -r BAG; do
  NAME=$(basename "${BAG%/}")
  NAME=${NAME%.mcap}
  DIR=$OUT_ROOT/$CONFIG/$NAME

  if [ -f "$DIR/done" ] && [ "$FORCE" != 1 ]; then
    echo "[$NAME] già fatto, salto"
    continue
  fi
  rm -rf "$DIR"
  mkdir -p "$DIR"
  echo "[$NAME] avvio cuVSLAM ($CONFIG)"

  setsid ros2 launch "$LAUNCH" "${LAUNCH_ARGS[@]}" > "$DIR/launch.log" 2>&1 < /dev/null &
  LAUNCH_PID=$!
  if ! wait_for_node; then
    echo "[$NAME] ERRORE: visual_slam_node non partito, vedi $DIR/launch.log"
    stop_group "$LAUNCH_PID"
    continue
  fi
  sleep 3

  setsid ros2 bag record -s mcap -o "$DIR/bag" --topics "${RECORD_TOPICS[@]}" \
    --regex '^/visual_slam/vis/.*' \
    > "$DIR/record.log" 2>&1 < /dev/null &
  RECORD_PID=$!
  sleep 3
  if ! kill -0 "$RECORD_PID" 2>/dev/null; then
    echo "[$NAME] ERRORE: ros2 bag record si è chiuso, vedi $DIR/record.log"
    tail -5 "$DIR/record.log"
    RECORD_PID=""
    stop_group "$LAUNCH_PID"; LAUNCH_PID=""
    continue
  fi

  echo "[$NAME] play"
  ros2 bag play "$BAG" --topics "${PLAY_TOPICS[@]}" > "$DIR/play.log" 2>&1 < /dev/null
  PLAY_RC=$?

  # lascia a cuVSLAM il tempo di pubblicare gli ultimi frame
  sleep 5
  stop_group "$RECORD_PID"; RECORD_PID=""
  stop_group "$LAUNCH_PID"; LAUNCH_PID=""
  sleep 2

  if [ $PLAY_RC -ne 0 ]; then
    echo "[$NAME] ERRORE nel play, vedi $DIR/play.log"
  elif [ -f "$DIR/bag/metadata.yaml" ] && ls "$DIR"/bag/*.mcap > /dev/null 2>&1; then
    touch "$DIR/done"
    echo "[$NAME] ok -> $DIR/bag"
    # Rerun non visualizza Odometry e Path: si crea un secondo MCAP con tipi che Rerun legge
    if python3 "$RERUN_SCRIPT" "$DIR/bag" "$DIR/rerun" > "$DIR/rerun.log" 2>&1; then
      echo "[$NAME] rerun -> $(ls "$DIR"/rerun/*.mcap)"
    else
      echo "[$NAME] ATTENZIONE: MCAP per Rerun non generato, vedi $DIR/rerun.log"
    fi
  else
    echo "[$NAME] ERRORE: registrazione non chiusa correttamente (manca metadata.yaml), vedi $DIR/record.log"
  fi
done < <(grep -v -e '^\s*$' -e '^\s*#' "$BENCH_LIST")

echo "Finito: $OUT_ROOT/$CONFIG"