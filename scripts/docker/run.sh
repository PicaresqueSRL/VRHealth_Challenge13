#!/bin/bash
# run.sh - Starts the vrhealth-cuvslam container with GPU access, the Rerun viewer on the host display (X11)
# and the data folder mounted at the same path as on the host, so that the absolute paths written in
# bag_report.json and run_info.json stay valid inside the container.
#
# Usage:
#   docker/run.sh                                   interactive shell
#   docker/run.sh <command> [args ...]              run one command, e.g.
#   docker/run.sh track_bag_rerun.py ~/dotlumen/Captures/9_Loop_closure/loop_closure --mode inertial --slam
#
# Environment variables:
#   IMAGE=vrhealth-cuvslam      image name
#   DATA=$HOME/dotlumen         data folder mounted at the same path (captures, results, repository)
#   NO_X11=1                    do not share the display (use --save / --no-viewer in the tracking scripts)
#   EXTRA_ARGS="..."            additional docker run arguments (e.g. "-v /mnt/disk:/mnt/disk")
set -euo pipefail

IMAGE=${IMAGE:-vrhealth-cuvslam}
DATA=${DATA:-$HOME/dotlumen}
SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -d "${DATA}" ] || { echo "Data folder not found: ${DATA} (set DATA=...)"; exit 1; }
DATA="$(realpath "${DATA}")"

args=(--rm --gpus all --ipc=host
      --user "$(id -u):$(id -g)"
      -e HOME="${HOME}" -e XDG_CACHE_HOME=/tmp/.cache -e MPLCONFIGDIR=/tmp/.matplotlib
      -v "${DATA}:${DATA}"
      -v "${SCRIPTS}:/opt/vrhealth/scripts")
[ -t 0 ] && [ -t 1 ] && args+=(-it)

# Rerun viewer on the host display
if [ "${NO_X11:-0}" != 1 ] && [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
	command -v xhost > /dev/null && xhost "+SI:localuser:$(id -un)" > /dev/null 2>&1 || true
	args+=(-e DISPLAY="${DISPLAY}" -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
	[ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ] && args+=(-e XAUTHORITY=/tmp/.Xauthority -v "${XAUTHORITY}:/tmp/.Xauthority:ro")
fi

# start in the current folder when it is inside the data folder, otherwise in the scripts folder
case "${PWD}/" in
	"${DATA}"/*) args+=(-w "${PWD}") ;;
	*) args+=(-w /opt/vrhealth/scripts) ;;
esac

# shellcheck disable=SC2206
[ -n "${EXTRA_ARGS:-}" ] && args+=(${EXTRA_ARGS})

exec docker run "${args[@]}" "${IMAGE}" "${@:-bash}"
