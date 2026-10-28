#!/bin/bash
# Converts a ROS 2 bag (.db3) to compressed MCAP, keeping only the topics cuVSLAM needs.
# Usage: ./db3_to_mcap.sh <source bag folder> <destination folder>
set -euo pipefail

CAMS=('cam1' 'cam2')

INCLUDED_TOPICS=('color/image_raw' 'color/camera_info'                                   # RGB
                 'infra1/image_rect_raw' 'infra1/camera_info'                            # Infra1
                 'infra2/image_rect_raw' 'infra2/camera_info'                            # Infra2
                 'aligned_depth_to_color/image_raw' 'aligned_depth_to_color/camera_info'  # Depth
                 'imu'                                                                    # IMU
)
GLOBAL_TOPICS=('/tf_static')   # camera/IMU extrinsics: required by bag_to_cuvslam.py

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

command -v rosbags-convert > /dev/null 2>&1 || {
	echo "rosbags-convert not found. Install it with 'pip install rosbags'"; exit 1; }

if [ $# -ne 2 ]; then
	echo "Usage: $0 <source bag> <destination folder>"
	exit 1
fi

source_bag="$(realpath "$1")"
dest_folder="$2"

if [ -e "${dest_folder}" ]; then
	echo "Destination ${dest_folder} already exists: rosbags-convert does not overwrite"; exit 1
fi

topics=("${GLOBAL_TOPICS[@]}")
for cam in "${CAMS[@]}"; do
	for t in "${INCLUDED_TOPICS[@]}"; do
		topics+=("/${cam}/${t}")
	done
done

# Some bags have a metadata.yaml pointing to a .db3 whose name differs from the file actually present
# (e.g. low_detail_0.db3 instead of low_detail_0-003.db3), and rosbags cannot open them.
# In that case a temporary folder is created with a link from the expected name to the real file.
src="${source_bag}"
tmp=""
cleanup() { [ -n "${tmp}" ] && rm -rf "${tmp}"; }
trap cleanup EXIT
if [ -f "${source_bag}/metadata.yaml" ]; then
	mapfile -t listed < <(grep -oE '[^[:space:]"'"'"'-][^[:space:]"'"'"']*\.db3' "${source_bag}/metadata.yaml" | sort -u)
	missing=0
	for f in "${listed[@]}"; do [ -e "${source_bag}/${f}" ] || missing=1; done
	if [ "${missing}" -eq 1 ]; then
		mapfile -t real < <(find "${source_bag}" -maxdepth 1 -name '*.db3' -printf '%f\n' | sort)
		if [ "${#listed[@]}" -ne 1 ] || [ "${#real[@]}" -ne 1 ]; then
			echo "metadata.yaml does not match the .db3 files and they cannot be paired automatically:"
			echo "  in metadata.yaml: ${listed[*]}"; echo "  in the folder:    ${real[*]}"; exit 1
		fi
		tmp="$(mktemp -d)"
		cp "${source_bag}/metadata.yaml" "${tmp}/"
		ln -s "${source_bag}/${real[0]}" "${tmp}/${listed[0]}"
		echo "metadata.yaml points to ${listed[0]}, using ${real[0]}"
		src="${tmp}"
	fi
fi

# --compress-mode storage: MCAP internal compression (zstd chunks), readable by
# bag_to_cuvslam.py and Rerun. The default 'file' would produce a .mcap.zstd, while 'message'
# would compress individual messages: bag_to_cuvslam.py can read neither.
rosbags-convert --src "${src}" --dst "${dest_folder}" --dst-storage mcap \
	--compress zstd --compress-mode storage --include-topic "${topics[@]}"

# Check: the reduced bag must still be convertible to EDEX (without writing images)
if [ -f "${HERE}/bag_to_cuvslam.py" ]; then
	echo "Checking with bag_to_cuvslam.py --dry-run ..."
	python3 "${HERE}/bag_to_cuvslam.py" "${dest_folder}" -o "$(mktemp -d)" --dry-run --reference auto
fi
du -sh "${source_bag}" "${dest_folder}"
