#!/bin/bash
# all_db3_to_mcap.sh - Runs db3_to_mcap.sh on every capture in a folder, preserving the folder structure.
#
# Rules (same as convert_all.py):
#   * a "capture" is a folder that directly contains .db3 or .mcap files;
#   * if the capture has a converted/ subfolder containing a bag, that copy is used
#     (the output still goes to the capture path, without "converted");
#   * vslam*, converted, edex folders and the output folder are skipped.
#
# Usage:
#   ./all_db3_to_mcap.sh [options] [captures folder]
#     -o DIR        output folder (default: <captures>/mcap)
#     -l            only list the captures found
#     -s            skip captures already converted (destination exists)
#     -f            reconvert captures already converted (deletes the destination)
#     -k PATTERN    convert only captures whose relative path contains PATTERN (repeatable)
#
# Example:
#   ./all_db3_to_mcap.sh -o ~/dotlumen/Captures_mcap ~/dotlumen/Captures
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONVERTER="${HERE}/db3_to_mcap.sh"

out_root=""
list_only=0
skip_existing=0
force=0
only=()
while getopts "o:lsfk:h" opt; do
	case "${opt}" in
		o) out_root="${OPTARG}" ;;
		l) list_only=1 ;;
		s) skip_existing=1 ;;
		f) force=1 ;;
		k) only+=("${OPTARG}") ;;
		*) sed -n '2,19p' "$0"; exit 1 ;;
	esac
done
shift $((OPTIND - 1))

root="$(realpath "${1:-${HOME}/dotlumen/Captures}")"
out_root="$(realpath -m "${out_root:-${root}/mcap}")"

[ -d "${root}" ] || { echo "Captures folder not found: ${root}"; exit 1; }
[ -x "${CONVERTER}" ] || { echo "Script not found or not executable: ${CONVERTER} (chmod +x?)"; exit 1; }
if [ "${skip_existing}" -eq 1 ] && [ "${force}" -eq 1 ]; then
	echo "-s and -f are mutually exclusive"; exit 1
fi

has_bag() { compgen -G "$1/*.db3" > /dev/null || compgen -G "$1/*.mcap" > /dev/null; }

# --- capture search ------------------------------------------------------------
rels=(); srcs=(); kinds=()
while IFS= read -r -d '' dir; do
	[ "${dir}" = "${root}" ] && continue
	case "${dir}" in "${out_root}"|"${out_root}"/*) continue ;; esac
	rel="${dir#"${root}"/}"
	skip=0
	IFS='/' read -ra parts <<< "${rel}"
	for p in "${parts[@]}"; do
		case "${p}" in vslam*|converted|edex|mcap) skip=1 ;; esac
	done
	[ "${skip}" -eq 1 ] && continue
	if [ -d "${dir}/converted" ] && has_bag "${dir}/converted"; then
		src="${dir}/converted"; kind="converted"
	elif has_bag "${dir}"; then
		src="${dir}"; kind="original"
	else
		continue
	fi
	if [ "${#only[@]}" -gt 0 ]; then
		match=0
		for k in "${only[@]}"; do [[ "${rel}" == *"${k}"* ]] && match=1; done
		[ "${match}" -eq 0 ] && continue
	fi
	rels+=("${rel}"); srcs+=("${src}"); kinds+=("${kind}")
done < <(find "${root}" -type d -print0 | sort -z)

echo "Captures found: ${#rels[@]}   (output: ${out_root})"
for i in "${!rels[@]}"; do
	size=$(du -sh "${srcs[$i]}" 2>/dev/null | cut -f1)
	printf "  %-45s %-9s %6s\n" "${rels[$i]}" "${kinds[$i]}" "${size}"
done
[ "${list_only}" -eq 1 ] && exit 0
[ "${#rels[@]}" -eq 0 ] && exit 0

mkdir -p "${out_root}"
echo "Free space at destination: $(df -h --output=avail "${out_root}" | tail -1 | tr -d ' ')"

# --- conversion ------------------------------------------------------------------
summary="${out_root}/conversion_summary.tsv"
printf "capture\tsource\tkind\tstatus\tseconds\tsize_in\tsize_out\n" > "${summary}"
ok=0; failed=0; skipped=0
n=${#rels[@]}
for i in "${!rels[@]}"; do
	rel="${rels[$i]}"; src="${srcs[$i]}"; kind="${kinds[$i]}"
	dst="${out_root}/${rel}"
	log="${out_root}/${rel}.log"
	mkdir -p "$(dirname "${dst}")"
	if [ -e "${dst}" ]; then
		if [ "${skip_existing}" -eq 1 ]; then
			echo "[$((i + 1))/${n}] ${rel}: already converted, skipping"
			printf "%s\t%s\t%s\tskipped\t0\t\t\n" "${rel}" "${src}" "${kind}" >> "${summary}"
			skipped=$((skipped + 1)); continue
		elif [ "${force}" -eq 1 ]; then
			rm -rf "${dst}"
		else
			echo "[$((i + 1))/${n}] ${rel}: ${dst} exists (use -s to skip or -f to reconvert)"
			printf "%s\t%s\t%s\tdestination exists\t0\t\t\n" "${rel}" "${src}" "${kind}" >> "${summary}"
			failed=$((failed + 1)); continue
		fi
	fi
	echo -n "[$((i + 1))/${n}] ${rel} (${kind}) ... "
	t0=$(date +%s)
	"${CONVERTER}" "${src}" "${dst}" > "${log}" 2>&1
	rc=$?
	dt=$(( $(date +%s) - t0 ))
	size_in=$(du -sh "${src}" 2>/dev/null | cut -f1)
	size_out=$(du -sh "${dst}" 2>/dev/null | cut -f1)
	if [ "${rc}" -eq 0 ]; then
		status="ok"; ok=$((ok + 1))
		echo "ok in ${dt} s (${size_in} -> ${size_out})"
	else
		if grep -q "No cameras found" "${log}"; then status="no cameras"; else status="error"; fi
		failed=$((failed + 1))
		echo "${status} (rc ${rc}), see ${log}"
		tail -n 3 "${log}" | sed 's/^/      /'
	fi
	printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\n" "${rel}" "${src}" "${kind}" "${status}" "${dt}" "${size_in}" "${size_out}" >> "${summary}"
done

echo
echo "Done: ${ok} ok, ${failed} failed, ${skipped} skipped. Summary: ${summary}"
echo "Total size: sources $(printf '%s\0' "${srcs[@]}" | du -shc --files0-from=- | tail -1 | cut -f1), output $(du -sh "${out_root}" | cut -f1)"
