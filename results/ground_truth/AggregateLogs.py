import sys
import os
import csv
import re
from pathlib import Path

# Keys in the .log file that represent file paths and must be excluded
EXCLUDED_KEYS = {
    "Source File",
    "Target File",
    "Aligned path saved to"
}

def parse_log_file(log_path, subfolder_name):
    """
    Parses an ICP alignment .log file and extracts metrics, excluding file paths.
    Splits 'Target Points' into raw and densified counts and strips units ('m').
    """
    row_data = {"Subfolder": subfolder_name}

    with open(log_path, mode="r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Optional: capture the ICP mode from the header line if needed
            if line.startswith("===") and line.endswith("==="):
                mode = line.strip("=").replace("Completed", "").strip()
                row_data["Mode"] = mode
                continue

            if ":" in line:
                key, val = line.split(":", 1)
                key = key.strip()
                val = val.strip()

                if key in EXCLUDED_KEYS:
                    continue

                # Handle "4 raw -> 2001 densified" either as separate columns or raw string
                if key == "Target Points":
                    match = re.match(r"(\d+)\s*raw\s*->\s*(\d+)\s*densified", val)
                    if match:
                        row_data["Target Points (Raw)"] = match.group(1)
                        row_data["Target Points (Densified)"] = match.group(2)
                    else:
                        row_data[key] = val
                else:
                    # Strip trailing ' m' unit from error metrics for clean numeric CSV cells
                    if val.endswith(" m"):
                        val = val[:-2].strip()
                    row_data[key] = val

    return row_data

def main():
    if len(sys.argv) < 3:
        print("Usage: python aggregate_logs.py <input_root_directory> <output_file.csv>")
        sys.exit(1)

    base_dir = Path(sys.argv[1]).resolve()
    output_csv = Path(sys.argv[2]).resolve()

    if not base_dir.is_dir():
        print(f"Error: Directory '{base_dir}' does not exist.")
        sys.exit(1)

    subdirs = sorted([d for d in base_dir.iterdir() if d.is_dir()])
    if not subdirs:
        print(f"No subdirectories found in '{base_dir}'.")
        sys.exit(0)

    rows = []
    fieldnames = ["Subfolder"]

    for folder in subdirs:
        log_files = sorted(folder.glob("*.log"))
        if not log_files:
            continue

        for log_file in log_files:
            parsed = parse_log_file(log_file, folder.name)
            rows.append(parsed)
            for k in parsed.keys():
                if k not in fieldnames:
                    fieldnames.append(k)

    if not rows:
        print(f"No .log files found in any subdirectory of '{base_dir}'.")
        sys.exit(0)

    # Write aggregated table to CSV
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(output_csv, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
            delimiter=",",
            quoting=csv.QUOTE_NONE,
            escapechar="\\"
        )
        writer.writeheader()
        writer.writerows(rows)

    print(f"Processed {len(rows)} log file(s). Summary CSV saved to: {output_csv}")

if __name__ == "__main__":
    main()