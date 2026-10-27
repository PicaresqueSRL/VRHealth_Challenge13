import subprocess
import sys
import os
from pathlib import Path

def main():
    # Parse command line arguments:
    # 1st arg: Target root folder (defaults to current directory '.')
    # 2nd arg: Source root folder (defaults to target root folder if omitted)
    if len(sys.argv) >= 2:
        target_base_dir = Path(sys.argv[1]).resolve()
    else:
        target_base_dir = Path(__file__).resolve().parent

    if len(sys.argv) >= 3:
        source_base_dir = Path(sys.argv[2]).resolve()
    else:
        source_base_dir = target_base_dir

    if not target_base_dir.is_dir():
        print(f"Error: Target directory '{target_base_dir}' does not exist.")
        sys.exit(1)

    if not source_base_dir.is_dir():
        print(f"Error: Source directory '{source_base_dir}' does not exist.")
        sys.exit(1)

    # Find all subdirectories in the target base directory
    target_subdirs = [d for d in target_base_dir.iterdir() if d.is_dir()]

    if not target_subdirs:
        print(f"No subdirectories found in target directory '{target_base_dir}'.")
        return

    print(f"Target Root Directory : {target_base_dir}")
    print(f"Source Root Directory : {source_base_dir}")
    print(f"Found {len(target_subdirs)} subdirectory/directories. Starting batch pipeline...\n")

    success_count = 0
    fail_count = 0

    for target_folder in sorted(target_subdirs):
        folder_name = target_folder.name
        print(f"--------------------------------------------------")
        print(f"Processing directory: {folder_name}")

        # Construct the matching source folder path using the same last directory name
        source_folder = source_base_dir / folder_name

        # Expected files
        source_csv = source_folder / "slam_poses.csv"
        target_csv = target_folder / "paths3d.csv"
        
        # Outputs and intermediates saved inside the target folder structure
        aligned_csv = target_folder / "aligned_odometry.csv"
        log_file = target_folder / "alignment_results.log"

        # Look for any .svg file inside the target subfolder (excluding already processed ones)
        svg_files = list(target_folder.glob("*.svg"))
        svg_files = [svg for svg in svg_files if "_with_odometry" not in svg.name]

        # Validation checks
        if not source_csv.is_file():
            print(f"[{folder_name}] Skipped: Missing 'source.csv' in source path ({source_csv}).")
            fail_count += 1
            continue

        if not target_csv.is_file():
            print(f"[{folder_name}] Skipped: Missing 'target.csv' in target path ({target_csv}).")
            fail_count += 1
            continue

        if not svg_files:
            print(f"[{folder_name}] Skipped: No input SVG file found in target folder.")
            fail_count += 1
            continue

        input_svg = svg_files[0]
        output_svg = target_folder / f"{input_svg.stem}_with_odometry.svg"

        try:
            # Step 1: Run icp_align.py
            print(f"[{folder_name}] Running ICP alignment...")
            icp_cmd = [
                sys.executable, "IcpAlign.py",
                str(source_csv),
                str(target_csv),
                str(aligned_csv),
                str(log_file)
                ,"--no-scale"                
            ]
            result_icp = subprocess.run(icp_cmd, capture_output=True, text=True)

            if result_icp.returncode != 0:
                print(f"[{folder_name}] ICP Alignment failed:\n{result_icp.stderr}")
                fail_count += 1
                continue

            # Step 2: Run add_odometry_to_svg.py
            print(f"[{folder_name}] Adding odometry layer to SVG...")
            svg_cmd = [
                sys.executable, "AddAlignedOdometryToSvg.py",
                str(input_svg),
                str(aligned_csv),
                str(output_svg)
            ]
            result_svg = subprocess.run(svg_cmd, capture_output=True, text=True)

            if result_svg.returncode != 0:
                print(f"[{folder_name}] Adding SVG odometry failed:\n{result_svg.stderr}")
                fail_count += 1
                continue

            print(f"[{folder_name}] Successfully processed!")
            success_count += 1

        except Exception as e:
            print(f"[{folder_name}] Error during processing: {e}")
            fail_count += 1

    print("\n==================================================")
    print(f"Batch Pipeline Finished: {success_count} succeeded, {fail_count} failed.")

if __name__ == "__main__":
    main()