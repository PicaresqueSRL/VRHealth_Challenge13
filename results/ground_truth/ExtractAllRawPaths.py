import subprocess
import sys
from pathlib import Path

def main():
    script_dir = Path(__file__).resolve().parent
    worker_script = script_dir / "ExtractRawPathsFromSvg.py"

    if not worker_script.is_file():
        print(f"Error: '{worker_script.name}' not found in {script_dir}")
        sys.exit(1)

    svg_files = [f for f in script_dir.iterdir() if f.is_file() and f.suffix.lower() == ".svg"]

    if not svg_files:
        print("No SVG files found in the current folder.")
        return

    print(f"Found {len(svg_files)} SVG file(s). Starting batch processing...\n")

    success_count = 0
    fail_count = 0

    for svg in sorted(svg_files):
        print(f"Processing: {svg.name} ...")
        result = subprocess.run([sys.executable, str(worker_script), str(svg)])
        
        if result.returncode == 0:
            success_count += 1
        else:
            fail_count += 1
            print(f"--> Failed processing {svg.name}")

    print("\n----------------------------------------")
    print(f"Batch completed: {success_count} succeeded, {fail_count} failed.")

if __name__ == "__main__":
    main()