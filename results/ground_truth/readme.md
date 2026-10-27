# SLAM vs. Ground Truth Trajectory Alignment & SVG Visualization Toolkit

This repository provides a suite of Python scripts to extract 2D/3D ground-truth trajectories from floorplan SVG files, align 3D SLAM odometry trajectories onto the ground truth using a bidirectional Iterative Closest Point (ICP) algorithm, log quantitative trajectory error metrics, and visually overlay the aligned SLAM paths onto the original SVG floorplans.

---

## Overview of the Automated Pipeline (`RunBatchPipeline.py`)

`RunBatchPipeline.py` automates the comparison between SLAM trajectories and ground-truth floorplan paths across multiple dataset folders:

1. **Directory Matching**: It scans all subdirectories inside a **Target Root Folder** (which holds the ground-truth `paths3d.csv` and the floorplan `.svg`) and pairs each subfolder with the identically named subdirectory inside a **Source Root Folder** (which holds the SLAM trajectory `slam_poses.csv`).


2. **Trajectory Alignment (`IcpAlign.py`)**: It aligns the SLAM path (`slam_poses.csv`) to the ground-truth path (`paths3d.csv`) using rigid ICP (`--no-scale`), outputting the transformed coordinates to `aligned_odometry.csv` and writing detailed alignment error metrics to `alignment_results.log`.


3. **Visual SVG Overlay (`AddAlignedOdometryToSvg.py`)**: It projects the aligned SLAM trajectory back into the metric scale of the original floorplan SVG and adds a new `odometry` layer containing a green polyline (`#00FF00`), saving the visual comparison as `<input_svg_stem>_with_odometry.svg`.



---

## Scripts Reference & Usage

### 1. `ExtractRawPathsFromSvg.py`

Extracts 2D ground-truth paths from an SVG file and converts coordinates from SVG pixels to meters.

* **How it works**:
* Reads the reference line inside the `MISURA` layer and equates its Euclidean pixel length to **5.0 meters** to compute the `meters_per_unit` scale factor.


* Iterates through each `<g>` group (subpath) inside the `PERCORSI` layer.


* Identifies the starting marker in each subpath by finding the circle (`<circle>`, `<ellipse>`, or closed curve `<path>`) with a unique CSS `class`, `fill`, or `style` attribute.


* Orders all `<line>`, `<polyline>`, and `<path>` vertices sequentially starting from the endpoint closest to the starting circle, assigning an incremental `SUBPATH` ID starting from `0`.


* Exports an unquoted, comma-delimited CSV (`<input_name>.csv`) with columns `X,Y,SUBPATH`.




* **Usage**:
```bash
python ExtractRawPathsFromSvg.py <input_file.svg>

```



### 2. `ExtractAllRawPaths.py`

A cross-platform batch runner that executes `ExtractRawPathsFromSvg.py` on every `.svg` file in the current directory.

* **Usage**:
```bash
python ExtractAllRawPaths.py

```



### 3. `3DifyPath.py`

Converts a 2D path CSV into a 3D path CSV by inserting a `Z` column filled with `0` values right before the `SUBPATH` column (or at column index 2 if `SUBPATH` is absent).

* **Usage**:
```bash
# Overwrite the file in-place:
python 3DifyPath.py <input_file.csv>

# Or save to a new file:
python 3DifyPath.py <input_file.csv> <output_file.csv>

```



### 4. `IcpAlign.py`

Aligns a 3D source path CSV (`X,Y,Z`) onto a 3D target path CSV (`X,Y,Z`) using global trajectory initialization followed by Iterative Closest Point (ICP) refinement. Preserves any additional columns (such as `SUBPATH`) in the output CSV and writes the error summary to a `.log` file.

* **Arguments & Options**:
* `source`: Path to the source CSV file (e.g., `slam_poses.csv`).


* `target`: Path to the target CSV file (e.g., `paths3d.csv`).


* `output`: Path for the aligned output CSV file.


* `log_file`: Path for the output `.log` file (automatically appends `.log` if omitted).


* `--no-scale`: Disables scale estimation, performing rigid 6-DoF rotation and translation only ($s = 1.0$).


* `--target-step`: Maximum spacing in meters between interpolated target points (default: `0.02`).


* `--min-target-points`: Minimum total points on the densified target path (default: `2000`).




* **Usage**:
```bash
# Scaled ICP alignment:
python IcpAlign.py slam_poses.csv paths3d.csv aligned_odometry.csv alignment_results.log

# Rigid ICP alignment (no scaling):
python IcpAlign.py slam_poses.csv paths3d.csv aligned_odometry.csv alignment_results.log --no-scale

```



### 5. `AddAlignedOdometryToSvg.py`

Overlays a 3D metric trajectory CSV onto an existing floorplan SVG inside a dedicated `odometry` layer.

* **How it works**:
* Computes the `units_per_meter` conversion factor from the 5.0-meter line in the `MISURA` layer of the input SVG.


* Reads `X`, `Y`, `Z` (and optional `SUBPATH`) from the CSV, converts `X` and `Y` from meters to SVG pixel units, and draws a green `<polyline>` (`stroke="#00FF00"`, `stroke-width="2"`) for each subpath inside a new `<g id="odometry">` layer.




* **Usage**:
```bash
python AddAlignedOdometryToSvg.py <input.svg> <aligned_odometry.csv> <output_with_odometry.svg>

```



### 6. `RunBatchPipeline.py`

Cycles through subdirectories to run `IcpAlign.py` and `AddAlignedOdometryToSvg.py` in batch mode.

* **Expected Directory Layout**:
* **Target Subfolder**: `<target_root>/<sequence_name>/paths3d.csv` and `<floorplan>.svg`.


* **Source Subfolder**: `<source_root>/<sequence_name>/slam_poses.csv`.


* **Generated Outputs (saved in Target Subfolder)**: `aligned_odometry.csv`, `alignment_results.log`, and `<floorplan>_with_odometry.svg`.




* **Usage**:
```bash
# Specify both Target Root and Source Root folders:
python RunBatchPipeline.py /path/to/target_root /path/to/source_root

# Run in a single root folder where source and target files share the same subdirectories:
python RunBatchPipeline.py /path/to/root_folder

```



### 7. `Paths3DToSVG.py`

Generates a standalone vector plot (`<input_name>_plots.svg`) showing side-by-side **Top View (XY)** and **Front View (XZ)** projections of a 3D CSV trajectory.

* **Features**:
* Automatically repairs malformed rows where decimal commas were accidentally split into extra columns.


* Groups trajectories by `SUBPATH`, marking the starting vertex of each subpath with a red circle (`#FF0000`) and subsequent vertices with color-coded circles and polylines.




* **Usage**:
```bash
python Paths3DToSVG.py <input_file.csv>

```



---

## ICP Algorithm & Error Metrics (`IcpAlign.py`)

### How the Alignment Algorithm Works

1. **Subpath-Aware Target Densification (`densify_path`)**:
Ground-truth paths extracted from SVG floorplans typically contain only sparse corner vertices. To prevent ICP from pulling SLAM points toward corners instead of along walls and corridors, the target path is linearly interpolated within each `SUBPATH` so that vertices are spaced at most `0.02 m` apart and contain at least `2,000` points.


2. **Arc-Length & Multi-Orientation Global Initialization**:
* Both source and target trajectories are resampled into `500` equidistant points along their cumulative arc length (`resample_by_arclength`) to form an initial 1-to-1 ordered correspondence.


* The script tests `6` base 3D axis permutations/flips across `36` yaw angles ($10^\circ$ increments around the Z-axis) to bridge coordinate frame differences between 3D camera SLAM odometry and 2D floorplan coordinates, selecting the orientation that minimizes bidirectional nearest-neighbor loss.




3. **Non-Collapsing Bidirectional ICP Refinement**:
* In each iteration (up to `150` iterations or until convergence within `1e-7`), the algorithm finds both **forward nearest neighbors** (each source point to the closest target point) and **backward nearest neighbors** (each target point to the closest source point).


* Combining forward and backward correspondences forces the aligned trajectory to span the entire ground-truth path and prevents it from collapsing onto a single cluster.


* Rotation $R$ is solved via Singular Value Decomposition (SVD) of the cross-covariance matrix. When scaling is enabled, the scale factor $s$ is computed from the ratio of root-mean-square spatial spreads around the centroids ($\sigma_{\text{dst}} / \sigma_{\text{src}}$) and clamped to $[0.5\times, 2.0\times]$ of the initial scale estimate.





### Error Metrics Logged in `alignment_results.log`

Given $N_{\text{src}}$ aligned source points $P'_i = s R P_i + t$ and $N_{\text{dst}}$ densified target points $Q_j$, let $d(P'_i, Q) = \min_j \Vert{} P'_i - Q_j \Vert{}_2$ denote the Euclidean distance from $P'_i$ to its closest target point, and $d(Q_j, P') = \min_i \Vert{} Q_j - P'_i \Vert{}_2$ denote the distance from $Q_j$ to its closest aligned source point. Three error metrics (in meters) are recorded in the `.log` file:

* **Point-to-Target RMSE (Root Mean Square Error)**:



$$\text{RMSE} = \sqrt{\frac{1}{N_{\text{src}}} \sum_{i=1}^{N_{\text{src}}} d(P'_i, Q)^2}$$



Measures how closely the aligned SLAM trajectory adheres to the ground-truth path, heavily weighting larger outliers.


* **Symmetric Chamfer RMSE**:



$$\text{Symmetric RMSE} = \sqrt{\frac{1}{2} \left( \frac{1}{N_{\text{src}}} \sum_{i=1}^{N_{\text{src}}} d(P'_i, Q)^2 + \frac{1}{N_{\text{dst}}} \sum_{j=1}^{N_{\text{dst}}} d(Q_j, P')^2 \right)}$$



Evaluates bidirectional alignment quality by penalizing both deviations of the SLAM path from the ground truth and any uncovered segments of the ground-truth trajectory.


* **Mean Euclidean Error (MAE)**:



$$\text{MAE} = \frac{1}{N_{\text{src}}} \sum_{i=1}^{N_{\text{src}}} d(P'_i, Q)$$



The average straight-line distance (in meters) between each aligned SLAM pose and the ground-truth floorplan trajectory.



---

## Requirements

* **Python 3.8+**
* **Dependencies**:
```bash
pip install numpy pandas

```