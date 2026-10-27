import sys
import os
import csv
import argparse
import numpy as np

def load_xyz_from_csv(file_path):
    """Loads X, Y, Z columns from a CSV file along with SUBPATH IDs and extra columns."""
    points = []
    subpaths = []
    extra_cols = []
    extra_headers = []

    with open(file_path, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f, delimiter=',')
        header = next(reader, None)
        if not header:
            raise ValueError(f"CSV file '{file_path}' is empty.")

        header_upper = [col.strip().upper() for col in header]
        try:
            x_idx = header_upper.index('X')
            y_idx = header_upper.index('Y')
            z_idx = header_upper.index('Z')
        except ValueError:
            raise ValueError(f"File '{file_path}' must contain 'X', 'Y', and 'Z' columns.")

        subpath_idx = header_upper.index('SUBPATH') if 'SUBPATH' in header_upper else None
        other_indices = [i for i in range(len(header)) if i not in (x_idx, y_idx, z_idx)]
        extra_headers = [header[i] for i in other_indices]

        for row in reader:
            if not row:
                continue
            x = float(row[x_idx])
            y = float(row[y_idx])
            z = float(row[z_idx])
            points.append([x, y, z])
            sub_id = row[subpath_idx].strip() if subpath_idx is not None else '0'
            subpaths.append(sub_id)
            extra_cols.append([row[i] for i in other_indices])

    return np.array(points, dtype=np.float64), subpaths, extra_headers, extra_cols

def densify_path(points, subpaths=None, max_step=0.02, min_total_points=2000):
    """
    Densifies a 3D trajectory by linearly interpolating along each segment
    within the same subpath.
    """
    if len(points) < 2:
        return points

    if subpaths is None:
        subpaths = ['0'] * len(points)

    total_length = 0.0
    valid_segments = []
    for i in range(len(points) - 1):
        if subpaths[i] == subpaths[i + 1]:
            p1, p2 = points[i], points[i + 1]
            seg_len = np.linalg.norm(p2 - p1)
            if seg_len > 1e-9:
                total_length += seg_len
                valid_segments.append((p1, p2, seg_len))

    if not valid_segments:
        return points

    effective_step = min(max_step, total_length / max(min_total_points, 1))

    dense_points = []
    for p1, p2, seg_len in valid_segments:
        num_samples = max(int(np.ceil(seg_len / effective_step)), 2)
        t_values = np.linspace(0.0, 1.0, num_samples, endpoint=False)
        seg_pts = p1[np.newaxis, :] + t_values[:, np.newaxis] * (p2 - p1)[np.newaxis, :]
        dense_points.append(seg_pts)

    dense_points.append(valid_segments[-1][1][np.newaxis, :])
    return np.vstack(dense_points)

def resample_by_arclength(points, num_samples=500):
    """
    Resamples an ordered trajectory into `num_samples` equidistant points along its cumulative arc length.
    """
    if len(points) < 2:
        return np.repeat(points, num_samples, axis=0)

    diffs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cum_length = np.concatenate(([0.0], np.cumsum(seg_lengths)))
    total_length = cum_length[-1]

    if total_length < 1e-9:
        return np.repeat(points[:1], num_samples, axis=0)

    target_lengths = np.linspace(0.0, total_length, num_samples)
    resampled = np.empty((num_samples, 3), dtype=np.float64)
    for dim in range(3):
        resampled[:, dim] = np.interp(target_lengths, cum_length, points[:, dim])

    return resampled

def find_nearest_neighbors(src_pts, dst_pts):
    """Finds the closest target point for each source point using chunked computation."""
    n_src = len(src_pts)
    nearest_indices = np.empty(n_src, dtype=np.int64)
    min_distances = np.empty(n_src, dtype=np.float64)

    chunk_size = 500
    for start in range(0, n_src, chunk_size):
        end = min(start + chunk_size, n_src)
        chunk = src_pts[start:end]
        diff = chunk[:, np.newaxis, :] - dst_pts[np.newaxis, :, :]
        dist_matrix = np.linalg.norm(diff, axis=2)
        idx = np.argmin(dist_matrix, axis=1)
        nearest_indices[start:end] = idx
        min_distances[start:end] = dist_matrix[np.arange(end - start), idx]

    return dst_pts[nearest_indices], min_distances

def robust_similarity_transform(src, dst, allow_scale=True, fixed_scale=None):
    """
    Computes rotation R, scale s, and translation t aligning src to dst.
    Uses the non-collapsing root-mean-square spread ratio for scaling.
    """
    mu_src = np.mean(src, axis=0)
    mu_dst = np.mean(dst, axis=0)

    src_centered = src - mu_src
    dst_centered = dst - mu_dst

    cov = (dst_centered.T @ src_centered) / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1

    R = U @ S @ Vt

    if fixed_scale is not None:
        scale = fixed_scale
    elif allow_scale:
        rms_src = np.sqrt(np.mean(np.sum(src_centered ** 2, axis=1)))
        rms_dst = np.sqrt(np.mean(np.sum(dst_centered ** 2, axis=1)))
        scale = (rms_dst / rms_src) if rms_src > 1e-9 else 1.0
    else:
        scale = 1.0

    t = mu_dst - scale * (R @ mu_src)
    return scale, R, t

def run_trajectory_icp(source, dense_target, src_subpaths=None, allow_scale=True, max_iterations=150, tolerance=1e-7):
    """
    Globally initializes scale, rotation, and translation and refines with bidirectional ICP.
    """
    src_resampled = resample_by_arclength(
        densify_path(source, src_subpaths, max_step=0.05, min_total_points=500),
        num_samples=500
    )
    dst_resampled = resample_by_arclength(dense_target, num_samples=500)

    init_scale, init_R, init_t = robust_similarity_transform(
        src_resampled, dst_resampled, allow_scale=allow_scale
    )

    mu_src = np.mean(src_resampled, axis=0)
    mu_dst = np.mean(dst_resampled, axis=0)

    best_R = init_R
    best_scale = init_scale
    best_t = init_t

    def bidirectional_loss(pts_a, pts_b):
        _, d_a2b = find_nearest_neighbors(pts_a, pts_b)
        _, d_b2a = find_nearest_neighbors(pts_b, pts_a)
        return np.mean(d_a2b ** 2) + np.mean(d_b2a ** 2)

    best_loss = bidirectional_loss(init_scale * (src_resampled @ init_R.T) + init_t, dst_resampled)

    angles = np.linspace(0, 2 * np.pi, 36, endpoint=False)
    base_orientations = [
        np.eye(3),
        np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=float),
        np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float),
        np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float),
        np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], dtype=float),
        np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float),
    ]

    for B in base_orientations:
        for theta in angles:
            c, s = np.cos(theta), np.sin(theta)
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)
            R_cand = Rz @ B
            t_cand = mu_dst - init_scale * (R_cand @ mu_src)
            transformed_cand = init_scale * (src_resampled @ R_cand.T) + t_cand
            loss = bidirectional_loss(transformed_cand, dst_resampled)
            if loss < best_loss:
                best_loss = loss
                best_R = R_cand
                best_t = t_cand

    current_src = best_scale * (source @ best_R.T) + best_t
    scale, R, t = best_scale, best_R, best_t

    min_allowed_scale = init_scale * 0.5
    max_allowed_scale = init_scale * 2.0

    prev_loss = float('inf')

    for iteration in range(1, max_iterations + 1):
        closest_dst_for_src, d_src2dst = find_nearest_neighbors(current_src, dense_target)

        _, d_dst2src_idx_dists = find_nearest_neighbors(dense_target, current_src)
        diff_back = dense_target[:, np.newaxis, :] - current_src[np.newaxis, :, :]
        back_idx = np.argmin(np.linalg.norm(diff_back, axis=2), axis=1)
        src_for_dst = source[back_idx]

        combined_src = np.vstack([source, src_for_dst])
        combined_dst = np.vstack([closest_dst_for_src, dense_target])

        current_loss = np.mean(d_src2dst ** 2) + np.mean(d_dst2src_idx_dists ** 2)
        if abs(prev_loss - current_loss) < tolerance:
            break
        prev_loss = current_loss

        scale, R, t = robust_similarity_transform(
            combined_src, combined_dst, allow_scale=allow_scale
        )
        if allow_scale:
            scale = np.clip(scale, min_allowed_scale, max_allowed_scale)
            t = np.mean(combined_dst, axis=0) - scale * (R @ np.mean(combined_src, axis=0))

        current_src = scale * (source @ R.T) + t

    _, final_d_src2dst = find_nearest_neighbors(current_src, dense_target)
    _, final_d_dst2src = find_nearest_neighbors(dense_target, current_src)

    rmse = np.sqrt(np.mean(final_d_src2dst ** 2))
    mae = np.mean(final_d_src2dst)
    sym_rmse = np.sqrt(0.5 * (np.mean(final_d_src2dst ** 2) + np.mean(final_d_dst2src ** 2)))

    return current_src, scale, rmse, mae, sym_rmse, iteration

def main():
    parser = argparse.ArgumentParser(
        description="Align and scale a 3D source trajectory CSV to a target path CSV and save logs to a .log file."
    )
    parser.add_argument("source", help="Path to the source CSV file")
    parser.add_argument("target", help="Path to the target CSV file")
    parser.add_argument("output", help="Path to the output aligned CSV file")
    parser.add_argument("log_file", help="Path/name for the output .log file")
    parser.add_argument(
        "--no-scale",
        action="store_true",
        help="Disable scale estimation (perform rigid rotation + translation only)"
    )
    parser.add_argument(
        "--target-step",
        type=float,
        default=0.02,
        help="Maximum spacing in meters between interpolated target points (default: 0.02m)"
    )
    parser.add_argument(
        "--min-target-points",
        type=int,
        default=2000,
        help="Minimum total number of points on the densified target path (default: 2000)"
    )

    args = parser.parse_args()

    # Ensure the log file ends with .log
    log_path = args.log_file if args.log_file.lower().endswith(".log") else f"{args.log_file}.log"

    for path in (args.source, args.target):
        if not os.path.isfile(path):
            print(f"Error: File '{path}' not found.")
            sys.exit(1)

    try:
        src_pts, src_subpaths, extra_headers, extra_cols = load_xyz_from_csv(args.source)
        raw_dst_pts, dst_subpaths, _, _ = load_xyz_from_csv(args.target)
    except Exception as e:
        print(f"Error loading CSV files: {e}")
        sys.exit(1)

    if len(src_pts) == 0 or len(raw_dst_pts) == 0:
        print("Error: Source or target CSV contains no valid 3D points.")
        sys.exit(1)

    dense_dst_pts = densify_path(
        raw_dst_pts,
        dst_subpaths,
        max_step=args.target_step,
        min_total_points=args.min_target_points
    )

    allow_scale = not args.no_scale

    aligned_pts, final_scale, rmse, mae, sym_rmse, num_iters = run_trajectory_icp(
        src_pts, dense_dst_pts, src_subpaths=src_subpaths, allow_scale=allow_scale
    )

    # Save aligned points to output CSV
    with open(args.output, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_NONE, escapechar='\\')
        writer.writerow(['X', 'Y', 'Z'] + extra_headers)
        for i, (x, y, z) in enumerate(aligned_pts):
            row = [f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"] + extra_cols[i]
            writer.writerow(row)

    # Prepare and write the output log to the specified .log file
    mode_str = "Scaled Trajectory ICP" if allow_scale else "Rigid Trajectory ICP (No Scaling)"
    log_lines = [
        f"=== {mode_str} Completed ===",
        f"Source File             : {args.source}",
        f"Target File             : {args.target}",
        f"Target Points           : {len(raw_dst_pts)} raw -> {len(dense_dst_pts)} densified",
        f"Source Points           : {len(src_pts)}",
        f"Iterations              : {num_iters}",
        f"Applied Scale Factor    : {final_scale:.6f}",
        f"Point-to-Target RMSE    : {rmse:.6f} m",
        f"Symmetric Chamfer RMSE  : {sym_rmse:.6f} m",
        f"Mean Euclidean Error    : {mae:.6f} m",
        f"Aligned path saved to   : {args.output}"
    ]

    with open(log_path, mode='w', encoding='utf-8') as log_f:
        log_f.write("\n".join(log_lines) + "\n")

if __name__ == "__main__":
    main()