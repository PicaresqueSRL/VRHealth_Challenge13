import xml.etree.ElementTree as ET
import sys
import os
import math
import csv
import re
from collections import Counter

def compute_distance(p1, p2):
    """Calculates the Euclidean distance between two points (x, y)."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def extract_points_from_path(d_str):
    """Extracts a list of (x, y) coordinates from a simple SVG path 'd' attribute (M, L, H, V commands)."""
    tokens = re.findall(r'[a-zA-Z]|[-+]?\d*\.?\d+', d_str)
    points = []
    curr_x, curr_y = 0.0, 0.0
    cmd = None
    i = 0
    while i < len(tokens):
        token = tokens[i]
        if token.isalpha():
            cmd = token
            i += 1
        else:
            if cmd in ('M', 'L'):
                curr_x = float(tokens[i])
                curr_y = float(tokens[i + 1])
                points.append((curr_x, curr_y))
                i += 2
            elif cmd in ('m', 'l'):
                curr_x += float(tokens[i])
                curr_y += float(tokens[i + 1])
                points.append((curr_x, curr_y))
                i += 2
            elif cmd == 'H':
                curr_x = float(tokens[i])
                points.append((curr_x, curr_y))
                i += 1
            elif cmd == 'h':
                curr_x += float(tokens[i])
                points.append((curr_x, curr_y))
                i += 1
            elif cmd == 'V':
                curr_y = float(tokens[i])
                points.append((curr_x, curr_y))
                i += 1
            elif cmd == 'v':
                curr_y += float(tokens[i])
                points.append((curr_x, curr_y))
                i += 1
            else:
                i += 1
    return points

def extract_geometric_elements(container):
    """
    Extracts circles with their class/style signatures and lines/polylines from an SVG element.
    Returns:
        circles: list of dicts [{'center': (x, y), 'class': str}]
        segments: list of lists of (x, y) points
    """
    circles = []
    segments = []

    for elem in container.iter():
        tag = elem.tag.split('}')[-1]
        elem_class = elem.attrib.get('class') or elem.attrib.get('fill') or elem.attrib.get('style', 'default')

        # 1. Circle / Ellipse elements
        if tag in ('circle', 'ellipse'):
            cx = float(elem.attrib.get('cx', 0))
            cy = float(elem.attrib.get('cy', 0))
            circles.append({'center': (cx, cy), 'class': elem_class})

        # 2. Line elements
        elif tag == 'line':
            x1 = float(elem.attrib.get('x1', 0))
            y1 = float(elem.attrib.get('y1', 0))
            x2 = float(elem.attrib.get('x2', 0))
            y2 = float(elem.attrib.get('y2', 0))
            segments.append([(x1, y1), (x2, y2)])

        # 3. Polyline elements
        elif tag == 'polyline':
            points_str = elem.attrib.get('points', '').strip()
            coords = [float(v) for v in re.split(r'[\s,]+', points_str) if v]
            pts = [(coords[i], coords[i + 1]) for i in range(0, len(coords) - 1, 2)]
            if len(pts) >= 2:
                segments.append(pts)

        # 4. Path elements
        elif tag == 'path':
            d = elem.attrib.get('d', '')
            pts = extract_points_from_path(d)
            if ('z' in d.lower() or 'c' in d.lower() or 'a' in d.lower()) and len(pts) > 0:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                circles.append({'center': ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2), 'class': elem_class})
            elif len(pts) >= 2:
                segments.append(pts)

    return circles, segments

def find_layer(root, layer_name):
    """Searches for a <g> layer by ID or Inkscape label (case-insensitive)."""
    target = layer_name.upper()
    for elem in root.iter():
        if elem.tag.split('}')[-1] == 'g':
            id_attr = elem.attrib.get('id', '').upper()
            label_attr = elem.attrib.get('{http://www.inkscape.org/namespaces/inkscape}label', '').upper()
            if id_attr == target or label_attr == target or id_attr.startswith(target + '_'):
                return elem
    return None

def find_unique_class_circle(circles):
    """Finds the circle with a unique class among the group's circles."""
    if not circles:
        return None
    if len(circles) == 1:
        return circles[0]['center']

    class_counts = Counter(c['class'] for c in circles)
    unique_classes = [cls for cls, count in class_counts.items() if count == 1]

    if unique_classes:
        target_class = unique_classes[0]
        for c in circles:
            if c['class'] == target_class:
                return c['center']

    # Fallback: pick the circle belonging to the least frequent class
    minority_class = min(class_counts, key=class_counts.get)
    for c in circles:
        if c['class'] == minority_class:
            return c['center']
    return circles[0]['center']

def order_group_subpath(circles, segments, tolerance=1e-2):
    """
    Orders the points of a single subpath group starting from the point
    closest to the unique-class starting circle.
    """
    if not segments:
        return []

    start_center = find_unique_class_circle(circles)
    available = [list(s) for s in segments]
    ordered = []

    # 1. Pick the segment endpoint closest to the starting circle
    if start_center is not None:
        best_idx = 0
        min_dist = float('inf')
        should_reverse = False

        for i, seg in enumerate(available):
            d_start = compute_distance(start_center, seg[0])
            d_end = compute_distance(start_center, seg[-1])

            if d_start < min_dist:
                min_dist = d_start
                best_idx = i
                should_reverse = False
            if d_end < min_dist:
                min_dist = d_end
                best_idx = i
                should_reverse = True

        first_seg = available.pop(best_idx)
        if should_reverse:
            first_seg.reverse()
        ordered.extend(first_seg)
    else:
        ordered.extend(available.pop(0))

    # 2. Chain remaining segments in the group starting from the last added point
    while available:
        last_pt = ordered[-1]
        best_idx = -1
        min_dist = float('inf')
        should_reverse = False

        for i, seg in enumerate(available):
            d_start = compute_distance(last_pt, seg[0])
            d_end = compute_distance(last_pt, seg[-1])

            if d_start < min_dist:
                min_dist = d_start
                best_idx = i
                should_reverse = False
            if d_end < min_dist:
                min_dist = d_end
                best_idx = i
                should_reverse = True

        next_seg = available.pop(best_idx)
        if should_reverse:
            next_seg.reverse()

        # Avoid duplicating shared vertices if segments touch within tolerance
        if min_dist <= tolerance:
            ordered.extend(next_seg[1:])
        else:
            ordered.extend(next_seg)

    return ordered

def extract_all_subpaths(layer_percorsi):
    """
    Iterates through each <g> subpath group inside the 'PERCORSI' layer
    and extracts ordered points along with their subpath index (starting from 0).
    Returns:
        list of tuples: [(subpath_id, x, y), ...]
    """
    subpath_groups = [child for child in layer_percorsi if child.tag.split('}')[-1] == 'g']

    if not subpath_groups:
        subpath_groups = [layer_percorsi]

    all_ordered_points = []
    subpath_id = 0

    for group in subpath_groups:
        circles, segments = extract_geometric_elements(group)
        if segments:
            subpath_points = order_group_subpath(circles, segments)
            for x_px, y_px in subpath_points:
                all_ordered_points.append((subpath_id, x_px, y_px))
            subpath_id += 1

    return all_ordered_points

def main():
    if len(sys.argv) < 2:
        print("Usage: python svg_to_csv.py <input_file.svg>")
        sys.exit(1)

    svg_path = sys.argv[1]
    if not os.path.isfile(svg_path):
        print(f"Error: File '{svg_path}' not found.")
        sys.exit(1)

    tree = ET.parse(svg_path)
    root = tree.getroot()

    # 1. Determine scaling factor from the 'MISURA' layer (1 line = 5.0 meters)
    layer_misura = find_layer(root, "MISURA")
    if layer_misura is None:
        print("Error: Layer 'MISURA' not found in SVG.")
        sys.exit(1)

    _, measure_segments = extract_geometric_elements(layer_misura)
    if not measure_segments:
        print("Error: No lines found in 'MISURA' layer.")
        sys.exit(1)

    measure_line = measure_segments[0]
    line_length_px = compute_distance(measure_line[0], measure_line[-1])
    if line_length_px == 0:
        print("Error: Reference line in 'MISURA' layer has zero length.")
        sys.exit(1)

    meters_per_unit = 5.0 / line_length_px

    width_str = re.sub(r'[^\d.]+', '', root.attrib.get('width', '0'))
    height_str = re.sub(r'[^\d.]+', '', root.attrib.get('height', '0'))
    if width_str and height_str:
        w_m = float(width_str) * meters_per_unit
        h_m = float(height_str) * meters_per_unit
        print(f"Image metric dimensions: {w_m:.2f} m x {h_m:.2f} m")

    # 2. Extract and order trajectory points for each subpath group in 'PERCORSI'
    layer_percorsi = find_layer(root, "PERCORSI")
    if layer_percorsi is None:
        print("Error: Layer 'PERCORSI' not found in SVG.")
        sys.exit(1)

    ordered_points = extract_all_subpaths(layer_percorsi)

    # 3. Write points to CSV output
    base_name = os.path.splitext(svg_path)[0]
    csv_path = f"{base_name}.csv"

    with open(csv_path, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_NONE, escapechar='\\')
        writer.writerow(['X', 'Y', 'SUBPATH'])
        for subpath_id, x_px, y_px in ordered_points:
            x_m = x_px * meters_per_unit
            y_m = y_px * meters_per_unit
            writer.writerow([f"{x_m:.4f}", f"{y_m:.4f}", subpath_id])

    print(f"CSV successfully generated: {csv_path}")

if __name__ == "__main__":
    main()