import xml.etree.ElementTree as ET
import sys
import os
import math
import csv
import re
from collections import defaultdict

# Register standard SVG and Inkscape namespaces to preserve clean XML output
ET.register_namespace('', 'http://www.w3.org/2000/svg')
ET.register_namespace('xlink', 'http://www.w3.org/1999/xlink')
ET.register_namespace('inkscape', 'http://www.inkscape.org/namespaces/inkscape')

def compute_distance(p1, p2):
    """Calculates the Euclidean distance between two points (x, y)."""
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def extract_points_from_path(d_str):
    """Extracts a list of (x, y) coordinates from a simple SVG path 'd' attribute."""
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

def extract_measure_segments(container):
    """Extracts line, polyline, or path segments from the 'MISURA' layer."""
    segments = []
    for elem in container.iter():
        tag = elem.tag.split('}')[-1]
        if tag == 'line':
            x1 = float(elem.attrib.get('x1', 0))
            y1 = float(elem.attrib.get('y1', 0))
            x2 = float(elem.attrib.get('x2', 0))
            y2 = float(elem.attrib.get('y2', 0))
            segments.append([(x1, y1), (x2, y2)])
        elif tag == 'polyline':
            points_str = elem.attrib.get('points', '').strip()
            coords = [float(v) for v in re.split(r'[\s,]+', points_str) if v]
            pts = [(coords[i], coords[i + 1]) for i in range(0, len(coords) - 1, 2)]
            if len(pts) >= 2:
                segments.append(pts)
        elif tag == 'path':
            d = elem.attrib.get('d', '')
            pts = extract_points_from_path(d)
            if len(pts) >= 2:
                segments.append(pts)
    return segments

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

def load_csv_points(csv_path):
    """
    Loads X, Y, Z coordinates (and optional SUBPATH) from the input CSV.
    Returns a dictionary mapping subpath_id -> list of (x, y, z) tuples in meters.
    """
    subpaths = defaultdict(list)

    with open(csv_path, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f, delimiter=',')
        header = next(reader, None)
        if not header:
            raise ValueError(f"CSV file '{csv_path}' is empty.")

        header_upper = [col.strip().upper() for col in header]
        try:
            x_idx = header_upper.index('X')
            y_idx = header_upper.index('Y')
            z_idx = header_upper.index('Z')
        except ValueError:
            raise ValueError(f"CSV file '{csv_path}' must contain 'X', 'Y', and 'Z' columns.")

        subpath_idx = header_upper.index('SUBPATH') if 'SUBPATH' in header_upper else None

        for row in reader:
            if not row:
                continue
            x = float(row[x_idx])
            y = float(row[y_idx])
            z = float(row[z_idx])
            sub_id = row[subpath_idx].strip() if subpath_idx is not None else '0'
            subpaths[sub_id].append((x, y, z))

    return subpaths

def main():
    if len(sys.argv) < 4:
        print("Usage: python add_odometry_to_svg.py <input.svg> <input_points.csv> <output.svg>")
        sys.exit(1)

    input_svg_path = sys.argv[1]
    input_csv_path = sys.argv[2]
    output_svg_path = sys.argv[3]

    for path in (input_svg_path, input_csv_path):
        if not os.path.isfile(path):
            print(f"Error: File '{path}' not found.")
            sys.exit(1)

    # 1. Parse SVG and compute scale from 'MISURA' layer (5 meters = reference line length)
    tree = ET.parse(input_svg_path)
    root = tree.getroot()

    layer_misura = find_layer(root, "MISURA")
    if layer_misura is None:
        print("Error: Layer 'MISURA' not found in SVG.")
        sys.exit(1)

    measure_segments = extract_measure_segments(layer_misura)
    if not measure_segments:
        print("Error: No reference line found in 'MISURA' layer.")
        sys.exit(1)

    measure_line = measure_segments[0]
    line_length_px = compute_distance(measure_line[0], measure_line[-1])
    if line_length_px == 0:
        print("Error: Reference line in 'MISURA' layer has zero length.")
        sys.exit(1)

    # Conversion factor from meters to SVG units (pixels)
    units_per_meter = line_length_px / 5.0

    # 2. Load points (X, Y, Z) in meters from CSV
    try:
        subpaths_meters = load_csv_points(input_csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    if not subpaths_meters:
        print("Error: No valid points found in the CSV file.")
        sys.exit(1)

    # 3. Create the new 'odometry' layer
    ns_match = re.match(r'\{.*\}', root.tag)
    ns = ns_match.group(0) if ns_match else ''

    odometry_layer = ET.SubElement(root, f"{ns}g", {
        'id': 'odometry',
        '{http://www.inkscape.org/namespaces/inkscape}groupmode': 'layer',
        '{http://www.inkscape.org/namespaces/inkscape}label': 'odometry'
    })

    # 4. Add a green polyline for each subpath (projecting 3D metric points onto 2D SVG coordinates)
    for sub_id, pts_m in subpaths_meters.items():
        if len(pts_m) < 2:
            continue

        points_attr_list = []
        for x_m, y_m, z_m in pts_m:
            x_px = x_m * units_per_meter
            y_px = y_m * units_per_meter
            points_attr_list.append(f"{x_px:.2f},{y_px:.2f}")

        points_str = " ".join(points_attr_list)

        ET.SubElement(odometry_layer, f"{ns}polyline", {
            'id': f"odometry_path_{sub_id}",
            'fill': 'none',
            'stroke': '#00FF00',
            'stroke-width': '2',
            'stroke-linecap': 'round',
            'stroke-linejoin': 'round',
            'points': points_str
        })

    # 5. Write out the updated SVG to the target file
    tree.write(output_svg_path, encoding='utf-8', xml_declaration=True)
    print(f"Successfully added 'odometry' layer and saved SVG to: {output_svg_path}")

if __name__ == "__main__":
    main()