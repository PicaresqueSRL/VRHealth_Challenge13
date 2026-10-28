import sys
import os
import csv
import pandas as pd

def load_path_csv(file_path):
    """Loads CSV data, handling potential decimal comma formatting issues."""
    rows = []
    with open(file_path, mode='r', encoding='utf-8') as f:
        reader = csv.reader(f, delimiter=',')
        header = next(reader, None)
        if not header:
            raise ValueError(f"CSV file '{file_path}' is empty.")
        
        for row in reader:
            if not row or len(row) < 3:
                continue
            try:
                cleaned_row = [float(val) for val in row]
            except ValueError:
                # Fallback: fix accidental comma splitting inside numbers
                fixed = []
                i = 0
                while i < len(row):
                    val_str = row[i].strip()
                    if i + 1 < len(row) and len(row[i+1].strip()) == 4 and row[i+1].isdigit():
                        fixed.append(float(f"{val_str}.{row[i+1].strip()}"))
                        i += 2
                    else:
                        fixed.append(float(val_str))
                        i += 1
                cleaned_row = fixed
            rows.append(cleaned_row)

    df = pd.DataFrame(rows, columns=header[:len(rows[0])])
    return df

def generate_svg_plot(df, output_svg_path, width=1200, height=600):
    """Generates an SVG file with side-by-side Top (XY) and Front (XZ) projections."""
    cols_upper = [c.upper() for c in df.columns]
    x_col = df.columns[cols_upper.index('X')]
    y_col = df.columns[cols_upper.index('Y')]
    z_col = df.columns[cols_upper.index('Z')]
    subpath_col = df.columns[cols_upper.index('SUBPATH')] if 'SUBPATH' in cols_upper else None

    # Calculate bounding boxes for scaling
    x_min, x_max = df[x_col].min(), df[x_col].max()
    y_min, y_max = df[y_col].min(), df[y_col].max()
    z_min, z_max = df[z_col].min(), df[z_col].max()

    # Add padding to bounds
    x_pad = max((x_max - x_min) * 0.1, 1.0)
    y_pad = max((y_max - y_min) * 0.1, 1.0)
    z_pad = max((z_max - z_min) * 0.1, 1.0)

    x_min, x_max = x_min - x_pad, x_max + x_pad
    y_min, y_max = y_min - y_pad, y_max + y_pad
    z_min, z_max = z_min - z_pad, z_max + z_pad

    # Layout dimensions
    margin = 80
    panel_w = (width - 3 * margin) / 2
    panel_h = height - 2 * margin

    # Mapping functions (SVG Y goes downwards, so we invert Y and Z)
    def map_xy(x, y):
        svg_x = margin + (x - x_min) / (x_max - x_min) * panel_w
        svg_y = margin + (y - y_min) / (y_max - y_min) * panel_h
        return svg_x, svg_y

    def map_xz(x, z):
        svg_x = 2 * margin + panel_w + (x - x_min) / (x_max - x_min) * panel_w
        svg_y = margin + (z - z_min) / (z_max - z_min) * panel_h
        return svg_x, svg_y

    # Distinct color palette for subpaths
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

    svg_elements = []
    svg_elements.append(f'<?xml version="1.0" encoding="UTF-8" standalone="no"?>')
    svg_elements.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">')
    svg_elements.append('<style>')
    svg_elements.append('  .bg { fill: #f9f9f9; stroke: #ccc; stroke-width: 1; }')
    svg_elements.append('  .grid { stroke: #e0e0e0; stroke-width: 1; stroke-dasharray: 4,4; }')
    svg_elements.append('  .axis { stroke: #333; stroke-width: 1.5; }')
    svg_elements.append('  .title { font-family: sans-serif; font-size: 16px; font-weight: bold; fill: #333; text-anchor: middle; }')
    svg_elements.append('  .label { font-family: sans-serif; font-size: 12px; fill: #666; }')
    svg_elements.append('  .legend { font-family: sans-serif; font-size: 12px; fill: #333; }')
    svg_elements.append('</style>')

    # Background containers for panels
    # Panel 1: Top View (XY)
    svg_elements.append(f'<rect x="{margin}" y="{margin}" width="{panel_w}" height="{panel_h}" class="bg" />')
    svg_elements.append(f'<text x="{margin + panel_w/2}" y="{margin - 25}" class="title">Top View (XY)</text>')

    # Panel 2: Front View (XZ)
    x2_start = 2 * margin + panel_w
    svg_elements.append(f'<rect x="{x2_start}" y="{margin}" width="{panel_w}" height="{panel_h}" class="bg" />')
    svg_elements.append(f'<text x="{x2_start + panel_w/2}" y="{margin - 25}" class="title">Front View (XZ)</text>')

    groups = df.groupby(subpath_col) if subpath_col else [(0, df)]

    # Draw paths and markers
    legend_items = []
    startcolor = "#FF0000"
    for idx, (sub_id, group) in enumerate(groups):
        color = colors[idx % len(colors)]
        label_str = f"Subpath {sub_id}" if subpath_col else "Path"

        # Build polyline points for XY and XZ
        xy_pts, xz_pts = [], []
        circles_xy, circles_xz = [], []
        first = True
        for _, row in group.iterrows():
            x, y, z = row[x_col], row[y_col], row[z_col]
            px_xy, py_xy = map_xy(x, y)
            px_xz, py_xz = map_xz(x, z)
            xy_pts.append(f"{px_xy:.1f},{py_xy:.1f}")
            xz_pts.append(f"{px_xz:.1f},{py_xz:.1f}")
            if(first):
                circles_xy.append(f'<circle cx="{px_xy:.1f}" cy="{py_xy:.1f}" r="4" fill="{startcolor}" />')
                circles_xz.append(f'<circle cx="{px_xz:.1f}" cy="{py_xz:.1f}" r="4" fill="{startcolor}" />')
                first = False
            else:
                circles_xy.append(f'<circle cx="{px_xy:.1f}" cy="{py_xy:.1f}" r="4" fill="{color}" />')
                circles_xz.append(f'<circle cx="{px_xz:.1f}" cy="{py_xz:.1f}" r="4" fill="{color}" />')

        # Polyline lines
        svg_elements.append(f'<polyline fill="none" stroke="{color}" stroke-width="2.5" points="{" ".join(xy_pts)}" />')
        svg_elements.append(f'<polyline fill="none" stroke="{color}" stroke-width="2.5" points="{" ".join(xz_pts)}" />')

        # Vertex markers
        svg_elements.extend(circles_xy)
        svg_elements.extend(circles_xz)

    # Draw Legend
    legend_y = margin + panel_h + 30
    leg_x = margin
    for label, col in legend_items:
        svg_elements.append(f'<circle cx="{leg_x + 10}" cy="{legend_y}" r="5" fill="{col}" />')
        svg_elements.append(f'<text x="{leg_x + 25}" y="{legend_y + 4}" class="legend">{label}</text>')
        leg_x += 120

    svg_elements.append('</svg>')

    with open(output_svg_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(svg_elements))

def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_3d_to_svg.py <input_file.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"Error: File '{csv_path}' not found.")
        sys.exit(1)

    try:
        df = load_path_csv(csv_path)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

    output_svg = os.path.splitext(csv_path)[0] + "_plots.svg"
    generate_svg_plot(df, output_svg)
    print(f"SVG plot successfully generated and saved to: {output_svg}")

if __name__ == "__main__":
    main()