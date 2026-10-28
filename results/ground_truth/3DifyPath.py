import sys
import os
import csv

def main():
    if len(sys.argv) < 2:
        print("Usage: python add_z_column.py <input_file.csv> [output_file.csv]")
        sys.exit(1)

    input_path = sys.argv[1]
    if not os.path.isfile(input_path):
        print(f"Error: File '{input_path}' not found.")
        sys.exit(1)

    # If no output file is specified, overwrite the input file in-place
    output_path = sys.argv[2] if len(sys.argv) >= 3 else input_path

    updated_rows = []

    with open(input_path, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.reader(f, delimiter=',')
        header = next(reader, None)

        if not header:
            print("Error: The input CSV file is empty.")
            sys.exit(1)

        # Find the position of 'SUBPATH' (or default to index 2 right after X, Y)
        header_upper = [col.strip().upper() for col in header]
        if 'Z' in header_upper:
            print(f"Skipping: '{input_path}' already contains a 'Z' column.")
            sys.exit(0)

        insert_idx = header_upper.index('SUBPATH') if 'SUBPATH' in header_upper else 2

        # Insert 'Z' into the header
        new_header = header[:insert_idx] + ['Z'] + header[insert_idx:]
        updated_rows.append(new_header)

        # Insert '0' (or '0.0000' if you prefer decimals) into each data row
        for row in reader:
            if not row:
                continue
            new_row = row[:insert_idx] + ['0'] + row[insert_idx:]
            updated_rows.append(new_row)

    with open(output_path, mode='w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f, delimiter=',', quoting=csv.QUOTE_NONE, escapechar='\\')
        writer.writerows(updated_rows)

    print(f"Successfully added 'Z' column to: {output_path}")

if __name__ == "__main__":
    main()