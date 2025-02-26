import csv
import numpy as np
import os
import glob

# Directory containing CSV files with collected data
data_folder = r'C:\Users\Louis\MSc-Thesis-Louis\2. Perform Measurement\Collected Data'

# Get a list of all CSV files in the folder.
csv_files = glob.glob(os.path.join(data_folder, '*.csv'))
if not csv_files:
    print("No CSV files found in the folder.")
    exit()

# Select the most recent CSV file based on modification time.
most_recent_file = max(csv_files, key=os.path.getmtime)
print(f"Most recent CSV file found: {most_recent_file}")

def read_sensor_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Handle rows with exactly 6 elements.
            if len(row) == 6:
                try:
                    float_row = list(map(float, row))
                    data.append(float_row)
                except ValueError:
                    continue
            # Handle rows with 7 elements by ignoring the first element (timestamp).
            elif len(row) == 7:
                try:
                    float_row = list(map(float, row[1:]))  # Skip the timestamp column
                    data.append(float_row)
                except ValueError:
                    continue
            # Skip rows that don't have 6 or 7 elements.
    if data:
        return np.array(data).T
    else:
        return np.array([])

data_arrays = read_sensor_data(most_recent_file)
if data_arrays.size:
    ax, ay, az, gx, gy, gz = data_arrays

    # Print first few values as a sample
    print("ax:", ax[:5])
    print("ay:", ay[:5])
    print("az:", az[:5])
    print("gx:", gx[:5])
    print("gy:", gy[:5])
    print("gz:", gz[:5])

    # Derive the output NPZ file name from the CSV file name.
    base_name = os.path.splitext(os.path.basename(most_recent_file))[0]
    npz_file_name = base_name + ".npz"

    # Set the output folder.
    output_folder = r"C:\Users\Louis\MSc-Thesis-Louis\3. Data Preparation\Extracted Data"
    # Combine folder path and file name.
    output_path = os.path.join(output_folder, npz_file_name)

    # Save the numpy arrays to an NPZ file.
    np.savez(output_path, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz)
    print(f"Sensor data saved to '{output_path}'.")
else:
    print("No valid sensor data found.")
