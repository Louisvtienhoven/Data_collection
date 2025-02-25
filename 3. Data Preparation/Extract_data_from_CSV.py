import csv
import numpy as np
import os

# Path to CSV file with the collected data
file_path = r'C:\Users\Louis\MSc-Thesis-Louis\2. Perform Measurement\Collected Data\25Hz_2.00min_2025-02-25_14-04-01.csv'
def read_sensor_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Skip rows that do not have exactly 6 elements.
            if len(row) != 6:
                continue
            try:
                # Convert all values in the row to float.
                float_row = list(map(float, row))
                data.append(float_row)
            except ValueError:
                # If conversion fails, skip this row.
                continue
    # If valid data exists, transpose it to separate columns.
    if data:
        return np.array(data).T
    else:
        return np.array([])



data_arrays = read_sensor_data(file_path)
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
    base_name = os.path.splitext(os.path.basename(file_path))[0]
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
