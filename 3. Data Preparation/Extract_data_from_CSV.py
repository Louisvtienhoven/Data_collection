import csv
import numpy as np

def read_sensor_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Skip rows that do not have exactly 6 elements.
            if len(row) != 6:
                continue
            try:
                # Try converting all values in the row to float.
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

# Path to the downloaded CSV file
file_path = r'C:\Users\Louis\MSc-Thesis-Louis\3. Collected Data\25Hz_0.05min_2025-02-20_17-14-14.csv'

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
else:
    print("No valid sensor data found.")
