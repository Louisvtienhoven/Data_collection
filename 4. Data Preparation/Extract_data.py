import csv
import numpy as np

# Path to the downloaded CSV file
file_path = r'C:\Users\Louis\MSc-Thesis-Louis\3. Collected Data\downloaded_75Hz_0.05min_2025-02-19_17-19-58.csv'
def read_sensor_data(file_path):
    data = []

    with open(file_path, 'r') as file:
        reader = csv.reader(file)

        for row in reader:
            # Skip metadata and empty lines
            if not row or not row[0][0].isdigit() and not row[0][0] == '-':
                continue

            # Convert row values to float and append to data list
            data.append(list(map(float, row)))

    return np.array(data).T  # Transpose to separate columns into individual arrays

data_arrays = read_sensor_data(file_path)
ax, ay, az, gx, gy, gz = data_arrays

# Print first few values as a sample
print("ax:", ax[:5])
print("ay:", ay[:5])
print("az:", az[:5])
print("gx:", gx[:5])
print("gy:", gy[:5])
print("gz:", gz[:5])
