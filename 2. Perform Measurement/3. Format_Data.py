import csv

def read_sensor_data(filename):
    time = []
    ax = []
    ay = []
    az = []
    gyrox = []
    gyroy = []
    gyroz = []

    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        
        # Skip the first two lines (header and "Sending sensorData.csv...")
        next(csvreader)
        next(csvreader)
        
        for row in csvreader:
            time.append(int(row[0]))
            ax.append(float(row[1]))
            ay.append(float(row[2]))
            az.append(float(row[3]))
            gyrox.append(float(row[4]))
            gyroy.append(float(row[5]))
            gyroz.append(float(row[6]))

    return time, ax, ay, az, gyrox, gyroy, gyroz

# Usage
filename = filename = r'C:\Users\Louis\MSc-Thesis-Louis\Retrieve Data\sensorData.csv'

time, ax, ay, az, gyrox, gyroy, gyroz = read_sensor_data(filename)

# Print the length of each list to verify the data
print(f"Number of data points: {len(time)}")
print(f"First few values of each list:")
print(f"Time: {time[:5]}")
print(f"Ax: {ax[:5]}")
print(f"Ay: {ay[:5]}")
print(f"Az: {az[:5]}")
print(f"Gyro X: {gyrox[:5]}")
print(f"Gyro Y: {gyroy[:5]}")
print(f"Gyro Z: {gyroz[:5]}")
