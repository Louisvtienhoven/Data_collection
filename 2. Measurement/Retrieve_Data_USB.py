import serial
import time
import os
from datetime import datetime
import pytz  # for timezone conversion

# Measurement parameters
frequency = 75  # Hz
duration = 0.05  # in minutes
remove_flag = 1  # 0 = false, 1 = true

# USB Serial configuration (update port name as necessary)
SERIAL_PORT = 'COM3'  # For Windows (e.g., "COM3"); use "/dev/ttyACM0" on Linux/macOS
BAUD_RATE = 115200

# Directory to save the collected data
SAVE_DIR = r"C:\Users\Louis\MSc-Thesis-Louis\3. Collected Data"

# Ensure the directory exists
os.makedirs(SAVE_DIR, exist_ok=True)


def retrieve_csv():
    # Open the serial port.
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
    time.sleep(2)  # Allow time for the serial connection to initialize

    # Clear any existing data in the input buffer.
    ser.reset_input_buffer()

    # Send the GETCSV command.
    ser.write(b'GETCSV\n')
    time.sleep(1)  # Wait a moment for the device to respond.

    # Read all available data.
    csv_data = ser.read_all().decode('utf-8', errors='replace')

    # Now send the CLRFLAG command to clear done.txt.
    ser.write(b'CLRFLAG\n')
    time.sleep(0.5)  # Wait for the Arduino to process the command.
    flag_response = ser.read_all().decode('utf-8', errors='replace')

    ser.close()
    return csv_data, flag_response


if __name__ == '__main__':
    csv_content, flag_resp = retrieve_csv()
    print("Retrieved CSV Data:")
    print(csv_content)
    print("Flag response:")
    print(flag_resp)

    # Get current time for the filename. Adjust the timezone if desired.
    now = datetime.now(pytz.timezone("Europe/Amsterdam"))
    timestamp_str = now.strftime("%Y-%m-%d_%H-%M-%S")

    # Build the filename using the measurement parameters and the timestamp.
    filename = f"{frequency}Hz_{duration}min_{timestamp_str}.csv"
    file_path = os.path.join(SAVE_DIR, filename)

    # Save the CSV data to the local file in the specified directory.
    with open(file_path, "w", encoding="utf-8") as f:
        f.write(csv_content)
    print(f"CSV file saved as '{file_path}'.")
