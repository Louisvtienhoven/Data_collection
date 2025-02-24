import serial
import time
import os
from datetime import datetime
import pytz  # for timezone conversion

# Measurement parameters (fallback defaults)
frequency = 75  # Hz
duration = 0.5  # in minutes
remove_flag = 1  # 0 = false, 1 = true

# USB Serial configuration (update port name as necessary)
SERIAL_PORT = 'COM3'  # For Windows (e.g., "COM3"); use "/dev/ttyACM0" on Linux/macOS
BAUD_RATE = 115200

# Directory to save the collected data
SAVE_DIR = r"C:\Users\Louis\MSc-Thesis-Louis\2. Perform Measurement\Collected Data"

# Ensure the directory exists
os.makedirs(SAVE_DIR, exist_ok=True)


def retrieve_csv():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
    except serial.SerialException as e:
        print("Error: Serial connection could not be established:", e)
        return None, None

    time.sleep(2)  # Allow time for the serial connection to initialize

    if ser.is_open:
        print("Serial connection established successfully on", SERIAL_PORT)
    else:
        print("Serial connection is not open!")
        return None, None

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


def parse_parameters(csv_data):
    """
    Extract the measurement parameters from the CSV header.
    Expected header lines:
      Sampling Frequency [Hz]:25
      Sample Duration [Min]:0.50
      Time:2025-02-20 11:45:38
    """
    frequency_param = None
    duration_param = None
    time_param = None
    # Split into individual lines
    lines = csv_data.splitlines()
    for line in lines:
        if "Sampling Frequency" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                frequency_param = parts[1].strip()
        elif "Sample Duration" in line:
            parts = line.split(":")
            if len(parts) >= 2:
                duration_param = parts[1].strip()
        elif line.startswith("Time:"):
            parts = line.split("Time:")
            if len(parts) >= 2:
                time_param = parts[1].strip()
        if frequency_param and duration_param and time_param:
            break
    return frequency_param, duration_param, time_param


if __name__ == '__main__':
    csv_content, flag_resp = retrieve_csv()
    if csv_content is None:
        print("CSV retrieval failed due to serial connection issues.")
    else:
        print("Retrieved CSV Data:")
        print(csv_content)

        # Optionally, print the flag response:
        # print("Flag response:")
        # print(flag_resp)

        # Try to parse the measurement parameters from the CSV header.
        freq_par, dur_par, time_par = parse_parameters(csv_content)
        if freq_par and dur_par and time_par:
            # Replace spaces and colons in time string for filename safety.
            safe_time = time_par.replace(" ", "_").replace(":", "-")
            filename = f"{freq_par}Hz_{dur_par}min_{safe_time}.csv"
        else:
            now = datetime.now(pytz.timezone("Europe/Amsterdam"))
            timestamp_str = now.strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"{frequency}Hz_{duration}min_{timestamp_str}.csv"

        file_path = os.path.join(SAVE_DIR, filename)

        with open(file_path, "w", encoding="utf-8") as f:
            f.write(csv_content)
        print(f"CSV file saved as '{file_path}'.")
