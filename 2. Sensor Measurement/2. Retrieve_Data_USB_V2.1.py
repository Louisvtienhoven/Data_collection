import serial
import time
import os
import sys
from datetime import datetime
import pytz  # for timezone conversion

# USB Serial configuration (update port name as necessary)
SERIAL_PORT = 'COM3'  # For Windows (e.g., "COM3"); use "/dev/ttyACM0" on Linux/macOS
BAUD_RATE = 460800 #Default baud rate is 115200

# Directory to save the collected data
SAVE_DIR = r"/2. Sensor Measurement/Collected Data"

# Ensure the directory exists
os.makedirs(SAVE_DIR, exist_ok=True)

# Flag to determine whether to clear device memory after retrieval
clear_memory = True


def retrieve_csv():
    try:
        # Open serial port with additional flow control settings.
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1, rtscts=True, xonxoff=True)
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
    print("Sent GETCSV command to device.")

    # Write incoming data directly to a temporary file.
    temp_filename = "temp_csv_data.txt"
    with open(temp_filename, "w", encoding="utf-8") as temp_file:
        timeout_period = 2.0  # seconds to wait after the last received data
        last_data_time = time.time()
        total_bytes = 0
        while True:
            if ser.in_waiting:
                try:
                    # Read all available bytes and decode them.
                    new_data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                except Exception as e:
                    print("Decoding error:", e)
                    new_data = ""
                if new_data:
                    temp_file.write(new_data)
                    total_bytes += len(new_data)
                    last_data_time = time.time()  # Reset timer when new data arrives.
                    print(f"Received {len(new_data)} bytes, total {total_bytes} bytes")
            # End loop if no new data arrives within the timeout period.
            if time.time() - last_data_time > timeout_period:
                print(f"No new data received for {timeout_period} seconds. Ending read.")
                break
            time.sleep(0.1)

    # Optionally, send CLRFLAG command to clear the device's memory.
    if clear_memory:
        ser.write(b'CLRFLAG\n')
        print("Sent CLRFLAG command to device.")
        time.sleep(0.5)  # Allow time for the device to process the command.

    # Read any flag response.
    flag_response = ""
    while ser.in_waiting:
        flag_response += ser.read(ser.in_waiting).decode('utf-8', errors='replace')
        time.sleep(0.1)

    ser.close()
    print("Serial port closed.")

    # Read the entire content of the temporary file.
    with open(temp_filename, "r", encoding="utf-8") as temp_file:
        csv_data = temp_file.read()
    # Clean up temporary file.
    os.remove(temp_filename)

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
    # Split CSV content into individual lines.
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
        sys.exit(1)
    else:
        print("Retrieved CSV Data (first 500 characters):")
        print(csv_content[:500])

        # Parse measurement parameters from the CSV header.
        freq_par, dur_par, time_par = parse_parameters(csv_content)
        if not (freq_par and dur_par and time_par):
            print("Error: CSV header does not contain all required measurement parameters.")
            sys.exit(1)

        # Replace spaces and colons in the time string for filename safety.
        safe_time = time_par.replace(" ", "_").replace(":", "-")
        filename = f"{freq_par}Hz_{dur_par}min_{safe_time}.csv"
        file_path = os.path.join(SAVE_DIR, filename)

        with open(file_path, "w", encoding="utf-8") as f:
            f.write(csv_content)
        print(f"CSV file saved as '{file_path}'.")
