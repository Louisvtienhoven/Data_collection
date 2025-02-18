import serial
import time
from datetime import datetime
import pytz  # for timezone conversion

# Measurement parameters
frequency = 65  # Hz
duration = 0.05  # in minutes
remove_flag = 1  # 0 = false, 1 = true

# USB Serial configuration (update port name as necessary)
SERIAL_PORT = "COM3"  # For Windows (e.g., "COM3"); use "/dev/ttyACM0" on Linux/macOS
BAUD_RATE = 115200


def send_measurement_parameters():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10) as ser:
            # Give Arduino time to reset after opening the port
            time.sleep(2)

            # (Optional) Read any startup messages from Arduino
            while ser.in_waiting:
                line = ser.readline().decode().strip()
                print(line)

            # Get the actual Unix timestamp in the correct timezone (Europe/Amsterdam)
            tz = pytz.timezone("Europe/Amsterdam")  # Adjust as needed
            real_world_time = int(datetime.now(tz).timestamp())

            # Format the command string (must match the Arduino format)
            command = f"{frequency},{real_world_time},{duration},{remove_flag}\n"
            print(f"Sending command: {command.strip()}")

            # Send the command over USB Serial
            ser.write(command.encode())
            ser.flush()

            # (Optional) Read and print Arduino's response for a short time
            time.sleep(1)
            while ser.in_waiting:
                print(ser.readline().decode().strip())

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    send_measurement_parameters()
