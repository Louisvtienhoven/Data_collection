import serial
import time
from datetime import datetime
import pytz  # for timezone conversion

# Measurement parameters
frequency = 25  # Hz
duration = 0.05  # in minutes
remove_flag = 1  # 0 = false, 1 = true

# Manually set the COM port
SERIAL_PORT = "COM3"  # Change this if needed
BAUD_RATE = 115200

# Delay before the measurement starts (in seconds)
# Delay stems from the uploaded Arduino code

delay = 10 # seconds

def send_measurement_parameters():
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10) as ser:
            # Allow Arduino time to reset after opening the port.
            time.sleep(2)

            # (Optional) Read any startup messages from Arduino.
            while ser.in_waiting:
                line = ser.readline().decode(errors='replace').strip()
                print(line)

            # Get the actual Unix timestamp in the Europe/Amsterdam timezone.
            tz = pytz.timezone("Europe/Amsterdam")
            real_world_time = int(datetime.now(tz).timestamp())

            # Format the command string (must match the Arduino format).
            command = f"{frequency},{real_world_time},{duration},{remove_flag}\n"
            print(f"Sending command: {command.strip()}")
            print(f"Change power source within {delay} seconds before measurement starts")
            print(f"Be aware that the measurement command is deleted once the measurement starts")

            # Send the command over USB Serial.
            ser.write(command.encode())
            ser.flush()

            # (Optional) Read and print Arduino's response for a short time.
            time.sleep(1)
            while ser.in_waiting:
                print(ser.readline().decode(errors='replace').strip())

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


if __name__ == "__main__":
    send_measurement_parameters()
