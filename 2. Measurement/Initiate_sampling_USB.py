import serial
import serial.tools.list_ports
import time
from datetime import datetime
import pytz  # for timezone conversion

# Measurement parameters
frequency = 75  # Hz
duration = 0.05  # in minutes
remove_flag = 1  # 0 = false, 1 = true

BAUD_RATE = 115200


def find_nicla_port():
    """Search for a COM port with 'nicla' in its description."""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "Nicla" in port.description.lower():
            print(f"Found Nicla device on port: {port.device}")
            return port.device
    print("Nicla device not found! Using default port.")
    return None


def send_measurement_parameters():
    # Attempt to automatically detect the COM port
    detected_port = find_nicla_port()
    serial_port = detected_port if detected_port is not None else "COM3"  # fallback

    try:
        with serial.Serial(serial_port, BAUD_RATE, timeout=10) as ser:
            # Allow Arduino time to reset after opening the port.
            time.sleep(2)

            # (Optional) Read any startup messages from Arduino.
            while ser.in_waiting:
                line = ser.readline().decode().strip()
                print(line)

            # Get the actual Unix timestamp in the Europe/Amsterdam timezone.
            tz = pytz.timezone("Europe/Amsterdam")
            real_world_time = int(datetime.now(tz).timestamp())

            # Format the command string (must match the Arduino format).
            command = f"{frequency},{real_world_time},{duration},{remove_flag}\n"
            print(f"Sending command: {command.strip()}")

            # Send the command over USB Serial.
            ser.write(command.encode())            ser.flush()

            # (Optional) Read and print Arduino's response for a short time.
            time.sleep(1)
            while ser.in_waiting:
                print(ser.readline().decode().strip())

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    send_measurement_parameters()
