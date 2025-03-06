import serial
import time
from datetime import datetime
import pytz  # for timezone conversion

# Measurement parameters
frequency = 26  # Hz, in practice this should be about 1-2 Hz higher because of flashing the memory
duration = 60  # in minutes
remove_flag = 1  # 0 = false, 1 = true

# Manually set the COM port and baud rate
SERIAL_PORT = "COM3"  # Change this if needed
BAUD_RATE = 460800    #Default is 115200



def send_measurement_parameters():
    try:
        # Attempt to open the serial port.
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=10)
    except serial.SerialException as e:
        print("Error: Serial connection could not be established:", e)
        return

    time.sleep(2)  # Allow time for the serial connection to initialize

    if ser.is_open:
        print("Serial connection established successfully on", SERIAL_PORT)
    else:
        print("Serial connection is not open!")
        ser.close()
        return

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

    # Send the command over USB Serial.
    ser.write(command.encode())
    ser.flush()

    # (Optional) Read and print Arduino's response for a short time.
    time.sleep(1)
    while ser.in_waiting:
        print(ser.readline().decode(errors='replace').strip())

    ser.close()

if __name__ == "__main__":
    send_measurement_parameters()
