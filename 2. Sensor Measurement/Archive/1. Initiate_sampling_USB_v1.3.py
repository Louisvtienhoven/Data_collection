import serial
import time
from datetime import datetime
import pytz  # for timezone conversion

# This script sends measurement parameters to an Arduino device over a serial connection.
# The parameters include frequency, duration, additional delay, and whether to collect gyroscope data.


#------------------------------

# Measurement parameters
frequency = 2000           # Hz
duration = 1             # in minutes
additional_delay = 2      # extra delay in minutes before sampling starts
collect_gyro = 0          # set to 0 to collect only accelerometer data, 1 for both

# Manually set the COM port and baud rate
SERIAL_PORT = "COM4"  # Change if needed
BAUD_RATE = 460800    # Default is 115200

#Please note! Because of integer division, when sampleFrequencyHz is greater than 1000, 1000 divided
# by that value yields 0 (e.g., 1000/2000 equals 0). Thus, any sampling frequency above 1000 Hz
# induces no additional delay. The maximum sampling frequency here is rougly 240 Hz

#------------------------------



# Clear memory command, not advised to use
remove_flag = 1           # 0 = false, 1 = true

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
    # Add additional_delay (converted to seconds) to the real world time.
    adjusted_time = real_world_time + additional_delay * 60

    # Format the command string with the six parameters.
    command = f"{frequency},{adjusted_time},{duration},{remove_flag},{additional_delay},{collect_gyro}\n"
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
