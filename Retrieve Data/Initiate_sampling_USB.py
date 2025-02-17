import time
import serial
from datetime import datetime
import pytz

# Measurement parameters
frequency = 60      # Hz
duration = 0.05     # in minutes
remove_flag = 0     # 0 = false, 1 = true

# Get current Unix timestamp in Europe/Amsterdam timezone
tz = pytz.timezone("Europe/Amsterdam")
real_world_time = int(datetime.now(tz).timestamp())

# Format the command string (e.g., "55,1708531300,0.05,0")
command = f"{frequency},{real_world_time},{duration},{remove_flag}"
print(f"Measurement command to send: {command}")

# Serial port configuration (update PORT as needed)
PORT = "COM3"      # e.g., "COM3" on Windows or "/dev/ttyACM0" on Linux/macOS
BAUD_RATE = 115200

try:
    with serial.Serial(PORT, BAUD_RATE, timeout=2) as ser:
        # Wait longer to allow the Arduino to reset and initialize
        time.sleep(4)
        ser.flushInput()
        ser.write((command + "\n").encode('utf-8'))
        print("Measurement command sent via Serial.")
except Exception as e:
    print("Error:", e)
