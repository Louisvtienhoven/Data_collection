import asyncio
from datetime import datetime
import pytz
from bleak import BleakClient, BleakScanner

# Measurement parameters
frequency = 65      # Hz
duration = 0.05     # in minutes
waiting_time = 0.2  # waiting time in minutes (new)
remove_flag = 1     # 0 = false, 1 = true

# BLE Variables
n_scan_retries = 5
n_conn_retries = 10
scan_timeout = 10   # seconds
conn_timeout = 15   # seconds

# Device search criteria
DEVICE_NAME = "NiclaVoice1"  # Use either NiclaVoice1 or NiclaVoice2

# UUID of the characteristic to write measurement parameters
CHAR_UUID = "abcdef01-1234-5678-1234-56789abcdef0"  # Must match the Arduino UUID

async def find_device():
    for scan_attempt in range(1, n_scan_retries + 1):
        print(f"Scan attempt {scan_attempt} of {n_scan_retries} (timeout: {scan_timeout}s)...")
        devices = await BleakScanner.discover(timeout=scan_timeout)
        for d in devices:
            print(f"  Found: {d.address} - {d.name}")
            if d.name and DEVICE_NAME.lower() in d.name.lower():
                print(f"Device matching '{DEVICE_NAME}' found: {d.address}")
                return d
        print("Device not found in this scan. Retrying...")
        await asyncio.sleep(1)
    print(f"Device '{DEVICE_NAME}' was not found after {n_scan_retries} scan attempts.")
    return None

async def send_measurement_parameters():
    device = await find_device()
    if device is None:
        return

    for attempt in range(1, n_conn_retries + 1):
        print(f"Connection attempt {attempt} to {device.address}...")
        try:
            async with BleakClient(device.address, timeout=conn_timeout) as client:
                if not client.is_connected:
                    print("Connection failed.")
                    continue
                print("Connected!")
                # Get the actual Unix timestamp in Europe/Amsterdam timezone
                tz = pytz.timezone("Europe/Amsterdam")
                real_world_time = int(datetime.now(tz).timestamp())

                # New measurement command format:
                # frequency,realWorldTime,duration,waiting_time,remove_flag
                command = f"{frequency},{real_world_time},{duration},{waiting_time},{remove_flag}"
                print(f"Sending command: {command}")
                await client.write_gatt_char(CHAR_UUID, command.encode())
                print("Parameters sent successfully!")
                return  # Exit after success
        except Exception as e:
            print(f"Error on connection attempt {attempt}: {e}")
        print("Retrying connection in 1 second...")
        await asyncio.sleep(1)
    print("Exceeded maximum connection retry attempts.")

if __name__ == "__main__":
    asyncio.run(send_measurement_parameters())
