import asyncio
import time  # to get the current timestamp
from datetime import datetime
import pytz  # for timezone conversion
from bleak import BleakClient, BleakScanner

# Measurement parameters
frequency = 60      # Hz
duration = 0.05     # in minutes
remove_flag = 0     # 0 = false, 1 = true

# BLE Variables
n_retries = 10
t_timeout = 10

# Use the MAC determined through find_MAC.py
DEVICE_ADDRESS = "B6:BE:83:89:82:AA"  # Replace with your Nicla Voice MAC address

# UUID of the characteristic to write measurement parameters
CHAR_UUID = "abcdef01-1234-5678-1234-56789abcdef0"  # Ensure this matches the Arduino UUID

async def send_measurement_parameters():
    max_retries = n_retries
    for attempt in range(1, max_retries + 1):
        print(f"Attempt {attempt} to connect to Nicla Voice at {DEVICE_ADDRESS}...")
        try:
            async with BleakClient(DEVICE_ADDRESS, timeout=t_timeout) as client:
                if not client.is_connected:
                    print(f"Failed to connect on attempt {attempt}.")
                    continue

                print("Connected! Sending measurement parameters...")

                # Get the actual Unix timestamp in the correct timezone (Europe/Amsterdam)
                tz = pytz.timezone("Europe/Amsterdam")  # UTC+1 in winter, UTC+2 in summer
                real_world_time = int(datetime.now(tz).timestamp())

                # Format the command string
                command = f"{frequency},{real_world_time},{duration},{remove_flag}"
                print(f"Sending command: {command}")

                await client.write_gatt_char(CHAR_UUID, command.encode())
                print("Parameters sent successfully! Disconnecting...")
                return  # exit function after success
        except Exception as e:
            print(f"Error on attempt {attempt}: {e}")
        print("Retrying connection in 1 second...")
        await asyncio.sleep(1)
    print("Exceeded maximum retry attempts. Could not connect to Nicla Voice.")

if __name__ == "__main__":
    asyncio.run(send_measurement_parameters())
