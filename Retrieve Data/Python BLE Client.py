import asyncio
from bleak import BleakClient, BleakScanner

# BLE Device Information
DEVICE_NAME = "NiclaVoice_CSV"
FILEDATA_UUID = "abcdef01-1234-5678-1234-56789abcdef0"
OUTPUT_FILE = "received_sensorData.csv"

# Scan timeout
SCAN_TIMEOUT = 15  # Increase for better reliability
RECONNECT_ATTEMPTS = 5  # Number of times to retry BLE connection

async def scan_for_device():
    """Scans for the Nicla Voice and returns its address."""
    print(f"Scanning for '{DEVICE_NAME}'...")

    for attempt in range(3):  # Retry scanning multiple times
        devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
        for device in devices:
            if device.name and DEVICE_NAME in device.name:
                print(f"Found '{DEVICE_NAME}' at {device.address}")
                return device.address
        
        print(f"Attempt {attempt + 1}: '{DEVICE_NAME}' not found, retrying...")
        await asyncio.sleep(2)  # Short delay before retrying

    print("Failed to find Nicla Voice. Ensure it's powered on and advertising.")
    return None

async def receive_file(device_address):
    """Connects to the Nicla Voice and receives the file over BLE notifications."""
    attempt = 0
    while attempt < RECONNECT_ATTEMPTS:
        try:
            print(f"Connecting to {device_address} (attempt {attempt + 1}/{RECONNECT_ATTEMPTS})...")
            async with BleakClient(device_address) as client:
                if not await client.is_connected():
                    print("Failed to connect.")
                    attempt += 1
                    continue

                print("Connected! Discovering services...")
                await asyncio.sleep(2)  # Delay before fetching services
                services = await client.get_services()
                print(f"Discovered {len(services)} services.")

                with open(OUTPUT_FILE, "wb") as f:
                    file_transfer_complete = asyncio.Event()

                    def notification_handler(sender, data):
                        """Handles incoming BLE file chunks."""
                        nonlocal file_transfer_complete
                        if b"<EOF>" in data:  # Detect End of File marker
                            print("File transfer complete.")
                            file_transfer_complete.set()
                            return
                        
                        print(f"Received {len(data)} bytes")
                        f.write(data)

                    print("Starting BLE notifications for file transfer...")
                    await client.start_notify(FILEDATA_UUID, notification_handler)

                    # Wait until the file transfer is marked complete
                    try:
                        await asyncio.wait_for(file_transfer_complete.wait(), timeout=20)
                    except asyncio.TimeoutError:
                        print("Warning: File transfer timeout!")

                    await client.stop_notify(FILEDATA_UUID)
                    print(f"File received and saved as {OUTPUT_FILE}")
                return  # Exit function if successful

        except Exception as e:
            print(f"Error: {e}. Retrying...")
            attempt += 1
            await asyncio.sleep(3)

    print("Failed to connect after multiple attempts.")

async def main():
    """Main function to scan for the device and receive the file."""
    device_address = await scan_for_device()
    if device_address:
        await receive_file(device_address)

asyncio.run(main())
