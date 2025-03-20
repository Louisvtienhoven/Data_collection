import asyncio
from bleak import BleakScanner


async def scan_ble_devices():
    print("Scanning for BLE devices (Low Energy mode)... Running multiple scans.")

    found = False
    for attempt in range(5):  # Try scanning 5 times
        print(f"Scan attempt {attempt + 1}/5...")
        devices = await BleakScanner.discover(timeout=10)  # 10 sec per scan

        for d in devices:
            name = d.name if d.name else "Unknown"
            print(f"Found device: {name} - Address: {d.address}")

            if name == "NiclaVoice":
                found = True
                break  # Stop scanning if found

        if found:
            print("NiclaVoice found!")
            break
        else:
            print("Retrying...")

    if not found:
        print("NiclaVoice was not found. Try restarting the device or moving closer.")


if __name__ == "__main__":
    asyncio.run(scan_ble_devices())
