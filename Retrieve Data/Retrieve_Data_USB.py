import serial
import time
import os

# Automatically detect the Arduino serial port (Windows, macOS, Linux)
def find_nicla_port():
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Nicla" in port.description or "Arduino" in port.description:
            return port.device  # Return the first matching port
    return None  # No matching port found

# Find port automatically
PORT = "COM3" 
if PORT is None:
    print("ERROR: Nicla Voice not found! Connect via USB.")
    exit(1)

BAUDRATE = 115200
FILENAME = "sensorData.csv"

def main():
    print(f"Connecting to Nicla Voice on {PORT}...")

    # Open serial connection
    with serial.Serial(PORT, BAUDRATE, timeout=3) as ser:
        time.sleep(2)  # Allow time for connection
        
        print("Requesting file transfer...")
        ser.write(b"GET_FILE\n")  # Send request to Nicla
        
        # Read data until EOF
        data = []
        while True:
            line = ser.readline().decode("utf-8").strip()
            if line == "EOF":
                break
            data.append(line)
        
        # Save file in script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, FILENAME)

        with open(file_path, "w") as file:
            file.write("\n".join(data))
        
        print(f"File saved as {file_path}")

if __name__ == "__main__":
    main()
