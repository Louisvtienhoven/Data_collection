import serial
import time

# Configure the serial port and baud rate.
SERIAL_PORT = 'COM3'  # Change this to your serial port, e.g., '/dev/ttyACM0'
BAUD_RATE = 115200


def retrieve_csv():
    # Open the serial port.
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    time.sleep(2)  # Allow time for the serial connection to initialize

    # Clear any existing data in the input buffer.
    ser.reset_input_buffer()

    # Send the GETCSV command.
    ser.write(b'GETCSV\n')
    time.sleep(1)  # Wait a moment for the device to respond.

    # Read all available data.
    csv_data = ser.read_all().decode('utf-8', errors='replace')

    # Now send the CLRFLAG command to clear done.txt.
    ser.write(b'CLRFLAG\n')
    time.sleep(0.5)  # Wait for the Arduino to process the command.
    flag_response = ser.read_all().decode('utf-8', errors='replace')

    ser.close()
    return csv_data, flag_response


if __name__ == '__main__':
    csv_content, flag_resp = retrieve_csv()
    print("Retrieved CSV Data:")
    print(csv_content)
    print("Flag response:")
    print(flag_resp)

    # Save the CSV data to a local file.
    with open("downloaded.csv", "w", encoding="utf-8") as f:
        f.write(csv_content)
    print("CSV file saved as 'downloaded.csv'.")
