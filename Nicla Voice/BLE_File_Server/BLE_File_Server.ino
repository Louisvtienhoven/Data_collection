#include <ArduinoBLE.h>
#include <File.h>
#include <LittleFileSystem.h>

constexpr auto userRoot{"fs"};
mbed::LittleFileSystem fs{userRoot};

// BLE Service and Characteristics
BLEService csvService("12345678-1234-5678-1234-56789abcdef0");  // Custom UUID
BLECharacteristic fileDataChar("abcdef01-1234-5678-1234-56789abcdef0", 
                              BLENotify, 128); // Sends file data
BLECharacteristic controlChar("abcdef02-1234-5678-1234-56789abcdef0",
                              BLEWrite, 1); // Client requests next chunk

const char* filename = "sensorData.csv";
mbed::File logFile;
bool fileOpen = false;
size_t fileSize = 0;
size_t bytesSent = 0;
const size_t CHUNK_SIZE = 128;
char buffer[CHUNK_SIZE];

// Function to send next file chunk
void sendNextChunk() {
  if (!fileOpen) return;

  ssize_t bytesRead = logFile.read(buffer, CHUNK_SIZE);
  if (bytesRead > 0) {
    fileDataChar.writeValue((const void*)buffer, (int)bytesRead);
    bytesSent += bytesRead;
  } else {
    Serial.println("File transfer complete.");
    logFile.close();
    fileOpen = false;
    bytesSent = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n--- Nicla Voice BLE CSV Transfer ---");

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set device name and advertise service
  BLE.setLocalName("NiclaVoice_CSV");
  BLE.setAdvertisedService(csvService);

  // Add characteristics to service
  csvService.addCharacteristic(fileDataChar);
  csvService.addCharacteristic(controlChar);
  BLE.addService(csvService);

  // Start advertising
  BLE.advertise();
  Serial.println("BLE advertising started.");

  // Event handler for controlChar
  controlChar.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic characteristic) {
    uint8_t command;
    characteristic.readValue(command);
    if (command == 1 && !fileOpen) {
      // Open the file
      int err = logFile.open(&fs, filename, O_RDONLY);
      if (err) {
        Serial.println("Cannot open file!");
        return;
      }
      fileSize = logFile.size();
      fileOpen = true;
      bytesSent = 0;
      Serial.println("File opened, starting transfer...");
    }
    sendNextChunk();
  });

  Serial.println("Setup complete. Waiting for BLE connection.");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (fileOpen) {
        sendNextChunk();
        delay(50);  // Avoid overloading BLE with rapid packets
      }
    }

    Serial.println("Client disconnected. Resetting.");
    fileOpen = false;
  }
}
