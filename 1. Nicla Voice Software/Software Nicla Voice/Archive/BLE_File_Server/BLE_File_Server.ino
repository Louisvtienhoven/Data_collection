#include <ArduinoBLE.h>
#include <File.h>
#include <LittleFileSystem.h>

constexpr auto userRoot{"fs"};
mbed::LittleFileSystem fs{userRoot};

// BLE Service & Characteristics
BLEService csvService("12345678-1234-5678-1234-56789abcdef0");
BLECharacteristic fileDataChar("abcdef01-1234-5678-1234-56789abcdef0", BLENotify, 244);  // Max BLE 5.0 packet size
BLECharacteristic controlChar("abcdef02-1234-5678-1234-56789abcdef1", BLEWrite, 1);

const char* filename = "sensorData.csv";
mbed::File logFile;
bool fileOpen = false;
bool transferInProgress = false;
size_t bytesSent = 0;
const size_t CHUNK_SIZE = 244;  // BLE 5.0 allows up to 244 bytes per packet
char buffer[CHUNK_SIZE];

// Function to send next data chunk
void sendNextChunk() {
  if (!fileOpen || !transferInProgress) return;

  ssize_t bytesRead = logFile.read(buffer, CHUNK_SIZE);
  if (bytesRead > 0) {
    fileDataChar.writeValue((const void*)buffer, (int)bytesRead);
    bytesSent += bytesRead;
  } else {
    // End of file marker
    const char eofMarker[] = "<EOF>";
    fileDataChar.writeValue((const void*)eofMarker, sizeof(eofMarker));

    Serial.println("File transfer complete.");
    logFile.close();
    fileOpen = false;
    transferInProgress = false;
    bytesSent = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);  // Allow serial monitor setup

  Serial.println("\n--- BLE 5.0 CSV File Transfer ---");

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1);
  }
  Serial.println("BLE Initialized!");

  // Set BLE parameters
  BLE.setLocalName("NiclaVoice_CSV");
  BLE.setAdvertisedService(csvService);
  BLE.setConnectionInterval(7.5, 15);  // Minimize latency

  csvService.addCharacteristic(fileDataChar);
  csvService.addCharacteristic(controlChar);
  BLE.addService(csvService);

  Serial.println("Starting BLE advertising...");
  BLE.advertise();

  // Event handler for file transfer start
  controlChar.setEventHandler(BLEWritten, [](BLEDevice central, BLECharacteristic characteristic) {
    uint8_t command;
    characteristic.readValue(command);
    if (command == 1 && !transferInProgress) {
      Serial.println("Received start transfer command.");

      if (!fileOpen) {
        int err = logFile.open(&fs, filename, O_RDONLY);
        if (err) {
          Serial.println("Cannot open file!");
          return;
        }
        fileOpen = true;
        transferInProgress = true;
        bytesSent = 0;
        Serial.println("File opened, starting transfer...");
      }
      sendNextChunk();
    }
  });
}


void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      sendNextChunk();
      delay(5);  // Small delay for BLE stack processing
    }

    Serial.println("Client disconnected. Resetting...");
    fileOpen = false;
    transferInProgress = false;
    BLE.advertise();
  }
}
