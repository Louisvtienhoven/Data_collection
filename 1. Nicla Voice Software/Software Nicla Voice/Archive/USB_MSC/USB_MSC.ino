#include <Arduino.h>
#include <BlockDevice.h>
#include <FileSystem.h>
#include <LittleFileSystem.h>

// Define the file system root
constexpr auto userRoot{"fs"};

// Initialize SPI Flash block device & LittleFS
mbed::BlockDevice* spif = mbed::BlockDevice::get_default_instance();
mbed::LittleFileSystem fs(userRoot);

// Filename stored in SPI Flash
static const char* LOG_FILENAME = "/fs/sensorData.csv";

// ---------------------------------------------------
// Function: Read and Send File Over Serial
// ---------------------------------------------------
void sendFileOverSerial() {
    Serial.println("Checking for sensorData.csv...");

    FILE* file = fopen(LOG_FILENAME, "r");  
    if (!file) {
        Serial.println("ERROR: File not found on SPI Flash!");
        return;
    }

    Serial.println("Sending sensorData.csv...");
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), file)) {
        Serial.print(buffer);  
    }

    fclose(file);
    Serial.println("\nEOF");
}

// ---------------------------------------------------
// Setup Function
// ---------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(3000);
    
    Serial.println("\n--- Nicla Voice Debug: Startup ---");

    // Mount SPI Flash
    Serial.print("Initializing SPI Flash... ");
    if (!spif) {
        Serial.println("ERROR: No SPI Flash found!");
        while (true);
    }

    int status = spif->init();
    if (status) {
        Serial.print("ERROR: SPI Flash init failed: ");
        Serial.println(status);
        while (true);
    }

    Serial.println("done.");
    Serial.print("Mounting LittleFS... ");
    if (fs.mount(spif)) {
        Serial.println("FAILED! Attempting format...");
        if (fs.reformat(spif)) {
            Serial.println("ERROR: Reformat failed!");
            while (true);
        }
    }
    Serial.println("done.");

    Serial.println("Waiting for Python script...");
}



// ---------------------------------------------------
// Loop Function - Waits for Python Trigger
// ---------------------------------------------------
void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "GET_FILE") {
            sendFileOverSerial();
        }
    }
}
