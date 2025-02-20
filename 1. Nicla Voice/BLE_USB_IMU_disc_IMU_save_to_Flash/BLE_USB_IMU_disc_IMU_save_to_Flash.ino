//To include the Arduino framework
#include <Arduino.h>
#include <TimeLib.h>         // Provides setTime(), year(), etc.

// ---------- MbedOS / LittleFS Includes ----------
#include <BlockDevice.h>
#include <File.h>
#include <FileSystem.h>
#include <LittleFileSystem.h>

// ---------- BLE Includes ----------
#include <ArduinoBLE.h>

// The name of our filesystem root:
constexpr auto userRoot{"fs"};

// The SPIF block device pointer
mbed::BlockDevice* spif;
// LittleFileSystem instance
mbed::LittleFileSystem fs{userRoot};

// ---------- BMI270 / Nicla Includes ----------
#include "NDP.h"
#include "BMI270_Init.h"

// Named constants for BMI270
#define READ_START_ADDRESS  0x0C
#define READ_BYTE_COUNT     16
#define SENSOR_DATA_LENGTH  16

// Accel range = +/- 2g => scale:
#define ACCEL_SCALE_FACTOR ((2.0f / 32767.0f) * 9.8f)
// Gyro range = 2000 dps => scale:
#define GYRO_SCALE_FACTOR (1.0f / 16.4f)

// Macro to check sensor status
#define CHECK_STATUS(s)         \
  do {                          \
    if (s) {                    \
      Serial.print("SPI access error in line "); \
      Serial.println(__LINE__); \
      while (1);                \
    }                           \
  } while (0)

// Filenames used for logging and for storing the command
static const char* LOG_FILENAME = "sensorData.csv";
static const char* CMD_FILENAME = "command.txt";

// ---------------------------------------------------
// Global measurement parameters (default values)
// These will be updated when a command is received via Serial or BLE.
unsigned int sampleFrequencyHz = 100;   // in Hz
float sampleDurationMin = 0.2;          // in minutes
unsigned long realWorldTime = 1708531200; // Unix timestamp
bool removeOldFile = false;             // Default is NOT to remove the log file

// These will be computed based on the measurement parameters
unsigned long g_sampleDurationMs = 0;
unsigned long g_sampleDelayMs = 0;

// ---------------------------------------------------
// Globals for timing and file usage
// ---------------------------------------------------
mbed::File logFile;
unsigned long startTime = 0;

// Define a simple state machine for measurement operation
enum MeasurementState {
  WAITING_FOR_COMMAND,
  SAMPLING,
  FINISHED
};

MeasurementState currentState = WAITING_FOR_COMMAND;

// ---------------------------------------------------
// Global variables for LED blinking in battery mode
// ---------------------------------------------------
bool batteryMode = false;               // set true if measurement loaded from flash
unsigned long previousBlinkTime = 0;
const unsigned long blinkInterval = 500;  // blink every 500ms
bool ledOn = false;

// ---------------------------------------------------
// BLE: Define service and characteristic for receiving commands.
// The service UUID is arbitrary; the characteristic UUID must match your Python script.
BLEService measurementService("abcdef00-1234-5678-1234-56789abcdef0");
BLEStringCharacteristic measurementCharacteristic("abcdef01-1234-5678-1234-56789abcdef0", BLEWrite, 64);

// ---------------------------------------------------
// Startup Blink Function: Blink red followed by blue
// ---------------------------------------------------
void startupBlink() {
  // Blink red for 500ms then off for 250ms
  nicla::leds.setColor(255, 0, 0);
  delay(500);
  nicla::leds.setColor(0, 0, 0);
  delay(250);
  
  // Blink blue for 500ms then off for 250ms
  nicla::leds.setColor(0, 0, 255);
  delay(500);
  nicla::leds.setColor(0, 0, 0);
  delay(250);
}

// ---------------------------------------------------
// Function: Write one CSV line to the open file
// ---------------------------------------------------
void writeToFile(float ax, float ay, float az,
                 float gx, float gy, float gz)
{
  char lineBuf[128];
  int n = snprintf(lineBuf, sizeof(lineBuf),
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                   ax, ay, az, gx, gy, gz);
  if (n > 0 && n < (int)sizeof(lineBuf)) {
    ssize_t written = logFile.write(lineBuf, n);
    if (written < 0) {
      Serial.print("File write error: ");
      Serial.println(written);
    }
  } else {
    Serial.println("Error formatting CSV line or buffer too small.");
  }
}

// ---------------------------------------------------
// Function: Read entire file from flash and print via Serial
// ---------------------------------------------------
void readAndPrintFile(const char* filename)
{
  mbed::File file;
  int err = file.open(&fs, filename, O_RDONLY);
  if (err) {
    Serial.print("Cannot open file '");
    Serial.print(filename);
    Serial.println("' for reading.");
    return;
  }
  size_t fsize = file.size();
  Serial.print("File '");
  Serial.print(filename);
  Serial.print("' size: ");
  Serial.print(fsize);
  Serial.println(" bytes.");
  const size_t chunkSize = 128;
  char buf[chunkSize + 1] = {0};
  while (true) {
    ssize_t readBytes = file.read(buf, chunkSize);
    if (readBytes <= 0) break;
    buf[readBytes] = '\0';
    Serial.print(buf);
  }
  Serial.println();
  file.close();
}

// ---------------------------------------------------
// Function: Convert Unix timestamp to human-readable format
// (Adjusted to show one hour later, e.g. Amsterdam time)
// ---------------------------------------------------
String convertUnixToReadable(unsigned long unixTime) {
  if (unixTime == 0) {
    return "Time not set";
  }
  // Add one hour (3600 seconds) for Amsterdam time adjustment
  setTime(unixTime + 3600);
  char timeStr[25];
  snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
  return String(timeStr);
}

// ---------------------------------------------------
// Function: Write metadata (frequency, duration, real-world time) to file
// ---------------------------------------------------
void writeMetadataToFile() {
  char metadata[128];
  String readableTime = convertUnixToReadable(realWorldTime);
  int n = snprintf(metadata, sizeof(metadata),
                   "Sampling Frequency [Hz]:%u\nSample Duration [Min]:%.2f\nTime:%s\n\n",
                   sampleFrequencyHz, sampleDurationMin, readableTime.c_str());
  if (n > 0 && n < (int)sizeof(metadata)) {
    ssize_t written = logFile.write(metadata, n);
    if (written < 0) {
      Serial.print("File write error: ");
      Serial.println(written);
    }
    logFile.sync();
  } else {
    Serial.println("Error formatting metadata.");
  }
}

// ---------------------------------------------------
// Function: Process a measurement command string (from Serial or BLE)
// ---------------------------------------------------
void processMeasurementCommand(String command) {
  command.trim();
  if (command.length() == 0) return;

  // Persist the command in flash.
  storeMeasurementCommand(command);
  
  // Parse command: expected format "frequency,realWorldTime,duration,remove_flag"
  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);
  int thirdComma = command.indexOf(',', secondComma + 1);
  if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
    Serial.println("Invalid command format. Please use: frequency,realWorldTime,duration,remove_flag");
    return;
  }
  String freqStr = command.substring(0, firstComma);
  String timeStr = command.substring(firstComma + 1, secondComma);
  String durStr  = command.substring(secondComma + 1, thirdComma);
  String removeStr = command.substring(thirdComma + 1);
  sampleFrequencyHz = freqStr.toInt();
  realWorldTime = timeStr.toInt();
  sampleDurationMin = durStr.toFloat();
  removeOldFile = (removeStr.toInt() != 0);

  // Recalculate timing parameters.
  g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
  g_sampleDelayMs = (sampleFrequencyHz > 0) ? (1000 / sampleFrequencyHz) : 10;

  Serial.println("Measurement parameters updated:");
  Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
  Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
  Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
  Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");

  // Open the log file.
  int flags = O_WRONLY | O_CREAT;
  if (removeOldFile) {
    flags |= O_TRUNC;
  } else {
    flags |= O_APPEND;
  }
  int err = logFile.open(&fs, LOG_FILENAME, flags);
  if (err) {
    Serial.print("Error opening log file: ");
    Serial.println(err);
    return;
  }
  writeMetadataToFile();

  Serial.println("Measurement command received.");
  Serial.println("Waiting 5 seconds for cable disconnection and power source change...");
  delay(5000);
  startTime = millis();
  currentState = SAMPLING;
  Serial.println("Sampling started...");
}

// ---------------------------------------------------
// BLE Characteristic callback: Called when a central writes to our characteristic.
// ---------------------------------------------------
void measurementCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  String command = String((const char*) characteristic.value());
  Serial.print("BLE command received: ");
  Serial.println(command);
  processMeasurementCommand(command);
}

// ---------------------------------------------------
// Store the received measurement command persistently
// ---------------------------------------------------
void storeMeasurementCommand(const String &command) {
  mbed::File file;
  int err = file.open(&fs, CMD_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
  if (err) {
    Serial.println("Error opening command file for writing.");
    return;
  }
  file.write(command.c_str(), command.length());
  file.sync();
  file.close();
  Serial.println("Measurement command stored in flash.");
}

// ---------------------------------------------------
// Load the measurement command from persistent storage.
// Returns true if a valid command was loaded.
bool loadMeasurementCommand() {
  mbed::File file;
  int err = file.open(&fs, CMD_FILENAME, O_RDONLY);
  if (err) {
    return false; // No command stored
  }
  char buffer[128] = {0};
  ssize_t n = file.read(buffer, sizeof(buffer)-1);
  file.close();
  if(n <= 0) return false;
  String command = String(buffer);
  command.trim();
  if (command.length() == 0) return false;
  
  // Parse the command string: expecting "frequency,realWorldTime,duration,remove_flag"
  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);
  int thirdComma = command.indexOf(',', secondComma + 1);
  if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
    Serial.println("Stored command has invalid format.");
    return false;
  }
  String freqStr = command.substring(0, firstComma);
  String timeStr = command.substring(firstComma + 1, secondComma);
  String durStr  = command.substring(secondComma + 1, thirdComma);
  String removeStr = command.substring(thirdComma + 1);
  sampleFrequencyHz = freqStr.toInt();
  realWorldTime = timeStr.toInt();
  sampleDurationMin = durStr.toFloat();
  removeOldFile = (removeStr.toInt() != 0);
  
  // Compute timing parameters
  g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
  g_sampleDelayMs = (sampleFrequencyHz > 0) ? (1000 / sampleFrequencyHz) : 10;
  
  Serial.println("Measurement command loaded from flash:");
  Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
  Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
  Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
  Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");
  
  // Remove the stored command so it doesn't restart measurement again unintentionally
  fs.remove(CMD_FILENAME);
  
  return true;
}

// ---------------------------------------------------
// Setup: Initialize SPI Flash, LittleFS, BMI270, BLE, and check for stored command.
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n--- Nicla Voice: BMI270 to SPI Flash Demo ---");

  // 1) Initialize SPI Flash & LittleFS
  Serial.print("Mounting LittleFS... ");
  spif = mbed::BlockDevice::get_default_instance();
  if (!spif) {
    Serial.println("No default block device found!");
    while (true) {}
  }
  int err = spif->init();
  if (err) {
    Serial.print("BlockDevice init failed: ");
    Serial.println(err);
    while (true) {}
  }
  err = fs.mount(spif);
  if (err) {
    Serial.print("Mount failed (error ");
    Serial.print(err);
    Serial.println("). Not reformatting to preserve data.");
    while (true) { delay(1000); }
  }
  Serial.println("done.");
  
  // Check if measurement was already completed by looking for the flag file.
  {
    mbed::File doneFile;
    if (doneFile.open(&fs, "done.txt", O_RDONLY) == 0) {
      Serial.println("Measurement already completed. Skipping new measurement.");
      doneFile.close();
      // Open sensorData.csv in append mode so that data is preserved.
      int flags = O_WRONLY | O_CREAT | O_APPEND;
      err = logFile.open(&fs, LOG_FILENAME, flags);
      if (err) {
        Serial.print("Error opening log file: ");
        Serial.println(err);
        while (true) {}
      }
      currentState = FINISHED;
      return; // Exit setup; loop() will handle retrieval commands.
    }
  }
  
  // 2) Setup Nicla & BMI270
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();
  // Call the startup blink: red then blue.
  startupBlink();
  
  Serial.println("- NDP processor initialization...");
  NDP.begin("mcu_fw_120_v91.synpkg");
  NDP.load("dsp_firmware_v91.synpkg");
  Serial.println("- NDP processor initialization done!");
  
  {
    uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];
    int status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x7E, 0xB6);
    CHECK_STATUS(status);
    delay(20);
    status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x7C, 0x00);
    CHECK_STATUS(status);
    delay(20);
    status = NDP.sensorBMI270Write(0x59, 0x00);
    CHECK_STATUS(status);
    Serial.println("- BMI270 initialization starting...");
    status = NDP.sensorBMI270Write(
      0x5E,
      sizeof(bmi270_maximum_fifo_config_file),
      (uint8_t*)bmi270_maximum_fifo_config_file
    );
    CHECK_STATUS(status);
    Serial.println("- BMI270 Initialization done!");
    status = NDP.sensorBMI270Write(0x59, 0x01);
    CHECK_STATUS(status);
    delay(200);
    status = NDP.sensorBMI270Read(0x21, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x7D, 0x0E);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x40, 0xA8);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x41, 0x00);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x42, 0xA9);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x43, 0x00);
    CHECK_STATUS(status);
  }
  
  // 3) Setup BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("NiclaVoice");
  BLE.setAdvertisedService(measurementService);
  measurementService.addCharacteristic(measurementCharacteristic);
  BLE.addService(measurementService);
  measurementCharacteristic.setEventHandler(BLEWritten, measurementCharacteristicWritten);
  BLE.advertise();
  Serial.println("BLE device active, waiting for connections...");
  
  // 4) Check if a measurement command was previously stored.
  if (loadMeasurementCommand()) {
    // A stored command indicates we are now on battery power.
    batteryMode = true;
    
    if (removeOldFile) {
      fs.remove(LOG_FILENAME);
    }
    
    int flags = O_WRONLY | O_CREAT;
    if (removeOldFile) {
      flags |= O_TRUNC;
    } else {
      flags |= O_APPEND;
    }
    
    err = logFile.open(&fs, LOG_FILENAME, flags);
    if (err) {
      Serial.print("Error opening log file: ");
      Serial.println(err);
      while (true) {}
    }
    writeMetadataToFile();
    Serial.println("Stored measurement command found.");
    Serial.println("Waiting 5 seconds for cable disconnection and power source change...");
    delay(5000);
    startTime = millis();
    currentState = SAMPLING;
    Serial.println("Sampling started...");
  }
  else {
    Serial.println("Waiting for measurement command in the format:");
    Serial.println("  frequency,realWorldTime,duration,remove_flag");
    Serial.println("Example: 65,1708531200,0.05,1");
    currentState = WAITING_FOR_COMMAND;
  }
}

// ---------------------------------------------------
// Loop: Process commands (from Serial and BLE) and perform sampling.
// ---------------------------------------------------
void loop() {
  // Poll BLE events.
  BLE.poll();

  // If waiting for a measurement command, check Serial for input.
  if (currentState == WAITING_FOR_COMMAND) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      if (command.equalsIgnoreCase("GETCSV")) {
        Serial.println("Sending CSV file contents:");
        readAndPrintFile(LOG_FILENAME);
        Serial.println("=== End of CSV ===");
        return;
      }
      else if (command.equalsIgnoreCase("CLRFLAG")) {
        mbed::File flagCheck;
        int ret = flagCheck.open(&fs, "done.txt", O_RDONLY);
        if(ret == 0) {
          flagCheck.close();
          int rem = fs.remove("done.txt");
          if(rem == 0) {
            Serial.println("done.txt removed.");
          } else {
            Serial.print("Error removing done.txt, error code: ");
            Serial.println(rem);
          }
        } else {
          Serial.println("done.txt does not exist.");
        }
        int remCSV = fs.remove(LOG_FILENAME);
        if(remCSV == 0) {
          Serial.println("sensorData.csv removed.");
        } else {
          Serial.print("Error removing sensorData.csv, error code: ");
          Serial.println(remCSV);
        }
        currentState = WAITING_FOR_COMMAND;
        return;
      }
      else if (command.equalsIgnoreCase("STOP")) {
        Serial.println("Stop command received. Halting execution.");
        while (true) {}
      }
      // Process as measurement command.
      if (command.length() > 0) {
        processMeasurementCommand(command);
      }
    }
  }
  // SAMPLING state: record sensor data.
  else if (currentState == SAMPLING) {
    if (batteryMode) {
      if (millis() - previousBlinkTime >= blinkInterval) {
        previousBlinkTime = millis();
        ledOn = !ledOn;
        if (ledOn) {
          nicla::leds.setColor(0, 255, 0);
        } else {
          nicla::leds.setColor(0, 0, 0);
        }
      }
    }
  
    unsigned long elapsed = millis() - startTime;
    if (elapsed < g_sampleDurationMs) {
      uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];
      int16_t x_acc_raw, y_acc_raw, z_acc_raw;
      int16_t x_gyr_raw, y_gyr_raw, z_gyr_raw;
      
      int status = NDP.sensorBMI270Read(READ_START_ADDRESS, READ_BYTE_COUNT, sensor_data);
      CHECK_STATUS(status);
      
      x_acc_raw = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
      y_acc_raw = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
      z_acc_raw = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);
      x_gyr_raw = (int16_t)((sensor_data[7] << 8) | sensor_data[6]);
      y_gyr_raw = (int16_t)((sensor_data[9] << 8) | sensor_data[8]);
      z_gyr_raw = (int16_t)((sensor_data[11] << 8) | sensor_data[10]);
      
      float x_acc = x_acc_raw * ACCEL_SCALE_FACTOR;
      float y_acc = y_acc_raw * ACCEL_SCALE_FACTOR;
      float z_acc = z_acc_raw * ACCEL_SCALE_FACTOR;
      float x_gyr = x_gyr_raw * GYRO_SCALE_FACTOR;
      float y_gyr = y_gyr_raw * GYRO_SCALE_FACTOR;
      float z_gyr = z_gyr_raw * GYRO_SCALE_FACTOR;
      
      writeToFile(x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr);
      delay(g_sampleDelayMs);
    } else {
      Serial.println("Sampling finished. File closed.");
      logFile.close();
      
      // Create flag file to indicate measurement completion.
      mbed::File flagFile;
      int err = flagFile.open(&fs, "done.txt", O_WRONLY | O_CREAT | O_TRUNC);
      if (!err) {
        flagFile.write("done", 4);
        flagFile.sync();
        flagFile.close();
      } else {
        Serial.println("Error creating done.txt flag file.");
      }
      
      currentState = FINISHED;
    }
  }
  // FINISHED state: process retrieval commands and blink red/blue.
  else if (currentState == FINISHED) {
    // Blink red and blue continuously.
    static unsigned long finishedPreviousBlinkTime = 0;
    static bool finishedBlinkToggle = false;
    if (millis() - finishedPreviousBlinkTime >= blinkInterval) {
      finishedPreviousBlinkTime = millis();
      finishedBlinkToggle = !finishedBlinkToggle;
      if (finishedBlinkToggle) {
        nicla::leds.setColor(255, 0, 0);  // Red
      } else {
        nicla::leds.setColor(0, 0, 255);  // Blue
      }
    }
    
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("GETCSV")) {
        Serial.println("Sending CSV file contents:");
        readAndPrintFile(LOG_FILENAME);
        Serial.println("=== End of CSV ===");
      }
      else if (cmd.equalsIgnoreCase("CLRFLAG")) {
        mbed::File flagCheck;
        int ret = flagCheck.open(&fs, "done.txt", O_RDONLY);
        if(ret == 0) {
          flagCheck.close();
          int rem = fs.remove("done.txt");
          if(rem == 0) {
            Serial.println("done.txt removed.");
          } else {
            Serial.print("Error removing done.txt, error code: ");
            Serial.println(rem);
          }
        } else {
          Serial.println("done.txt does not exist.");
        }
        int remCSV = fs.remove(LOG_FILENAME);
        if(remCSV == 0) {
          Serial.println("sensorData.csv removed.");
        } else {
          Serial.print("Error removing sensorData.csv, error code: ");
          Serial.println(remCSV);
        }
        currentState = WAITING_FOR_COMMAND;
  
        Serial.println();
        Serial.print("currentState has been set to WAITING_FOR_COMMAND");
        Serial.println();
      }
      else if (cmd.equalsIgnoreCase("STOP")) {
        Serial.println("Stop command received. Halting execution.");
        while (true) {}
      }
      else if (cmd.length() > 0) {
        Serial.println("Unknown command. Available commands: GETCSV, CLRFLAG, STOP");
      }
    }
  }
}
