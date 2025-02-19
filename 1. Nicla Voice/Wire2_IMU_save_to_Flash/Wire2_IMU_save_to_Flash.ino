#include <Arduino.h>
#include <TimeLib.h>         // Provides setTime(), year(), etc.

// ---------- MbedOS / LittleFS Includes ----------
#include <BlockDevice.h>
#include <File.h>
#include <FileSystem.h>
#include <LittleFileSystem.h>

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

// Filename used for logging
static const char* LOG_FILENAME = "sensorData.csv";

// ---------------------------------------------------
// Global measurement parameters (default values)
// These will be updated when a command is received via Serial
// ---------------------------------------------------
unsigned int sampleFrequencyHz = 100;   // in Hz
float sampleDurationMin = 0.2;          // in minutes
unsigned long realWorldTime = 1708531200; // Unix timestamp
bool removeOldFile = true;

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
// Function: Write one CSV line to the open file
// ---------------------------------------------------
void writeToFile(float ax, float ay, float az,
                 float gx, float gy, float gz)
{
  unsigned long t = millis();
  char lineBuf[128];
  int n = snprintf(lineBuf, sizeof(lineBuf),
                   "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                   t, ax, ay, az, gx, gy, gz);
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
  Serial.println();  // final newline
  file.close();
}

// ---------------------------------------------------
// Function: Convert Unix timestamp to human-readable format
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
                   "Frequency [Hz]:%u\nSample Duration [Min]:%.2f\nReal-World Time:%s\n\n",
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
// Setup: Initialize SPI Flash, LittleFS, BMI270 and Serial
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
    Serial.println("Mount failed, trying reformat...");
    err = fs.reformat(spif);
    if (err) {
      Serial.print("Reformat failed: ");
      Serial.println(err);
      while (true) {}
    }
  }
  Serial.println("done.");

  // 2) Setup Nicla & BMI270
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();
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

  Serial.println("Setup complete.");
  Serial.println("Waiting for measurement command in the format:");
  Serial.println("  frequency,realWorldTime,duration,remove_flag");
  Serial.println("Example: 65,1708531200,0.05,1");
}

// ---------------------------------------------------
// Loop: Wait for a measurement command, then sample, then respond to queries.
// ---------------------------------------------------
void loop() {
  // State: WAITING_FOR_COMMAND
  if (currentState == WAITING_FOR_COMMAND) {
    if (Serial.available() > 0) {
      // Read the incoming command line (terminated by newline)
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.length() > 0) {
        // Expecting comma-separated values: frequency,realWorldTime,duration,remove_flag
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);
        if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
          Serial.println("Invalid command format. Please use: frequency,realWorldTime,duration,remove_flag");
        } else {
          String freqStr = command.substring(0, firstComma);
          String timeStr = command.substring(firstComma + 1, secondComma);
          String durStr  = command.substring(secondComma + 1, thirdComma);
          String removeStr = command.substring(thirdComma + 1);
          sampleFrequencyHz = freqStr.toInt();
          realWorldTime = timeStr.toInt();
          sampleDurationMin = durStr.toFloat();
          removeOldFile = (removeStr.toInt() != 0);

          // Recalculate delay and duration in milliseconds based on new parameters
          g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
          if (sampleFrequencyHz > 0) {
            g_sampleDelayMs = 1000 / sampleFrequencyHz;
          } else {
            g_sampleDelayMs = 10; // fallback delay
          }

          Serial.println("Measurement parameters updated:");
          Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
          Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
          Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
          Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");

          // Remove the old file if needed
          if (removeOldFile) {
            fs.remove(LOG_FILENAME);
          }

          // Open log file (truncate mode) and write metadata
          int err = logFile.open(&fs, LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
          if (err) {
            Serial.print("Error opening log file: ");
            Serial.println(err);
            return;
          }
          writeMetadataToFile();

          // Mark start time for sampling and change state to SAMPLING
          startTime = millis();
          currentState = SAMPLING;
          Serial.println("Sampling started...");
        }
      }
    }
  }
  // State: SAMPLING - record sensor data until duration has elapsed
  else if (currentState == SAMPLING) {
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
      currentState = FINISHED;
    }
  }
  // State: FINISHED - wait for further serial commands (e.g., GETCSV or STOP)
  else if (currentState == FINISHED) {
    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("GETCSV")) {
        Serial.println("Sending CSV file contents:");
        readAndPrintFile(LOG_FILENAME);
        Serial.println("=== End of CSV ===");
      }
      else if (cmd.equalsIgnoreCase("STOP")) {
        Serial.println("Stop command received. Halting execution.");
        while (true) {}
      }
      // Optionally, you could allow restarting measurements by returning to WAITING_FOR_COMMAND
      else if (cmd.length() > 0) {
        Serial.println("Unknown command. Available commands: GETCSV, STOP");
      }
    }
  }
}
