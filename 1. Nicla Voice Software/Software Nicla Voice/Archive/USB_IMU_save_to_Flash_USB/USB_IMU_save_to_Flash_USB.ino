// Only required if not used within the Arduino IDE
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
// GLOBAL VARIABLES FOR MEASUREMENT PARAMETERS
// ---------------------------------------------------
// (These will be updated via the command string sent from the PC.)
float sampleDurationMin = 0.2;             // in minutes (default)
unsigned int sampleFrequencyHz = 100;      // in Hz (default)
unsigned long sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
unsigned long sampleDelayMs = 1000 / sampleFrequencyHz;
unsigned long realWorldTime = 1708531200;    // default real-world time (Unix timestamp)
bool removeOldFile = true;                   // default flag

// ---------------------------------------------------
// Measurement State Machine
// ---------------------------------------------------
enum MeasurementState { WAITING_FOR_COMMAND, SAMPLING, SAMPLING_DONE };
MeasurementState measState = WAITING_FOR_COMMAND;

// ---------------------------------------------------
// Globals for timing and file usage
// ---------------------------------------------------
mbed::File logFile;
unsigned long startTime = 0;

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
  setTime(unixTime);
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
// Setup: Initialize SPI Flash, LittleFS, BMI270, etc.
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n--- Nicla Voice: BMI270 to SPI Flash Demo (USB Mode) ---");

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
  
  // Now waiting for a measurement command via USB Serial.
  Serial.println("Waiting for measurement command...");
  Serial.println("Send command in format: frequency,real_world_time,duration,remove_flag");
  Serial.println("For example: 55,1708531300,0.05,0");
}

void loop() {
  if (measState == WAITING_FOR_COMMAND) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.indexOf(',') != -1) {
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);
        if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
          Serial.println("Invalid command format. Expected: frequency,real_world_time,duration,remove_flag");
          return;
        }
        sampleFrequencyHz = command.substring(0, firstComma).toInt();
        realWorldTime = command.substring(firstComma + 1, secondComma).toInt();
        sampleDurationMin = command.substring(secondComma + 1, thirdComma).toFloat();
        int removeFlag = command.substring(thirdComma + 1).toInt();
        removeOldFile = (removeFlag == 1);

        sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
        sampleDelayMs = 1000 / sampleFrequencyHz;

        Serial.println("Received measurement command:");
        Serial.print("Frequency [Hz]: ");
        Serial.println(sampleFrequencyHz);
        Serial.print("Real-world time (Unix timestamp): ");
        Serial.println(realWorldTime);
        Serial.print("Duration [Min]: ");
        Serial.println(sampleDurationMin);
        Serial.print("Remove old file: ");
        Serial.println(removeOldFile ? "Yes" : "No");

        if (removeOldFile) {
          fs.remove(LOG_FILENAME);
        }

        int err = logFile.open(&fs, LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
        if (err) {
          Serial.print("Error opening log file: ");
          Serial.println(err);
          while (true) {}
        }
        writeMetadataToFile();
        
        startTime = millis();
        measState = SAMPLING;
        Serial.println("Sampling started...");
      }
    }
  }
  else if (measState == SAMPLING) {

    if (millis() - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = millis();
      ledOn = !ledOn;
      if (ledOn) {
        nicla::leds.setColor(0, 255, 0);
      } else {
        nicla::leds.setColor(0, 0, 0);
      }
    }
    
    unsigned long elapsed = millis() - startTime;
    if (elapsed < sampleDurationMs) {
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
      delay(sampleDelayMs);
    } else {
      Serial.println("Sampling finished. File closed.");
      logFile.close();
      measState = SAMPLING_DONE;
    }
  }


  else if (measState == SAMPLING_DONE) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("GETCSV")) {
        Serial.println("Sending CSV file contents:");
        readAndPrintFile(LOG_FILENAME);
        Serial.println("=== End of CSV ===");
      }
      else if (command.equalsIgnoreCase("STOP")) {
        Serial.println("Stop command received. Halting execution.");
        while (true) {}
      }
    }
  }
}
