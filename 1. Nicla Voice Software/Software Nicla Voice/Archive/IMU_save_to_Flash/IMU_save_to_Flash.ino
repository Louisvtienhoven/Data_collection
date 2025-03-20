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
// CONFIGURABLE VARIABLES
// ---------------------------------------------------

// Sampling duration in minutes (e.g., 0.2 = 12 seconds)
static const float SAMPLE_DURATION_MIN = 2;  // Change as needed
// Sampling frequency in Hz
static const unsigned int SAMPLE_FREQUENCY_HZ = 100;  // Change as needed
// Real-world time (Unix timestamp; will be updated later via BLE)
static unsigned long REAL_WORLD_TIME = 1708531200;  // Feb 21, 2024

// If true, old file is removed
const bool REMOVE_OLD_FILE = true;

// Convert sampling duration to milliseconds
static const unsigned long SAMPLE_DURATION_MS = (unsigned long)(SAMPLE_DURATION_MIN * 60 * 1000);
// Compute the delay time between samples in milliseconds
static const unsigned long SAMPLE_DELAY_MS = 1000 / SAMPLE_FREQUENCY_HZ;

// ---------------------------------------------------
// Globals for timing and file usage
// ---------------------------------------------------
mbed::File logFile;
unsigned long startTime = 0;
bool doneSampling = false;
bool fileClosed = false;

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
  String readableTime = convertUnixToReadable(REAL_WORLD_TIME);
  int n = snprintf(metadata, sizeof(metadata),
                   "Frequency [Hz]:%u\nSample Duration [Min]:%.2f\nReal-World Time:%s\n\n",
                   SAMPLE_FREQUENCY_HZ, SAMPLE_DURATION_MIN, readableTime.c_str());
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
// Setup: Initialize SPI Flash, LittleFS, BMI270
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
  
  if (REMOVE_OLD_FILE) {
    fs.remove(LOG_FILENAME);
  }
  
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
  
  // 3) Open log file (truncate mode) and write metadata
  err = logFile.open(&fs, LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
  if (err) {
    Serial.print("Error opening log file: ");
    Serial.println(err);
    while (true) {}
  }
  writeMetadataToFile();
  
  // Mark start time for sampling
  startTime = millis();
  
  Serial.println("Setup complete. Sampling in progress...");
  Serial.println("After sampling, type 'GETCSV' in the Serial Monitor to retrieve the CSV file.");
}

void loop() {
  // While sampling is in progress, record sensor data
  if (!doneSampling) {
    unsigned long elapsed = millis() - startTime;
    if (elapsed < SAMPLE_DURATION_MS) {
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
      delay(SAMPLE_DELAY_MS);
    } else {
      Serial.println("Sampling finished. File closed.");
      logFile.close();
      doneSampling = true;
    }
  }
  
  // After sampling, check for serial commands to retrieve CSV file
  if (doneSampling && Serial.available() > 0) {
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
