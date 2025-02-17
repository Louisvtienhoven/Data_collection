// Only required if not used within the Arduino IDE
#include <Arduino.h>
#include <TimeLib.h>         // Provides setTime(), year(), etc.
#include <ArduinoBLE.h>      // Arduino BLE library

// ---------- MbedOS / LittleFS Includes ----------
#include <BlockDevice.h>
#include <File.h>
#include <FileSystem.h>
#include <LittleFileSystem.h>

// The name of our filesystem root:
constexpr auto userRoot{"fs"};

// The SPIF block device pointer and LittleFS instance
mbed::BlockDevice* spif;
mbed::LittleFileSystem fs{userRoot};

// ---------- BMI270 / Nicla Includes ----------
#include "NDP.h"
#include "BMI270_Init.h"

// Named constants for BMI270
#define READ_START_ADDRESS  0x0C
#define READ_BYTE_COUNT     16
#define SENSOR_DATA_LENGTH  16

// Scale factors
#define ACCEL_SCALE_FACTOR ((2.0f / 32767.0f) * 9.8f)
#define GYRO_SCALE_FACTOR  (1.0f / 16.4f)

// Macro to check sensor status
#define CHECK_STATUS(s)         \
  do {                          \
    if (s) {                    \
      Serial.print("SPI access error in line "); \
      Serial.println(__LINE__); \
      while (1);                \
    }                           \
  } while (0)

// Filename for logging
static const char* LOG_FILENAME = "sensorData.csv";

// ---------------------------------------------------
// Global measurement parameters (set via BLE)
// ---------------------------------------------------
float measurementDurationMin = 0.2;         // in minutes (default: 0.2 = 12 sec)
unsigned int measurementFrequencyHz = 100;  // default frequency
unsigned long measurementRealWorldTime = 1708531200; // default Unix timestamp
bool removeOldFileFlag = true;              // default: remove any old file

// Derived variables (computed when measurement starts)
unsigned long measurementDurationMS = 0;
unsigned long sampleDelayMS = 0;

// Flags for measurement state
volatile bool measurementInitiated = false;  // set via BLE when new parameters are received
volatile bool measurementInProgress = false; // true during an ongoing measurement session

// ---------------------------------------------------
// Globals for file usage and timing
// ---------------------------------------------------
mbed::File logFile;
unsigned long startTime = 0;

// ---------------------------------------------------
// BLE service and characteristic for initiating measurement
// ---------------------------------------------------
BLEService measurementService("12345678-1234-5678-1234-56789abcdef0");  // Arbitrary UUID
BLECharacteristic measurementChar("abcdef01-1234-5678-1234-56789abcdef0", BLEWrite, 64);

// BLE event handler: called when a central writes to measurementChar
// Expected format: "frequency,realWorldTime,duration,remove"
// Example: "200,1708531300,0.5,0" means 200Hz, Unix time=1708531300, 0.5 minutes, do not remove old file.
void onMeasurementWritten(BLEDevice central, BLECharacteristic characteristic) {
  // If a measurement is already in progress, ignore new commands.
  if (measurementInProgress) {
    Serial.println("Measurement already in progress. Ignoring new command.");
    return;
  }
  
  String value = String((const char*)characteristic.value());
  Serial.print("Received measurement parameters via BLE: ");
  Serial.println(value);

  int firstComma = value.indexOf(',');
  int secondComma = value.indexOf(',', firstComma + 1);
  int thirdComma = value.indexOf(',', secondComma + 1);
  if (firstComma > 0 && secondComma > 0 && thirdComma > 0) {
    String freqStr = value.substring(0, firstComma);
    String timeStr = value.substring(firstComma + 1, secondComma);
    String durStr  = value.substring(secondComma + 1, thirdComma);
    String removeStr = value.substring(thirdComma + 1);

    measurementFrequencyHz = freqStr.toInt();
    measurementRealWorldTime = timeStr.toInt();
    measurementDurationMin = durStr.toFloat();
    removeOldFileFlag = (removeStr.toInt() != 0);

    Serial.print("Parsed frequency: "); Serial.println(measurementFrequencyHz);
    Serial.print("Parsed real-world time: "); Serial.println(measurementRealWorldTime);
    Serial.print("Parsed duration (min): "); Serial.println(measurementDurationMin);
    Serial.print("Parsed remove flag: "); Serial.println(removeOldFileFlag);

    // Compute derived variables
    measurementDurationMS = (unsigned long)(measurementDurationMin * 60 * 1000);
    sampleDelayMS = 1000 / measurementFrequencyHz;

    measurementInitiated = true;
  }
  else {
    Serial.println("Invalid parameter format received.");
  }
}

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
  // Adjust for Paris (UTC+1 in winter) by adding 3600 seconds.
  unsigned long localTime = unixTime + 3600;
  setTime(localTime);
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
  String readableTime = convertUnixToReadable(measurementRealWorldTime);
  int n = snprintf(metadata, sizeof(metadata),
                   "Frequency [Hz]:%u\nSample Duration [Min]:%.2f\nReal-World Time:%s\n\n",
                   measurementFrequencyHz, measurementDurationMin, readableTime.c_str());
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
// Function: Execute a measurement session
// ---------------------------------------------------
void executeMeasurementSession() {
  measurementInProgress = true;
  
  int err;
  // Check the remove flag for this session:
  // If true, remove any existing file and open in truncate mode.
  // If false, open the file in append mode.
  if (removeOldFileFlag) {
    fs.remove(LOG_FILENAME);
    Serial.println("Old file removed as requested.");
    err = logFile.open(&fs, LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
  } else {
    err = logFile.open(&fs, LOG_FILENAME, O_WRONLY | O_CREAT | O_APPEND);
  }
  if (err) {
    Serial.print("Error opening log file: ");
    Serial.println(err);
    measurementInProgress = false;
    measurementInitiated = false;
    return;
  }
  
  // Write metadata for this measurement session
  writeMetadataToFile();
  
  // Mark start time for sampling
  startTime = millis();
  Serial.println("Measurement session started.");
  
  // Sampling loop for the duration of the measurement
  while (millis() - startTime < measurementDurationMS) {
    // Read sensor data
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
    
    delay(sampleDelayMS);
    BLE.poll(); // handle BLE events during sampling
  }
  
  logFile.close();
  Serial.println("Measurement session finished and file closed.");
  
  // Reset flags so that a new measurement can be initiated
  measurementInProgress = false;
  measurementInitiated = false;
}

// ---------------------------------------------------
// Setup: Initialize SPI Flash, LittleFS, BMI270, and BLE
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n--- Nicla Voice: BLE-Initiated Measurement Demo ---");

  // --- Initialize BLE ---
  BLE.setLocalName("NiclaVoice");

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  measurementService.addCharacteristic(measurementChar);
  BLE.addService(measurementService);
  measurementChar.setEventHandler(BLEWritten, onMeasurementWritten);
  BLE.advertise();
  Serial.println("BLE advertising started. Waiting for measurement commands...");

  // --- Initialize SPI Flash & LittleFS ---
  Serial.print("Mounting LittleFS... ");
  spif = mbed::BlockDevice::get_default_instance();
  if (!spif) {
    Serial.println("No default block device found!");
    while (1);
  }
  int err = spif->init();
  if (err) {
    Serial.print("BlockDevice init failed: ");
    Serial.println(err);
    while (1);
  }
  err = fs.mount(spif);
  if (err) {
    Serial.println("Mount failed, trying reformat...");
    err = fs.reformat(spif);
    if (err) {
      Serial.print("Reformat failed: ");
      Serial.println(err);
      while (1);
    }
  }
  Serial.println("done.");

  // --- Initialize Nicla & BMI270 ---
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
  
  Serial.println("Setup complete. Waiting for BLE measurement commands...");
}

void loop() {
  BLE.poll();
  if (!BLE.connected()) {
    BLE.advertise();  
  }

  // If a new measurement command has been received and no measurement is in progress, run it.
  if (measurementInitiated && !measurementInProgress) {
    executeMeasurementSession();
  }

  // Check for Serial commands to retrieve CSV or stop execution (only when not measuring)
  if (!measurementInProgress && Serial.available() > 0) {
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
