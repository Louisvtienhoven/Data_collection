
#include "NDP.h"
#include "BMI270_Init.h"
#include <LittleFS.h>


// Named constants
#define READ_START_ADDRESS 0x0C
#define READ_BYTE_COUNT 16
#define SENSOR_DATA_LENGTH 16
#define ACCEL_SCALE_FACTOR ((2.0 / 32767.0) * 9.8)
#define GYRO_SCALE_FACTOR (1 / 16.4)

#define SAMPLE_RATE 100  // Hz (100 samples per second)
#define RECORD_DURATION 5  // Recording duration in seconds
#define WAIT_DURATION 1  // Time to wait before sending file (seconds)

#define NUM_SAMPLES (SAMPLE_RATE * RECORD_DURATION) // Total samples to store

const char* filename = "/sensor_data.csv";  // File on SPI Flash
bool recording_done = false;
unsigned long start_time, wait_start_time;
File dataFile;

// Macro for error handling
#define CHECK_STATUS(s) \
  do { \
    if (s) { \
      Serial.print("SPI access error in line "); \
      Serial.println(__LINE__); \
      return; \
    } \
  } while (0)

void setup() {
  Serial.begin(115200);

  // Initialize SPI Flash (LittleFS)
  if (!LittleFS.begin()) {
    Serial.println("LittleFS initialization failed!");
    while (1);
  }
  Serial.println("LittleFS initialized.");

  // Delete old file to avoid appending to previous data
  LittleFS.remove(filename);

  // Open file for writing
  dataFile = LittleFS.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("Failed to open SPI Flash file for writing.");
    while (1);
  }

  // Write CSV headers
  dataFile.println("Time (ms),x_acc (m/s²),y_acc (m/s²),z_acc (m/s²),x_gyr (°/s),y_gyr (°/s),z_gyr (°/s)");
  dataFile.flush();

  // Initialize Nicla Voice & BMI270 Sensor
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  int status;
  status = NDP.sensorBMI270Write(0x7E, 0xB6); // Reset sensor
  CHECK_STATUS(status);
  delay(20);

  status = NDP.sensorBMI270Write(0x7D, 0x0E); // Normal power mode
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x40, 0xA8); // Accelerometer ODR: 200Hz
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x41, 0x00); // ±2g range
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x42, 0xA9); // Gyroscope ODR: 400Hz
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x43, 0x00); // ±2000 dps range
  CHECK_STATUS(status);

  Serial.println("BMI270 sensor initialized!");

  start_time = millis();
}

void loop() {
  unsigned long elapsed_time = millis() - start_time;

  // Stop recording after RECORD_DURATION seconds
  if (!recording_done && elapsed_time > RECORD_DURATION * 1000) {
    Serial.println("Recording complete. Waiting 15 seconds before sending file...");
    recording_done = true;
    dataFile.close();  // Close the file after recording
    wait_start_time = millis();
  }

  // Store sensor data
  if (!recording_done) {
    uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];
    int16_t x_acc_raw, y_acc_raw, z_acc_raw, x_gyr_raw, y_gyr_raw, z_gyr_raw;
    float x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr;
    int status;

    // Read sensor data
    status = NDP.sensorBMI270Read(READ_START_ADDRESS, READ_BYTE_COUNT, &sensor_data[0]);
    CHECK_STATUS(status);

    // Parse accelerometer data
    x_acc_raw = (sensor_data[0] | sensor_data[1] << 8);
    y_acc_raw = (sensor_data[2] | sensor_data[3] << 8);
    z_acc_raw = (sensor_data[4] | sensor_data[5] << 8);
    x_gyr_raw = (sensor_data[6] | sensor_data[7] << 8);
    y_gyr_raw = (sensor_data[8] | sensor_data[9] << 8);
    z_gyr_raw = (sensor_data[10] | sensor_data[11] << 8);

    // Convert raw values to physical units
    x_acc = x_acc_raw * ACCEL_SCALE_FACTOR;
    y_acc = y_acc_raw * ACCEL_SCALE_FACTOR;
    z_acc = z_acc_raw * ACCEL_SCALE_FACTOR;
    x_gyr = x_gyr_raw * GYRO_SCALE_FACTOR;
    y_gyr = y_gyr_raw * GYRO_SCALE_FACTOR;
    z_gyr = z_gyr_raw * GYRO_SCALE_FACTOR;

    // Write data to SPI Flash file
    dataFile.print(elapsed_time);
    dataFile.print(",");
    dataFile.print(x_acc);
    dataFile.print(",");
    dataFile.print(y_acc);
    dataFile.print(",");
    dataFile.print(z_acc);
    dataFile.print(",");
    dataFile.print(x_gyr);
    dataFile.print(",");
    dataFile.print(y_gyr);
    dataFile.print(",");
    dataFile.println(z_gyr);
    dataFile.flush();  // Ensure data is saved

   // delay(10);  // Ensures 100 Hz sampling rate
    delayMicroseconds(625);
  }

  // Wait 15 seconds, then send file to computer
  if (recording_done && millis() - wait_start_time >= WAIT_DURATION * 1000) {
    Serial.println("Sending CSV file to computer...");

    // Provide file to the user
    if (LittleFS.exists(filename)) {
      File dataFile = LittleFS.open(filename, FILE_READ);
      if (dataFile) {
        Serial.println("READY_TO_DOWNLOAD"); // Signal for user to retrieve the file
        LittleFS.download(filename); // Trigger download to computer
        dataFile.close();
      } else {
        Serial.println("Error: Could not open file for download.");
      }
    } else {
      Serial.println("Error: CSV file not found.");
    }

    while (1);  // Stop execution after sending file
  }
}
