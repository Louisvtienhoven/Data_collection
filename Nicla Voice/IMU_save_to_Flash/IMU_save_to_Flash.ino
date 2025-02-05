#include <Arduino.h>

// ---------- MbedOS / LittleFS Includes ----------
#include <BlockDevice.h>
#include <File.h>
#include <FileSystem.h>
#include <LittleFileSystem.h>

// The name of our filesystem root:
constexpr auto userRoot{"fs"};

// The SPIF block device pointer
mbed::BlockDevice* spif;
// LittleFS instance
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

// Macros to check sensor status
#define CHECK_STATUS(s) \
  do { \
    if (s) { \
      Serial.print("SPI access error in line "); \
      Serial.println(__LINE__); \
      for (;;) {} \
    } \
  } while (0)

// Filename used for logging
static const char* LOG_FILENAME = "sensorData.csv";

// ---------------------------------------------------
// CONFIGURABLE VARIABLES
// ---------------------------------------------------

// How long to sample, in milliseconds:
static const unsigned long SAMPLE_DURATION_MS = 5000;

// After sampling finishes, we'll wait another 5s
// before reading and printing the file. Change as needed.
static const unsigned long WAIT_BEFORE_READING_MS = 5000;

// ---------------------------------------------------
// Globals for timing and file usage
// ---------------------------------------------------
mbed::File logFile;

unsigned long startTime   = 0;
bool doneSampling         = false;
bool fileClosed           = false;
bool doneReading          = false;

// ---------------------------------------------------
// Function: Write one CSV line to the *already-open* file
// ---------------------------------------------------
void writeToFile(float ax, float ay, float az,
                 float gx, float gy, float gz)
{
  unsigned long t = millis();
  char lineBuf[128];

  // Format: time_ms,accX,accY,accZ,gyroX,gyroY,gyroZ
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
// Function: Read entire file from flash and print
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

  // Read in small chunks
  const size_t chunkSize = 128;
  char buf[chunkSize + 1] = {0};

  while (true) {
    ssize_t readBytes = file.read(buf, chunkSize);
    if (readBytes <= 0) {
      break; // EOF or error
    }
    buf[readBytes] = '\0'; // null-terminate
    Serial.print(buf);
  }
  Serial.println(); // final newline

  file.close();
}

// ---------------------------------------------------
// Setup: SPI Flash + LittleFS + BMI270
// ---------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n--- Nicla Voice: BMI270 to SPI Flash (Single-Open) Demo ---");

  // 1) Initialize the SPI Flash & LittleFS
  {
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
    // Attempt to mount
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
  }

  // Remove any old file so we start fresh
  fs.remove(LOG_FILENAME);

  // 2) Nicla & BMI270 Setup
  nicla::begin();
  // If you power the board externally at 3.3V, disabling the LDO is OK.
  // But if there's any chance the IMU is unpowered, comment out the next line:
  nicla::disableLDO();  
  nicla::leds.begin();

  Serial.println("- NDP processor initialization...");
  NDP.begin("mcu_fw_120_v91.synpkg");
  NDP.load("dsp_firmware_v91.synpkg");
  Serial.println("- NDP processor initialization done!");

  {
    uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];

    // Dummy reads
    int status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);

    // Soft reset
    status = NDP.sensorBMI270Write(0x7E, 0xB6);
    CHECK_STATUS(status);
    delay(20);

    status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
    CHECK_STATUS(status);

    // PWR_CONF = 0x00
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

    // 0x7D = 0x0E => enable sensor
    status = NDP.sensorBMI270Write(0x7D, 0x0E);
    CHECK_STATUS(status);

    // ACC & GYR config
    status = NDP.sensorBMI270Write(0x40, 0xA8);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x41, 0x00);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x42, 0xA9);
    CHECK_STATUS(status);
    status = NDP.sensorBMI270Write(0x43, 0x00);
    CHECK_STATUS(status);
  }

  // 3) **Open the log file ONCE** (truncate mode)
  {
    int err = logFile.open(&fs, LOG_FILENAME, O_WRONLY | O_CREAT | O_TRUNC);
    if (err) {
      Serial.print("Error opening log file: ");
      Serial.println(err);
      while(true){}
    }
  }

  // Mark the start time
  startTime = millis();
}

void loop() {
  unsigned long elapsed = millis() - startTime;

  // 1) For the configured sampling duration, read sensor & write lines
  if (!doneSampling && elapsed < SAMPLE_DURATION_MS) {
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

    // Optionally print in real-time (this can slow your loop if you do it every sample)
    Serial.print("ACC: ");
    Serial.print(x_acc, 2); Serial.print(", ");
    Serial.print(y_acc, 2); Serial.print(", ");
    Serial.print(z_acc, 2); Serial.print(" | GYRO: ");
    Serial.print(x_gyr, 2); Serial.print(", ");
    Serial.print(y_gyr, 2); Serial.print(", ");
    Serial.println(z_gyr, 2);

    // Write data to the *open file*
    writeToFile(x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr);

    // ~100 Hz sampling
    delay(10);
  }
  else if (!doneSampling && elapsed >= SAMPLE_DURATION_MS) {
    // Done sampling => close the file
    Serial.println("Sampling finished. Closing file.");
    logFile.close();
    fileClosed = true;

    // Wait before reading
    doneSampling = true;
    startTime = millis();  // reset for the next phase
  }

  // 2) After WAIT_BEFORE_READING_MS, read entire file & print
  if (doneSampling && fileClosed && !doneReading) {
    unsigned long waitAfterSampling = millis() - startTime;
    if (waitAfterSampling >= WAIT_BEFORE_READING_MS) {
      Serial.println("Reading file from flash and printing:");
      readAndPrintFile(LOG_FILENAME);
      Serial.println("--- Done. Halting. ---");
      doneReading = true;
      while (true) { /* halt */ }
    }
  }
}
