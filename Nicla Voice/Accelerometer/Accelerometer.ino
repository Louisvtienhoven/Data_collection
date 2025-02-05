#include "NDP.h"
#include "BMI270_Init.h"

// Named constants
#define READ_START_ADDRESS 0x0C
#define READ_BYTE_COUNT 16
#define SENSOR_DATA_LENGTH 16

// Accelerometer range is set to +/-2g
// Raw accelerometer data is represented as a signed 16-bit integer
// Raw accelerometer data can be converted to acceleration in m/s^2 units using the following scale factor:
#define ACCEL_SCALE_FACTOR ((2.0 / 32767.0) * 9.8)

// Gyroscope has a sensitivity of 16.4 LSB/dps
#define GYRO_SCALE_FACTOR (1 / 16.4)

// Macros for checking the sensor status.
#define CHECK_STATUS(s) \
  do { \
    if (s) { \
      Serial.print("SPI access error in line "); \
      Serial.println(__LINE__); \
      for (;;) \
        ; \
    } \
  } while (0)

void setup() {
  int status;
  uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];

  // Initiate Serial communication for debugging and monitoring.
  Serial.begin(115200);

  // Initialize Nicla Voice board's system functions.
  // Disable the LDO regulator on the Nicla Voice board for power saving.
  // Initialize the built-in RGB LED of the Nicla Voice board.
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  // NDP processor initialization with firmwares and models
  Serial.println("- NDP processor initialization...");
  NDP.begin("mcu_fw_120_v91.synpkg");
  NDP.load("dsp_firmware_v91.synpkg");
  Serial.println("- NDP processor initialization done!");

  // Set the BMI270 sensor in SPI mode, then read sensor data.
  status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
  CHECK_STATUS(status);

  // Perform a software reset of the sensor.
  status = NDP.sensorBMI270Write(0x7E, 0xB6);
  CHECK_STATUS(status);
  delay(20);

  // Set the sensor back to SPI mode after the software reset.
  status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
  CHECK_STATUS(status);

  // Disable power configurations.
  status = NDP.sensorBMI270Write(0x7C, 0x00);
  CHECK_STATUS(status);
  delay(20);

  // Prepare to load the sensor configuration.
  status = NDP.sensorBMI270Write(0x59, 0x00);
  CHECK_STATUS(status);

  // Sensor configuration.
  Serial.println("- BMI270 initialization starting...");
  status = NDP.sensorBMI270Write(0x5E, sizeof(bmi270_maximum_fifo_config_file), (uint8_t*)bmi270_maximum_fifo_config_file);
  CHECK_STATUS(status);
  Serial.println("- BMI270 Initialization done!");
  status = NDP.sensorBMI270Write(0x59, 0x01);
  CHECK_STATUS(status);
  delay(200);

  // Check sensor status.
  status = NDP.sensorBMI270Read(0x21, 1, sensor_data);
  CHECK_STATUS(status);

  // Configure the device to normal power mode with both accelerometer and gyroscope operational.
  // Set the accelerometer and gyroscope settings such as measurement range and data rate.
  status = NDP.sensorBMI270Write(0x7D, 0x0E);  // Normal power mode
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x40, 0xA8);  // Accelerometer configuration.
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x41, 0x00);  // Set the accelerometer range to +/- 2g.
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x42, 0xA9);  // Gyroscope configuration.
  CHECK_STATUS(status);
  status = NDP.sensorBMI270Write(0x43, 0x00);  // Set the gyroscope range to +/- 2000 dps.
  CHECK_STATUS(status);
}

void loop() {
  // Allocate space for raw sensor data.
  uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];

  // Declare variables for accelerometer and gyroscope data.
  int16_t x_acc_raw, y_acc_raw, z_acc_raw, x_gyr_raw, y_gyr_raw, z_gyr_raw;
  float x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr;

  // Read operation status variable.
  int status;

  // Perform data read from the BMI270 sensor. The data read includes accelerometer and gyroscope data.
  // The sensor's read function is called with 0x0C as the start address and 16 as the number of bytes to read.
  // Collected data is placed into sensor_data array.
  status = NDP.sensorBMI270Read(READ_START_ADDRESS, READ_BYTE_COUNT, &sensor_data[0]);

  // Check the status of the read operation.
  CHECK_STATUS(status);

  // Parse the read sensor data. Data is in 16-bit format, where lower byte comes first (little endian).
  // Data for each axis (X, Y, Z) of the accelerometer and gyroscope is extracted from the array.
  x_acc_raw = (0x0000 | sensor_data[0] | sensor_data[1] << 8);
  y_acc_raw = (0x0000 | sensor_data[2] | sensor_data[3] << 8);
  z_acc_raw = (0x0000 | sensor_data[4] | sensor_data[5] << 8);
  x_gyr_raw = (0x0000 | sensor_data[6] | sensor_data[7] << 8);
  y_gyr_raw = (0x0000 | sensor_data[8] | sensor_data[9] << 8);
  z_gyr_raw = (0x0000 | sensor_data[10] | sensor_data[11] << 8);

  // Convert raw accelerometer data to acceleration expressed in m/s^2.
  x_acc = x_acc_raw * ACCEL_SCALE_FACTOR;
  y_acc = y_acc_raw * ACCEL_SCALE_FACTOR;
  z_acc = z_acc_raw * ACCEL_SCALE_FACTOR;

  // Convert raw gyroscope data to angular velocity expressed in °/s.
  x_gyr = x_gyr_raw * GYRO_SCALE_FACTOR;
  y_gyr = y_gyr_raw * GYRO_SCALE_FACTOR;
  z_gyr = z_gyr_raw * GYRO_SCALE_FACTOR;

  // Print accelerometer data (expressed in m/s^2).
  Serial.print("x_acc:");
  Serial.print(x_acc);
  Serial.print(",");
  Serial.print("y_acc:");
  Serial.print(y_acc);
  Serial.print(",");
  Serial.print("z_acc:");
  Serial.println(z_acc);

  // Print gyroscope data (expressed in °/s). 
  Serial.print("x_gyr:");
  Serial.print(x_gyr);
  Serial.print(",");
  Serial.print("y_gyr:");
  Serial.print(y_gyr);
  Serial.print(",");
  Serial.print("z_gyr:");
  Serial.println(z_gyr);

  delay(10);
}