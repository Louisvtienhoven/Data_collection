#include "NDP.h"
#include "BMI270_Init.h"

// Named constants
#define READ_START_ADDRESS 0x0C
#define READ_BYTE_COUNT 16
#define SENSOR_DATA_LENGTH 16

// Accelerometer range is set to +/-2g
#define ACCEL_SCALE_FACTOR ((2.0 / 32767.0) * 9.8)
#define GYRO_SCALE_FACTOR (1 / 16.4)

// Maximum number of data entries (assuming sampling every 10ms for 15s)
#define MAX_ENTRIES 1500  // 15,000 ms / 10 ms = 1500 samples

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

// Structure to store sensor readings
struct SensorData {
  float x_acc, y_acc, z_acc;
  float x_gyr, y_gyr, z_gyr;
};

// Array to store sensor readings
SensorData dataBuffer[MAX_ENTRIES];
int dataIndex = 0;  // Index to track stored data

unsigned long startTime;

void setup() {
  int status;
  uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];

  delay(3000);
  Serial.begin(115200);

  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  Serial.println("- NDP processor initialization...");
  NDP.begin("mcu_fw_120_v91.synpkg");
  NDP.load("dsp_firmware_v91.synpkg");
  Serial.println("- NDP processor initialization done!");

  status = NDP.sensorBMI270Read(0x0, 1, sensor_data);
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
  status = NDP.sensorBMI270Write(0x5E, sizeof(bmi270_maximum_fifo_config_file), (uint8_t*)bmi270_maximum_fifo_config_file);
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

  startTime = millis();
}

void loop() {
  unsigned long elapsedTime = millis() - startTime;

  // Stop collecting data after 15 seconds
  if (elapsedTime >= 5000) {
    Serial.println("5 seconds elapsed. Printing collected data...");
    delay(5000);
    printStoredData();
    while (1);  // Stop execution after printing data
  }

  uint8_t __attribute__((aligned(4))) sensor_data[SENSOR_DATA_LENGTH];
  int16_t x_acc_raw, y_acc_raw, z_acc_raw, x_gyr_raw, y_gyr_raw, z_gyr_raw;
  float x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr;

  int status = NDP.sensorBMI270Read(READ_START_ADDRESS, READ_BYTE_COUNT, &sensor_data[0]);
  CHECK_STATUS(status);

  x_acc_raw = (0x0000 | sensor_data[0] | sensor_data[1] << 8);
  y_acc_raw = (0x0000 | sensor_data[2] | sensor_data[3] << 8);
  z_acc_raw = (0x0000 | sensor_data[4] | sensor_data[5] << 8);
  x_gyr_raw = (0x0000 | sensor_data[6] | sensor_data[7] << 8);
  y_gyr_raw = (0x0000 | sensor_data[8] | sensor_data[9] << 8);
  z_gyr_raw = (0x0000 | sensor_data[10] | sensor_data[11] << 8);

  x_acc = x_acc_raw * ACCEL_SCALE_FACTOR;
  y_acc = y_acc_raw * ACCEL_SCALE_FACTOR;
  z_acc = z_acc_raw * ACCEL_SCALE_FACTOR;

  x_gyr = x_gyr_raw * GYRO_SCALE_FACTOR;
  y_gyr = y_gyr_raw * GYRO_SCALE_FACTOR;
  z_gyr = z_gyr_raw * GYRO_SCALE_FACTOR;

  Serial.print("x_acc:");
  Serial.print(x_acc);
  Serial.print(",");
  Serial.print("y_acc:");
  Serial.print(y_acc);
  Serial.print(",");
  Serial.print("z_acc:");
  Serial.println(z_acc);

  Serial.print("x_gyr:");
  Serial.print(x_gyr);
  Serial.print(",");
  Serial.print("y_gyr:");
  Serial.print(y_gyr);
  Serial.print(",");
  Serial.print("z_gyr:");
  Serial.println(z_gyr);

  // // Store the data
  // if (dataIndex < MAX_ENTRIES) {
  //   dataBuffer[dataIndex].x_acc = x_acc;
  //   dataBuffer[dataIndex].y_acc = y_acc;
  //   dataBuffer[dataIndex].z_acc = z_acc;
  //   dataBuffer[dataIndex].x_gyr = x_gyr;
  //   dataBuffer[dataIndex].y_gyr = y_gyr;
  //   dataBuffer[dataIndex].z_gyr = z_gyr;
  //   dataIndex++;
  // }

  delay(10);
}

// Function to print the stored data
void printStoredData() {
  Serial.println("Time (ms), x_acc, y_acc, z_acc, x_gyr, y_gyr, z_gyr");
  for (int i = 0; i < dataIndex; i++) {
    Serial.print(i * 10);  // Time in milliseconds (10ms intervals)
    Serial.print(",");
    Serial.print(dataBuffer[i].x_acc);
    Serial.print(",");
    Serial.print(dataBuffer[i].y_acc);
    Serial.print(",");
    Serial.print(dataBuffer[i].z_acc);
    Serial.print(",");
    Serial.print(dataBuffer[i].x_gyr);
    Serial.print(",");
    Serial.print(dataBuffer[i].y_gyr);
    Serial.print(",");
    Serial.println(dataBuffer[i].z_gyr);
  }
}
