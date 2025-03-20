/*
 * Nicla Voice BMI270 to SPI Flash Demo with Buffered Sampling
 *
 * This program runs on a Nicla Voice board and uses the BMI270 sensor to collect
 * accelerometer data and, optionally, gyroscope data. The sensor data is logged to a CSV file on SPI Flash
 * via the LittleFS filesystem. A state machine governs the program operation:
 *
 *   - WAITING_FOR_COMMAND: The device waits for a measurement command via Serial.
 *   - SAMPLING: Once a command is received (or stored), the device collects sensor data
 *     at a specified frequency for a defined duration.
 *   - FINISHED: When sampling is complete, the system creates a flag file and waits for
 *     retrieval commands (e.g., GETCSV, CLRFLAG, STOP).
 *
 * The measurement command format is now:
 *   frequency,realWorldTime,duration,remove_flag,extraDelayMin,collectGyro
 *
 * - frequency: Sampling frequency in Hz.
 * - realWorldTime: Unix timestamp (in seconds), already adjusted if needed.
 * - duration: Measurement duration (in minutes).
 * - remove_flag: 1 to remove old file, 0 to append.
 * - extraDelayMin: Additional delay (in minutes) before sampling starts.
 * - collectGyro: 1 to collect gyroscope data, 0 to collect only accelerometer data.
 */

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

// Accelerometer range = +/- 2g, so scale:
#define ACCEL_SCALE_FACTOR ((2.0f / 32767.0f) * 9.8f)
// Gyro range = 2000 dps, so scale:
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

// Global measurement parameters (default values)
unsigned int sampleFrequencyHz = 100;    // in Hz
float sampleDurationMin = 0.2;           // in minutes
unsigned long realWorldTime = 1708531200;  // Unix timestamp
bool removeOldFile = true;               // remove old file if commanded
float extraDelayMin = 0.0;               // additional delay (in minutes) before sampling starts
bool collectGyroData = true;             // if true, collect gyroscope data; if false, only accelerometer data

// Computed timing parameters
unsigned long g_sampleDurationMs = 0;
unsigned long g_sampleDelayMs = 0;

// Global flag to ensure extra delay is applied only once
bool extraDelayApplied = false;

// Globals for timing and file usage
mbed::File logFile;
unsigned long startTime = 0;

// Define a simple state machine for measurement operation
enum MeasurementState {
  WAITING_FOR_COMMAND,
  SAMPLING,
  FINISHED
};

MeasurementState currentState = WAITING_FOR_COMMAND;

// Globals for LED blinking
bool batteryMode = false;               // true if measurement loaded from flash
unsigned long previousBlinkTime = 0;
const unsigned long blinkInterval = 500;  // for sampling state blinking
bool ledOn = false;

// --------------------
// Buffer for CSV data in RAM
// --------------------
constexpr size_t BUFFER_SIZE = 4096; // Adjust as needed
char dataBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// Flush the data buffer to the open log file.
void flushBuffer() {
  if (bufferIndex > 0) {
    ssize_t written = logFile.write(dataBuffer, bufferIndex);
    if (written < 0) {
      Serial.print("File write error: ");
      Serial.println(written);
    }
    bufferIndex = 0;
  }
}

// Append one CSV-formatted line to the buffer.
// If gyroscope data is disabled, only accelerometer values are logged.
void writeToBuffer(float ax, float ay, float az,
                   float gx, float gy, float gz)
{
  char lineBuf[128];
  int n;
  if (collectGyroData) {
    n = snprintf(lineBuf, sizeof(lineBuf),
                 "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                 ax, ay, az, gx, gy, gz);
  }
  else {
    n = snprintf(lineBuf, sizeof(lineBuf),
                 "%.4f,%.4f,%.4f\r\n",
                 ax, ay, az);
  }
  
  if (n > 0 && (bufferIndex + n) < BUFFER_SIZE) {
    memcpy(&dataBuffer[bufferIndex], lineBuf, n);
    bufferIndex += n;
  } else {
    // Buffer full or line too large; flush and then try writing the line.
    flushBuffer();
    if (n < BUFFER_SIZE) { 
      memcpy(&dataBuffer[bufferIndex], lineBuf, n);
      bufferIndex += n;
    } else {
      Serial.println("Error: CSV line too large for buffer.");
    }
  }
}

// Read an entire file from flash and print its contents via Serial.
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

// Convert a Unix timestamp to a human-readable string.
String convertUnixToReadable(unsigned long unixTime) {
  if (unixTime == 0) {
    return "Time not set";
  }
  // Add one hour (3600 sec) for Amsterdam time adjustment.
  setTime(unixTime + 3600);
  char timeStr[25];
  snprintf(timeStr, sizeof(timeStr), "%04d-%02d-%02d %02d:%02d:%02d",
           year(), month(), day(), hour(), minute(), second());
  return String(timeStr);
}

// Write measurement metadata to the log file.
void writeMetadataToFile() {
  char metadata[128];
  String readableTime = convertUnixToReadable(realWorldTime);
  int n = snprintf(metadata, sizeof(metadata),
                   "Sampling Frequency [Hz]:%u\nSample Duration [Min]:%.2f\nTime:%s\nGyro Data:%s\n\n",
                   sampleFrequencyHz, sampleDurationMin, readableTime.c_str(),
                   (collectGyroData ? "Enabled" : "Disabled"));
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

// Store the received measurement command persistently.
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

// Load the measurement command from persistent storage.
// Returns true if a valid command was loaded.
// Expected formats:
//   With 6 parameters: "frequency,realWorldTime,duration,remove_flag,extraDelayMin,collectGyro"
//   With 5 parameters: "frequency,realWorldTime,duration,remove_flag,extraDelayMin" (defaults to collecting gyro data)
bool loadMeasurementCommand() {
  mbed::File file;
  int err = file.open(&fs, CMD_FILENAME, O_RDONLY);
  if (err) {
    return false; // No command stored.
  }
  char buffer[128] = {0};
  ssize_t n = file.read(buffer, sizeof(buffer) - 1);
  file.close();
  if(n <= 0) return false;
  String command = String(buffer);
  command.trim();
  if (command.length() == 0) return false;
  
  // Determine comma positions.
  int firstComma = command.indexOf(',');
  int secondComma = command.indexOf(',', firstComma + 1);
  int thirdComma = command.indexOf(',', secondComma + 1);
  int fourthComma = command.indexOf(',', thirdComma + 1);
  int fifthComma = command.indexOf(',', fourthComma + 1);
  
  if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
    Serial.println("Stored command has invalid format.");
    return false;
  }
  
  // If only five parameters are provided, default to collecting gyro data.
  if (fifthComma < 0) {
    String freqStr = command.substring(0, firstComma);
    String timeStr = command.substring(firstComma + 1, secondComma);
    String durStr  = command.substring(secondComma + 1, thirdComma);
    String removeStr = command.substring(thirdComma + 1, fourthComma);
    String extraDelayStr = command.substring(fourthComma + 1);
    
    sampleFrequencyHz = freqStr.toInt();
    realWorldTime = timeStr.toInt();
    sampleDurationMin = durStr.toFloat();
    removeOldFile = (removeStr.toInt() != 0);
    extraDelayMin = extraDelayStr.toFloat();
    collectGyroData = true;  // default
  } else {
    String freqStr = command.substring(0, firstComma);
    String timeStr = command.substring(firstComma + 1, secondComma);
    String durStr = command.substring(secondComma + 1, thirdComma);
    String removeStr = command.substring(thirdComma + 1, fourthComma);
    String extraDelayStr = command.substring(fourthComma + 1, fifthComma);
    String gyroStr = command.substring(fifthComma + 1);
    
    sampleFrequencyHz = freqStr.toInt();
    realWorldTime = timeStr.toInt();
    sampleDurationMin = durStr.toFloat();
    removeOldFile = (removeStr.toInt() != 0);
    extraDelayMin = extraDelayStr.toFloat();
    collectGyroData = (gyroStr.toInt() != 0);
  }
  
  // Compute timing parameters.
  g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
  g_sampleDelayMs = (sampleFrequencyHz > 0) ? (1000 / sampleFrequencyHz) : 10;
  
  Serial.println("Measurement command loaded from flash:");
  Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
  Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
  Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
  Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");
  Serial.print("  Extra Delay (min): "); Serial.println(extraDelayMin);
  Serial.print("  Collect Gyro Data: "); Serial.println(collectGyroData ? "Yes" : "No");
  
  // Remove stored command to avoid re-triggering measurement.
  fs.remove(CMD_FILENAME);
  
  return true;
}

// Check if a file exists.
bool fileExists(const char* filename) {
  mbed::File file;
  int err = file.open(&fs, filename, O_RDONLY);
  if (err == 0) {
    file.close();
    return true;
  } else {
    return false;
  }
}

// Startup blink: Red then blue.
void startupBlink() {
  // Blink red for 500ms then off for 250ms.
  nicla::leds.setColor(255, 0, 0);
  delay(500);
  nicla::leds.setColor(0, 0, 0);
  delay(250);
  
  // Blink blue for 500ms then off for 250ms.
  nicla::leds.setColor(0, 0, 255);
  delay(500);
  nicla::leds.setColor(0, 0, 0);
  delay(250);
}

// --------------------
// Setup: Initialize SPI Flash, LittleFS, Nicla, BMI270, etc.
// --------------------
void setup() {
  Serial.begin(460800);
  Serial.println("Baud rate set to 460800");
  delay(3000);
  Serial.println("\n--- Nicla Voice: BMI270 to SPI Flash Demo ---");

  // 1) Initialize SPI Flash & LittleFS.
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

  // 2) Initialize Nicla and its peripherals.
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  // 3) Initialize the sensor and NDP firmware.
  Serial.println("- NDP processor initialization...");
  NDP.begin("mcu_fw_120_v91.synpkg");
  NDP.load("dsp_firmware_v91.synpkg");
  Serial.println("- NDP processor initialization done!");
  
  // Sensor configuration sequence.
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
  
  // 4) If sensorData.csv exists, skip new measurement.
  if (fileExists(LOG_FILENAME)) {
    Serial.println("sensorData.csv already exists. Skipping new measurement.");
    currentState = FINISHED;
    startupBlink();
    return;
  }
  
  // 5) Check for a stored measurement command.
  if (loadMeasurementCommand()) {
    batteryMode = true;
    if (removeOldFile) {
      fs.remove(LOG_FILENAME);
    }
    int flags = O_WRONLY | O_CREAT;
    if (removeOldFile) {
      flags |= O_TRUNC;
    }
    err = logFile.open(&fs, LOG_FILENAME, flags);
    if (err) {
      Serial.print("Error opening log file: ");
      Serial.println(err);
      while (true) {}
    }
    writeMetadataToFile();
    Serial.println("Stored measurement command found.");
    currentState = SAMPLING;
    Serial.println("Sampling started...");
  } else {
    Serial.println("Waiting for measurement command in the format:");
    Serial.println("  frequency,realWorldTime,duration,remove_flag,extraDelayMin,collectGyro");
    Serial.println("Example (for only accelerometer data): 65,1708531200,0.05,1,2,0");
    currentState = WAITING_FOR_COMMAND;
  }
  
  startupBlink();
}

// --------------------
// Loop: Process commands and perform sampling.
// --------------------
void loop() {
  if (currentState == WAITING_FOR_COMMAND) {
    // Blink LED slowly (toggle every 1 sec) to indicate waiting.
    unsigned long currentMillis = millis();
    if (currentMillis - previousBlinkTime >= 1000) {
      previousBlinkTime = currentMillis;
      if (ledOn) {
        nicla::leds.setColor(0, 0, 0);
        ledOn = false;
      } else {
        nicla::leds.setColor(0, 0, 255);
        ledOn = true;
      }
    }

    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      // Retrieval commands.
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
        int remCMD = fs.remove(CMD_FILENAME);
        if(remCMD == 0){
          Serial.println("command.txt removed.");
        } else {
          Serial.print("Error removing command.txt, error code: ");
          Serial.println(remCMD);
        }
        currentState = WAITING_FOR_COMMAND;
        return;
      }
      else if (command.equalsIgnoreCase("STOP")) {
        Serial.println("Stop command received. Halting execution.");
        while (true) {}
      }
      
      // Otherwise, treat as measurement command.
      if (command.length() > 0) {
        storeMeasurementCommand(command);
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);
        int fourthComma = command.indexOf(',', thirdComma + 1);
        int fifthComma = command.indexOf(',', fourthComma + 1);
        if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
          Serial.println("Invalid command format.");
          Serial.println("Command format: frequency,realWorldTime,duration,remove_flag,extraDelayMin,collectGyro");
        } else {
          if (fifthComma < 0) {
            // Only 5 parameters provided; default to collecting gyro data.
            String freqStr = command.substring(0, firstComma);
            String timeStr = command.substring(firstComma + 1, secondComma);
            String durStr = command.substring(secondComma + 1, thirdComma);
            String removeStr = command.substring(thirdComma + 1, fourthComma);
            String extraDelayStr = command.substring(fourthComma + 1);
            
            sampleFrequencyHz = freqStr.toInt();
            realWorldTime = timeStr.toInt();
            sampleDurationMin = durStr.toFloat();
            removeOldFile = (removeStr.toInt() != 0);
            extraDelayMin = extraDelayStr.toFloat();
            collectGyroData = true;  // default
          } else {
            String freqStr = command.substring(0, firstComma);
            String timeStr = command.substring(firstComma + 1, secondComma);
            String durStr = command.substring(secondComma + 1, thirdComma);
            String removeStr = command.substring(thirdComma + 1, fourthComma);
            String extraDelayStr = command.substring(fourthComma + 1, fifthComma);
            String gyroStr = command.substring(fifthComma + 1);
            
            sampleFrequencyHz = freqStr.toInt();
            realWorldTime = timeStr.toInt();
            sampleDurationMin = durStr.toFloat();
            removeOldFile = (removeStr.toInt() != 0);
            extraDelayMin = extraDelayStr.toFloat();
            collectGyroData = (gyroStr.toInt() != 0);
          }

          g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
          g_sampleDelayMs = (sampleFrequencyHz > 0) ? (1000 / sampleFrequencyHz) : 10;

          Serial.println("Measurement command received.");
          Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
          Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
          Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
          Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");
          Serial.print("  Extra Delay (min): "); Serial.println(extraDelayMin);
          Serial.print("  Collect Gyro Data: "); Serial.println(collectGyroData ? "Yes" : "No");

          Serial.println("Waiting 10 seconds for cable disconnection and power source change...");
          delay(10000);
          // Also wait the extra delay specified (in minutes).
          if (extraDelayMin > 0) {
            Serial.print("Applying additional delay of ");
            Serial.print(extraDelayMin);
            Serial.println(" minute(s)...");
            delay((unsigned long)(extraDelayMin * 60 * 1000));
          }

          if (removeOldFile) {
            fs.remove(LOG_FILENAME);
          }

          int flags = O_WRONLY | O_CREAT | (removeOldFile ? O_TRUNC : 0);
          int err = logFile.open(&fs, LOG_FILENAME, flags);
          if (err) {
            Serial.print("Error opening log file: ");
            Serial.println(err);
            while (true) {}
          }
          writeMetadataToFile();
          currentState = SAMPLING;
          Serial.println("Sampling started...");
        }
      }
    }
  }
  else if (currentState == SAMPLING) {
    // On the very first pass in SAMPLING, if an extra delay is specified and not yet applied, delay here.
    if (startTime == 0 && extraDelayMin > 0 && !extraDelayApplied) {
      Serial.print("Applying additional delay of ");
      Serial.print(extraDelayMin);
      Serial.println(" minute(s) before starting sampling...");
      delay((unsigned long)(extraDelayMin * 60 * 1000));
      extraDelayApplied = true;
      startTime = millis();
    }
    else if (startTime == 0) {
      startTime = millis();
    }

    // Blink LED during sampling.
    unsigned long currentMillis = millis();
    if (currentMillis - previousBlinkTime >= blinkInterval) {
      previousBlinkTime = currentMillis;
      if (ledOn) {
        nicla::leds.setColor(0, 0, 0);
        ledOn = false;
      } else {
        nicla::leds.setColor(0, 255, 0);
        ledOn = true;
      }
    }

    // Use non-blocking timing for sensor reads.
    static unsigned long nextSampleTime = 0;
    unsigned long now = millis();
    if (now < nextSampleTime) {
      return;
    }
    nextSampleTime = now + g_sampleDelayMs;

    // Read sensor data.
    uint8_t sensor_data[SENSOR_DATA_LENGTH];
    int status = NDP.sensorBMI270Read(READ_START_ADDRESS, READ_BYTE_COUNT, sensor_data);
    CHECK_STATUS(status);

    // Extract and convert accelerometer raw values.
    int16_t ax_raw = (int16_t)((sensor_data[1] << 8) | sensor_data[0]);
    int16_t ay_raw = (int16_t)((sensor_data[3] << 8) | sensor_data[2]);
    int16_t az_raw = (int16_t)((sensor_data[5] << 8) | sensor_data[4]);
    float ax = (float)ax_raw * ACCEL_SCALE_FACTOR;
    float ay = (float)ay_raw * ACCEL_SCALE_FACTOR;
    float az = (float)az_raw * ACCEL_SCALE_FACTOR;

    // If gyroscope data is to be collected, extract and convert it.
    float gx = 0.0, gy = 0.0, gz = 0.0;
    if (collectGyroData) {
      int16_t gx_raw = (int16_t)((sensor_data[7] << 8) | sensor_data[6]);
      int16_t gy_raw = (int16_t)((sensor_data[9] << 8) | sensor_data[8]);
      int16_t gz_raw = (int16_t)((sensor_data[11] << 8) | sensor_data[10]);
      gx = (float)gx_raw * GYRO_SCALE_FACTOR;
      gy = (float)gy_raw * GYRO_SCALE_FACTOR;
      gz = (float)gz_raw * GYRO_SCALE_FACTOR;
    }

    // Append the CSV-formatted line (only 3 columns if gyro is disabled).
    writeToBuffer(ax, ay, az, gx, gy, gz);

    // Flush the buffer if it is nearly full.
    if (bufferIndex > (BUFFER_SIZE * 3) / 4) {
      flushBuffer();
    }

    // End sampling when duration has elapsed.
    if (millis() - startTime >= g_sampleDurationMs) {
      flushBuffer();  // Write any remaining data.
      Serial.println("Sampling finished.");
      logFile.close();
      currentState = FINISHED;

      // Create a flag file to indicate measurement completion.
      mbed::File doneFile;
      int doneErr = doneFile.open(&fs, "done.txt", O_WRONLY | O_CREAT | O_TRUNC);
      if (doneErr) {
        Serial.print("Error creating done.txt: ");
        Serial.println(doneErr);
      } else {
        doneFile.close();
      }
      if(batteryMode) {
        nicla::leds.setColor(0,0,0);
      }
    }
  }
  else if (currentState == FINISHED) {
    // Blink red and blue every 3 seconds.
    if (millis() - previousBlinkTime >= 3000) {
      previousBlinkTime = millis();
      nicla::leds.setColor(255, 0, 0);  // Red
      delay(100);
      nicla::leds.setColor(0, 0, 0);
      delay(100);
      nicla::leds.setColor(0, 0, 255);  // Blue
      delay(100);  
      nicla::leds.setColor(0, 0, 0);
    }

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
        int remCMD = fs.remove(CMD_FILENAME);
        if(remCMD == 0){
          Serial.println("command.txt removed.");
        } else {
          Serial.print("Error removing command.txt, error code: ");
          Serial.println(remCMD);
        }
        currentState = WAITING_FOR_COMMAND;
        return;
      }
      else if (command.equalsIgnoreCase("STOP")) {
        Serial.println("Stop command received. Halting execution.");
        while (true) {}
      }
    }
  }
}
