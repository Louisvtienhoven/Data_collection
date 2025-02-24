/*
 * Nicla Voice BMI270 to SPI Flash Demo with Optimized Single Buffer Sampling
 *
 * This program runs on a Nicla Voice board and uses the BMI270 sensor to collect
 * accelerometer and gyroscope data. Sensor data is logged as CSV to a file on SPI Flash
 * via the LittleFS filesystem.
 *
 * A state machine governs operation:
 *   - WAITING_FOR_COMMAND: Wait for a measurement command via Serial.
 *   - SAMPLING: Collect sensor data at the specified frequency for a given duration.
 *   - FINISHED: When sampling completes, a flag file is created and retrieval commands (GETCSV, CLRFLAG, STOP)
 *     are processed.
 *
 * The measurement command (format: frequency,realWorldTime,duration,remove_flag) can also be stored in flash.
 */

#include <Arduino.h>
#include <TimeLib.h>         // Provides setTime(), year(), etc.

// ---------- MbedOS / LittleFS Includes ----------
#include <BlockDevice.h>
#include <File.h>
#include <FileSystem.h>
#include <LittleFileSystem.h>

// Filesystem root name.
constexpr auto userRoot{"fs"};

// SPI Flash block device pointer and LittleFS instance.
mbed::BlockDevice* spif;
mbed::LittleFileSystem fs{userRoot};

// ---------- BMI270 / Nicla Includes ----------
#include "NDP.h"
#include "BMI270_Init.h"

// BMI270 constants.
#define READ_START_ADDRESS  0x0C
#define READ_BYTE_COUNT     16
#define SENSOR_DATA_LENGTH  16

// Scale factors.
#define ACCEL_SCALE_FACTOR ((2.0f / 32767.0f) * 9.8f)
#define GYRO_SCALE_FACTOR (1.0f / 16.4f)

// Macro to check sensor status.
#define CHECK_STATUS(s)         \
  do {                          \
    if (s) {                    \
      Serial.print("SPI access error in line "); \
      Serial.println(__LINE__); \
      while (1);                \
    }                           \
  } while (0)

// Filenames for log and command.
static const char* LOG_FILENAME = "sensorData.csv";
static const char* CMD_FILENAME = "command.txt";

// Global measurement parameters (default values).
unsigned int sampleFrequencyHz = 100;   // Hz
float sampleDurationMin = 0.2;          // minutes
unsigned long realWorldTime = 1708531200; // Unix timestamp
bool removeOldFile = true;              // remove old file if commanded

// Computed timing parameters.
unsigned long g_sampleDurationMs = 0;
unsigned long g_sampleDelayUs = 0;  // in microseconds

// Global file and timing variables.
mbed::File logFile;
unsigned long startTime = 0;

// State machine.
enum MeasurementState {
  WAITING_FOR_COMMAND,
  SAMPLING,
  FINISHED
};
MeasurementState currentState = WAITING_FOR_COMMAND;

// LED blinking globals.
bool batteryMode = false;               // true if measurement loaded from flash
unsigned long previousBlinkTime = 0;
const unsigned long blinkInterval = 500;  // in ms
bool ledOn = false;

// --------------------
// Buffer for CSV data in RAM
// --------------------
constexpr size_t BUFFER_SIZE = 8192;  // Increased buffer size for fewer flash writes
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
void writeToBuffer(float ax, float ay, float az,
                   float gx, float gy, float gz) {
  char lineBuf[128];
  int n = snprintf(lineBuf, sizeof(lineBuf),
                   "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                   ax, ay, az, gx, gy, gz);
  if (n > 0 && (bufferIndex + n) < BUFFER_SIZE) {
    memcpy(&dataBuffer[bufferIndex], lineBuf, n);
    bufferIndex += n;
  } else {
    // Buffer full or line too large; flush and then write.
    flushBuffer();
    if (n < BUFFER_SIZE) { 
      memcpy(&dataBuffer[bufferIndex], lineBuf, n);
      bufferIndex += n;
    } else {
      Serial.println("Error: CSV line too large for buffer.");
    }
  }
}

// Read an entire file from flash and print via Serial.
void readAndPrintFile(const char* filename) {
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
  if (unixTime == 0) return "Time not set";
  // Adjust for Amsterdam time (+1 hour).
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
                   "Sampling Frequency [Hz]:%u\nSample Duration [Min]:%.2f\nTime:%s\n\n",
                   sampleFrequencyHz, sampleDurationMin, readableTime.c_str());
  if (n > 0 && n < (int)sizeof(metadata)) {
    ssize_t written = logFile.write(metadata, n);
    if (written < 0) {
      Serial.print("File write error: ");
      Serial.println(written);
    }
    logFile.sync();  // Sync only for metadata.
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
bool loadMeasurementCommand() {
  mbed::File file;
  int err = file.open(&fs, CMD_FILENAME, O_RDONLY);
  if (err) return false; // No command stored.
  char buffer[128] = {0};
  ssize_t n = file.read(buffer, sizeof(buffer) - 1);
  file.close();
  if(n <= 0) return false;
  String command = String(buffer);
  command.trim();
  if (command.length() == 0) return false;
  
  // Expected format: "frequency,realWorldTime,duration,remove_flag"
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
  
  // Compute timing parameters.
  g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
  g_sampleDelayUs = (sampleFrequencyHz > 0) ? (1000000UL / sampleFrequencyHz) : 10000;
  
  Serial.println("Measurement command loaded from flash:");
  Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
  Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
  Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
  Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");
  
  // Remove the stored command.
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
  // Blink red.
  nicla::leds.setColor(255, 0, 0);
  delay(500);
  nicla::leds.setColor(0, 0, 0);
  delay(250);
  // Blink blue.
  nicla::leds.setColor(0, 0, 255);
  delay(500);
  nicla::leds.setColor(0, 0, 0);
  delay(250);
}

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n--- Nicla Voice: BMI270 to SPI Flash Demo ---");

  // 1) Initialize SPI Flash & LittleFS.
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

  // 3) Initialize sensor and NDP firmware.
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
  
  // 4) If sensorData.csv exists, skip a new measurement.
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
    startTime = millis();
    currentState = SAMPLING;
    Serial.println("Sampling started...");
  } else {
    Serial.println("Waiting for measurement command in the format:");
    Serial.println("  frequency,realWorldTime,duration,remove_flag");
    Serial.println("Example: 65,1708531200,0.05,1");
    currentState = WAITING_FOR_COMMAND;
  }
  
  startupBlink();
}

void loop() {
  if (currentState == WAITING_FOR_COMMAND) {
    // Blink LED every second.
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
      
      // Process retrieval commands.
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
      
      if (command.length() > 0) {
        storeMeasurementCommand(command);
        int firstComma = command.indexOf(',');
        int secondComma = command.indexOf(',', firstComma + 1);
        int thirdComma = command.indexOf(',', secondComma + 1);
        if (firstComma < 0 || secondComma < 0 || thirdComma < 0) {
          Serial.println("Invalid command format.");
          Serial.println("Command format: frequency,realWorldTime,duration,remove_flag");
        } else {
          String freqStr = command.substring(0, firstComma);
          String timeStr = command.substring(firstComma + 1, secondComma);
          String durStr = command.substring(secondComma + 1, thirdComma);
          String removeStr = command.substring(thirdComma + 1);
          
          sampleFrequencyHz = freqStr.toInt();
          realWorldTime = timeStr.toInt();
          sampleDurationMin = durStr.toFloat();
          removeOldFile = (removeStr.toInt() != 0);
          
          g_sampleDurationMs = (unsigned long)(sampleDurationMin * 60 * 1000);
          g_sampleDelayUs = (sampleFrequencyHz > 0) ? (1000000UL / sampleFrequencyHz) : 10000;
          
          Serial.println("Measurement command received.");
          Serial.print("  Frequency (Hz): "); Serial.println(sampleFrequencyHz);
          Serial.print("  Duration (min): "); Serial.println(sampleDurationMin);
          Serial.print("  Real-world Time: "); Serial.println(realWorldTime);
          Serial.print("  Remove old file: "); Serial.println(removeOldFile ? "Yes" : "No");
          
          Serial.println("Waiting 10 seconds for cable disconnection and power source change...");
          delay(10000);
          
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
          startTime = millis();
          currentState = SAMPLING;
          Serial.println("Sampling started...");
        }
      }
    }
  }
  else if (currentState == SAMPLING) {
    // Use micros() for high-resolution timing.
    static unsigned long nextSampleTime = 0;
    unsigned long currentMicros = micros();
    if (currentMicros < nextSampleTime) {
      return;
    }
    nextSampleTime = currentMicros + g_sampleDelayUs;
    
    // Read sensor data.
    uint8_t sensor_data[SENSOR_DATA_LENGTH];
    int status = NDP.sensorBMI270Read(READ_START_ADDRESS, READ_BYTE_COUNT, sensor_data);
    CHECK_STATUS(status);
    
    int16_t ax_raw = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
    int16_t ay_raw = (int16_t)((sensor_data[2] << 8) | sensor_data[3]);
    int16_t az_raw = (int16_t)((sensor_data[4] << 8) | sensor_data[5]);
    int16_t gx_raw = (int16_t)((sensor_data[6] << 8) | sensor_data[7]);
    int16_t gy_raw = (int16_t)((sensor_data[8] << 8) | sensor_data[9]);
    int16_t gz_raw = (int16_t)((sensor_data[10] << 8) | sensor_data[11]);
    
    float ax = (float)ax_raw * ACCEL_SCALE_FACTOR;
    float ay = (float)ay_raw * ACCEL_SCALE_FACTOR;
    float az = (float)az_raw * ACCEL_SCALE_FACTOR;
    float gx = (float)gx_raw * GYRO_SCALE_FACTOR;
    float gy = (float)gy_raw * GYRO_SCALE_FACTOR;
    float gz = (float)gz_raw * GYRO_SCALE_FACTOR;
    
    writeToBuffer(ax, ay, az, gx, gy, gz);
    
    // Flush buffer if nearly full.
    if (bufferIndex > (BUFFER_SIZE * 3) / 4) {
      flushBuffer();
    }
    
    // Check if sampling duration has elapsed.
    if (millis() - startTime >= g_sampleDurationMs) {
      flushBuffer();
      Serial.println("Sampling finished.");
      logFile.close();
      currentState = FINISHED;
      
      mbed::File doneFile;
      int doneErr = doneFile.open(&fs, "done.txt", O_WRONLY | O_CREAT | O_TRUNC);
      if (doneErr) {
        Serial.print("Error creating done.txt: ");
        Serial.println(doneErr);
      } else {
        doneFile.close();
      }
      if (batteryMode) {
        nicla::leds.setColor(0, 0, 0);
      }
    }
  }
  else if (currentState == FINISHED) {
    // Blink red and blue every 3 seconds.
    if (millis() - previousBlinkTime >= 3000) {
      previousBlinkTime = millis();
      nicla::leds.setColor(255, 0, 0);
      delay(100);
      nicla::leds.setColor(0, 0, 0);
      delay(100);
      nicla::leds.setColor(0, 0, 255);
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
