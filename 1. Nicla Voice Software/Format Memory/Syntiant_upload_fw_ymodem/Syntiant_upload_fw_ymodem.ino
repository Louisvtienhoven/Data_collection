/*
  YModem uploader / downloader

  Available commands:

  L  -> list files
  S  -> list files + sha256 checksum
  F  -> format external flash (remove all files)
  FF -> remove all files except:
         mcu_fw_120_v91.synpkg and dsp_firmware_v91.synpkg
  R  -> send file (Ymodem transmit)
  Y  -> receive file (Ymodem receive)

    To send a file, overwriting the existing one if it exists:
      ./syntiant-uploader send -m "Y" -w "Y" -p $portName $filename

    To receive a file (will be copied in CWD folder):
      ./syntiant-uploader receive -m "R$filename" -w "R" -p $portName

  For example:
      ./syntiant-uploader receive -m "Raudiodump.mp3" -w "R" -p /dev/ttyACM1
*/

#include "SPIFBlockDevice.h"
#include "LittleFileSystem.h"
#include "sha256.h"
#include "ymodem.h"

SPIFBlockDevice spif(SPI_PSELMOSI0, SPI_PSELMISO0, SPI_PSELSCK0, CS_FLASH, 16000000);
mbed::LittleFileSystem fs("fs");

FILE *file;
mbedtls_sha256_context ctx;
int timeout = 0;

char filename[256] = {'\0'};

long getFileLen(FILE *file) {
    fseek(file, 0, SEEK_END);
    long len = ftell(file);
    fseek(file, 0, SEEK_SET);
    // Decrement len by 1 to remove the CRC from the count (if needed)
    return len;
}

uint8_t* sha256(char* filename, uint8_t* output) {
    mbedtls_sha256_init(&ctx);
    uint8_t packet[256];
    int total = 0;

    String name = String("/fs/") + filename;
    file = fopen(name.c_str(), "rb");

    if (file != NULL) {
        Serial.print(getFileLen(file));
        Serial.print("    ");
    }

    mbedtls_sha256_starts(&ctx, 0);
    while (!feof(file)) {
        int howMany = fread(packet, 1, sizeof(packet), file);
        mbedtls_sha256_update(&ctx, packet, howMany);
        total += howMany;
    }
    mbedtls_sha256_finish(&ctx, output);
    fclose(file);

    printSha256Sum(output);
    return output;
}

void printSha256Sum(uint8_t* output) {
    for (int i = 0; i < 32; i++) {
        if (output[i] < 0x10) {
            Serial.print("0");
        }
        Serial.print(output[i], HEX);
    }
    Serial.println();
}

void listFiles(bool shasum = false);
void deleteAllFilesExcept();  // Function for selective deletion

void listFiles(bool shasum) {
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir("/fs")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            Serial.println(ent->d_name);
            if (shasum) {
                Serial.print("    ");
                uint8_t output[32];
                sha256(ent->d_name, output);
            }
        }
        closedir(dir);
    } else {
        Serial.println("Failed to open directory /fs");
    }
}

// This function deletes every file except the two excluded firmware packages.
void deleteAllFilesExcept() {
    const char* exclude1 = "mcu_fw_120_v91.synpkg";
    const char* exclude2 = "dsp_firmware_v91.synpkg";
    
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir("/fs")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            String fname = String(ent->d_name);
            // Skip the current and parent directory entries.
            if (fname == "." || fname == "..") continue;
            
            // Delete the file if it is not one of the excluded ones.
            if (fname != exclude1 && fname != exclude2) {
                String fullPath = String("/fs/") + fname;
                int ret = remove(fullPath.c_str());
                if (ret != 0) {
                    Serial.print("Error removing file: ");
                    Serial.println(fullPath);
                } else {
                    Serial.print("Removed file: ");
                    Serial.println(fullPath);
                }
            } else {
                Serial.print("Preserved file: ");
                Serial.println(fname);
            }
        }
        closedir(dir);
    } else {
        Serial.println("Could not open directory /fs");
    }
}

void setup() {
    Serial.begin(115200);
    // Mount the filesystem on the SPI flash.
    int err = fs.mount(&spif);
    memset(filename, 0, sizeof(filename));
}

void loop() {
    // Check if a full command line is available
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        // Command "Y": receive a file via YModem
        if (cmd == "Y") {
            FILE* f = fopen("/fs/temp.bin", "wb");
            // Flush any extra data from Serial
            while (Serial.available()) { Serial.read(); }
            Serial.print("Y");
            int ret = Ymodem_Receive(f, 1024 * 1024, filename);
            String name = String(filename);
            if (ret > 0 && name != "") {
                name = "/fs/" + name;
                fclose(f);
                ret = rename("/fs/temp.bin", name.c_str());
            }
        }
        // Command "R": send a file via YModem
        else if (cmd.startsWith("R")) {
            // Remove the 'R' from the command to get the filename.
            String fileToSend = cmd.substring(1);
            fileToSend.trim();
            String filename_abs = String("/fs/") + fileToSend;
            FILE* f = fopen(filename_abs.c_str(), "rb");
            // Flush any extra data from Serial
            while (Serial.available()) { Serial.read(); }
            if (f != NULL) {
                Serial.print("R");
                int ret = Ymodem_Transmit((char*)fileToSend.c_str(), getFileLen(f), f);
                fclose(f);
            }
        }
        // Command "L": list files in the filesystem.
        else if (cmd == "L") {
            listFiles();
        }
        // Command "S": list files and print sha256 checksums.
        else if (cmd == "S") {
            listFiles(true);
        }
        // Command "F": reformat external flash (remove all files).
        else if (cmd == "F") {
            Serial.println("Executing F: Reformatting external flash (removing all files)");
            fs.reformat(&spif);
        }
        // Command "FF": delete all files except the two firmware packages.
        else if (cmd == "FF") {
            Serial.println("Executing FF: Deleting all files except firmware packages");
            deleteAllFilesExcept();
        }
    } else {
        delay(10);
    }
}
