#include "BR.h"
#include <math.h>

bool BR::begin(bool useBiasCorrection) {
    myusb.begin();
    //resetSensorValues();
    return init();
}

bool BR::init() {
    initialized = true;
    return true;
}

void BR::update() {
    read();
}

void BR::resetSensorValues() {
    altitude = pressure = temperature = velocity = angle = 0;
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;
    rollAngle = tiltAngle = 0;
}

void BR::read() {
    if (blueRaven.available()) {
        int bytesRead = blueRaven.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';

            //Serial.print("Raw message: ");
            //Serial.println(buffer);
            if (strncmp(buffer, "@ BLR_STAT", 10) == 0) {
                if (parseMessage(buffer)) {
                    lastReadTime = millis();
                    //mmfs::getLogger().recordLogData(mmfs::INFO_, "Message parsed successfully");
                } else {
                    mmfs::getLogger().recordLogData(mmfs::INFO_, "Failed to parse message");
                }
            } else {
                mmfs::getLogger().recordLogData(mmfs::INFO_, "Received message does not start with @ BLR_STAT");
            }
        } else {
            mmfs::getLogger().recordLogData(mmfs::INFO_, "No bytes read from blueRaven");
        }
    } else {
        //mmfs::getLogger().recordLogData(mmfs::INFO_, "No data available from blueRaven");
    }
    
    if (!isConnected()) {
        resetSensorValues();
        //Serial.println("Connection timed out. Sensor values reset.");
    }
}



bool BR::parseMessage(const char* message) {
    if (!message) return false;
    
    char* ptr;
    bool success = false;
    float tempValues[3];  // Temporary storage for vector values
    
    // Parse accelerometer data
    ptr = strstr(message, "HG:");
    if (ptr && sscanf(ptr, "HG: %f %f %f", &tempValues[0], &tempValues[1], &tempValues[2]) == 3) {
        accelX = tempValues[0];
        accelY = tempValues[1];
        accelZ = tempValues[2];
        success = true;
    }
    
    // Parse pressure and temperature
    ptr = strstr(message, "Bo:");
    if (ptr && sscanf(ptr, "Bo: %f %f", &pressure, &temperature) == 2) {
        success = true;
    }
    
    // Parse gyroscope data
    ptr = strstr(message, "gy:");
    if (ptr && sscanf(ptr, "gy: %f %f %f", &tempValues[0], &tempValues[1], &tempValues[2]) == 3) {
        gyroX = tempValues[0];
        gyroY = tempValues[1];
        gyroZ = tempValues[2];
        success = true;
    }
    
    // Parse angle data
    ptr = strstr(message, "ang:");
    if (ptr && sscanf(ptr, "ang: %f %f", &tiltAngle, &rollAngle) == 2) {
        success = true;
    }
    
    // Parse velocity
    ptr = strstr(message, "vel");
    if (ptr && sscanf(ptr, "vel %f", &velocity) == 1) {
        success = true;
    }
    
    // Parse altitude
    ptr = strstr(message, "AGL");
    if (ptr && sscanf(ptr, "AGL %f", &altitude) == 1) {
        success = true;
    }
    
    return success;
}