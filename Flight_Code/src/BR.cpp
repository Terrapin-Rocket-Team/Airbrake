#include "BR.h"
#include <math.h>

bool BR::init() {
    myusb.begin();
    delay(100);  // Allow time for USB enumeration

    unsigned long startTime = millis();
    bool deviceResponding = false;

    while (millis() - startTime < 3000) {  // Try for up to 3 seconds
        myusb.Task();

        if (blueRaven.available()) {
            int bytesRead = blueRaven.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
            if (bytesRead > 0) {
                buffer[bytesRead] = '\0';

                mmfs::getLogger().recordLogData(mmfs::INFO_, (std::string("Init: Received message: ") + buffer).c_str());

                if (strncmp(buffer, "@ BLR_STAT", 10) == 0) {
                    deviceResponding = true;
                    break;
                }
            }
        }
    }

    if (deviceResponding) {
        initialized = true;
        mmfs::getLogger().recordLogData(mmfs::INFO_, "BlueRaven connection successful.");
    } else {
        initialized = false;
        mmfs::getLogger().recordLogData(mmfs::ERROR_, "BlueRaven did not respond during init.");
    }

    return initialized;
}

void BR::resetSensorValues() {
    altitude = pressure = temperature = velocity = angle = 0;
    accelX = accelY = accelZ = 0;
    gyroX = gyroY = gyroZ = 0;
    rollAngle = tiltAngle = 0;
}

void BR::read() {
    myusb.Task();
    if (blueRaven.available()) {
        int bytesRead = blueRaven.readBytesUntil('\n', buffer, BUFFER_SIZE - 1);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0';

            // Serial.print("Raw message: ");
            // Serial.println(buffer);
            if (strncmp(buffer, "@ BLR_STAT", 10) == 0) {
                if (parseMessage(buffer)) {
                    lastReadTime = millis();
                    // mmfs::getLogger().recordLogData(mmfs::INFO_, "Message parsed successfully");
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
        // mmfs::getLogger().recordLogData(mmfs::INFO_, "No data available from blueRaven");
    }
    
    if (!isConnected()) {
        resetSensorValues();
        mmfs::getLogger().recordLogData(mmfs::INFO_, "Connection timed out. Sensor values reset.");
    }
}



bool BR::parseMessage(const char* message) {
    if (!message) return false;
    
    char* ptr;
    bool success = false;
    float tempValues[3];  // Temporary storage for vector values
    
    // Parse accelerometer data
    ptr = strstr(message, "XYZ:");
    if (ptr && sscanf(ptr, "XYZ: %f %f %f", &tempValues[0], &tempValues[1], &tempValues[2]) == 3) {
        accelX = tempValues[0] / 1000 * 9.81; // Value stored as Gs x1000
        accelY = tempValues[1] / 1000 * 9.81; // Value stored as Gs x1000
        accelZ = tempValues[2] / 1000 * 9.81; // Value stored as Gs x1000
        success = true;
    }
    
    // Parse pressure and temperature
    ptr = strstr(message, "Bo:");
    if (ptr && sscanf(ptr, "Bo: %f %f", &tempValues[0], &tempValues[1]) == 2) {
        pressure = tempValues[0] / 10000 * 101325; // Value stored as atm x 10000
        temperature = ((tempValues[1] / 100) - 32) * (5.0/9.0); // Value stored as deg F x 100
        success = true;
    }

    // Parse battery voltage
    ptr = strstr(message, "bt");
    if (ptr && sscanf(ptr, "bt %f", &tempValues[0]) == 1) {
        batteryVoltage = tempValues[0] * 1000; // Value stored as mV
        success = true;
    }
    
    // Parse gyroscope data
    ptr = strstr(message, "gy:");
    if (ptr && sscanf(ptr, "gy: %f %f %f", &tempValues[0], &tempValues[1], &tempValues[2]) == 3) {
        gyroX = tempValues[0] / 100 * M_PI / 180; // Value stored as deg/s x 100
        gyroY = tempValues[1] / 100 * M_PI / 180; // Value stored as deg/s x 100
        gyroZ = tempValues[2] / 100 * M_PI / 180; // Value stored as deg/s x 100
        success = true;
    }
    
    // Parse angle data
    ptr = strstr(message, "ang:");
    if (ptr && sscanf(ptr, "ang: %f %f", &tempValues[0], &tempValues[1]) == 2) {
        tiltAngle = tempValues[0] / 10; // Value stored as deg x 10
        rollAngle = tempValues[1]; // Value stored as deg
        success = true;
    }
    
    // Parse velocity
    ptr = strstr(message, "vel");
    if (ptr && sscanf(ptr, "vel %f", &tempValues[0]) == 1) {
        velocity = tempValues[0] * 0.3048; // Value stored as ft/s
        success = true;
    }
    
    // Parse altitude
    ptr = strstr(message, "AGL");
    if (ptr && sscanf(ptr, "AGL %f", &tempValues[0]) == 1) {
        altitude = tempValues[0] * 0.3048; // Value stored as ft
        success = true;
    }
    
    return success;
}