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
    packData();
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
                    //Serial.println("Message parsed successfully");
                } else {
                    logger.recordLogData(mmfs::INFO_, "Failed to parse message");
                }
            } else {
                logger.recordLogData(mmfs::INFO_, "Received message does not start with @ BLR_STAT");
            }
        } else {
            logger.recordLogData(mmfs::INFO_, "No bytes read from blueRaven");
        }
    } else {
        //Serial.println("No data available from blueRaven");
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

const int BR::getNumPackedDataPoints() const {
    return 13;  // Updated to match all data points
}

const mmfs::PackedType* BR::getPackedOrder() const {
    static const mmfs::PackedType result[13] = {
        mmfs::FLOAT,  // altitude
        mmfs::FLOAT,  // pressure
        mmfs::FLOAT,  // temperature
        mmfs::FLOAT,  // velocity
        mmfs::FLOAT,  // angle
        mmfs::FLOAT,  // accelX
        mmfs::FLOAT,  // accelY
        mmfs::FLOAT,  // accelZ
        mmfs::FLOAT,  // gyroX
        mmfs::FLOAT,  // gyroY
        mmfs::FLOAT,  // gyroZ
        mmfs::FLOAT,  // rollAngle
        mmfs::FLOAT   // tiltAngle
    };
    return result;
}

const char** BR::getPackedDataLabels() const {
    static const char* labels[13] = {
        "BR-ALT (ft)",
        "BR-PRES (Pa)",
        "BR-TEMP (C)",
        "BR-VEL (m/s)",
        "BR-ANG (deg)",
        "BR-ACCX (m/s^2)",
        "BR-ACCY (m/s^2)",
        "BR-ACCZ (m/s^2)",
        "BR-GYROX (rad/s)",
        "BR-GYROY (rad/s)",
        "BR-GYROZ (rad/s)",
        "BR-ROLL (deg)",
        "BR-TILT (deg)"
    };
    return labels;
}

void BR::packData() {
    if (!initialized) return;
    PackedData data = {
        altitude,
        pressure,
        temperature,
        velocity,
        angle,
        accelX,
        accelY,
        accelZ,
        gyroX,
        gyroY,
        gyroZ,
        rollAngle,
        tiltAngle
    };
    memcpy(packedData, &data, sizeof(PackedData));
}