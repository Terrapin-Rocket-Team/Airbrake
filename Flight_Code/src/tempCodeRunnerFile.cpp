#include "BR.h"


bool BR::begin(bool useBiasCorrection) {
    myusb.begin();
    blueRaven.begin(115200);  // Baud rate doesn't matter but required
    return init();
}


bool BR::init() {
    return initialized = true;
}

void BR::update() {
    read();
    packData();
}

void BR::read() {
    if (blueRaven.available()) {
        int bytesRead = blueRaven.readBytesUntil('\n', buffer, BUFFER_SIZE);
        buffer[bytesRead] = '\0';
        //Serial.println(" Intiallizing ");
        if (strncmp(buffer, "@ BLR_STAT", 10) == 0) {
            parseMessage(buffer);
            //Serial.println(" receving ");

        }
    }
}

bool BR::parseMessage(const char* message) {
    char* ptr;
    
    // Parse pressure   
    ptr = strstr(message, "Bo:");
    if (ptr) {
        if (ptr) sscanf(ptr, "Bo: %f %f", &pressure, &temperature);
        Serial.println(pressure);    
    }
    
    // Parse altitude
    ptr = strstr(message, "AGL");
    if (ptr) {
        sscanf(ptr, "AGL %f", &altitude);
    }
    
   
    //Parse Velocity
    ptr = strstr(message, "vel");
    if (ptr) {
        sscanf(ptr, "vel %f", &velocity);
    }


    return true;
}

const int BR::getNumPackedDataPoints() const {
    return 4; // altitude, pressure, temperature
}

const mmfs::PackedType* BR::getPackedOrder() const {
    static const mmfs::PackedType result[] = {
        mmfs::FLOAT, // altitude
        mmfs::FLOAT, // pressure
        mmfs::FLOAT,  // temperature
        mmfs::FLOAT  // velocity
    };
    return result;
}

const char** BR::getPackedDataLabels() const {
    static const char* labels[] = {
        "BR-ALT (ft)",
        "BR-PRES (Pa)",
        "BR-TEMP (C)"
        "BR-Velocity(m/s)"
    };
    return labels;
}

void BR::packData() {
    struct PackedData data;
    data.altitude = altitude;
    data.pressure = pressure;
    data.temperature = temperature;
    data.velocity = velocity;
    memcpy(packedData, &data, sizeof(PackedData));
}

 // namespace mmfs