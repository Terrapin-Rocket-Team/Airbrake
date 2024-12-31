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
        
        if (strncmp(buffer, "@ BLR_STAT", 10) == 0) {
            parseMessage(buffer);
        }
    }
}

bool BR::parseMessage(const char* message) {
    char* ptr;
    
    // Parse pressure
    ptr = strstr(message, "Bo:");
    if (ptr) {
        sscanf(ptr, "Bo: %f", &pressure);
    }
    
    // Parse altitude
    ptr = strstr(message, "AGL");
    if (ptr) {
        sscanf(ptr, "AGL %f", &altitude);
    }
    
    // Parse temperature (if available in your data)
    ptr = strstr(message, "temp:");
    if (ptr) {
        sscanf(ptr, "temp: %f", &temperature);
    }
    
    return true;
}

const int BR::getNumPackedDataPoints() const {
    return 3; // altitude, pressure, temperature
}

const mmfs::PackedType* BR::getPackedOrder() const {
    static const mmfs::PackedType result[] = {
        mmfs::FLOAT, // altitude
        mmfs::FLOAT, // pressure
        mmfs::FLOAT  // temperature
    };
    return result;
}

const char** BR::getPackedDataLabels() const {
    static const char* labels[] = {
        "BR-ALT (ft)",
        "BR-PRES (Pa)",
        "BR-TEMP (C)"
    };
    return labels;
}

void BR::packData() {
    struct PackedData data;
    data.altitude = altitude;
    data.pressure = pressure;
    data.temperature = temperature;
    memcpy(packedData, &data, sizeof(PackedData));
}

 // namespace mmfs