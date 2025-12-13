#include "Sensors/GPS/GPS.h"
#include <Arduino.h>
#include "Radio/ESP32BluetoothRadio.h"
using namespace mmfs;

class BTGPS : public GPS
{
public:
    BTGPS(const char *name, ESP32BluetoothRadio *radio);

    void read() override;

    bool init() override;

private:
    ESP32BluetoothRadio *radio;
};