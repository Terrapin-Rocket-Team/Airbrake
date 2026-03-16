#ifndef ASTRA_ROCKET_BLUE_RAVEN_H
#define ASTRA_ROCKET_BLUE_RAVEN_H

#include <Arduino.h>

#include <Math/Vector.h>
#include <Sensors/Sensor.h>

#if defined(ENV_TEENSY) && !defined(NATIVE)
#include <USBHost_t36.h>
#endif

namespace astra_rocket
{
    class BlueRaven : public astra::Sensor
    {
    public:
        struct Sample
        {
            bool valid = false;
            uint32_t sequence = 0;
            uint16_t year = 0;
            uint8_t month = 0;
            uint8_t day = 0;
            char timeOfDay[16] = {0};
            astra::Vector<3> accelMps2 = astra::Vector<3>(0.0, 0.0, 0.0);
            astra::Vector<3> highGAccelMps2 = astra::Vector<3>(0.0, 0.0, 0.0);
            astra::Vector<3> gyroRadPerSec = astra::Vector<3>(0.0, 0.0, 0.0);
            double pressureHpa = 0.0;
            double temperatureC = 0.0;
            double batteryVolts = 0.0;
            double tiltDeg = 0.0;
            double rollDeg = 0.0;
            double verticalVelocityMps = 0.0;
            double altitudeAglM = 0.0;
            uint16_t crc = 0;
        };

        BlueRaven();

        int begin() override;
        bool poll();
        bool ingestLine(const char *line);
        void setStream(Stream *stream);
        void useUsbHost(uint32_t baud = DEFAULT_USB_BAUD);

        bool hasValidSample() const { return latest.valid; }
        uint32_t getSampleCount() const { return sampleCount; }
        bool isConnected() const;
        const Sample &getLatestSample() const { return latest; }

        astra::Vector<3> getAccel() const { return latest.accelMps2; }
        astra::Vector<3> getHighGAccel() const { return latest.highGAccelMps2; }
        astra::Vector<3> getGyro() const { return latest.gyroRadPerSec; }
        double getPressureHpa() const { return latest.pressureHpa; }
        double getTemperatureC() const { return latest.temperatureC; }
        double getBatteryVolts() const { return latest.batteryVolts; }
        double getTiltDeg() const { return latest.tiltDeg; }
        double getRollDeg() const { return latest.rollDeg; }
        double getVerticalVelocityMps() const { return latest.verticalVelocityMps; }
        double getAltitudeAglM() const { return latest.altitudeAglM; }

        static uint16_t crc16Buypass(const uint8_t *data, size_t len);

    private:
        static constexpr uint32_t DEFAULT_USB_BAUD = 115200;
        static constexpr size_t MAX_LINE_LEN = 256;
        static constexpr double STANDARD_ATMOSPHERE_HPA = 1013.25;
        static constexpr double FEET_TO_METERS = 0.3048;

        int init() override;
        int read() override;

        bool parseLine(const char *line, Sample &parsed) const;
        void resetLineBuffer();
        void serviceStream(Stream &stream);

#if defined(ENV_TEENSY) && !defined(NATIVE)
        void startUsbHostIfNeeded();
        void serviceUsbHost();
#endif

        bool started = false;
        Stream *inputStream = nullptr;
        Sample latest;
        uint32_t sampleCount = 0;
        char lineBuffer[MAX_LINE_LEN] = {0};
        size_t lineLength = 0;

#if defined(ENV_TEENSY) && !defined(NATIVE)
        USBHost usbHost;
        USBHub usbHub1;
        USBHub usbHub2;
        USBHub usbHub3;
        USBSerial_BigBuffer usbSerial;
        bool usbHostEnabled = true;
        bool usbHostStarted = false;
        bool usbSerialConnected = false;
        uint32_t usbBaud = DEFAULT_USB_BAUD;
#endif
    };
}

#endif // ASTRA_ROCKET_BLUE_RAVEN_H
