#include "BlueRaven.h"

#include <cstdio>
#include <cstring>

#include "BGHGAccel.h"
#include "BRAccel.h"
#include "BRBaro.h"
#include "BRIMU.h"

namespace
{
    constexpr double kStandardGravity = 9.80665;
    constexpr double kDegreesToRadians = 0.01745329251994329577;
    constexpr double kCentiGToMps2 = 0.01 * kStandardGravity;
    constexpr double kMilliGToMps2 = 0.001 * kStandardGravity;
    constexpr double kCentiDegPerSecToRadPerSec = 0.01 * kDegreesToRadians;
}

namespace astra_rocket
{
#if defined(ENV_TEENSY) && !defined(NATIVE)
    BlueRaven::BlueRaven()
        : usbHub1(usbHost), usbHub2(usbHost), usbHub3(usbHost), usbSerial(usbHost, 1)
    {
    }
#else
    BlueRaven::BlueRaven() = default;
#endif

    int BlueRaven::begin()
    {
        if (started)
            return 0;

        started = true;
        resetLineBuffer();

#if defined(ENV_TEENSY) && !defined(NATIVE)
        startUsbHostIfNeeded();
#endif

        return 0;
    }

    bool BlueRaven::poll()
    {
        if (!started)
            begin();

#if defined(ENV_TEENSY) && !defined(NATIVE)
        serviceUsbHost();
#endif

        if (inputStream)
            serviceStream(*inputStream);

        return latest.valid;
    }

    void BlueRaven::setStream(Stream *stream)
    {
        inputStream = stream;

#if defined(ENV_TEENSY) && !defined(NATIVE)
        usbHostEnabled = false;
        usbSerialConnected = false;
#endif
    }

    void BlueRaven::useUsbHost(uint32_t baud)
    {
#if defined(ENV_TEENSY) && !defined(NATIVE)
        usbBaud = baud;
        usbHostEnabled = true;
        inputStream = nullptr;

        if (started)
            startUsbHostIfNeeded();
#else
        (void)baud;
#endif
    }

    bool BlueRaven::isConnected() const
    {
#if defined(ENV_TEENSY) && !defined(NATIVE)
        if (usbHostEnabled)
            return usbSerialConnected;
#endif

        return inputStream != nullptr;
    }

    bool BlueRaven::ingestLine(const char *line)
    {
        if (!line)
            return false;

        Sample parsed;
        if (!parseLine(line, parsed))
            return false;

        latest = parsed;
        sampleCount++;
        return true;
    }

    uint16_t BlueRaven::crc16Buypass(const uint8_t *data, size_t len)
    {
        uint16_t crc = 0x0000;
        for (size_t i = 0; i < len; i++)
        {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (uint8_t bit = 0; bit < 8; bit++)
            {
                if ((crc & 0x8000u) != 0)
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x8005u);
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

    bool BlueRaven::parseLine(const char *line, Sample &parsed) const
    {
        if (!line || std::strncmp(line, "@ BLR_STAT ", 11) != 0)
            return false;

        const char *crcTag = std::strstr(line, " CRC:");
        if (!crcTag)
            return false;

        const uint16_t computedCrc =
            crc16Buypass(reinterpret_cast<const uint8_t *>(line), static_cast<size_t>(crcTag - line));

        unsigned long sequence = 0;
        int year = 0;
        int month = 0;
        int day = 0;
        char timeText[16] = {0};
        long hgx = 0;
        long hgy = 0;
        long hgz = 0;
        long ax = 0;
        long ay = 0;
        long az = 0;
        long pressureAtmX10000 = 0;
        long tempFx100 = 0;
        long batteryMv = 0;
        long gyx = 0;
        long gyy = 0;
        long gyz = 0;
        long tiltDeciDeg = 0;
        long rollDeg = 0;
        long velFps = 0;
        long aglFt = 0;
        unsigned int parsedCrc = 0;

        const int fields = std::sscanf(
            line,
            "@ BLR_STAT %lu %d %d %d %15s HG: %ld %ld %ld XYZ: %ld %ld %ld Bo: %ld %ld bt: %ld gy: %ld %ld %ld ang: %ld %ld vel %ld AGL %ld CRC: %x",
            &sequence,
            &year,
            &month,
            &day,
            timeText,
            &hgx,
            &hgy,
            &hgz,
            &ax,
            &ay,
            &az,
            &pressureAtmX10000,
            &tempFx100,
            &batteryMv,
            &gyx,
            &gyy,
            &gyz,
            &tiltDeciDeg,
            &rollDeg,
            &velFps,
            &aglFt,
            &parsedCrc);

        if (fields != 22)
            return false;

        if (computedCrc != static_cast<uint16_t>(parsedCrc))
            return false;

        parsed.valid = true;
        parsed.sequence = static_cast<uint32_t>(sequence);
        parsed.year = static_cast<uint16_t>(year);
        parsed.month = static_cast<uint8_t>(month);
        parsed.day = static_cast<uint8_t>(day);
        std::strncpy(parsed.timeOfDay, timeText, sizeof(parsed.timeOfDay) - 1);
        parsed.timeOfDay[sizeof(parsed.timeOfDay) - 1] = '\0';
        parsed.highGAccelMps2 =
            astra::Vector<3>(hgx * kCentiGToMps2, hgy * kCentiGToMps2, hgz * kCentiGToMps2);
        parsed.accelMps2 =
            astra::Vector<3>(ax * kMilliGToMps2, ay * kMilliGToMps2, az * kMilliGToMps2);
        parsed.pressureHpa = (static_cast<double>(pressureAtmX10000) / 10000.0) * STANDARD_ATMOSPHERE_HPA;
        parsed.temperatureC = ((static_cast<double>(tempFx100) / 100.0) - 32.0) * (5.0 / 9.0);
        parsed.batteryVolts = static_cast<double>(batteryMv) / 1000.0;
        parsed.gyroRadPerSec =
            astra::Vector<3>(gyx * kCentiDegPerSecToRadPerSec,
                             gyy * kCentiDegPerSecToRadPerSec,
                             gyz * kCentiDegPerSecToRadPerSec);
        parsed.tiltDeg = static_cast<double>(tiltDeciDeg) / 10.0;
        parsed.rollDeg = static_cast<double>(rollDeg);
        parsed.verticalVelocityMps = static_cast<double>(velFps) * FEET_TO_METERS;
        parsed.altitudeAglM = static_cast<double>(aglFt) * FEET_TO_METERS;
        parsed.crc = static_cast<uint16_t>(parsedCrc);

        return true;
    }

    void BlueRaven::resetLineBuffer()
    {
        lineLength = 0;
        lineBuffer[0] = '\0';
    }

    void BlueRaven::serviceStream(Stream &stream)
    {
        while (stream.available() > 0)
        {
            const int ch = stream.read();
            if (ch < 0)
                break;

            if (ch == '\r')
                continue;

            if (ch == '\n')
            {
                lineBuffer[lineLength] = '\0';
                if (lineLength > 0)
                    ingestLine(lineBuffer);
                resetLineBuffer();
                continue;
            }

            if (lineLength >= (MAX_LINE_LEN - 1))
            {
                resetLineBuffer();
                continue;
            }

            lineBuffer[lineLength++] = static_cast<char>(ch);
            lineBuffer[lineLength] = '\0';
        }
    }

#if defined(ENV_TEENSY) && !defined(NATIVE)
    void BlueRaven::startUsbHostIfNeeded()
    {
        if (!usbHostEnabled || usbHostStarted)
            return;

        usbHost.begin();
        usbSerial.begin(usbBaud);
        usbHostStarted = true;
        usbSerialConnected = static_cast<bool>(usbSerial);
    }

    void BlueRaven::serviceUsbHost()
    {
        if (!usbHostEnabled)
            return;

        startUsbHostIfNeeded();
        usbHost.Task();

        const bool connectedNow = static_cast<bool>(usbSerial);
        if (connectedNow && !usbSerialConnected)
        {
            usbSerial.begin(usbBaud);
            usbSerial.setDTR(true);
            usbSerial.setRTS(true);
        }

        usbSerialConnected = connectedNow;
        if (usbSerialConnected)
            serviceStream(usbSerial);
    }
#endif

    BRAccel::BRAccel(BlueRaven &parentIn, const char *name)
        : Accel(name), parent(&parentIn)
    {
        setUpdateRate(5);
    }

    int BRAccel::init()
    {
        return parent ? parent->begin() : -1;
    }

    int BRAccel::read()
    {
        if (!parent)
            return -1;

        parent->poll();
        if (!parent->hasValidSample())
            return -2;

        acc = parent->getAccel();
        healthy = true;
        return 0;
    }

    BRIMU::BRIMU(BlueRaven &parentIn, const char *name)
        : IMU6DoF(name), parent(&parentIn)
    {
        setUpdateRate(5);
    }

    int BRIMU::init()
    {
        return parent ? parent->begin() : -1;
    }

    int BRIMU::read()
    {
        if (!parent)
            return -1;

        parent->poll();
        if (!parent->hasValidSample())
            return -2;

        acc = parent->getAccel();
        angVel = parent->getGyro();
        healthy = true;
        return 0;
    }

    BRBaro::BRBaro(BlueRaven &parentIn, const char *name)
        : Barometer(name), parent(&parentIn)
    {
        setUpdateRate(5);
    }

    int BRBaro::init()
    {
        return parent ? parent->begin() : -1;
    }

    int BRBaro::read()
    {
        if (!parent)
            return -1;

        parent->poll();
        if (!parent->hasValidSample())
            return -2;

        pressure = parent->getPressureHpa();
        temp = parent->getTemperatureC();
        altitudeAglM = parent->getAltitudeAglM();
        healthy = true;
        return 0;
    }

    BGHGAccel::BGHGAccel(BlueRaven &parentIn, const char *name)
        : Accel(name), parent(&parentIn)
    {
        setUpdateRate(5);
    }

    int BGHGAccel::init()
    {
        return parent ? parent->begin() : -1;
    }

    int BGHGAccel::read()
    {
        if (!parent)
            return -1;

        parent->poll();
        if (!parent->hasValidSample())
            return -2;

        acc = parent->getHighGAccel();
        healthy = true;
        return 0;
    }
}
