#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <string>

#include <unity.h>

#include "../../src/BlueRaven/BRAccel.h"
#include "../../src/BlueRaven/BRBaro.h"
#include "../../src/BlueRaven/BRIMU.h"
#include "../../src/BlueRaven/BlueRaven.h"

namespace test_blue_raven {

namespace {

constexpr double kTol = 1e-6;
constexpr double kStandardGravity = 9.80665;
constexpr double kDegreesToRadians = 0.01745329251994329577;

class TestStream : public Stream {
public:
    void push(const std::string &data)
    {
        for (char ch : data)
            buffer.push_back(static_cast<uint8_t>(ch));
    }

    bool available() override
    {
        return !buffer.empty();
    }

    int read() override
    {
        if (buffer.empty())
            return -1;

        const uint8_t value = buffer.front();
        buffer.pop_front();
        return value;
    }

    int peek() override
    {
        if (buffer.empty())
            return -1;

        return buffer.front();
    }

    void flush() override {}

    size_t write(uint8_t) override
    {
        return 1;
    }

private:
    std::deque<uint8_t> buffer;
};

std::string buildPacket(unsigned long sequence,
                        long hgx,
                        long hgy,
                        long hgz,
                        long ax,
                        long ay,
                        long az,
                        long pressureAtmX10000,
                        long tempFx100,
                        long batteryMv,
                        long gyx,
                        long gyy,
                        long gyz,
                        long tiltDeciDeg,
                        long rollDeg,
                        long velFps,
                        long aglFt)
{
    char body[256] = {0};
    std::snprintf(body,
                  sizeof(body),
                  "@ BLR_STAT %lu 2026 3 15 12:34:56 HG: %ld %ld %ld XYZ: %ld %ld %ld Bo: %ld %ld bt: %ld gy: %ld %ld %ld ang: %ld %ld vel %ld AGL %ld",
                  sequence,
                  hgx,
                  hgy,
                  hgz,
                  ax,
                  ay,
                  az,
                  pressureAtmX10000,
                  tempFx100,
                  batteryMv,
                  gyx,
                  gyy,
                  gyz,
                  tiltDeciDeg,
                  rollDeg,
                  velFps,
                  aglFt);

    const uint16_t crc = astra_rocket::BlueRaven::crc16Buypass(
        reinterpret_cast<const uint8_t *>(body),
        std::strlen(body));

    char packet[320] = {0};
    std::snprintf(packet, sizeof(packet), "%s CRC: %x", body, crc);
    return packet;
}

} // namespace

void test_poll_waits_for_complete_line()
{
    astra_rocket::BlueRaven blueRaven;
    TestStream stream;

    TEST_ASSERT_EQUAL(0, blueRaven.begin());
    blueRaven.setStream(&stream);

    const std::string packet = buildPacket(1, 100, -100, 50, 1000, -1000, 500, 10000, 6800, 3700, 9000, 0, -4500, 150, 10, 100, 500);
    const size_t split = packet.size() / 2;

    stream.push(packet.substr(0, split));
    TEST_ASSERT_FALSE(blueRaven.poll());
    TEST_ASSERT_FALSE(blueRaven.hasValidSample());
    TEST_ASSERT_EQUAL_UINT32(0, blueRaven.getSampleCount());

    stream.push(packet.substr(split));
    stream.push("\n");
    TEST_ASSERT_TRUE(blueRaven.poll());
    TEST_ASSERT_TRUE(blueRaven.hasValidSample());
    TEST_ASSERT_EQUAL_UINT32(1, blueRaven.getSampleCount());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 3.7, blueRaven.getBatteryVolts());
}

void test_children_only_consume_new_samples_once()
{
    astra_rocket::BlueRaven blueRaven;
    astra_rocket::BRBaro baro(blueRaven);
    astra_rocket::BRIMU imu(blueRaven);

    TEST_ASSERT_EQUAL(0, blueRaven.begin());
    TEST_ASSERT_EQUAL(0, baro.begin());
    TEST_ASSERT_EQUAL(0, imu.begin());
    TEST_ASSERT_FALSE(baro.shouldUpdate(0.0));
    TEST_ASSERT_FALSE(imu.shouldUpdate(0.0));

    const std::string packet1 = buildPacket(1, 100, 0, -100, 1000, 0, -1000, 10000, 6800, 3700, 9000, 0, -4500, 120, 10, 100, 500);
    TEST_ASSERT_TRUE(blueRaven.ingestLine(packet1.c_str()));

    TEST_ASSERT_TRUE(baro.shouldUpdate(0.0));
    TEST_ASSERT_TRUE(imu.shouldUpdate(0.0));
    TEST_ASSERT_EQUAL(0, baro.update());
    TEST_ASSERT_EQUAL(0, imu.update());

    TEST_ASSERT_DOUBLE_WITHIN(kTol, 1013.25, baro.getPressure());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 20.0, baro.getTemp());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 1000.0 * kStandardGravity / 1000.0, imu.getAccel().x());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 9000.0 * 0.01 * kDegreesToRadians, imu.getAngVel().x());

    TEST_ASSERT_FALSE(baro.shouldUpdate(0.0));
    TEST_ASSERT_FALSE(imu.shouldUpdate(0.0));

    const std::string packet2 = buildPacket(2, 200, 0, -200, 2000, 1000, -500, 9500, 7700, 3650, 0, 4500, -9000, 200, 20, 150, 800);
    TEST_ASSERT_TRUE(blueRaven.ingestLine(packet2.c_str()));

    TEST_ASSERT_TRUE(baro.shouldUpdate(0.0));
    TEST_ASSERT_TRUE(imu.shouldUpdate(0.0));
    TEST_ASSERT_EQUAL(0, baro.update());
    TEST_ASSERT_EQUAL(0, imu.update());

    TEST_ASSERT_DOUBLE_WITHIN(kTol, 0.95 * 1013.25, baro.getPressure());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 25.0, baro.getTemp());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 2000.0 * kStandardGravity / 1000.0, imu.getAccel().x());
    TEST_ASSERT_DOUBLE_WITHIN(kTol, 4500.0 * 0.01 * kDegreesToRadians, imu.getAngVel().y());
    TEST_ASSERT_FALSE(baro.shouldUpdate(0.0));
    TEST_ASSERT_FALSE(imu.shouldUpdate(0.0));
}

void run_test_blue_raven_tests()
{
    RUN_TEST(test_poll_waits_for_complete_line);
    RUN_TEST(test_children_only_consume_new_samples_once);
}

} // namespace test_blue_raven
