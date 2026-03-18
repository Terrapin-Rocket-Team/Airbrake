#ifndef AIRBRAKE_TELEMETRY_H
#define AIRBRAKE_TELEMETRY_H

#include <Arduino.h>
#include <stdint.h>

namespace astra
{
class VoltageSensor;
} // namespace astra

namespace astra_rocket
{
class AstraRocketConfig;
class RocketState;
} // namespace astra_rocket

namespace avionics_packet
{
struct PacketBuffer;
} // namespace avionics_packet

class AirbrakeController;
class MotorDriver;
class PacketStreams;

class AirbrakeTelemetryPublisher
{
public:
    AirbrakeTelemetryPublisher(astra_rocket::RocketState &rocketState,
                               astra_rocket::AstraRocketConfig &config,
                               AirbrakeController &airbrakeCtrl,
                               astra::VoltageSensor &voltageSensor,
                               MotorDriver &motorDriver,
                               PacketStreams &telemetryStreams,
                               uint32_t publishPeriodMs = 500);

    void publishIfDue();

private:
    bool buildPacket(avionics_packet::PacketBuffer &packet) const;
    static float metersToFeet(double meters);

    astra_rocket::RocketState &rocketState;
    astra_rocket::AstraRocketConfig &config;
    AirbrakeController &airbrakeCtrl;
    astra::VoltageSensor &voltageSensor;
    MotorDriver &motorDriver;
    PacketStreams &telemetryStreams;
    const uint32_t publishPeriodMs;
    uint32_t lastPublishMs = 0;
    bool reportedOnline = false;
};

void setupAirbrakeTelemetrySerial(PacketStreams &telemetryStreams,
                                  HardwareSerial &serialPort,
                                  uint32_t baud = 115200);

#endif // AIRBRAKE_TELEMETRY_H
