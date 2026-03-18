#include "AirbrakeTelemetry.h"

#include <AstraRocketConfig.h>
#include <RocketState.h>
#include <Sensors/SensorManager/SensorManager.h>
#include <Sensors/VoltageSensor/VoltageSensor.h>

#include "../AirbrakeController.h"
#include "../AvionicsPacketProtocol.h"
#include "../Helpers/PacketStreams.h"
#include "../MotorDriver/MotorDriver.h"

using namespace astra;
using namespace astra_rocket;

AirbrakeTelemetryPublisher::AirbrakeTelemetryPublisher(RocketState &rocketState,
                                                       AstraRocketConfig &config,
                                                       AirbrakeController &airbrakeCtrl,
                                                       VoltageSensor &voltageSensor,
                                                       MotorDriver &motorDriver,
                                                       PacketStreams &telemetryStreams,
                                                       uint32_t publishPeriodMs)
    : rocketState(rocketState),
      config(config),
      airbrakeCtrl(airbrakeCtrl),
      voltageSensor(voltageSensor),
      motorDriver(motorDriver),
      telemetryStreams(telemetryStreams),
      publishPeriodMs(publishPeriodMs)
{
}

float AirbrakeTelemetryPublisher::metersToFeet(double meters)
{
    return static_cast<float>(meters * 3.28083989501312);
}

bool AirbrakeTelemetryPublisher::buildPacket(avionics_packet::PacketBuffer &packet) const
{
    avionics_packet::AbTelemetry telemetry = {};

    telemetry.positionZFeet = metersToFeet(rocketState.getAltitudeAGL());

    const Vector<3> velocity = rocketState.getVelocity();
    telemetry.velocityZMs = static_cast<float>(velocity.z());

    const Vector<3> acceleration = rocketState.getAcceleration();
    telemetry.accelZMs2 = static_cast<float>(acceleration.z());

    SensorManager *sensorManager = config.getSensorManager();
    if (sensorManager != nullptr)
    {
        if (Barometer *baroSource = sensorManager->getBaroSource();
            baroSource != nullptr && baroSource->isInitialized())
        {
            telemetry.hasBaroAgl = true;
            telemetry.baroAglFeet = metersToFeet(baroSource->getASLAltM() - rocketState.getGroundLevelMSL());
        }
    }

    const Quaternion orientation = rocketState.getRocketOrientation();
    telemetry.quatW = static_cast<float>(orientation.w());
    telemetry.quatX = static_cast<float>(orientation.x());
    telemetry.quatY = static_cast<float>(orientation.y());
    telemetry.quatZ = static_cast<float>(orientation.z());

    telemetry.hasBattery = voltageSensor.isInitialized();
    telemetry.batteryVolts = static_cast<float>(voltageSensor.getVoltage());

    telemetry.hasMotorBattery = motorDriver.isInitialized();
    telemetry.motorBatteryVolts = motorDriver.getBatVoltage();

    telemetry.desiredAngleDeg = static_cast<float>(airbrakeCtrl.getCommandedDeployment());
    telemetry.actualAngleDeg = static_cast<float>(airbrakeCtrl.getCurrentDeployment());
    telemetry.predictedApogeeFeet = metersToFeet(airbrakeCtrl.getPredictedApogee());

    return avionics_packet::encodeAbTelemetry(telemetry, packet);
}

void AirbrakeTelemetryPublisher::publishIfDue()
{
    const uint32_t now = millis();
    if ((now - lastPublishMs) < publishPeriodMs)
        return;

    lastPublishMs = now;

    avionics_packet::PacketBuffer packet;
    if (!buildPacket(packet))
        return;

    const bool sent = telemetryStreams.send(packet);
    if (!sent)
    {
        LOGW("ABTELEM send failed");
        return;
    }

    if (!reportedOnline)
    {
        reportedOnline = true;
        LOGI("ABTELEM stream online on Serial2");
    }
}

void setupAirbrakeTelemetrySerial(PacketStreams &telemetryStreams,
                                  HardwareSerial &serialPort,
                                  uint32_t baud)
{
    serialPort.begin(baud);
    delay(100);
    telemetryStreams.addStream(serialPort);
}
