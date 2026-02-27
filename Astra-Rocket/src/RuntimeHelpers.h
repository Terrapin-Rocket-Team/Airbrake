#ifndef RUNTIME_HELPERS_H
#define RUNTIME_HELPERS_H

#include <Arduino.h>
#include <stdint.h>

class MotorDriver;
namespace astra
{
class VoltageSensor;
} // namespace astra

namespace astra_rocket
{
class AstraRocket;
class AstraRocketConfig;
class RocketState;
} // namespace astra_rocket

class AirbrakeController;

namespace runtime_helpers
{
void runOrientationCalibration(astra_rocket::AstraRocket &rocket,
                               astra_rocket::AstraRocketConfig &config,
                               bool usingHitlSensors,
                               Stream &serialOut,
                               uint32_t stationaryCalTimeMs,
                               uint32_t magCalTimeMs);

void emitCompactHeader(Stream &out);

void emitCompactData(Stream &out,
                     astra_rocket::RocketState *state,
                     astra_rocket::AstraRocketConfig &config,
                     AirbrakeController *airbrakeCtrl,
                     astra::VoltageSensor *voltageSensor,
                     MotorDriver *motorDriver);
} // namespace runtime_helpers

#endif // RUNTIME_HELPERS_H
