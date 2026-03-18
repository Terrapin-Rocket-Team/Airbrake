#pragma once

#include <Arduino.h>

class MotorDriver;
class AirbrakeController;

void handleAirbrakeMessage(const char *message, const char *prefix, Stream *source, MotorDriver &motorDriver);
void updateAirbrakeSweep(MotorDriver &motorDriver);

