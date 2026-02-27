#pragma once

#include <Arduino.h>

class MotorDriver;
class AirbrakeController;

void handleAirbrakeMessage(const char *message, const char *prefix, Stream *source,
                          AirbrakeController &airbrakeCtrl, MotorDriver &motorDriver);

void handleHitlMessage(const char *message, const char *prefix, Stream *source, bool hitlRuntimeReady);

