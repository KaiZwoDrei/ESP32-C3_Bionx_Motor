#pragma once

#include <Arduino.h>

// Select OTA method: set to 1 for Arduino OTA (espota), 0 for ElegantOTA (web)
#define USE_ARDUINO_OTA 0
void mongooseTask(void *pvParameters);
