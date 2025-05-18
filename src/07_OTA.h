#ifndef OTA_H
#define OTA_H

#include <Arduino.h>

// Select OTA method: set to 1 for Arduino OTA (espota), 0 for ElegantOTA (web)
#define USE_ARDUINO_OTA 1

void setupOTA(const char* hostname = "esp32controller");
void handleOTA();
void otaTask(void *parameter);

#endif // OTA_H
