#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

void setupBLE();
void updateBLEData(int16_t humanPower, int16_t torqueValue, uint16_t speed, int16_t motorPower, uint8_t batteryLevel) ;


//void handleBLEConnections();

