#pragma once

#include <Arduino.h>
#include "10_bionxregg.h"
#include "05_can_functions.h"


// Button state struct for RTOS-safe handling
struct ButtonState {
    bool lastState = HIGH;
    bool debouncedState = HIGH;
    TickType_t lastDebounceTime = 0;
    TickType_t pressStartTime = 0;
    TickType_t lastReleaseTime = 0;
    TickType_t lastPressTime = 0;
    int pressCount = 0;
    bool longPressHandled = false;
};



// Funktionsdeklarationen
void setupDisplay();
int16_t handleButton(int16_t& assistLevel, bool& light, int16_t& recuLevel);
void updateDisplay(int16_t assistLevel,int8_t rekupLevel, uint8_t motorSpeed, uint8_t motorStatus, uint8_t batteryLevel,uint8_t motorlevel,bool light);
uint8_t getModeDisplay(int16_t assistLevel, int8_t recuplevel);
int16_t readUART();
void uart_event_task(void *parameter);

