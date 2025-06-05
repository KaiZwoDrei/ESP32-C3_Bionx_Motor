#pragma once

#include <Arduino.h>

// Pin Definitionen
const uint8_t RX_PIN = 1;
const uint8_t Button_PIN = 0;


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
int16_t handleButton(int16_t& assistLevel, bool& light);
void updateDisplay(int8_t assistLevel,int8_t rekupLevel, uint8_t motorSpeed, uint8_t motorStatus, uint8_t batteryLevel,uint8_t motorlevel);
uint8_t getModeDisplay(int8_t assistLevel, int8_t recuplevel);
int16_t readUART();
void uart_event_task(void *parameter);

