#ifndef UART_DISPLAY_H
#define UART_DISPLAY_H

#include <Arduino.h>

// Pin Definitionen
const uint8_t RX_PIN = 1;
const uint8_t Button_PIN = 0;

// Button Struktur
struct ButtonState {
    bool lastState = HIGH;
    bool debouncedState = HIGH;
    unsigned long lastDebounceTime = 0;
    unsigned long lastPressTime = 0;
    unsigned long lastReleaseTime = 0;
    unsigned long pressStartTime = 0;
    bool longPressHandled = false;
    uint8_t pressCount = 0;
};




// Funktionsdeklarationen
void setupDisplay();
int16_t handleButton(int16_t& assistLevel, bool& light);
void updateDisplay(int8_t assistLevel,int8_t rekupLevel, uint8_t motorSpeed, uint8_t motorStatus, uint8_t batteryLevel,uint8_t motorlevel);
uint8_t getModeDisplay(int8_t assistLevel, int8_t recuplevel);
int16_t readUART();
void uart_event_task(void *parameter);

#endif
