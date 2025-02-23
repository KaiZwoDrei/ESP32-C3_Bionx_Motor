#ifndef CAN_FUNCTIONS_H
#define CAN_FUNCTIONS_H

#include <Arduino.h>

void setupCAN();
bool readBionxRegister(int canId, uint16_t address, uint16_t &result);
bool writeBionxRegister(int canId, uint16_t address, uint16_t value);

#endif // CAN_FUNCTIONS_H
