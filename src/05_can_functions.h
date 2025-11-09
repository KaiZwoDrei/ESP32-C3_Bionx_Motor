#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif


void setupCAN(void);
bool readBionxRegister(int canId, uint16_t address, uint16_t *result);
bool writeBionxRegister(int canId, uint16_t address, uint16_t value);

#ifdef __cplusplus
}
#endif
