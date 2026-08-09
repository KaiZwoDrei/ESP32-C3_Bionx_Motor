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
void canbus_reset(void);
void canWatchdogTask(void *pvParameters);

// Wird von updateDisplay() ausgewertet:
// > 0 → Fehlercode-Anzeige aktiv (läuft in canWatchdogTask herunter)
extern volatile uint8_t  can_display_error_code;   // Fehlercode (z.B. 1 = BUS_OFF, 2 = Timeout)
extern volatile uint32_t can_error_display_until_ms; // millis()-Zeitstempel bis wann anzeigen

#ifdef __cplusplus
}
#endif
