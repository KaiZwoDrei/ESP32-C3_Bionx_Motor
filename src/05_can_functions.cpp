// ============================================================
// 05_can_functions.cpp
// CAN-Bus Kommunikation mit automatischem Reconnect-Watchdog
// ============================================================
#include "05_can_functions.h"
#include <esp32_can.h>
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <Arduino.h>

#include "10_bionxregg.h"

// --- Mutex für exklusiven Bus-Zugriff ---
SemaphoreHandle_t canMutex;
static uint32_t mutexBlockCount = 0;

// --- Fehlerzähler für aufeinanderfolgende Lesefehler ---
static volatile int consecutiveReadErrors = 0;
#define CAN_RESET_THRESHOLD 5   // Nach 5 Fehlern in Folge → Reset

// --- Fehleranzeige-Variablen (von 08_uartdisplay.cpp gelesen) ---
volatile uint8_t  can_display_error_code    = 0;
volatile uint32_t can_error_display_until_ms = 0;

// --- Interner Fehlercode-Setter mit 5s Anzeigedauer ---
static void set_can_error_display(uint8_t code) {
    can_display_error_code     = code;
    can_error_display_until_ms = (uint32_t)millis() + 5000UL;
    Serial.printf("[CAN] Fehlercode %d wird 5s angezeigt\n", code);
}

// ============================================================
// setupCAN – Initialisierung
// ============================================================
void setupCAN() {
    CAN0.setCANPins(GPIO_NUM_0, GPIO_NUM_1);
    CAN0.begin(125000);
    CAN0.watchFor(0x08, 0x7FF);
    canMutex = xSemaphoreCreateMutex();
    Serial.println("[CAN] Initialisiert");
}

// ============================================================
// canbus_reset – Robuste Reset-Sequenz
//   Aufruf aus canWatchdogTask (mit Mutex) oder direkt
// ============================================================
void canbus_reset() {
    Serial.println("[CAN] === RESET wird durchgeführt ===");

    // Mutex holen damit kein anderer Task gleichzeitig sendet
    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        CAN0.disable();
        vTaskDelay(pdMS_TO_TICKS(80));   // Transceiver Zeit zum Beruhigen
        CAN0.setCANPins(GPIO_NUM_0, GPIO_NUM_1);
        CAN0.begin(125000);
        vTaskDelay(pdMS_TO_TICKS(50));
        CAN0.watchFor(0x08, 0x7FF);

        consecutiveReadErrors = 0;
        xSemaphoreGive(canMutex);
        Serial.println("[CAN] Reset abgeschlossen, Bus wieder aktiv");
    } else {
        Serial.println("[CAN] Reset: Mutex-Timeout – versuche erzwungenen Reset");
        // Erzwungener Reset ohne Mutex als letzter Ausweg
        CAN0.disable();
        vTaskDelay(pdMS_TO_TICKS(100));
        CAN0.setCANPins(GPIO_NUM_0, GPIO_NUM_1);
        CAN0.begin(125000);
        vTaskDelay(pdMS_TO_TICKS(50));
        CAN0.watchFor(0x08, 0x7FF);
        consecutiveReadErrors = 0;
        Serial.println("[CAN] Erzwungener Reset abgeschlossen");
    }
}

// ============================================================
// canWatchdogTask – prüft Bus-Gesundheit alle 500ms
// ============================================================
void canWatchdogTask(void *pvParameters) {
    static uint32_t resetCount = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // 1) TWAI Hardware-Status prüfen
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            bool busOff = (status.state == TWAI_STATE_BUS_OFF);
            bool stopped = (status.state == TWAI_STATE_STOPPED);

            if (busOff || stopped) {
                resetCount++;
                Serial.printf("[CAN-WDT] BUS_OFF/STOPPED erkannt! Reset #%lu\n", resetCount);
                set_can_error_display(busOff ? 1 : 2);  // E1 = BUS_OFF, E2 = STOPPED
                canbus_reset();
                continue;  // Sofort neu prüfen nach Reset
            }

            // Warnung bei hohen Fehlerzählern (noch kein BUS_OFF)
            if (status.tx_error_counter > 96 || status.rx_error_counter > 96) {
                Serial.printf("[CAN-WDT] Warnung: TX-Err=%d RX-Err=%d\n",
                              status.tx_error_counter, status.rx_error_counter);
            }
        }

        // 2) Aufeinanderfolgende Lesefehler aus readBionxRegister prüfen
        if (consecutiveReadErrors >= CAN_RESET_THRESHOLD) {
            resetCount++;
            Serial.printf("[CAN-WDT] %d Lesefehler in Folge → Reset #%lu\n",
                          consecutiveReadErrors, resetCount);
            set_can_error_display(3);  // E3 = Timeout-Fehler
            canbus_reset();
        }

        // 3) Fehleranzeige automatisch löschen nach Ablauf der 5s
        if (can_display_error_code != 0 &&
            (uint32_t)millis() >= can_error_display_until_ms) {
            can_display_error_code = 0;
        }
    }
}

// ============================================================
// readBionxRegister
// ============================================================
bool readBionxRegister(int canId, uint16_t address, uint16_t *result) {
    bool readSuccess = false;

    if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        CAN_FRAME txFrame;
        txFrame.rtr = 0;
        txFrame.id = canId;
        txFrame.extended = false;
        txFrame.length = 2;
        txFrame.data.uint8[0] = 0;
        txFrame.data.uint8[1] = address;

        CAN0.sendFrame(txFrame);

        // Veraltete Frames verwerfen
        CAN_FRAME dummyFrame;
        while (CAN0.read(dummyFrame)) {}

        TickType_t startTime = xTaskGetTickCount();
        const TickType_t timeout = pdMS_TO_TICKS(4);

        while ((xTaskGetTickCount() - startTime) < timeout) {
            if (CAN0.read(txFrame) &&
                txFrame.data.uint8[0] == 0 &&
                txFrame.data.uint8[1] == address) {
                *result = (txFrame.data.uint8[2] << 8) | txFrame.data.uint8[3];
                readSuccess = true;
                break;
            }
            taskYIELD();
        }
        xSemaphoreGive(canMutex);
    } else {
        mutexBlockCount++;
        if (mutexBlockCount >= 1000) {
            Serial.println("[CAN] Mutex 1000x blockiert!");
            mutexBlockCount = 0;
        }
    }

    // Aufeinanderfolgende Fehler zählen
    if (readSuccess) {
        consecutiveReadErrors = 0;
    } else {
        consecutiveReadErrors++;
        Serial.printf("[CAN] Lesefehler #%d – Adresse 0x%02X (%s)\n",
                      consecutiveReadErrors, address, getMotorText(address));
    }

    return readSuccess;
}

// ============================================================
// writeBionxRegister
// ============================================================
bool writeBionxRegister(int canId, uint16_t address, uint16_t value) {
    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 4;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;
    txFrame.data.uint8[2] = (value >> 8) & 0xFF;
    txFrame.data.uint8[3] = value & 0xFF;

    bool result  = CAN0.sendFrame(txFrame);
    bool result2 = CAN0.sendFrame(txFrame);
    return result || result2;
}
