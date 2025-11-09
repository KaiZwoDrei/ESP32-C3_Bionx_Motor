#include "08_uartdisplay.h"
#include <Arduino.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// Lokale Variablen
#define UART_NUM 1

#define Button_PIN  4
#define TXD_PIN 9   // ESP32-C3 Supermicro
#define RXD_PIN 10  // ESP32-C3 Supermicro

#define UART_BAUD_RATE 115200
#define UART_QUEUE_SIZE 10
#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024

// Externe Referenzen
extern ButtonState button;
bool  light;  // status Licht

// Assistenzmodi
const int16_t MODE_OFF = 0;
const int16_t MODE_ECO = 50;
const int16_t MODE_DRIVE = 100;
const int16_t MODE_SPORT = 150;

// Rekuperationswerte (negativ)
const int16_t Recu_1 = -16;
const int16_t Recu_2 = -32;
const int16_t Recu_3 = -64;

// Arrays für einfache Modi-Zyklen
const int16_t assistModes[] = {MODE_ECO, MODE_DRIVE, MODE_SPORT};
const size_t assistModesCount = sizeof(assistModes) / sizeof(assistModes[0]);

const int16_t recuLevels[] = {0, Recu_1, Recu_2, Recu_3};  // 0 = kein Reku
const size_t recuLevelsCount = sizeof(recuLevels) / sizeof(recuLevels[0]);

uint8_t tx_frame[14];
uint8_t uart_buf[8];
uint16_t throttle_value = 0;
uint16_t brake_value = 0;

QueueHandle_t uart_queue;

// Hilfsfunktion: Index finden
int findAssistIndex(int16_t mode) {
    for (size_t i = 0; i < assistModesCount; i++) {
        if (assistModes[i] == mode) return i;
    }
    return 0; // default MODE_OFF
}

// Zyklisches Umschalten Assistenzmodus
void cycleAssistMode(int16_t &assistLevel) {
    int idx = findAssistIndex(assistLevel);
    idx = (idx + 1) % assistModesCount;
    assistLevel = assistModes[idx];
}

// Umschalten Rekuperationsmodus an/aus
void toggleRecuMode(int16_t assistLevel, int16_t &recuLevel) {
    int idx = findAssistIndex(assistLevel);
    if (recuLevel == 0) {
        recuLevel = recuLevels[idx + 1];
        Serial.print("RECU Mode ON: ");
        Serial.println(recuLevel);
    } else {
        recuLevel = 0;
        Serial.println("RECU Mode OFF");
    }
}

// Überarbeitete handleButton-Funktion
int16_t handleButton(int16_t &assistLevel, bool &light, int16_t &recuLevel) {
    const unsigned long DEBOUNCE_DELAY = 5;       
    const unsigned long MULTI_CLICK_TIME = 300;   
    const unsigned long CLICK_TIMEOUT = 400;      
    const unsigned long LONG_PRESS_TIME = 700;    
    
    bool currentReading = digitalRead(Button_PIN);
    unsigned long currentTime = millis();

    // Entprellung
    if (currentReading != button.lastState) {
        button.lastDebounceTime = currentTime;
    }

    if ((currentTime - button.lastDebounceTime) > DEBOUNCE_DELAY) {
        if (button.debouncedState != currentReading) {
            button.debouncedState = currentReading;

            if (button.debouncedState == LOW) {
                button.pressStartTime = currentTime;
                button.longPressHandled = false;
                if ((currentTime - button.lastReleaseTime) < MULTI_CLICK_TIME) {
                    button.pressCount++;
                } else {
                    button.pressCount = 1;
                }
                button.lastPressTime = currentTime;

                Serial.print("Press detected: ");
                Serial.println(button.pressCount);
            } else {
                button.lastReleaseTime = currentTime;
            }
        }
        
        // Long Press: Licht toggeln
        if (button.debouncedState == LOW && !button.longPressHandled) {
            if ((currentTime - button.pressStartTime) > LONG_PRESS_TIME) {
                light = !light;
                Serial.print("Long Press - Light: ");
                Serial.println(light ? "ON" : "OFF");
                button.longPressHandled = true;
                button.pressCount = 0;
            }
        }
    }

    // Klickverarbeitung
    if (button.pressCount > 0 &&
    (currentTime - button.lastPressTime) > CLICK_TIMEOUT &&
    button.debouncedState == HIGH) {

    Serial.print("Processing clicks: ");
    Serial.println(button.pressCount);

    switch (button.pressCount) {
        case 2:  // Doppelklick: Wechsel zwischen Reku- und Normalmodus
            if (recuLevel == 0) {
                // Rekuperation einschalten, auf Basis aktuellen assistLevel
                toggleRecuMode(assistLevel, recuLevel);
            } else {
                // Rekuperation ausschalten, zurück zu normalem assistLevel (bleibt unverändert)
                recuLevel = 0;
                Serial.println("RECU Mode OFF due to double click");
            }
            break;

        case 1:  // Einzelklick: Modi wechseln
            if (recuLevel != 0) {
                // Im Rekuperationsmodus -> Rekuperationslevel zyklisch wechseln
                int idx = 0;
                // Finde Index des aktuellen Rekuperationslevels im recuLevels Array
                for (size_t i = 1; i < recuLevelsCount; i++) {
                    if (recuLevels[i] == recuLevel) {
                        idx = i;
                        break;
                    }
                }
                // Zyklisch das nächste Rekuperationslevel auswählen
                idx = (idx % (recuLevelsCount - 1)) + 1;
                recuLevel = recuLevels[idx];

                Serial.print("Cycling RECU Mode: ");
                Serial.println(recuLevel);
            } else {
                // Im normalen Modus -> Assistenzmodus zyklisch wechseln
                cycleAssistMode(assistLevel);
                Serial.print("New Assist Mode: ");
                Serial.println(assistLevel);
            }
            break;

        case 3:  // Dreifachklick: Beispiel System Shutdown
            writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_SHUTDOWN, 0x01);
            Serial.println("System Shutdown");
            break;

        default:
            // Optional weitere Klickanzahlen handhaben
            break;
    }
    button.pressCount = 0;
}
    button.lastState = currentReading;
    return assistLevel;
}

uint8_t getModeDisplay(int16_t assistLevel, int8_t recuplevel) {
    if (recuplevel == Recu_1) return 128 + 2;     // ECO Symbol
    if (recuplevel == Recu_2) return 128 + 1;     // DRIVE Symbol
    if (recuplevel == Recu_3) return 128 + 4;     // SPORT Symbol

    if (assistLevel == MODE_OFF) return 16;       // OFF Symbol
    if (assistLevel == MODE_ECO) return 2;        // ECO Symbol
    if (assistLevel == MODE_DRIVE) return 1;      // DRIVE Symbol
    if (assistLevel == MODE_SPORT) return 4;      // SPORT Symbol

    return 0;
}

void setupDisplay() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    uart_queue = xQueueCreate(1, sizeof(uint32_t));

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, 256, 0, 0, 0, 0));
    pinMode(Button_PIN, INPUT_PULLUP);
}

void updateDisplay(int16_t assistLevel, int8_t rekupLevel, uint8_t motorSpeed, uint8_t motorStatus, uint8_t batteryLevel, uint8_t motorlevel, bool light) {
    uart_set_pin(UART_NUM, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_disable_rx_intr(UART_NUM);
    uart_set_rts(UART_NUM, 0);

    int8_t segmentdisp;
    int8_t error = 0;

    if (rekupLevel < 0) {
        segmentdisp = 110 - rekupLevel / 16;  // 7-Segment Anzeige A0-A9, 110 = b0-b9 
    } else if (assistLevel < 0) {
        segmentdisp = 110 - (assistLevel) / 33;
    } else {
        segmentdisp = motorlevel + (motorlevel >> 1) + (motorlevel >> 4); // Umrechnung 0-64 --> 0-100
    }

    if (batteryLevel == 0) {
        error = 1;
    }

    tx_frame[0] = 0x55;
    tx_frame[1] = 0xAA;
    tx_frame[2] = 0x08;
    tx_frame[3] = 0x21;
    tx_frame[4] = 0x64;
    tx_frame[5] = 0x00;

    tx_frame[6] = getModeDisplay(assistLevel, rekupLevel);
    tx_frame[7] = batteryLevel;
    tx_frame[8] = light ? 1 : 0;  // Licht anzeigen
    tx_frame[9] = 0;              // Hupe
    tx_frame[10] = segmentdisp;
    tx_frame[11] = error;         

    // CRC Berechnung
    uint16_t crc = 0;
    for (int i = 2; i < 12; i++) {
        crc += tx_frame[i];
    }
    uint16_t c_out = crc ^ 0xFFFF;
    tx_frame[12] = c_out & 0xFF;
    tx_frame[13] = (c_out >> 8) & 0xFF;

    uart_write_bytes(UART_NUM, (const char*)tx_frame, 14);
    uart_wait_tx_done(UART_NUM, portMAX_DELAY);
    uart_flush(UART_NUM);
}

int16_t readUART() {
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_rts(UART_NUM, 1);
    uart_enable_rx_intr(UART_NUM);

    uint8_t uart_buf[8];
    int len = uart_read_bytes(UART_NUM, uart_buf, sizeof(uart_buf), portMAX_DELAY);

    for (size_t i = 0; i < 10; i++) {
        if (len < 8 || uart_buf[4] != 0x65) {
            uart_flush(UART_NUM);
            len = uart_read_bytes(UART_NUM, uart_buf, sizeof(uart_buf), portMAX_DELAY);
        }
    }

    if (uart_buf[4] == 0x65) {
        uint16_t throttle_val = std::min(std::max(uart_buf[7] - 43, 0) * 100 / 153, 99);
        uart_flush(UART_NUM);
        return throttle_val;
    }
    return 100;
}
