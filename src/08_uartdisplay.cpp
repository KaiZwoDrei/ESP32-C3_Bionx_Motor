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

//#define TXD_PIN 0  // Luatos ESP32-C3
//#define RXD_PIN 1
#define TXD_PIN 20  // ESP32-C3 Supermicro
#define RXD_PIN 21  // ESP32-C3 Supermicro

#define UART_BAUD_RATE 115200
#define UART_QUEUE_SIZE 10
#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 1024
// Externe Referenzen
extern ButtonState button;
extern bool light;
// Define modes as constants
const int8_t MODE_OFF = 0;
const int8_t MODE_ECO = 33;
const int8_t MODE_DRIVE = 66;
const int8_t MODE_SPORT = 100;

uint8_t tx_frame[14];
uint8_t uart_buf[8];
uint16_t throttle_value = 0;
uint16_t brake_value = 0;

QueueHandle_t uart_queue;
uint8_t getModeDisplay(int8_t assistLevel, int8_t recuplevel) 
    {
   // if (assistLevel <0) return 128+2; 
    if (assistLevel == -MODE_ECO) return 128+2;       // ECO symbol
    if (assistLevel == -MODE_DRIVE) return 128+1;     // DRIVE symbol
    if (assistLevel == -MODE_SPORT) return 128+4;     // SPORT symbol
    if(assistLevel == MODE_OFF) return 16;      // OFF symbol
    if (assistLevel == MODE_ECO) return 2;       // ECO symbol
    if (assistLevel == MODE_DRIVE) return 1;     // DRIVE symbol
    if (assistLevel == MODE_SPORT) return 4;     // SPORT symbol
    else return 0;
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
   // ESP_ERROR_CHECK(uart_set_mode(UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
    pinMode(Button_PIN, INPUT_PULLUP);
}


    
int16_t handleButton(int16_t& assistLevel, bool& light) {
    const unsigned long DEBOUNCE_DELAY = 5;     // 20ms
    const unsigned long MULTI_CLICK_TIME = 300;   // 300ms
    const unsigned long CLICK_TIMEOUT = 400;      // 400ms
    const unsigned long LONG_PRESS_TIME = 700;    // 800ms für Long Press
    
    bool currentReading = digitalRead(Button_PIN);
    unsigned long currentTime = millis();

    // Entprellung
    if (currentReading != button.lastState) {
        button.lastDebounceTime = currentTime;
    }

    // Wenn genug Zeit für Entprellung vergangen ist
    if ((currentTime - button.lastDebounceTime) > DEBOUNCE_DELAY) {
        // Wenn sich der entprellte Zustand ändert
        if (button.debouncedState != currentReading) {
            button.debouncedState = currentReading;
            
            if (button.debouncedState == LOW) {  // Button gedrückt
                button.pressStartTime = currentTime;  // Startzeit für Long Press
                button.longPressHandled = false;     // Reset Long Press Flag
                
                if ((currentTime - button.lastReleaseTime) < MULTI_CLICK_TIME) {
                    button.pressCount++;
                } else {
                    button.pressCount = 1;
                }
                button.lastPressTime = currentTime;
                Serial.print("Press detected: ");
                Serial.println(button.pressCount);
            } else {  // Button losgelassen
                button.lastReleaseTime = currentTime;
            }
        }
        
        // Prüfe auf Long Press während Button gedrückt ist
        if (button.debouncedState == LOW && !button.longPressHandled) {
            if ((currentTime - button.pressStartTime) > LONG_PRESS_TIME) {
                // Long Press erkannt
                light = !light;
                Serial.print("Long Press - Light: ");
                Serial.println(light ? "ON" : "OFF");
                button.longPressHandled = true;
                button.pressCount = 0;  // Reset press count
            }
        }
    }

    // Verarbeite normale Klicks nach Timeout
    if (button.pressCount > 0 && 
        (currentTime - button.lastPressTime) > CLICK_TIMEOUT &&
        button.debouncedState == HIGH) {  // Nur wenn Button nicht gedrückt
        
        Serial.print("Processing clicks: ");
        Serial.println(button.pressCount);
        
        switch (button.pressCount) {
            case 2:  // Doppelklick: recu
                if (assistLevel>0)
                {
                    assistLevel = -MODE_ECO;  
                }
                else assistLevel=MODE_ECO;

               Serial.print("New RECU Level: ");
                Serial.println(assistLevel);
                break;
                
            case 1:  // Single click: Cycle through assist modes
                switch(assistLevel) {
                case MODE_OFF:   assistLevel = MODE_ECO; break;
                case MODE_ECO:   assistLevel = MODE_DRIVE; break;
                case MODE_DRIVE: assistLevel = MODE_SPORT; break;
                case MODE_SPORT: assistLevel = MODE_ECO; break;
                case -MODE_ECO:   assistLevel = -MODE_DRIVE; break;
                case -MODE_DRIVE: assistLevel = -MODE_SPORT; break;
                case -MODE_SPORT: assistLevel = -MODE_ECO; break;

                default:         assistLevel = MODE_OFF; break;
                                    }
    Serial.print("New Mode: ");
    Serial.println(assistLevel);
    break;
                
            case 3:  // Dreifachklick: System Reset oder andere Funktion
                
                assistLevel = 0;  // Reset Assist
                Serial.println("System Reset");
                break;
        }
        button.pressCount = 0;
    }
    
    button.lastState = currentReading;
    return assistLevel;
}
void updateDisplay(int8_t assistLevel,int8_t rekupLevel, uint8_t motorSpeed, uint8_t motorStatus, uint8_t batteryLevel, uint8_t motorlevel) {
    // Header
    uart_set_pin(UART_NUM,RXD_PIN ,UART_PIN_NO_CHANGE , UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
      uart_disable_rx_intr(UART_NUM);
      uart_set_rts(UART_NUM, 0);
int8_t segmentdisp;
int8_t error=0;
if(rekupLevel>0)
{
segmentdisp=110+rekupLevel/10; //7 segment zeigt nach 99 A0-A9, 110= b0-b9 
}
else if(assistLevel<0){
segmentdisp=110-(assistLevel)/33;
}
else segmentdisp = motorSpeed;
if(batteryLevel==0)
{
error=1;
}

 //   uart_flush(UART_NUM);  
    tx_frame[0] = 0x55;
    tx_frame[1] = 0xAA;
    tx_frame[2] = 0x08;
    tx_frame[3] = 0x21;
    tx_frame[4] = 0x64;
    tx_frame[5] = 0x00;

    tx_frame[6] = getModeDisplay(assistLevel,rekupLevel); //(mode field (1=drive, 2=eco, 4=sport, 8=charge, 16=off, 32=lock)
    tx_frame[7] =(batteryLevel);
    tx_frame[8] = light ? 1 : 0; //light anzeige
    tx_frame[9] = 0; //hupe
    tx_frame[10] = segmentdisp; // weiße anzeige des 7 segment anzeige
    tx_frame[11] =error; //(batteryLevel); //schraubenschlüssel und rote anzeige des 7 degmentdisplay 0-99

    // CRC
    uint16_t crc = 0;
    for (int i = 2; i < 12; i++) {
        crc += tx_frame[i];
    }
    uint16_t c_out = crc ^ 0xFFFF;
    tx_frame[12] = c_out & 0xFF;
    tx_frame[13] = (c_out >> 8) & 0xFF;
    // Daten senden
    uart_write_bytes(UART_NUM, (const char*)tx_frame, 14);
    uart_wait_tx_done(UART_NUM,portMAX_DELAY);
    uart_flush(UART_NUM); 
}

int16_t readUART() {
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set RTS pin to enable receiving
    uart_set_rts(UART_NUM, 1);
// Re-enable UART interrupts
    uart_enable_rx_intr(UART_NUM);
    // Prüfe, ob genügend Bytes im UART-Puffer verfügbar sind
    // Lese die ersten 6 Bytes in den Puffer
    uint8_t uart_buf[8];
    
    //uart_flush(UART_NUM); 
    int len = uart_read_bytes(UART_NUM, uart_buf, sizeof(uart_buf), portMAX_DELAY);
    
    
    //vTaskDelay(pdMS_TO_TICKS(5));
    for (size_t i = 0; i < 10; i++)
    {
    if (len < 8 || uart_buf[4] != 0x65) // Mindestens 8 Bytes erforderlich: Header (2 Bytes) + Frame-Typ (1 Byte) + Throttle (1 Byte) + Brake (1 Byte)
    { 
    uart_flush(UART_NUM);     
      //  Serial.printf("Frame1: %d |Frame2: %d |Frame3: %d | Frame4: %d | Frame5: %d | Frame6: %d | Frame7: %d | Frame8: %d \n",
      //            uart_buf[1],uart_buf[2],uart_buf[3],uart_buf[4],uart_buf[5],uart_buf[6],uart_buf[7],uart_buf[8] );
    len = uart_read_bytes(UART_NUM, uart_buf, sizeof(uart_buf), portMAX_DELAY);
    
    }
    }
    // Prüfe den Frame-Typ (z. B. 0x65 für Throttle/Brake-Daten)
    if (uart_buf[4] == 0x65) 
    { // Frame-Typ ist korrekt
        uint16_t throttle_value = min(max(uart_buf[7]-43,0)*100/153,99); // Skalierung auf Spannung
        uint16_t brake_value = uart_buf[6] ;    // Skalierung auf Spannung

        // Berechne Rückgabewert basierend auf Throttle-Wert
      //  Serial.println(throttle_value);
        uart_flush(UART_NUM); 
        return throttle_value; 

    }
    return 100;
}
