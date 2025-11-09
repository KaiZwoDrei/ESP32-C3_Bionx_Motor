#include "05_can_functions.h"
#include <esp32_can.h>
#include "freertos/semphr.h"

#include "10_bionxregg.h"
SemaphoreHandle_t canMutex;  
static uint32_t mutexBlockCount = 0;

void setupCAN() {
    CAN0.setCANPins(GPIO_NUM_0, GPIO_NUM_1); // für supermicro
    CAN0.begin(125000);
    //CAN0.setRXFilter(0, 0x08, 0x7FF, false);  // Filter auf ID 0x08
    //CAN0.watchFor();
    CAN0.watchFor(0x08,0x7FF);
    canMutex = xSemaphoreCreateMutex();
}
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
        CAN_FRAME dummyFrame;
        while (CAN0.read(dummyFrame)) {
            // Alte Frames verwerfen
        }
        TickType_t startTime = xTaskGetTickCount();
        const TickType_t timeout = pdMS_TO_TICKS(4);

        while ((xTaskGetTickCount() - startTime) < timeout) {
            if (CAN0.read(txFrame) && txFrame.data.uint8[0] == 0 && txFrame.data.uint8[1] == address) {
                *result = (txFrame.data.uint8[2] << 8) | txFrame.data.uint8[3];
                readSuccess = true;
                break;
            }
            taskYIELD();
        }
        xSemaphoreGive(canMutex);
    } else {
        mutexBlockCount++;
        if (mutexBlockCount >= 10) {
            Serial.println("CAN 100 mal blockiert!");
            mutexBlockCount = 0;
        }
    }
    if (!readSuccess)
        Serial.printf("Fail to read address 0x%02X - %s\n", address, getMotorText(address));
    return readSuccess;
}

bool writeBionxRegister(int canId, uint16_t address, uint16_t value) {
    //vTaskDelay(pdMS_TO_TICKS(2)); // Kurze Pause nach dem Senden

    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 4;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;
    txFrame.data.uint8[2] = (value >> 8) & 0xFF;
    txFrame.data.uint8[3] = value & 0xFF;
    bool result = CAN0.sendFrame(txFrame);
    bool result2 = CAN0.sendFrame(txFrame);
    return result||result2;
   // vTaskDelay(pdMS_TO_TICKS(2)); 

}
