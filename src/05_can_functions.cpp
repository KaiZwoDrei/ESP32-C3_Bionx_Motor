#include "05_can_functions.h"
#include <esp32_can.h>
#include "freertos/semphr.h"

extern SemaphoreHandle_t canMutex;

void setupCAN() {
    CAN0.setCANPins(GPIO_NUM_0, GPIO_NUM_1); // für supermicro
//    CAN0.setCANPins(GPIO_NUM_19, GPIO_NUM_13);  für luatos esp32c3 

    CAN0.begin(125000);
}
bool readBionxRegister(int canId, uint16_t address, uint16_t *result) {
    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 2;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;

    CAN0.sendFrame(txFrame);

    TickType_t startTime = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(20);

    while ((xTaskGetTickCount() - startTime) < timeout) {
        CAN0.watchFor(0x08);
        if (CAN0.read(txFrame) && 
            txFrame.data.uint8[0] == 0 && 
            txFrame.data.uint8[1] == address) {
            *result = (txFrame.data.uint8[2] << 8) | txFrame.data.uint8[3];
            return true;
        }
        taskYIELD();  // Kritisch für RTOS-Stabilität
    }
    return false;
}

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
    return CAN0.sendFrame(txFrame);
}

/*bool readBionxRegister(int canId, uint16_t address, uint16_t *result) {
    if(xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdFALSE) return false;

    CAN_FRAME txFrame;
    memset(&txFrame, 0, sizeof(txFrame));
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 2;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;

    CAN0.sendFrame(txFrame);

    // Antwort-ID gemäß BionX-Protokoll (Request-ID + 0x08)
    const uint32_t responseId = canId + 0x08;
    const TickType_t startTime = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(20);

    bool success = false;
    while ((xTaskGetTickCount() - startTime) < timeout) {
        CAN_FRAME rxFrame;
        if(CAN0.read(rxFrame)) {
            if(rxFrame.id == responseId && 
               rxFrame.data.uint8[1] == address) {
                *result = (rxFrame.data.uint8[2] << 8) | rxFrame.data.uint8[3];
                success = true;
                break;
            }
        }
        taskYIELD();  // Kritisch für RTOS-Stabilität [2][3]
    }

    xSemaphoreGive(canMutex);
    return success;
}

bool writeBionxRegister(int canId, uint16_t address, uint16_t value) {
    if(xSemaphoreTake(canMutex, pdMS_TO_TICKS(50)) == pdFALSE) return false;

    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 4;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;
    txFrame.data.uint8[2] = (value >> 8) & 0xFF;
    txFrame.data.uint8[3] = value & 0xFF;
    bool ret = CAN0.sendFrame(txFrame);
    xSemaphoreGive(canMutex);
    return ret;
}*/

