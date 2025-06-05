#include "05_can_functions.h"
#include <esp32_can.h>

void setupCAN() {
    CAN0.setCANPins(GPIO_NUM_19, GPIO_NUM_13);
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
