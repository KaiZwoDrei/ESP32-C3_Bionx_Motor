#include <esp32_can.h>

//#include "can_functions.h"

void setupCAN() {
    CAN0.setCANPins(GPIO_NUM_19, GPIO_NUM_13);
    CAN0.begin(125000);
}
/*OLD VERSION 
bool readBionxRegister(int canId, uint16_t address, uint16_t &result) {
    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 2;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;

    CAN0.sendFrame(txFrame);

    for (int attempt = 0; attempt < 100; attempt++) {
        CAN0.watchFor(0x08);
        delay(5);
        if (CAN0.read(txFrame) && txFrame.data.uint8[0] == 0 && txFrame.data.uint8[1] == address) {
            result = (txFrame.data.uint8[2] << 8) | txFrame.data.uint8[3];
            return true;
        }
    }
    return false;
}*/
bool readBionxRegister(int canId, uint16_t address, uint16_t &result) {
    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 2;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;

    // Send the CAN frame
    CAN0.sendFrame(txFrame);

    // Wait for a response with timeout
    TickType_t startTime = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(20); // 500ms timeout

    while ((xTaskGetTickCount() - startTime) < timeout) {
        CAN0.watchFor(0x08); // Watch for specific CAN ID (response)

        if (CAN0.read(txFrame)) {
            // Check if response matches the request
            if (txFrame.data.uint8[0] == 0 && txFrame.data.uint8[1] == address) {
                result = (txFrame.data.uint8[2] << 8) | txFrame.data.uint8[3];
                return true; // Successful read
            }
        }

      //  vTaskDelay(pdMS_TO_TICKS(1)); // Yield to other tasks for 1ms
    }

    return false; // Timeout occurred
}

bool writeBionxRegister(int canId, uint16_t address, uint16_t value) {
    //vTaskDelay(pdMS_TO_TICKS(2));
    CAN_FRAME txFrame;
    txFrame.rtr = 0;
    txFrame.id = canId;
    txFrame.extended = false;
    txFrame.length = 4;
    txFrame.data.uint8[0] = 0;
    txFrame.data.uint8[1] = address;
    txFrame.data.uint8[2] = (value >> 8) & 0xff;
    txFrame.data.uint8[3] = value & 0xff;
    return CAN0.sendFrame(txFrame);
}
