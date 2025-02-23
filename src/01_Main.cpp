#include "06_bluetooth_functions.h"
#include "02_bionx_motor.h"
#include "05_can_functions.h"
#include "08_uartdisplay.h"
#include "09_project_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// Globale Task Handles
TaskHandle_t torqueTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
SemaphoreHandle_t uartSemaphore;
EventGroupHandle_t taskEventGroup;


void setup() {
    Serial.begin(115200);
    setupCAN();
    setupBionx();
    setupBLE();
    uartSemaphore = xSemaphoreCreateMutex();
    taskEventGroup = xEventGroupCreate();
    setupDisplay();

    xTaskCreatePinnedToCore(
        torqueTask,
        "Torque",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIO_TORQUE,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        statusTask,
        "Status",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIO_STATUS,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        buttonTask,
        "Button",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIO_BUTTON,
        NULL,
        0
    );
     xTaskCreatePinnedToCore(
        speedTask,
        "Display/speed",
        TASK_STACK_SIZE,
        NULL,
        TASK_PRIO_DISPLAY,
        NULL,
        0
    );
    
}
void loop() {

    
}
