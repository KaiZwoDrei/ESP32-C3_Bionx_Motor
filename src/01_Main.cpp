// Includes
//#include "06_bluetooth_functions.h"
#include "02_bionx_motor.h"
#include "05_can_functions.h"
#include "08_uartdisplay.h"
#include "09_project_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <WiFi.h>
#include <WiFiManager.h>
#include "mongoose/mongoose_glue.h"

// Konfigurierbar
#define WIFI_ENABLE 0

// Globale Task Handles und Synchronisationsobjekte
TaskHandle_t torqueTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
SemaphoreHandle_t uartSemaphore;
EventGroupHandle_t taskEventGroup;

// Mongoose Webserver Task (optional)
void mongooseTask(void *pvParameters) {
    while (1) {
        mongoose_poll();
        vTaskDelay(pdMS_TO_TICKS(30)); // Poll alle 30ms
    }
}

// Setup-Funktion zur Initialisierung aller Komponenten
void setup() {
    Serial.begin(115200);

    // WLAN initialisieren, falls aktiviert
    if(WIFI_ENABLE) {
        WiFiManager wm;
        wm.setHostname("BionX-Controller");

        if(!wm.autoConnect("BionX_SetupAP")) {
            Serial.println("Failed to connect and hit timeout");
            ESP.restart();
        }

        Serial.println("Connected to WiFi!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        mongoose_init();
        mg_log_set(MG_LL_ERROR);
        Serial.println("Mongoose dashboard started on port 80");
    } else {
        Serial.println("WiFiManager not enabled, running without WiFi");
    }

    // Peripherie Setup
    setupCAN();
    setupBionx();
    uartSemaphore = xSemaphoreCreateMutex();
    taskEventGroup = xEventGroupCreate();
    setupDisplay();

    // Tasks erstellen
    xTaskCreatePinnedToCore(torqueTask, "Torque", TASK_STACK_SIZE, NULL, TASK_PRIO_TORQUE, NULL, 0);
    xTaskCreatePinnedToCore(keepAliveTask, "KeepAlive", TASK_STACK_SIZE, NULL, TASK_PRIO_TORQUE, NULL, 0);
    xTaskCreatePinnedToCore(statusTask, "Status", TASK_STACK_SIZE, NULL, TASK_PRIO_STATUS, NULL, 0);
    xTaskCreatePinnedToCore(buttonTask, "Button", TASK_STACK_SIZE, NULL, TASK_PRIO_BUTTON, NULL, 0);
    xTaskCreatePinnedToCore(speedTask, "Display/speed", TASK_STACK_SIZE, NULL, TASK_PRIO_DISPLAY, NULL, 0);

    if(WIFI_ENABLE) {
        xTaskCreatePinnedToCore(
            mongooseTask,
            "Mongoose",
            TASK_STACK_SIZE * 4,
            NULL,
            4,
            NULL,
            1
        );
    }
}

// Empty loop for FreeRTOS cooperative multitasking
void loop() {
}
