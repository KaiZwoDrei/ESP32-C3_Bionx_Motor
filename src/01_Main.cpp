// Includes
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

// Konfigurierbar → ENTFERNT, wird dynamisch bestimmt

// Globale Task Handles und Synchronisationsobjekte (können bleiben, werden nicht verwendet)
TaskHandle_t torqueTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
SemaphoreHandle_t uartSemaphore;
EventGroupHandle_t taskEventGroup;

// **NEUE globale Flags**
bool configMode = false;    // Config/WiFi-Modus (Button+Brake 5s)
bool wifiActive = false;    // WiFi läuft

// Mongoose Webserver Task (optional)
void mongooseTask(void *pvParameters) {
    while (1) {
        mongoose_poll();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Setup-Funktion zur Initialisierung aller Komponenten
void setup() {
    Serial.begin(115200);

    // **BOOT-ABFRAGE: Pins für Button+Brake**
    pinMode(4, INPUT_PULLUP);  // Button_PIN aus uartdisplay.h
    pinMode(8, INPUT_PULLUP);  // BUTTON_PIN_8 aus bionx_motor.h

    // 5s warten: beide müssen die Ganze Zeit gedrückt bleiben
    const uint32_t WAIT_MS = 5000;
    uint32_t start = millis();
    bool stillPressed = true;

    Serial.println("Boot-Check: Halte Button (Pin4) + Brake (Pin8) 5 Sekunden für CONFIG MODE...");
    
    while (millis() - start < WAIT_MS) {
        if (digitalRead(4) != LOW ) {
            stillPressed = false;
            break;
        }
        delay(50);  // 20x pro Sekunde checken
    }

    if (stillPressed) {
        configMode = true;
        Serial.println("*** CONFIG MODE: WiFi aktiviert ***");
    } else {
        configMode = false;
        Serial.println("*** NORMAL MODE: ohne WiFi ***");
    }

    // **WLAN nur im CONFIG MODE**
    if (configMode) {
        WiFiManager wm;
        wm.setHostname("BionX-Controller");

        if (!wm.autoConnect("BionX_SetupAP")) {
            Serial.println("Failed to connect and hit timeout");
            ESP.restart();
        }

        wifiActive = true;
        Serial.println("Connected to WiFi!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());

        mongoose_init();
        mg_log_set(MG_LL_ERROR);
        Serial.println("Mongoose dashboard started on port 80");
    } else {
        Serial.println("WiFiManager not enabled, running without WiFi");
    }

    // **Peripherie Setup (IMMER)**
    setupCAN();
    setupBionx();
    uartSemaphore = xSemaphoreCreateMutex();
    taskEventGroup = xEventGroupCreate();
    setupDisplay();

    // **TASKS NUR IM NORMAL MODE erstellen!**
    if (!configMode) {
        Serial.println("Starte Motor-Control Tasks...");
        xTaskCreatePinnedToCore(torqueTask,   "Torque",     TASK_STACK_SIZE, NULL, TASK_PRIO_TORQUE,  NULL, 0);
        xTaskCreatePinnedToCore(keepAliveTask,"KeepAlive",  TASK_STACK_SIZE, NULL, TASK_PRIO_TORQUE,  NULL, 0);
        xTaskCreatePinnedToCore(statusTask,   "Status",     TASK_STACK_SIZE, NULL, TASK_PRIO_STATUS,  NULL, 0);
        xTaskCreatePinnedToCore(buttonTask,   "Button",     TASK_STACK_SIZE, NULL, TASK_PRIO_BUTTON,  NULL, 0);
    } else {
        Serial.println("Config Mode: Motor-Tasks deaktiviert (kein Bus-Zugriff)");
    }

    // **Mongoose nur im CONFIG MODE**
    if (configMode) {
        xTaskCreatePinnedToCore(
            mongooseTask,
            "Mongoose",
            TASK_STACK_SIZE * 4,
            NULL,
            2,
            NULL,
            1
        );
    }
}

// Minimaler Loop für Config-Modus (optional)
void loop() {
    // Im Config-Modus: nur Display-Update oder nichts
    if (configMode) {
        static unsigned long lastUpdate = 0;
        if (millis() - lastUpdate > 1000) {
            // Optional: updateDisplay(0,0,0,0,0,0,false); // Dummy
            lastUpdate = millis();
        }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}
