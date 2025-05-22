//#include "06_bluetooth_functions.h"
#include "07_OTA.h"
#include "02_bionx_motor.h"
#include "05_can_functions.h"
#include "08_uartdisplay.h"
#include "09_project_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <WiFi.h>
#include <WiFiManager.h> 
// Mongoose includes with proper C linkage

#include "mongoose/mongoose_glue.h"
extern int16_t assistLevel;
extern uint16_t motorSpeed;
extern uint16_t motorPower;


// Globale Task Handles
TaskHandle_t torqueTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
SemaphoreHandle_t uartSemaphore;
EventGroupHandle_t taskEventGroup;
static struct mg_mgr mgr;
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp) __attribute__((weak));
extern "C" int lwip_hook_ip6_input(struct pbuf *p, struct netif *inp) {
  if (ip6_addr_isany_val(inp->ip6_addr[0].u_addr.ip6)) {
    pbuf_free(p);
    return 1;
  }
  return 0;
}

void setup() {
    Serial.begin(115200);
    
    // Initialize WiFiManager
    WiFiManager wm;
    wm.setHostname("BionX-Controller"); // Set device hostname
    
    // Automatically connect or start configuration portal
    if(!wm.autoConnect("BionX_SetupAP")) {
        Serial.println("Failed to connect and hit timeout");
        ESP.restart();
   }

    // WiFi connected at this point
    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
      // Initialize Mongoose
    mongoose_init();
    mg_log_set(MG_LL_ERROR); 


    Serial.println("Mongoose dashboard started on port 80");

  //  setupOTA("bionx-controller");
    setupCAN();
    setupBionx();
 //   setupBLE();
    uartSemaphore = xSemaphoreCreateMutex();
    taskEventGroup = xEventGroupCreate();
    setupDisplay();

      // Create Mongoose FreeRTOS task
    xTaskCreatePinnedToCore(
    mongooseTask,
    "Mongoose",
    TASK_STACK_SIZE*4,        // Increased stack size
    NULL,
    4,           // Higher priority than other tasks
    NULL,
    1            // Core 1
  );
    xTaskCreatePinnedToCore(
        otaTask,         // Task function (defined in ota.cpp)
        "OTA",           // Task name
        TASK_STACK_SIZE*2,            // Stack size
        NULL,            // Parameter
        1,               // Priority (lowest is fine)
        NULL,            // Task handle (optional)
        1                // Core (choose 0 or 1, as fits your system)
    );
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
