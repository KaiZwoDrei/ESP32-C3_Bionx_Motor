#include "07_OTA.h"
#include <mongoose/mongoose_glue.h>

#if USE_ARDUINO_OTA
#include <ArduinoOTA.h>

void setupOTA(const char* hostname) {
    ArduinoOTA.setHostname(hostname);

    ArduinoOTA.onStart([]() {
        Serial.println("OTA Start");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("Arduino OTA ready");
}

void handleOTA() {
    ArduinoOTA.handle();
}
void otaTask(void *parameter) {
    while (1) {
        handleOTA();                  // Your OTA handler (ArduinoOTA.handle() or ElegantOTA's .handleClient())
        vTaskDelay(pdMS_TO_TICKS(200)); // Call every 200 ms
    }
}
#else // Use MongooseOTA

#endif
/*
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>

WebServer otaServer(80);

void setupOTA(const char* hostname) {
    // WiFi must already be connected!
    ElegantOTA.begin(&otaServer);  // Start ElegantOTA
    otaServer.begin();
    Serial.println("ElegantOTA ready at /update");
}

void handleOTA() {
    otaServer.handleClient();
}

#endif
*/
void mongooseTask(void *pvParameters) {
    while (1) {   
    mongoose_poll();
      vTaskDelay(pdMS_TO_TICKS(10)); // Call every 30 msfor(;;) {
    }
  }  

