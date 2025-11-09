#include "05_can_functions.h"
//#include "06_bluetooth_functions.h"
#include "10_bionxregg.h"
#include "02_bionx_motor.h"
#include "08_uartdisplay.h"
#include "freertos/timers.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "mongoose/mongoose_glue.h"

// Struktur für Button-Zustand
ButtonState button;

// Globale Variablen für Motorzustand
int16_t assistLevel = 100;
uint16_t motorSpeed = 0;
uint16_t motorPower = 0;
uint16_t motorTemperature = 0;
uint16_t motorStatus = 0;
uint32_t motorVoltage = 0;
int32_t gauge = 0;
int16_t rekupLevel = 0;
int32_t level = 0;
int16_t motorlevel = 0;
bool light = false;
uint16_t rawTorque;
uint16_t maxMotorCurrent = 0;
uint16_t currentMotorPower = 0;


SemaphoreHandle_t canMutex= NULL;
extern EventGroupHandle_t taskEventGroup;



// FreeRTOS Queues für Datenübertragung zwischen Tasks
QueueHandle_t torqueQueue;
QueueHandle_t speedQueue;
QueueHandle_t statusQueue;

// Funktion zur Initialisierung des Bionx-Systems
void setupBionx() {
    uint16_t softwareVersion;
   // canMutex = xSemaphoreCreateMutex();

    // Warte auf erfolgreiche Verbindung zum Motor
    for(int i=0; i<=10;i++) 
    {
        if(!readBionxRegister(BXID_MOTOR, BXR_MOTOR_SWVERS, &softwareVersion))
         {
        Serial.println("Cannot read software version from motor!");
        
        }
        delay(200);
    }
       
    //writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_CONTROL_VOLTAGE_ENABLE , 0x01); // Aktiviere Spannung  
    writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_ACCESSORY_ENABLED , 0x01); // Aktiviere Spannung Licht
    //writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_BATTINT_VOLTAGE_ENABLE, 0x01); // vBattInt
    
    
    Serial.print("Found motor with software version: ");
    Serial.println(softwareVersion);
    
    // Lese maximalen Motorstrom
    uint16_t currentHi, currentLo;
    readBionxRegister(BXID_MOTOR, REG_MOTOR_CONFIG_MAX_DISCHARGE_HI, &currentHi);
    readBionxRegister(BXID_MOTOR, REG_MOTOR_CONFIG_MAX_DISCHARGE_LO, &currentLo);
    maxMotorCurrent = (currentHi << 8) | currentLo;
    Serial.print("Maximaler Motor Strom: ");
    Serial.println(maxMotorCurrent);
    
    // Initialisiere FreeRTOS Queues
    torqueQueue = xQueueCreate(1, sizeof(uint16_t));
    speedQueue = xQueueCreate(1, sizeof(uint16_t));
    statusQueue = xQueueCreate(1, sizeof(uint32_t));
}

// Funktion zur Berechnung des Batterieladestands
uint8_t calculateBatteryLevel(uint32_t Voltage) {
    if (Voltage >= BATTERY_VOLTAGE_MAX) return 100;
    if (Voltage <= BATTERY_VOLTAGE_MIN) return 0;

    uint8_t soc = 0;
    /*if (Voltage > BATTERY_VOLTAGE_NOM) {
        // Upper range (non-linear, "exponential" approximation)
        int32_t diff = (int32_t)Voltage - (int32_t)BATTERY_VOLTAGE_NOM;
        uint32_t temp = ((int64_t)diff * BATTERY_INV_UPPER_RANGE) >> 24;
        soc = 85 + temp;
    } else {
        // Lower range (linear)
        
        int32_t diff = (int32_t)Voltage - (int32_t)BATTERY_VOLTAGE_MIN;
        uint32_t temp = ((int64_t)diff * BATTERY_INV_LOWER_RANGE) >> 24;
        soc = temp;
    }*/
    soc=map(Voltage, BATTERY_VOLTAGE_MIN, BATTERY_VOLTAGE_MAX, 0, 100);
    // Clamp to [0, 100]
    if (soc > 100) soc = 100;
    return soc;
}


// Task für Drehmomentmessung (500Hz)
void torqueTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 10ms = 100Hz
    static TickType_t lastWriteTime = 0;
    static TickType_t lastAliveTime = 0;
    //static int16_t motorlevel = 0;
    static int16_t lastwrittenLevel = 0;
    while(1) {                        
        uint16_t rawTorque;
        if(readBionxRegister(BXID_MOTOR, REG_MOTOR_TORQUE_GAUGE_VALUE, &rawTorque)) {
            xQueueOverwrite(torqueQueue, &rawTorque);
                motorlevel = processTorque(rawTorque);
        }
    
            // Enforce 50ms write interval
        //TickType_t currentTime = xTaskGetTickCount();
        if (xTaskGetTickCount()-lastAliveTime>=pdMS_TO_TICKS(400))
        {writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_POWER_VOLTAGE_ENABLE , 0x01); // Aktiviere Spannung),
            lastAliveTime = xTaskGetTickCount();
        }
        else if((xTaskGetTickCount() - lastWriteTime) >= pdMS_TO_TICKS(20)) {
            if(motorSpeed >= MINSPEED && motorlevel > 0) {
                writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, motorlevel);
                lastWriteTime = xTaskGetTickCount();
                //Serial.printf("Motorlevel: %d\n", motorlevel);
                lastwrittenLevel = motorlevel;
            }
            else if (motorlevel == 0 && lastwrittenLevel != 0) {
                writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, 0);
                lastWriteTime = xTaskGetTickCount();
                lastwrittenLevel = motorlevel;
                //Serial.printf("Motorlevel: %d\n", motorlevel);
            }
            if (rekupLevel > 0) {
                writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, -rekupLevel*100/150);
                lastWriteTime = xTaskGetTickCount();
                lastwrittenLevel = motorlevel;
            }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
}
}
}
// Task für Button-Handling (100Hz)
void buttonTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(30); // 30ms = 30Hz
    
    while(1) {
        uint16_t speed;
        assistLevel = handleButton(assistLevel, light);
        //if (xSemaphoreTake(uartSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {

       //xSemaphoreGive(uartSemaphore);
        //}
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
void keepAliveTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 5000 ms = 5 Sekunden

    while(1) {
        //writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_SHUTDOWN, 0x00);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
// Task für Geschwindigkeit & Display (20Hz)
void speedTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(150); // 150ms = 6Hz
    
    while(1) {
        // Lese Motordaten
        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_SPEED, &motorSpeed);
        //xQueueOverwrite(speedQueue, &motorSpeed);
        //rekupLevel = readUART(); //nur bei angeschlossenem throttle oder brake 

        //if(xQueueReceive(speedQueue, &speed, 0))
        int16_t batterylevel= calculateBatteryLevel(motorVoltage);
        //{
        updateDisplay(assistLevel,rekupLevel, motorSpeed, motorStatus, batterylevel, motorlevel);
        //}
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    }



// Task für Statusaktualisierung und BLE-Kommunikation (2Hz)
void statusTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 500ms = 2Hz
    
    while(1) {
        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_METER, &motorPower);
        
        // Lese Motorspannung
        uint16_t voltageHigh, voltageLow, motorStatus;
        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_MAIN, &motorStatus);
    
        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_VOLTAGE_HI, &voltageHigh);
        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_VOLTAGE_LO, &voltageLow);
        motorVoltage = (voltageHigh << 8) | voltageLow;
        
        // Berechne aktuelle Motorleistung
        currentMotorPower = (motorVoltage/1000 * maxMotorCurrent/1000 * motorPower) / 100;
        
        // Hole aktuelles Drehmoment
        xQueueReceive(torqueQueue, &rawTorque, 0);
        glue_update_state();

        // Aktualisiere BLE-Daten
        //updateBLEData(currentMotorPower, rawTorque, motorPower, motorSpeed, calculateBatteryLevel());
        
        // Debug-Ausgabe
       // Serial.printf("Speed: %d | Level: %d | Rekup %d | Power: %dW | Torque: %d | Motorlevel: %d | Motor Status: %d\n",
        //             motorSpeed, assistLevel, rekupLevel ,currentMotorPower, rawTorque, motorlevel, motorStatus);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


uint16_t processTorque(uint16_t rawTorque) {
    #define Q16_SHIFT       16
    #define Q16_SCALE       65536
    #define ALPHA_1PCT      ((uint32_t)(0.01 * 65536))
    #define DECAY_MIN       ((uint32_t)(0.99 * 65536))
    #define DECAY_MAX       ((uint32_t)(0.70 * 65536))
    #define DECAY_RANGE     (DECAY_MIN - DECAY_MAX)
    const TickType_t DECAY_INTERVAL = pdMS_TO_TICKS(5);

    static int32_t filtered = 0;
    static TickType_t lastDecay = 0;

    int32_t torque = (int32_t)rawTorque << Q16_SHIFT;

    if(torque >= (TORQUE_THRESHOLD_HIGH << Q16_SHIFT)) {
        int64_t temp = (int64_t)torque * ALPHA_1PCT + 
                      (int64_t)filtered * (Q16_SCALE - ALPHA_1PCT);
        filtered = (int32_t)((temp + (1 << 15)) >> Q16_SHIFT);  // Runden statt Truncate[2][5]
    } 
    else {
        if((xTaskGetTickCount() - lastDecay) >= DECAY_INTERVAL) {
            // 1. Korrekte Decay-Skalierung mit 64-bit[4]
            int64_t decay = DECAY_MIN - ((DECAY_RANGE * (int64_t)filtered) >> 40);
            
            // 2. Explizite Bereichsbegrenzung[1][3]
            decay = (decay > DECAY_MIN) ? DECAY_MIN : 
                   (decay < DECAY_MAX) ? DECAY_MAX : decay;
            
            // 3. 64-bit Multiplikation mit Rundung
            int64_t temp = (int64_t)filtered * decay;
            filtered = (int32_t)((temp + (1 << 15)) >> Q16_SHIFT);
            
            lastDecay = xTaskGetTickCount();
        }
    }

    // 4. Fixed-Point Division mit 24-bit Skalierung[6]
    
    int32_t gauge = (int32_t)((filtered * assistLevel/150) >> 16);
    
    // 5. Sicherheitsbegrenzung[1][4]
    return (uint16_t)(gauge);
}
