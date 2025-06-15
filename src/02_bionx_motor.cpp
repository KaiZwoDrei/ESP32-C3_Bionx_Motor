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


extern SemaphoreHandle_t uartSemaphore;
extern EventGroupHandle_t taskEventGroup;


// Puffer für Drehmomentdaten
TorqueData torqueBuffer[CURVE_BUFFER_SIZE] = {{0, 0, false}};
int bufferIndex = 0;

// FreeRTOS Queues für Datenübertragung zwischen Tasks
QueueHandle_t torqueQueue;
QueueHandle_t speedQueue;
QueueHandle_t statusQueue;

// Funktion zur Initialisierung des Bionx-Systems
void setupBionx() {
    uint16_t softwareVersion;
    
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
    //writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_ACCESSORY_ENABLED , 0x01); // Aktiviere Spannung Licht
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
uint8_t calculateBatteryLevel() {
    if (motorVoltage >= BATTERY_VOLTAGE_MAX) return 100;
    if (motorVoltage <= BATTERY_VOLTAGE_MIN) return 0;

    uint8_t soc = 0;
    if (motorVoltage > BATTERY_VOLTAGE_NOM) {
        // Upper range (non-linear, "exponential" approximation)
        int32_t diff = (int32_t)motorVoltage - (int32_t)BATTERY_VOLTAGE_NOM;
        uint32_t temp = ((int64_t)diff * BATTERY_INV_UPPER_RANGE) >> 24;
        soc = 85 + temp;
    } else {
        // Lower range (linear)
        int32_t diff = (int32_t)motorVoltage - (int32_t)BATTERY_VOLTAGE_MIN;
        uint32_t temp = ((int64_t)diff * BATTERY_INV_LOWER_RANGE) >> 24;
        soc = temp;
    }

    // Clamp to [0, 100]
    if (soc > 100) soc = 100;
    return soc;
}


// Task für Drehmomentmessung (500Hz)
void torqueTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 10ms = 100Hz
    int writeCounter = 0;
    while(1) {
        if (rekupLevel > 0) {
            writeCounter++;
            if (writeCounter >= 10) {
                writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, -rekupLevel*100/150);
                writeCounter = 0;
            }
        } else {
            writeCounter = 0;
        uint16_t rawTorque;
        if(readBionxRegister(BXID_MOTOR, REG_MOTOR_TORQUE_GAUGE_VALUE, &rawTorque)) {
            xQueueOverwrite(torqueQueue, &rawTorque);
            motorlevel = processTorque(rawTorque);
        }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
    const TickType_t xFrequency = pdMS_TO_TICKS(5000); // 5000 ms = 5 Sekunden

    while(1) {
        writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_SHUTDOWN, 0x00);
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
        //{
        updateDisplay(assistLevel,rekupLevel, motorSpeed, motorStatus, calculateBatteryLevel(), motorlevel);
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
        writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_POWER_VOLTAGE_ENABLE , 0x01); // Aktiviere Spannung
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
        Serial.printf("Speed: %d | Level: %d | Rekup %d | Power: %dW | Torque: %d | Motorlevel: %d | Motor Status: %d\n",
                      motorSpeed, assistLevel, rekupLevel ,currentMotorPower, rawTorque, motorlevel, motorStatus);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
uint16_t processTorque(uint16_t rawTorque) {
    static int32_t filtered_torque = 0;  // Q16-Format (16:16)
    const TickType_t currentTime = xTaskGetTickCount();
    static TickType_t lastWriteTime = 0;
    static int32_t lastTorque = 0;
    static uint16_t lastWrittenLevel = 0;
    const TickType_t WRITE_INTERVAL = pdMS_TO_TICKS(50); // Minimum time
    const TickType_t DECAY_INTERVAL = pdMS_TO_TICKS(100); // Minimum time
    // 1. Fixed-Point-Konversion (12:4 Format)
    int32_t torque = (int32_t)rawTorque * TORQUE_FACTOR >> 16;  // 12 Bit ADC → 16:16
    
    if(torque >= TORQUE_THRESHOLD) {
     
    // 2. IIR-Filter: y[n] = 0.25*x[n] + 0.75*y[n-1]
    filtered_torque = ((torque * IIR_ALPHA) >> 16) + 
                     ((filtered_torque * IIR_1MALPHA) >> 16);
    

    // 4. Skaliere auf 0-100% (8:24 → 0-100)
    int32_t gauge = (filtered_torque * assistLevel)*INV_100 >> 24; // 8:24 Festkomma-Format
    gauge = (gauge > 100) ? 100 : gauge;
    
    // 5. Motor-Update mit Assist
    level = gauge  ;
    if ((currentTime - lastWriteTime) >= WRITE_INTERVAL) {
        writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, level);
        lastWrittenLevel = level;
        lastWriteTime = currentTime;
    }

    
    }   
    if(torque < TORQUE_THRESHOLD)
    {
    const int32_t decay_shift = (filtered_torque > 30) ? 1 : 2; // 1/16 oder 1/4 pro Sekunde
    // Füge einen minimalen Decay von 1 hinzu, wenn filtered_torque > 0
    
    if (((currentTime - lastWriteTime) > DECAY_INTERVAL)  && lastWrittenLevel != 0) {
        if (filtered_torque > 0) {
        int32_t decay = (filtered_torque >> decay_shift);
        decay = (decay == 0) ? 1 : decay; // Mindestens 1
        filtered_torque -= decay;
        }
        int32_t gauge = (filtered_torque * assistLevel)*INV_100 >> 24; // 8:24 Festkomma-Format
        level=gauge;
        writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, level);
        lastWrittenLevel = level;
        lastWriteTime = currentTime;
    }
    }
    lastTorque = torque;
    return level;
}
