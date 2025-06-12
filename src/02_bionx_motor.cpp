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
    // Konstanten für 13S LG MJ1 Akku
    const uint32_t VOLTAGE_MIN = 39000;    // 39.0V in mV
    const uint32_t VOLTAGE_MAX = 54600;    // 54.6V in mV
    const uint32_t VOLTAGE_NOMINAL = 48100; // 48.1V in mV
    
    if (motorVoltage >= VOLTAGE_MAX) return 100;
    if (motorVoltage <= VOLTAGE_MIN) return 0;
    
    // Nicht-lineare Berechnung mit Integer-Arithmetik
    if (motorVoltage > VOLTAGE_NOMINAL) {
        // Oberer Bereich (exponentiell)
        uint32_t temp = ((motorVoltage - VOLTAGE_NOMINAL) * 15LL * 65536) / (VOLTAGE_MAX - VOLTAGE_NOMINAL);
        return 85 + (temp >> 16);
    } else {
        // Unterer Bereich (linear)
        uint32_t temp = ((motorVoltage - VOLTAGE_MIN) * 85LL * 65536) / (VOLTAGE_NOMINAL - VOLTAGE_MIN);
        return (temp >> 16);
    }
}

// Task für Drehmomentmessung (500Hz)
void torqueTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 2ms = 500Hz
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
    const TickType_t xFrequency = pdMS_TO_TICKS(30); // 10ms = 100Hz
    
    while(1) {
        uint16_t speed;
        assistLevel = handleButton(assistLevel, light);
        //if (xSemaphoreTake(uartSemaphore, pdMS_TO_TICKS(5)) == pdTRUE) {

       //xSemaphoreGive(uartSemaphore);
        //}
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Task für Geschwindigkeit & Display (20Hz)
void speedTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(300); // 50ms = 20Hz
    
    while(1) {
        // Lese Motordaten
        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_SPEED, &motorSpeed);
        //xQueueOverwrite(speedQueue, &motorSpeed);
        rekupLevel = readUART();

        //if(xQueueReceive(speedQueue, &speed, 0))
        //{
        updateDisplay(assistLevel,rekupLevel, motorSpeed, motorStatus, calculateBatteryLevel(), motorlevel);
        //}
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    }



// Task für Statusaktualisierung und BLE-Kommunikation (10Hz)
void statusTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 100ms = 10Hz
    
    while(1) {

        readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_METER, &motorPower);
        
        // Lese Motorspannung
        uint16_t voltageHigh, voltageLow;
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
        //Serial.printf("Speed: %d | Level: %d | Rekup %d | Power: %dW | Torque: %d | Motorlevel: %d \n",
        //              motorSpeed, assistLevel, rekupLevel ,currentMotorPower, rawTorque, motorlevel);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Funktion zur Verarbeitung des Drehmoments
uint16_t processTorque(uint16_t rawTorque) {
    TickType_t currentTime = xTaskGetTickCount();
    static int32_t lastTorque = 0;
    static TickType_t lastPeakTime = 0;
    static TickType_t lastWriteTime = 0;
    static uint16_t lastWrittenLevel = 0;
    const TickType_t WRITE_INTERVAL = pdMS_TO_TICKS(50); // Minimum time between writes
    const TickType_t PEAK_INTERVAL_TICKS = pdMS_TO_TICKS(PEAK_INTERVAL);
    
    // Umrechnung in Festkomma
    int32_t torque = ((int32_t)rawTorque * TORQUE_FACTOR) >> 16;
    
    bool isPeak = (torque > TORQUE_THRESHOLD && lastTorque <= TORQUE_THRESHOLD);
    
    if (torque > TORQUE_THRESHOLD) {
        // Aktualisiere Drehmomentpuffer
        torqueBuffer[bufferIndex].value = torque;
        torqueBuffer[bufferIndex].timestamp = currentTime;
        torqueBuffer[bufferIndex].isPeak = isPeak;
        
        if (isPeak) {
            lastPeakTime = currentTime;
        }
        
        bufferIndex = (bufferIndex + 1) % CURVE_BUFFER_SIZE;
        
        // Berechne Durchschnittswerte
        int32_t peakSum = 0;
        int32_t curveSum = 0;
        int peakCount = 0;
        int validSamples = 0;
        
        for (int i = 0; i < CURVE_BUFFER_SIZE; i++) {
            if ((currentTime - torqueBuffer[i].timestamp) < (PEAK_INTERVAL_TICKS + pdMS_TO_TICKS(2))) {
                if (torqueBuffer[i].isPeak) {
                    peakSum += torqueBuffer[i].value;
                    peakCount++;
                }
                curveSum += torqueBuffer[i].value;
                validSamples++;
            }
        }
        
        if (validSamples > 0) {
            int32_t peakAvg = peakCount > 0 ? ((peakSum * 65536) / peakCount) >> 16 : 0;
            int32_t curveAvg = ((curveSum * 65536) / validSamples) >> 16;
            
            // Gewichtete Summe mit Ganzzahlen
            int32_t combinedTorque = ((peakAvg * PEAK_WEIGHT) >> 16) +
                                     ((curveAvg * CURVE_WEIGHT) >> 16);
            
            gauge = ((((gauge*90) + (combinedTorque*10))*65536)/100) >> 16;
            
            if (motorSpeed >= 0 && motorSpeed < 50) {
                level = ((gauge * assistLevel*65536)/ 100) >> 16;
                if (level > 100) level = 100;
                if ((level > 0) && ((currentTime - lastWriteTime) >= WRITE_INTERVAL)) {
                    writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, level);
                    lastWrittenLevel = level;
                    lastWriteTime = currentTime;
                //    Serial.println("sent");
                }
            }
        }
    }
    
    if ((currentTime - lastPeakTime) > PEAK_INTERVAL_TICKS) {
        gauge = (gauge * GAUGE_DECAY) >> 16;
        if (gauge < 100 && lastWrittenLevel != 0) { // 1.0 in Festkomma
            writeBionxRegister(BXID_MOTOR, BXR_MOTOR_LEVEL, 0);
            lastWrittenLevel = 0;
            level = lastWrittenLevel;
            lastWriteTime = currentTime;
        }
    }
    
    lastTorque = torque;
    return level;
}

