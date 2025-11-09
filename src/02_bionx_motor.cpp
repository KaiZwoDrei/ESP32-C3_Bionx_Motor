#include "02_bionx_motor.h"
#include "05_can_functions.h"
// #include "06_bluetooth_functions.h"
#include "10_bionxregg.h"
#include "08_uartdisplay.h"
#include "freertos/timers.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "mongoose/mongoose_glue.h"


#define BUTTON_PIN_8 8
#define BUTTON_PIN_20 20


ButtonState button;


struct BionxMotorState motorState = {
    .assistLevel = 100,
    .motorSpeed = 0,
    .motorPower = 0,
    .motorTemperature = 0,
    .motorStatus = 0,
    .motorVoltage = 0,
    .gauge = 0,
    .rekupLevel = 0,
    .level = 0,
    .motorlevel = 0,
    .light = false,
    .rawTorque = 0,
    .maxMotorCurrent = 0,
    .currentMotorPower = 0
};


const TickType_t REKUP_RAMP_TIME_MS = 500;
TickType_t rekupStartTime = 0;
bool rekupActive = false;


extern EventGroupHandle_t taskEventGroup;


QueueHandle_t torqueQueue;
QueueHandle_t speedQueue;
QueueHandle_t statusQueue;






void setupBionx() {
    uint16_t softwareVersion;




    for(int i=0; i<=10; i++) {
        readBionxRegister(BXID_MOTOR, BXR_MOTOR_SWVERS, &softwareVersion);
        delay(200);
    }


    writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_ACCESSORY_ENABLED, 0x01);

    Serial.print("Found motor with software version: ");
    Serial.println(softwareVersion);


    uint16_t currentHi, currentLo;
    readBionxRegister(BXID_MOTOR, REG_MOTOR_CONFIG_MAX_DISCHARGE_HI, &currentHi);
    readBionxRegister(BXID_MOTOR, REG_MOTOR_CONFIG_MAX_DISCHARGE_LO, &currentLo);
    motorState.maxMotorCurrent = (currentHi << 8) | currentLo;

    Serial.print("Maximaler Motor Strom: ");
    Serial.println(motorState.maxMotorCurrent);


    torqueQueue = xQueueCreate(1, sizeof(uint16_t));
    speedQueue = xQueueCreate(1, sizeof(uint16_t));
    statusQueue = xQueueCreate(1, sizeof(uint32_t));
}


void setupButtons() {
    pinMode(BUTTON_PIN_8, INPUT_PULLUP);
    pinMode(BUTTON_PIN_20, INPUT_PULLUP);
}


uint8_t calculateBatteryLevel(uint32_t Voltage) {
    if (Voltage >= BATTERY_VOLTAGE_MAX) return 100;
    if (Voltage <= BATTERY_VOLTAGE_MIN) return 0;


    uint8_t soc = map(Voltage, BATTERY_VOLTAGE_MIN, BATTERY_VOLTAGE_MAX, 0, 100);
    if (soc > 100) soc = 100;
    return soc;
}


void torqueTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);
    static TickType_t lastWriteTime = 0;
    static TickType_t lastAliveTime = 0;
    static int16_t lastwrittenLevel = 0;


    static int errorCounter = 0;
    static TickType_t lastread = 0;
    static TickType_t errorBlockStart = 0;
    const int ERROR_THRESHOLD = 20;
    const TickType_t ERROR_BLOCK_TIME = pdMS_TO_TICKS(1000); // 1 Sekunde


    while(1) {
        TickType_t currentTime = xTaskGetTickCount();
        TickType_t start_time = currentTime;
        if (errorCounter >= ERROR_THRESHOLD) {
            if ((currentTime - errorBlockStart) >= ERROR_BLOCK_TIME) {
                errorCounter = 0;
            } else {
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }
        }


        uint16_t rawTorque = 0;
        readBionxRegister(BXID_MOTOR, REG_MOTOR_TORQUE_GAUGE_VALUE, &rawTorque);
        TickType_t end_time = xTaskGetTickCount()-lastread;
        lastread = xTaskGetTickCount();
        //Serial.println("TorqueTask Zeit: " + String(pdTICKS_TO_MS(end_time)) + " ticks");
        motorState.rawTorque = rawTorque;
        motorState.motorlevel = processTorque(rawTorque);
        errorCounter = 0; // Erfolg: Fehlerzähler resetten


        
        if ((currentTime - lastWriteTime) >= pdMS_TO_TICKS(25)) {
            if (motorState.motorSpeed >= MINSPEED && motorState.motorlevel > 0 && motorState.rekupLevel==0) {
                writeBionxRegister(BXID_MOTOR, REG_MOTOR_ASSIST_LEVEL, motorState.motorlevel);
                // Serial.println(" write motorlevel: " + String(motorState.motorlevel));
                lastWriteTime = currentTime;
                lastwrittenLevel = motorState.motorlevel;
            } else if (motorState.motorlevel == 0 && lastwrittenLevel != 0) {
                writeBionxRegister(BXID_MOTOR, REG_MOTOR_ASSIST_LEVEL, 0);
                lastWriteTime = currentTime;
                lastwrittenLevel = motorState.motorlevel;
            }
            if (motorState.rekupLevel < 0) {
                writeBionxRegister(BXID_MOTOR, REG_MOTOR_ASSIST_LEVEL, motorState.rekupLevel * 64 / 150);
                lastWriteTime = currentTime;
                lastwrittenLevel = motorState.motorlevel;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void buttonTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(30);


    while(1) {
        TickType_t now = xTaskGetTickCount();


        motorState.assistLevel = handleButton(motorState.assistLevel, motorState.light, motorState.rekupLevel);
        
        bool brakeLeftPressed = digitalRead(BUTTON_PIN_8) == LOW;
        bool brakeRightPressed = digitalRead(BUTTON_PIN_20) == LOW;
        bool brakePressed = brakeLeftPressed || brakeRightPressed;

        // Bremsdruck setzt motorState.rekupLevel nur, wenn nicht gerade Rekup-Modus aktiv ist
        if (brakePressed) {
            if (!rekupActive) {
                rekupStartTime = now;
                rekupActive = true;
            }
            TickType_t elapsed = (now - rekupStartTime) * portTICK_PERIOD_MS;
            if (elapsed > REKUP_RAMP_TIME_MS) elapsed = REKUP_RAMP_TIME_MS;

            if (motorState.rekupLevel <= 0) { // wenn kein button-bedingter Rekup-Modus
                motorState.rekupLevel = (int16_t)((-elapsed * 64) / REKUP_RAMP_TIME_MS);
            }
        } else {
            rekupActive = false;

        }
    }
}


void keepAliveTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);
    
    while(1) {
        TickType_t currentTime = xTaskGetTickCount();
        static TickType_t lastAliveTime = 0;
        if ((currentTime - lastAliveTime) >= pdMS_TO_TICKS(400)) {
            writeBionxRegister(ID_BATTERY, REG_BATTERY_CONFIG_POWER_VOLTAGE_ENABLE, 0x01);
            lastAliveTime = currentTime;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



void speedTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(150);


    while(1) {
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void statusTask(void *parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 50 ms Taskintervall


    static int caseCounter = 0;


    uint16_t voltageHigh = 0, voltageLow = 0;


    while (1) {
        switch (caseCounter) {
            case 0:
                readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_METER, &motorState.motorPower);
                break;
            case 1:
                readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_MAIN, &motorState.motorStatus);
                break;
            case 2:
                readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_VOLTAGE_HI, &voltageHigh);
                break;
            case 3:
                readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_POWER_VOLTAGE_LO, &voltageLow);
                motorState.motorVoltage = (voltageHigh << 8) | voltageLow;
                motorState.currentMotorPower = (motorState.motorVoltage / 1000 * motorState.maxMotorCurrent / 1000 * motorState.motorPower) / 100;
                break;
            case 4:
                readBionxRegister(BXID_MOTOR, REG_MOTOR_STATUS_SPEED, &motorState.motorSpeed);
                int16_t batterylevel = calculateBatteryLevel(motorState.motorVoltage);
                updateDisplay(motorState.assistLevel, motorState.rekupLevel, motorState.motorSpeed,
                              motorState.motorStatus, batterylevel, motorState.motorlevel, motorState.light);
                glue_update_state();              
                break;
        }


        caseCounter = (caseCounter + 1) % 5;  // Zyklisch in 5 Schritten


        //xQueueReceive(torqueQueue, &motorState.rawTorque, 0);



        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


uint16_t processTorque(uint16_t rawTorque) {
    #define Q16_SHIFT       16
    #define Q16_SCALE       65536
    #define ALPHA_1PCT      ((uint32_t)(0.0001 * 65536))
    #define ALPHA_2PCT      ((uint32_t)(0.015 * 65536))
    #define DECAY_MIN       ((uint32_t)(0.99 * 65536))
    #define DECAY_MAX       ((uint32_t)(0.8 * 65536))
    #define DECAY_RANGE     (DECAY_MIN - DECAY_MAX)
    const TickType_t DECAY_INTERVAL = pdMS_TO_TICKS(5);


    static int32_t filtered = 0;
    static TickType_t lastDecay = 0;


    int32_t torque = (int32_t)rawTorque << Q16_SHIFT;


    if(torque >= (TORQUE_THRESHOLD_HIGH << Q16_SHIFT)) {
        int64_t temp = (int64_t)torque * ALPHA_2PCT +
                      (int64_t)filtered * (Q16_SCALE - ALPHA_2PCT);
        filtered = (int32_t)((temp + (1 << 15)) >> Q16_SHIFT);
    }
    else if(torque >= (TORQUE_THRESHOLD_LOW << Q16_SHIFT)) {
          int64_t temp = (int64_t)torque * ALPHA_1PCT +
                        (int64_t)filtered * (Q16_SCALE - ALPHA_1PCT);
        filtered = (int32_t)((temp + (1 << 15)) >> Q16_SHIFT);     
    }
     else {
        if((xTaskGetTickCount() - lastDecay) >= DECAY_INTERVAL) {
            int64_t decay = DECAY_MIN - ((DECAY_RANGE * (int64_t)filtered) >> 40);
            decay = (decay > DECAY_MIN) ? DECAY_MIN :
                    (decay < DECAY_MAX) ? DECAY_MAX : decay;


            int64_t temp = (int64_t)filtered * decay;
            filtered = (int32_t)((temp + (1 << 15)) >> Q16_SHIFT);


            lastDecay = xTaskGetTickCount();
        }
    }


    int32_t gauge = (int32_t)((filtered * motorState.assistLevel / 150) >> 16);


    return (uint16_t)(gauge);
}
