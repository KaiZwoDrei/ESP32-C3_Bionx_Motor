#pragma once
#include <Arduino.h>


const int32_t TORQUE_FACTOR = (int32_t)(1.5625 * 65536);
const int32_t PEAK_WEIGHT = (int32_t)(0.8 * 65536);
const int32_t CURVE_WEIGHT = (int32_t)(0.2 * 65536);
//const int32_t GAUGE_DECAY = (int32_t)(0.90 * 65536);
// Konstanten für sanfteren Decay (10ms-Intervall)
    #define DECAY_PER_TICK      (int32_t)(0.95 * 65536)    // 99% skalierung pro tick
    #define FIXED_SCALE_16     65536UL  // 2^16 für 16:16 Festkomma
const int32_t TORQUE_THRESHOLD_HIGH = 20;
const int32_t TORQUE_THRESHOLD_LOW = 5;
// 8:24 Fixed-Point Format (32 Bit)
#define FIXED_SCALE 16777216UL  //2^24 for 8:24 fixed-point
#define INV_10 ((uint32_t)(FIXED_SCALE / 10))
#define INV_100 ((uint32_t)(FIXED_SCALE / 100))
#define INV_1000 ((uint32_t)(FIXED_SCALE / 1000))

#define INV_51 ((uint32_t)(FIXED_SCALE / 51))
#define BATTERY_VOLTAGE_MIN 39000UL    // 39.0V in mV
#define BATTERY_VOLTAGE_NOM 48100UL    // 48.1V in mV
#define BATTERY_VOLTAGE_MAX 54600UL    // 54.6V in mV
#define MINSPEED 0 // Minimalgeschwindigkeit für unterstützung (in U/min)
// Precomputed for 8:24 fixed-point
#define BATTERY_INV_UPPER_RANGE 165191UL   // (15 * FIXED_SCALE) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_NOM)
#define BATTERY_INV_LOWER_RANGE 934734UL   // (85 * FIXED_SCALE) / (BATTERY_VOLTAGE_NOM - BATTERY_VOLTAGE_MIN)

    #define IIR_ALPHA       (int32_t)(0.05 * 65536) //(Tiefpass bei ~20Hz)
    #define IIR_1MALPHA    (int32_t)(0.95 * 65536 )


// Zeitintervalle
const unsigned long STATUS_INTERVAL = 1000;  // 1 Hz
const unsigned long PEAK_INTERVAL = 400;     // Zeit zwischen Peaks bei 60 U/min
extern uint16_t maxMotorCurrent;  // in mA
extern uint16_t currentMotorPower; // in W

// Task-Deklarationen
void torqueTask(void *parameter);
void statusTask(void *parameter);
void buttonTask(void *parameter);
void speedTask(void *parameter);
void keepAliveTask(void *parameter);

void printTaskStats() ;



// Funktionsdeklarationen
void setupBionx();
uint16_t processTorque(uint16_t rawTorque);

// Externe Variablen
extern int16_t assistLevel;
extern uint16_t rawTorque;
extern uint16_t motorSpeed;
extern uint16_t motorPower;
extern uint16_t motorTemperature;
extern uint16_t motorStatus;
extern uint32_t motorVoltage;
extern int32_t gauge;





