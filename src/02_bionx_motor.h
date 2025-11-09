#pragma once
#include <Arduino.h>

// Struktur zur Bündelung aller motorspezifischen CAN-Bus-Daten
struct BionxMotorState {
    int16_t assistLevel;
    uint16_t motorSpeed;
    uint16_t motorPower;
    uint16_t motorTemperature;
    uint16_t motorStatus;
    uint32_t motorVoltage;
    int32_t gauge;
    int16_t rekupLevel;
    int32_t level;
    int16_t motorlevel;
    bool light;
    uint16_t rawTorque;
    uint16_t maxMotorCurrent;
    uint16_t currentMotorPower;
};
extern struct BionxMotorState motorState;

// Konstanten für Berechnungen und Regler
const int32_t TORQUE_FACTOR = (int32_t)(1.5625 * 65536);
const int32_t PEAK_WEIGHT = (int32_t)(0.8 * 65536);
const int32_t CURVE_WEIGHT = (int32_t)(0.2 * 65536);

#define DECAY_PER_TICK      (int32_t)(0.95 * 65536)    // 99% Skalierung pro Tick
#define FIXED_SCALE_16      65536UL  // 2^16 für 16:16 Festkomma

const int32_t TORQUE_THRESHOLD_HIGH = 15;
const int32_t TORQUE_THRESHOLD_LOW = 2;

#define FIXED_SCALE 16777216UL  // 2^24 für 8:24 Festkomma

#define INV_10 ((uint32_t)(FIXED_SCALE / 10))
#define INV_100 ((uint32_t)(FIXED_SCALE / 100))
#define INV_1000 ((uint32_t)(FIXED_SCALE / 1000))
#define INV_51 ((uint32_t)(FIXED_SCALE / 51))

#define BATTERY_VOLTAGE_MIN 39000UL    // 39.0V in mV
#define BATTERY_VOLTAGE_NOM 48100UL    // 48.1V in mV
#define BATTERY_VOLTAGE_MAX 54600UL    // 54.6V in mV

#define MINSPEED 0 // Minimalgeschwindigkeit für Unterstützung (in U/min)

// Precomputed für 8:24 Fixed-Point
#define BATTERY_INV_UPPER_RANGE 165191UL   // (15 * FIXED_SCALE) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_NOM)
#define BATTERY_INV_LOWER_RANGE 934734UL   // (85 * FIXED_SCALE) / (BATTERY_VOLTAGE_NOM - BATTERY_VOLTAGE_MIN)

#define IIR_ALPHA       (int32_t)(0.05 * 65536)
#define IIR_1MALPHA     (int32_t)(0.95 * 65536 )

const unsigned long STATUS_INTERVAL = 1000;  // 1 Hz
const unsigned long PEAK_INTERVAL = 400;     // Zeit zwischen Peaks bei 60 U/min

// Task-Deklarationen
void torqueTask(void *parameter);
void statusTask(void *parameter);
void buttonTask(void *parameter);
void speedTask(void *parameter);
void keepAliveTask(void *parameter);

// Mutex für CAN-Bus Zugriff
extern SemaphoreHandle_t canMutex;

// Geschützte Zugriffe auf Bionx Register mit Mutex
uint8_t safeReadBionxRegister(uint8_t device, uint8_t reg, uint16_t *value);
uint8_t safeWriteBionxRegister(uint8_t device, uint8_t reg, uint16_t value);

// Funktionsdeklarationen
void setupBionx();
uint16_t processTorque(uint16_t rawTorque);

void printTaskStats();
