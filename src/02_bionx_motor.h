#ifndef BIONX_MOTOR_H
#define BIONX_MOTOR_H
#include <Arduino.h>


const int32_t TORQUE_FACTOR = int32_t(1.5625 * 65536);
const int32_t PEAK_WEIGHT = int32_t(0.7 * 65536);
const int32_t CURVE_WEIGHT = int32_t(0.3 * 65536);
const int32_t GAUGE_DECAY = int32_t(0.95 * 65536);
const int32_t TORQUE_THRESHOLD = 5;
// Zeitintervalle
const unsigned long STATUS_INTERVAL = 1000;  // 1 Hz
const unsigned long PEAK_INTERVAL = 500;     // Zeit zwischen Peaks bei 60 U/min
extern uint16_t maxMotorCurrent;  // in mA
extern uint16_t currentMotorPower; // in W

// Task-Deklarationen
void torqueTask(void *parameter);
void statusTask(void *parameter);
void buttonTask(void *parameter);
void speedTask(void *parameter);
void printTaskStats() ;

// Puffer-Größe
const int CURVE_BUFFER_SIZE = 50;

// Datenstruktur für Torque-Messungen
struct TorqueData {
    int32_t value;           // Festkomma-Format
    unsigned long timestamp;
    bool isPeak;
};


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


#endif



