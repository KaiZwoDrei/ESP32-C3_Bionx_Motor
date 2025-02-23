#include "06_bluetooth_functions.h"
#include <Update.h>

// OTA Service and Characteristic UUIDs
#define SERVICE_OTA BLEUUID("1d14d6ee-fd63-4fa1-bfa4-8f47b42119f0")
#define CHAR_OTA_CONTROL BLEUUID("f7bf3564-fb6d-4e53-88a4-5e37e0326063")
#define CHAR_OTA_DATA BLEUUID("984227f3-34fc-4045-a5d0-2c581f81a153")

// Standard GATT Services
#define SERVICE_POWER      BLEUUID((uint16_t)0x1818)  // Cycling Power Service
#define CHAR_POWER_MEASURE BLEUUID((uint16_t)0x2A63)  // Power Measurement
#define CHAR_POWER_FEATURE BLEUUID((uint16_t)0x2A65)  // Power Feature

#define SERVICE_BATTERY    BLEUUID((uint16_t)0x180F)  // Battery Service
#define CHAR_BATTERY      BLEUUID((uint16_t)0x2A19)  // Battery Level
BLECharacteristic* pOtaControl = NULL;
BLECharacteristic* pOtaData = NULL;

class OtaCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        if (pCharacteristic == pOtaControl) {
            std::string value = pCharacteristic->getValue();
            if (value.length() > 0) {
                if (value[0] == 0x00) {
                    // Start OTA
                    Update.begin(UPDATE_SIZE_UNKNOWN);
                } else if (value[0] == 0x01) {
                    // End OTA
                    Update.end(true);
                    ESP.restart();
                }
            }
        } else if (pCharacteristic == pOtaData) {
            std::string value = pCharacteristic->getValue();
            if (value.length() > 0) {
                Update.write((uint8_t*)value.c_str(), value.length());
            }
        }
    }
};

// Global variables
BLEServer* pServer = NULL;
BLECharacteristic* pPowerMeasurement = NULL;
BLECharacteristic* pBatteryLevel = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
const uint32_t WHEEL_FACTOR = uint32_t((1000.0 * (1 << 16)) / (60 * 2.2));

// Callbacks class
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override { deviceConnected = true; }
    void onDisconnect(BLEServer* pServer) override { deviceConnected = false; }
};

uint16_t calculateWheelRevolutions(uint16_t speed_kmh) {
    return (uint32_t(speed_kmh) * WHEEL_FACTOR) >> 16;
}

void setupBLE() {
    BLEDevice::init("E-Bike BLE");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Power Service
    BLEService *pPowerService = pServer->createService(BLEUUID((uint16_t)0x1818));
    pPowerMeasurement = pPowerService->createCharacteristic(
        BLEUUID((uint16_t)0x2A63),
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pPowerMeasurement->addDescriptor(new BLE2902());

    // Battery Service
    BLEService *pBatteryService = pServer->createService(BLEUUID((uint16_t)0x180F));
    pBatteryLevel = pBatteryService->createCharacteristic(
        BLEUUID((uint16_t)0x2A19),
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pBatteryLevel->addDescriptor(new BLE2902());
    // OTA Service
    BLEService *pOtaService = pServer->createService(SERVICE_OTA);
    pOtaControl = pOtaService->createCharacteristic(
        CHAR_OTA_CONTROL,
        BLECharacteristic::PROPERTY_WRITE
    );
    pOtaData = pOtaService->createCharacteristic(
        CHAR_OTA_DATA,
        BLECharacteristic::PROPERTY_WRITE
    );
    pOtaControl->setCallbacks(new OtaCallbacks());
    pOtaData->setCallbacks(new OtaCallbacks());

    // Start OTA service
    pOtaService->start();
    // Start services
    pPowerService->start();
    pBatteryService->start();

    // Advertising
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x1818));
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->start();
}


void updateBLEData(int16_t humanPower, int16_t torqueValue, uint16_t speed, int16_t motorPower, uint8_t batteryLevel) {
    if (deviceConnected) {
        struct PowerMeasurement {
            uint16_t flags;
            int16_t instantaneousPower;
            int16_t accumulatedTorque;
            uint32_t wheelRevolutions;
            uint16_t wheelEventTime;
        } __attribute__((packed)) powerData;

        static uint16_t eventTime = 0;
        eventTime += 1024;

        powerData.flags = 0x14;  // Bits: 2(Torque) and 4(Wheel)
        powerData.instantaneousPower = humanPower;
        powerData.accumulatedTorque = torqueValue;
        powerData.wheelRevolutions = calculateWheelRevolutions(speed);
        powerData.wheelEventTime = eventTime;

        pPowerMeasurement->setValue((uint8_t*)&powerData, sizeof(PowerMeasurement));
        pPowerMeasurement->notify();

        pBatteryLevel->setValue(&batteryLevel, 1);
        pBatteryLevel->notify();
    }

    // Connection handling
    if (!deviceConnected && oldDeviceConnected) {
        BLEDevice::startAdvertising();
    }
    oldDeviceConnected = deviceConnected;
}
