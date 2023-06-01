#include "BatteryLevelService.h"

BatteryLevelServiceClass::BatteryLevelServiceClass() : _batteryService("180F"), _batteryLevelChar("2A19", BLERead | BLENotify) {
  // Empty Constructor
  currentLevel = 0;
}

void BatteryLevelServiceClass::begin() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setAdvertisedService(_batteryService);  // Advertise battery service

  _batteryService.addCharacteristic(_batteryLevelChar);  // Add battery level characteristic
  BLE.addService(_batteryService);  // Add battery service

  BLE.advertise();  // Start advertising
}

void BatteryLevelServiceClass::setBatteryLevel(int level) {
  if(currentLevel!=level){ // only if level changed
    _batteryLevelChar.writeValue(level);  // Update battery level characteristic
    currentLevel = level;
  }
}

BatteryLevelServiceClass BatteryLevelService;
