#include "BatteryLevelService.h"

BatteryLevelServiceClass::BatteryLevelServiceClass() : _batteryService("180F"), _batteryLevelChar("2A19", BLERead | BLENotify) {
  // Empty Constructor
  currentLevel = 0;
}

void BatteryLevelServiceClass::begin() {
  _batteryService.addCharacteristic(_batteryLevelChar);  // Add battery level characteristic
  BLE.addService(_batteryService);  // Add battery service
}

void BatteryLevelServiceClass::setBatteryLevel(int level) {
  if(currentLevel!=level){ // only if level changed
    _batteryLevelChar.writeValue(level);  // Update battery level characteristic
    currentLevel = level;
  }
}

BatteryLevelServiceClass BatteryLevelService;
