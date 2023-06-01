#ifndef BatteryLevelService_h
#define BatteryLevelService_h

#include <ArduinoBLE.h>

class BatteryLevelServiceClass {
  public:
    BatteryLevelServiceClass();
    void begin();
    void setBatteryLevel(int level);
    
  private:
    BLEService _batteryService;
    BLEByteCharacteristic _batteryLevelChar;
    int currentLevel;
};

extern BatteryLevelServiceClass BatteryLevelService;

#endif