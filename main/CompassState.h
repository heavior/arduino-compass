#ifndef _COMPASS_STATE_H_
#define _COMPASS_STATE_H_

#include "Arduino.h"
#include "ArduinoBLE.h"

struct CompassState{
  double lattitude = 0.0;
  double longtitude = 0.0; // Current coordinates from GPS
  bool havePosition = false; // do we have gps reading or not
  bool closed = true;   // closed lid (hall sensor)
  int servoSpeed = -1; // current servo speed
  int heading = 0; // direction from north (angles)
  int dial = 0; // current dial position (angles)
  float batteryVoltage = 0; // current voltage
  int batteryLevel = 0; // battery level - %%

  const double* destination = NULL;
  float direction = 0;
  float distance = 0;

  bool disableMotor = false;        // prod value: false
  bool spinMotor = false;           // prod value: false
  int spinSpeed = 0;

  bool calibrate = false; // are we in calibration state
  int calibrateTarget = -1; // dial position for calibration
  // TODO: add command to stop calibration


  float  currentCalibration[13];
};

void printCompassState(const CompassState& state);


void updateCompassStateBLE(const CompassState& state);
void setupCompassStateBLEService();

extern BLEService compassService;
extern BLECharacteristic compassStateCharacteristic;


#endif
