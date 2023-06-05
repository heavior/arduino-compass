#ifndef _COMPASS_STATE_H_
#define _COMPASS_STATE_H_

#include "Arduino.h"
#include "ArduinoBLE.h"

#include <pb_encode.h>
#include "compassData.pb.h"

#define CompassState compass_CompassState
  // TODO: add command to stop calibration
//  float  currentCalibration[13];


void printCompassState(const CompassState& state);


void updateCompassStateBLE(const compass_CompassState& state);
void sendCompassConfigBLE(const compass_CompassConfig& state);

void setupCompassStateBLEService();

extern BLEService compassService;
extern BLECharacteristic compassStateCharacteristic;


#endif
