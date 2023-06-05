#include "CompassState.h"

#include <pb_encode.h>
#include "compassData.pb.h"

// UUID for the service and characteristic
#define COMPASS_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define COMPASS_STATE_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define COMPASS_CONFIG_UUID "55c3d0bf-d6d5-47f5-9222-650e7249b6f6"

#ifndef compass_CompassConfig_size
#define compass_CompassConfig_size 512
#endif

BLEService compassService(COMPASS_SERVICE_UUID);
BLECharacteristic compassStateCharacteristic(COMPASS_STATE_UUID, BLENotify, sizeof(CompassState));
BLECharacteristic compassConfigCharacteristic(COMPASS_CONFIG_UUID, BLERead | BLEWrite | BLENotify, compass_CompassConfig_size);

void printCompassState(const CompassState& state) {
  Serial.print("\tClosed: ");
  Serial.print(state.closed);

  Serial.print("\tLocation: (");
  Serial.print(state.location.latitude,6);
  Serial.print(",");
  Serial.print(state.location.longitude,6);
  Serial.print(")");

  Serial.print(" Destination: (");
  Serial.print(state.destination.latitude,6);
  Serial.print(",");
  Serial.print(state.destination.longitude,6);
  Serial.print(")");

  Serial.print("\tDistance: ");
  Serial.print(state.distance,1);
  Serial.print("m\tDirection: ");
  Serial.print(state.direction,0);
  
  Serial.print("\tHeading: ");
  Serial.print(state.heading);
  
  Serial.print("\tDial: ");
  Serial.print(state.dial);

  Serial.print(" \tCurrent calibration: ");
  Serial.print(state.currentCalibration[0],0);

  Serial.print(" \tCalibrate: ");
  Serial.print(state.calibrate?String(state.calibrateTarget):"false");

  Serial.print("\tMotor: ");
  Serial.print(state.disableMotor?"off": (state.spinMotor?"spin":"on"));

  Serial.print("\tSpeed: ");
  Serial.print(state.spinMotor?state.spinSpeed:state.servoSpeed);

  Serial.print("\tBattery: ");
  Serial.print(state.batteryVoltage);
  Serial.print("v, ");
  Serial.print(state.batteryLevel);
  Serial.print("%");

  Serial.println();
}

void setupCompassStateBLEService() {
  compassService.addCharacteristic(compassStateCharacteristic);
  compassService.addCharacteristic(compassConfigCharacteristic);
  BLE.addService(compassService);
}


void updateCompassStateBLE(const compass_CompassState& state) {

  // Serialize the state structure
  uint8_t buffer[compass_CompassConfig_size];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  bool status = pb_encode(&stream, compass_CompassConfig_fields, &state);

  if (!status) {
    Serial.println("Compass state encoding failed");
    Serial.println(PB_GET_ERROR(&stream));
  } else {
    // send it over bluetooth
    compassStateCharacteristic.writeValue(buffer, stream.bytes_written);

    Serial.print("compass state len:");
    Serial.print(stream.bytes_written);
  }

}


void sendCompassConfigBLE(const compass_CompassConfig& config) {
  // Serialize the state structure
  uint8_t buffer[compass_CompassConfig_size];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  bool status = pb_encode(&stream, compass_CompassConfig_fields, &config);


  if (!status) {
    Serial.println("Compas eonfig encoding failed");
    Serial.println(PB_GET_ERROR(&stream));
  } else {
    // send it over bluetooth
    compassConfigCharacteristic.writeValue(buffer, stream.bytes_written);
  }
}
