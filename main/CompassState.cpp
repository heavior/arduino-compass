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

  Serial.print("\tPosition: (");
  Serial.print(state.lattitude,6);
  Serial.print(",");
  Serial.print(state.longtitude,6);
  Serial.print(")");

  if(state.destination){
    Serial.print(" Destination: (");
    Serial.print(state.destination[0],6);
    Serial.print(",");
    Serial.print(state.destination[1],6);
    Serial.print(")");
  }else{
    Serial.print(" Destination: (");
    Serial.print(0.0,6);
    Serial.print(",");
    Serial.print(0.0,6);
    Serial.print(")");
  }

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

size_t serializeCompassState(const CompassState& state, uint8_t* buffer) {
  size_t offset = 0;

  // Copy the data of each field into the buffer using memcpy

  // Latitude
  memcpy(buffer + offset, &state.lattitude, sizeof(state.lattitude));
  offset += sizeof(state.lattitude);
  // Serial.print(offset);Serial.print(" "); // 8

  // Longitude
  memcpy(buffer + offset, &state.longtitude, sizeof(state.longtitude));
  offset += sizeof(state.longtitude);
  // Serial.print(offset);Serial.print(" ");// 16 

  // Have position
  memcpy(buffer + offset, &state.havePosition, sizeof(state.havePosition));
  offset += sizeof(state.havePosition);
  // Serial.print(offset);Serial.print(" "); // 17

  // Closed
  memcpy(buffer + offset, &state.closed, sizeof(state.closed));
  offset += sizeof(state.closed);
  // Serial.print(offset);Serial.print(" "); // 18

  // Servo speed
  memcpy(buffer + offset, &state.servoSpeed, sizeof(state.servoSpeed));
  offset += sizeof(state.servoSpeed);
  // Serial.print(offset);Serial.print(" "); // 22

  // Heading
  memcpy(buffer + offset, &state.heading, sizeof(state.heading));
  offset += sizeof(state.heading);
  // Serial.print(offset);Serial.print(" "); // 26

  // Dial
  memcpy(buffer + offset, &state.dial, sizeof(state.dial));
  offset += sizeof(state.dial);
  // Serial.print(offset);Serial.print(" "); // 30

  // Battery voltage
  memcpy(buffer + offset, &state.batteryVoltage, sizeof(state.batteryVoltage));
  offset += sizeof(state.batteryVoltage);
  // Serial.print(offset);Serial.print(" ");// 34

  // Battery level
  memcpy(buffer + offset, &state.batteryLevel, sizeof(state.batteryLevel));
  offset += sizeof(state.batteryLevel);
  // Serial.print(offset);Serial.print(" "); // 38

  if(state.destination){
    // Destination
    memcpy(buffer + offset, &state.destination[0], sizeof(state.destination[0]));
    offset += sizeof(state.destination[0]);
    // Serial.print(offset);Serial.print("-Destination "); //46

    memcpy(buffer + offset, &state.destination[1], sizeof(state.destination[1]));
    offset += sizeof(state.destination[1]);
    // Serial.print(offset);Serial.print("-Destination "); // 54
  }else{
    double zero = 0;
    memcpy(buffer + offset, &zero, sizeof(zero));
    offset += sizeof(zero);
    memcpy(buffer + offset, &zero, sizeof(zero));
    offset += sizeof(zero);
  }

  // Direction
  memcpy(buffer + offset, &state.direction, sizeof(state.direction));
  offset += sizeof(state.direction);
  // Serial.print(offset);Serial.print(" "); // 58 

  // Distance
  memcpy(buffer + offset, &state.distance, sizeof(state.distance));
  offset += sizeof(state.distance);
  // Serial.print(offset);Serial.print(" "); // 62

  // Disable motor
  memcpy(buffer + offset, &state.disableMotor, sizeof(state.disableMotor));
  offset += sizeof(state.disableMotor);
  // Serial.print(offset);Serial.print(" "); // 63

  // Spin motor
  memcpy(buffer + offset, &state.spinMotor, sizeof(state.spinMotor));
  offset += sizeof(state.spinMotor);
  // Serial.print(offset);Serial.print(" "); // 64

  // Spin speed
  memcpy(buffer + offset, &state.spinSpeed, sizeof(state.spinSpeed));
  offset += sizeof(state.spinSpeed);
  // Serial.print(offset);Serial.print(" "); // 68

  // Calibrate
  memcpy(buffer + offset, &state.calibrate, sizeof(state.calibrate));
  offset += sizeof(state.calibrate);
  // Serial.print(offset);Serial.print(" "); // 69

  // Calibrate target
  memcpy(buffer + offset, &state.calibrateTarget, sizeof(state.calibrateTarget));
  offset += sizeof(state.calibrateTarget);
  // Serial.print(offset);Serial.print(" ");// 73 


  // Return the used buffer length
  // Serial.println();
  return offset;
}


void updateCompassStateBLE(const CompassState& state) {
  // Serialize the state structure
  uint8_t data[sizeof(state)];
  size_t len = serializeCompassState(state, data);
  // Update the characteristic value
  compassStateCharacteristic.writeValue(data, len);
}


void sendCompassConfigBLE(const compass_CompassConfig& state) {
  // Serialize the state structure
  uint8_t buffer[compass_CompassConfig_size];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  bool status = pb_encode(&stream, compass_CompassConfig_fields, &state);



  if (!status) {
    Serial.println("Encoding failed");
    Serial.println(PB_GET_ERROR(&stream));
  } else {
    // send it over bluetooth
    compassConfigCharacteristic.writeValue(buffer, stream.bytes_written);

    Serial.print("compass confid len:");
    Serial.print(stream.bytes_written);
  }
}
