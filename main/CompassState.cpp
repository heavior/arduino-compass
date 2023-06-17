#include "CompassState.h"

#include <pb_encode.h>
#include "compassData.pb.h"

// UUID for the service and characteristic
#define COMPASS_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define COMPASS_STATE_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define COMPASS_CONFIG_UUID "55c3d0bf-d6d5-47f5-9222-650e7249b6f6"

#define CALIBRATE_CHARACTERISTIC_UUID "8a257ab3-65e9-4647-98ab-d07fc9bc78b0"
#define CALIBRATE_CONTROL_CHARACTERISTIC_UUID "3c9d3165-a6ef-42c6-9956-01827683c3d6"



// #define BLUETOOTH_SERVICE_UUID "c2642aa8-51dd-456f-bdf3-542e6f6b8cc8"

// #define BLUETOOTH_CALIBRATE_SERVICE_UUID "66c41382-675a-4ec8-af51-82456e148c79"




#ifndef compass_CompassConfig_size
#define compass_CompassConfig_size 512
#endif

BLEService compassService(COMPASS_SERVICE_UUID);
BLECharacteristic compassStateCharacteristic(COMPASS_STATE_UUID, BLENotify, compass_CompassState_size);
BLECharacteristic compassConfigCharacteristic(COMPASS_CONFIG_UUID, BLERead | BLEWrite | BLENotify, compass_CompassConfig_size);
BLECharacteristic calibrationDataCharacteristic(CALIBRATE_CHARACTERISTIC_UUID, BLERead | BLENotify, compass_CalibrationData_size);
BLEUnsignedIntCharacteristic calibrationControlCharacteristic(CALIBRATE_CONTROL_CHARACTERISTIC_UUID, BLEWrite | BLENotify);

BLEDevice hostDevice;


void printCompassState(const CompassState& state) {
  Serial.print("\tClosed: ");
  Serial.print(state.closed);

  Serial.print("\tLocation: (");
  Serial.print(state.location.latitude,6);
  Serial.print(",");
  Serial.print(state.location.longitude,6);
  Serial.print(")");

  Serial.print(" Destination: ");
  Serial.print(state.destination.name);
  Serial.print("(");
  Serial.print(state.destination.coordinates.latitude,6);
  Serial.print(",");
  Serial.print(state.destination.coordinates.longitude,6);
  Serial.print(")");

  Serial.print("\tDistance: ");
  Serial.print(state.distance,1);
  Serial.print("m\tDirection: ");
  Serial.print(state.direction,0);
  
  Serial.print("\tHeading: ");
  Serial.print(state.heading);
  
  Serial.print("\tDial: ");
  Serial.print(state.dial);

  Serial.print(" \tCalibrate: ");
  Serial.print(state.calibrate?String(state.calibrateTarget):"false");

  Serial.print("\tMotor: ");
  Serial.print(state.disableMotor?"off": (state.spinMotor?"spin":"on"));

  Serial.print("\tSpeed: ");
  Serial.print(state.spinMotor?state.spinSpeed:state.motorSpeed);

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


  compassService.addCharacteristic(calibrationDataCharacteristic);
  compassService.addCharacteristic(calibrationControlCharacteristic);
  BLE.addService(compassService);
  BLE.setAdvertisedService(compassService);
}



bool checkBluetooth(){
  hostDevice = BLE.central();
  return (!!hostDevice) && (hostDevice.connected());
}

int checkCalibrationAngle(){
  // this is not very good, but will do for now
  // TODO: get rid of side effect special values
  if(!checkBluetooth()){
    return -1; // no calibration
  }
  if (calibrationControlCharacteristic.written()) {
    return (int)calibrationControlCharacteristic.value();
  }
  return -2; // no new value
}



void updateCompassStateBLE(const compass_CompassState& state) {

  // Serialize the state structure
  uint8_t buffer[compass_CompassState_size];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  bool status = pb_encode(&stream, compass_CompassState_fields, &state);

  if (!status) {
    Serial.println("Compass state encoding failed");
    Serial.println(PB_GET_ERROR(&stream));
  } else {
    // send it over bluetooth
    compassStateCharacteristic.writeValue(buffer, stream.bytes_written);

    //Serial.print("compass state len:");
    //Serial.print(stream.bytes_written);
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


void sendCalibrationData(float x, float y, float z, int angle){
  compass_CalibrationData calibrationData;
  calibrationData.x = x;
  calibrationData.y = y;
  calibrationData.z = z;
  calibrationData.angle = angle;


  uint8_t buffer[compass_CalibrationData_size];
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  bool status = pb_encode(&stream, compass_CalibrationData_fields, &calibrationData);

  if (!status) {
    Serial.println("Compas calibration data encoding failed");
    Serial.println(PB_GET_ERROR(&stream));
  } else {
    // send it over bluetooth
    calibrationDataCharacteristic.writeValue(buffer, stream.bytes_written);
  }
}
