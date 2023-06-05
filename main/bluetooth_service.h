#ifndef BLUETOOTH_SERVICE_H
#define BLUETOOTH_SERVICE_H

#define BLUETOOTH_SERVICE_UUID "c2642aa8-51dd-456f-bdf3-542e6f6b8cc8"

#define BLUETOOTH_CALIBRATE_SERVICE_UUID "66c41382-675a-4ec8-af51-82456e148c79"
#define BLUETOOTH_CALIBRATE_CHARACTERISTIC_UUID "8a257ab3-65e9-4647-98ab-d07fc9bc78b0"
#define BLUETOOTH_CALIBRATE_CONTROL_CHARACTERISTIC_UUID "3c9d3165-a6ef-42c6-9956-01827683c3d6"


  /*spare UUIDS:
  
  5018e176-c27d-4ba4-b55b-5b6eff1e5132
  5b87f336-3dc3-4a20-8654-02d5317b405e
  9343ce55-7a1f-4764-a2e0-dc49687fdd21
  1126b3d7-a028-468b-a60d-ba970c4a51eb
  8a09f0d1-2156-4480-8841-cefa83df474a
  20aae0ad-67aa-4193-8e44-0af0ca65b275
  */


struct CalibrationData {
  float x;
  float y;
  float z;
  int angle;
} calibrationData;

BLEService calibrateCompassService(BLUETOOTH_CALIBRATE_SERVICE_UUID);
BLECharacteristic calibrationDataCharacteristic(BLUETOOTH_CALIBRATE_CHARACTERISTIC_UUID, BLERead | BLENotify, sizeof(CalibrationData));
BLEUnsignedIntCharacteristic calibrationControlCharacteristic(BLUETOOTH_CALIBRATE_CONTROL_CHARACTERISTIC_UUID, BLEWrite | BLENotify);
BLEDevice hostDevice;

void setupCalibrationBLEService(){
  BLE.setAdvertisedService(calibrateCompassService);
  calibrateCompassService.addCharacteristic(calibrationDataCharacteristic);
  calibrateCompassService.addCharacteristic(calibrationControlCharacteristic);
  BLE.addService(calibrateCompassService);
}

bool checkBluetooth(){
  hostDevice = BLE.central();
  return (!!hostDevice) && (hostDevice.connected());
}

int checkBluetoothCalibrationAngle(){
  if(!checkBluetooth()){
    return -1;
  }

  if (calibrationControlCharacteristic.written()) {
    return (int)calibrationControlCharacteristic.value();
  }
  return -1;
}

bool writeCalibrationData(float x, float y, float z, int angle){
  calibrationData.x = x;
  calibrationData.y = y;
  calibrationData.z = z;
  calibrationData.angle = angle;
  return !! calibrationDataCharacteristic.writeValue((byte*)&calibrationData, sizeof(CalibrationData));
}

#endif