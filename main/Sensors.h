#ifndef Sensors_h
#define Sensors_h


#if BOARD_REVISION == 1   // Arduino nano ble sense rev1
  #include <Arduino_LSM9DS1.h>
  #include <Arduino_HTS221.h> // humidity and temperatur
#elif BOARD_REVISION == 2 // Arduino nano ble sense rev2
  #include <Arduino_BMI270_BMM150.h> // build in IMU
  #include <Arduino_HTS221.h> // humidity and temperatur

#elif BOARD_REVISION == 3 // Seedstudio XIAO ble sense
  #include <DFRobot_QMC5883.h>  // Universal magnetometer library
  #include <Wire.h>
  // Note: Patch library with SoftwareI2C.h if need special pins and software implementaion for I2C instead of Wire.h

  #include <LSM6DS3.h> // Seedstudio XIAO BLE SENSE IMU, more details: https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
#endif

class Sensors {
  public:
    #if BOARD_REVISION == 3
      Sensors(int pinSCL, int pinSDA);
    #else
      Sensors();
    #endif
    bool begin();
    void sleep();
    void wakeUp();
/*    void enableAccelGyro();
    void enableMagnetometer();
    void setAccelRange(uint8_t range);
    void setGyroRange(uint8_t range);
    void setMagRate(uint8_t rate);*/
    void readAcceleration(float& x, float& y, float& z);
    void readGyroscope(float& x, float& y, float& z);
    void readMagneticField(float& x, float& y, float& z);
    void readTemperature(int &temp);
    int readTemperature();
  private: 
  #if BOARD_REVISION == 3
    int pinSCL;
    int pinSDA;
    DFRobot_QMC5883 compass;

    LSM6DS3 IMU;   
  #endif
};


#if BOARD_REVISION == 3

/*
#define HMC5883L_ADDRESS             (0x1E)
#define QMC5883_ADDRESS              (0x0D)
#define VCM5883L_ADDRESS             (0x0C)
*/
  Sensors::Sensors(int compassPinSCL, int compassPinSDA):compass(&Wire, /*I2C addr*/QMC5883_ADDRESS){
    pinSCL = compassPinSCL;
    pinSDA = compassPinSDA;
  }
#else
  Sensors::Sensors(){}
#endif


bool Sensors::begin() {
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
//    return false;
  }else{
    Serial.println("IMU ready");
  }

  #if BOARD_REVISION == 3
    // TODO: explore what this compass can do
    // compass.init(pinSDA, pinSCL);
    compass.begin();

  #else
    HTS.begin();
    IMU.setContinuousMode(); // continous reading for 
  #endif

  // Implement your initialization code here
  // Return true if the initialization is successful, false otherwise
  return true;
}

void Sensors::sleep() {
  Serial.println("Sensors::sleep() not implemented");
}

void Sensors::wakeUp() {
  Serial.println("Sensors::wakeUp() not implemented");
}
/*
void Sensors::enableAccelGyro() {
  // Implement enabling accelerometer and gyroscope here
}

void Sensors::enableMagnetometer() {
  // Implement enabling magnetometer here
}
/*
void Sensors::setAccelRange(uint8_t range) {
  // Implement setting accelerometer range here
}

void Sensors::setGyroRange(uint8_t range) {
  // Implement setting gyroscope range here
}

void Sensors::setMagRate(uint8_t rate) {
  // Implement setting magnetometer rate here
}
*/

void Sensors::readAcceleration(float& x, float& y, float& z) {
  // Implement reading accelerometer data here
  #if BOARD_REVISION == 3
      //Returns the values as floats.  Inside, this calls readRaw___();
    x = IMU.readFloatAccelX();
    y = IMU.readFloatAccelY();
    z = IMU.readFloatAccelZ();
  #else
    IMU.readAcceleration(x,y,z);
  #endif
}

void Sensors::readGyroscope(float& x, float& y, float& z) {
  // Implement reading gyroscope data here
  #if BOARD_REVISION == 3
      //Returns the values as floats.  Inside, this calls readRaw___();
    x = IMU.readFloatGyroX();
    y = IMU.readFloatGyroY();
    z = IMU.readFloatGyroZ();
  #else
    IMU.readGyroscope(x,y,z);
  #endif
}

void Sensors::readMagneticField(float& x, float& y, float& z) {
  // Implement reading magnetometer data here
  #if BOARD_REVISION == 3
    // Read compass values
    /*compass.read();

    // Return XYZ readings
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();*/


    sVector_t mag = compass.readRaw();
    compass.getHeadingDegrees();
    x = mag.XAxis;
    y = mag.YAxis;
    z = mag.ZAxis;
  #else
    IMU.readMagneticField(x,y,z);
  #endif
  Serial.print("magnetic x:");
  Serial.print(x);
  Serial.print("magnetic y:");
  Serial.print(y);
  Serial.print("magnetic z:");
  Serial.println(z);
}

int Sensors::readTemperature(){
  #if BOARD_REVISION == 3
    return IMU.readTempC();
  #else
    // did not test on arduino nano
    // read here to learn more: https://docs.arduino.cc/tutorials/nano-33-ble-sense/humidity-and-temperature-sensor
    return HTS.readTemperature();
  #endif
}
void Sensors::readTemperature(int &temp){
  temp = readTemperature();
}

#endif
