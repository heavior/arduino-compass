/*
*/

#define BLE_REVISION 1
#define IGNORE_HALL_SENSOR false
#define DIABLE_MOTOR true
#define REQUIRE_PLOOTER true


#define DEBUG_HALL false

#define DELAY 100 // usually 100 ?
#define COMPENSATE_COMPASS false


#include <Arduino.h>

#if BLE_REVISION == 1
  #include <Arduino_LSM9DS1.h>
#else
  #include <Arduino_BMI270_BMM150.h>
#endif

#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>

#include "compass_utils.h"

/**
Components in use:
* Magnetometer to find magnetic north
* Gyroscope to compensate magnetometer for tilt
* GPS module to locate current position
* Angular encoder to check dial rotation
* Servo motor to rotate the dial
* Hall sensor to identify closed lid and send everything to sleep if possible
* Blutooth to read an update. Turns on after first boot, and shuts down after two minutes without activity
*/


#define HALL_SENSOR_PIN         A6    // hass sensor - analogue
#define HALL_SENSOR_THRESHOLD   500   // value below that is a magnet

// TODO: 
// 1. research better conversion to degrees, this one doesn't really work.
// 2. find difference between north and arrow
// 3. test servo rotating speeds, find nice values
// 4. send right speed and direction to servo
// 5. stop when angle is correct?
#define ENCODER_LOW         0     // 50      
#define ENCODER_HIGH        1024  // 950
#define ENCODER_PIN         A5    // angular encoder - analogue

// TODO: we can calculate this version using encoder for feedback
#define SERVO_PIN           D2    // servo - digital port
#define SERVO_ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
#define SERVO_MAX_SPEED     30    // max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
#define SERVO_MIN_SPEED     10    // min speed where servo becomes unresponsive or doesn't have enough power
Servo servoMotor;  // Servo Object

#define gpsPort             Serial1 // 
#define GPS_HDOP_THRESHOLD        500 //  HDOP*100 to consider pointing to the target. See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
TinyGPSPlus gps;          // GPS object, reads through Serial1

struct CompassState{
  double lattitude = 0.0;
  double longitude = 0.0; // Current coordinates from GPS
  bool havePosition = false; // do we have gps reading or not
  bool closed = true;   // closed lid (hall sensor)
  int servoSpeed = SERVO_ZERO_SPEED; // current servo speed
  int heading = 0; // direction from north (angles)
  int dial = 0; // current dial position (angles)
  float batteryVoltage = 0; // current voltage
  int batteryLevel =0; // battery level - %%

};
void plotHeadersCompassState() {
  if(REQUIRE_PLOOTER){
    while(!Serial);
  }

  //Serial.print("Latitude ");
  //Serial.print("Longitude ");
  //Serial.print("GPS ");
  //Serial.print("Lid ");
  //Serial.print("ServoSpeed ");
  Serial.print("Heading,");
  Serial.print("Dial");
  //Serial.print("Battery ");
  //Serial.print("BatteryLevel ");
  Serial.println();
}
void plotCompassState(const CompassState& compassState) {
  /*Serial.print("Latitude: ");
  Serial.print(compassState.lattitude);
  Serial.print(" Longitude: ");
  Serial.print(compassState.longitude);
  Serial.print(" GPS: ");
  Serial.print(compassState.havePosition);
  Serial.print(" Lid: ");
  Serial.print(compassState.closed);
  Serial.print(" ServoSpeed: ");
  Serial.print(compassState.servoSpeed);*/
  //Serial.print(" Heading: ");
  Serial.print(compassState.heading);
  //Serial.print(" DialPosition: ");
  //Serial.print(",");
  //Serial.print(compassState.dial);
  /*Serial.print(" BatteryVoltage: ");
  Serial.print(compassState.batteryVoltage);
  Serial.print(" BatteryLevel: ");
  Serial.print(compassState.batteryLevel);*/
  Serial.println();
}
void printCompassState(const CompassState& state) {
  Serial.print("Position: (");
  Serial.print(state.lattitude,6);
  Serial.print(",");
  Serial.print(state.longitude,6);
  Serial.print(")");
  
  Serial.print("\tClosed: ");
  Serial.print(state.closed);
  
  Serial.print("\tHeading: ");
  Serial.print(state.heading);
  
  Serial.print("\tDial: ");
  Serial.print(state.dial);

  Serial.print("\tSpeed: ");
  Serial.print(state.servoSpeed);

  Serial.print("\tBatt: ");
  Serial.print(state.batteryVoltage);
  Serial.print("v, ");
  Serial.print(state.batteryLevel);
  Serial.print("%");

  Serial.println();
}


CompassState compassState;
// TODO: do the calibration again after assembly, and iron matrix
// TODO: move to the library
float magBias[3] = {0, 0, 0};
float magScale[3] = {1.00, 1.00, 1.00};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
/*
float magBias[3] = {28.06, 8.76, 6.15};
float magScale[3] = {1.00, 1.00, 1.02};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
*/


float alpha = 0.7; // filtration for potentiometer
float encoderMin=10000;
float encoderMax=-1;

float readDialPosition(){ // функция получения угла с потенциометра/энкодера
  static float oldAngle = 0;
  float read = analogRead(ENCODER_PIN);

  if(read < encoderMin){
    encoderMin = read;
  }

  if(read > encoderMax){
    encoderMax = read;
  }

  float angle = map(read,ENCODER_LOW,ENCODER_HIGH,0,360);
  if(angle < 0 || angle > 360){
    angle = 0;
  }
/*  if(angle > 360){
    angle = 0;
  }*/
  //float angle = ENCODER_SCALE * read;
  
  /*Serial.print("Encoder value  = ");
  Serial.print(read);
  Serial.print("\t\t angle:");
  Serial.print(angle);fmy
  Serial.print("\t\t min:");
  Serial.print(encoderMin);
  Serial.print("\t\t max:");
  Serial.println(encoderMax);*/

  angle = lpFilter(angle, oldAngle, alpha);   // фильтруем показания с потенциометра, если надо
  oldAngle = angle;
  return angle;
}

float lpFilter(float value, float oldValue, float alp){
  return oldValue*(1.f-alp)+ alp*value;
}


#define REF_VOLTAGE   3.3 // reference voltage of the board
#define BATTERY_PIN   A7  // battery level reading pin
#define MIN_VOLTAGE   3.3 // minimum voltage for battery
#define MAX_VOLTAGE   4.2 // maximum voltage for battery
#define R1 5100.0  // resistance of R1 in the voltage divider circuit
#define R2 10000.0 //10000.0 // resistance of R2 in the voltage divider circuit
#define BATTERY_DANGER_THRESHOLD 10 //10% battery - dangerous level


bool checkBattery(){ // return false if level is dangerous
  // measure battery voltage using analog input
  int batteryReading = analogRead(BATTERY_PIN);
  compassState.batteryVoltage = (float)batteryReading / 1023.0 * REF_VOLTAGE * (R1 + R2) / R2;

/*
  Serial.print(batteryReading);
  Serial.print("\t pin voltage:");
  Serial.print((float)batteryReading / 1023.0 * REF_VOLTAGE);

  Serial.print("\t battery voltage:");
  Serial.println((float)compassState.batteryVoltage);
*/
  // calculate battery percentage
  float remainingCapacity = (compassState.batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0;
  compassState.batteryLevel = constrain(remainingCapacity, 0.0, 100.0);//to int

  return compassState.batteryLevel > BATTERY_DANGER_THRESHOLD;
}


void setup() {
  Serial.begin(9600); // non blocking - opening Serial port to connect to laptop for diagnostics
  Serial.println("Started");

  pinMode(ENCODER_PIN, INPUT);
  servoMotor.attach(SERVO_PIN);

  gpsPort.begin(9600);
  if (!gpsPort) {  
    Serial.println("Failed to initialize GPS!");   
  }else{
    Serial.println("GPS ready");
  }
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
  }else{
    Serial.println("IMU ready");
    //TODO: Calibrate magnetometer here
    IMU.setContinuousMode(); // continous reading for 
  }

  Serial.println("Ready");
}

// Methods to reduce power consumption when compass is closed
void softSleep(){

}
void wakeUp(){

}

void readGps(){
  while(gpsPort.available())//While there are characters to come from the GPS.
  {
    int data = gpsPort.read();
    gps.encode(data);
  }
  if (gps.location.isUpdated()){
    compassState.lattitude = gps.location.lat();
    compassState.longitude = gps.location.lng();
    int precision = gps.hdop.value(); // Horizontal Dim. of Precision (100ths-i32)
    compassState.havePosition = (precision < GPS_HDOP_THRESHOLD);

    /*Serial.print("gps. precision: ");
    Serial.print(precision);
    Serial.print("\t lat: ");
    Serial.print(compassState.lattitude,6);
    Serial.print("\t lon: ");
    Serial.print(compassState.longitude,6);
    Serial.println();*/

  }else{
    //Serial.print("no gps");
  }
}

float readCompass(){
  // compass value compensated with accelerometer. will freak out when shaking.
  // TODO: use gyro ro introduce noise when shaking

  if (!IMU.accelerationAvailable() || !IMU.magneticFieldAvailable()) {
    return -1;
  }

  const float declinationAngle = 0;
  // this code generated with chatGPT
  float mx, my, mz, ax, ay, az;
  IMU.readMagneticField(mx, my, mz);
  if(!COMPENSATE_COMPASS){
    return heading(mx, my, declinationAngle);
  }

  IMU.readAcceleration(ax, ay, az);
  calibrateMagReading(mx,my,mz,magBias,magScale, softIronMatrix);
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  float m_x_comp = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch;
  float m_y_comp = my*cos_roll - mz*sin_roll;
  float m_z_comp = -mx*sin_pitch + my*cos_roll*sin_pitch + mz*cos_pitch*sin_roll;

  return heading(m_x_comp, m_y_comp, declinationAngle);
}

void loop() {
  // 1. Check if lid is closed or open:
  #if IGNORE_HALL_SENSOR
  compassState.closed = false;
  #else
  int hallValue = analogRead(HALL_SENSOR_PIN);  // Read the value of the hall sensor
  compassState.closed = hallValue <= HALL_SENSOR_THRESHOLD;

  if(DEBUG_HALL){
    Serial.print("hall: ");
    Serial.print(hallValue);
    Serial.print(" ");
  }
  #endif


  if(!checkBattery()){
    //Serial.println("BATTERY LOW !!!");
  }
  
  // TODO: if state (close/open) changed - initiate wakeup or sleep sequence

  //Serial.print("Hall sensor: ");
  //Serial.print(hallValue);  // Print the value to the serial monitor
  //Serial.print("\t motorOn: ");
  //Serial.print(motorOn);  // Print the value to the serial monitor

  // 2. read GPS data
  readGps();

  // 3. Read magnetometer (with compensation for tilt from accelerometer)
  // TODO: compensate for upside down or other nonce related to initial installation
  // TODO: validate against true north
  int currentHeading = readCompass();
  if(currentHeading >= 0){
    compassState.heading = currentHeading;
  }
  
  // 4. Read angular position from encoder
  // TODO: compensate for initial position
  // TODO: figure out 360 degrees rotation range
  compassState.dial = readDialPosition();
  int compensationAngle = compassState.heading + compassState.dial;
  while (compensationAngle > 180){
    compensationAngle -= 360;
  }
  while (compensationAngle < -180){
    compensationAngle += 360;
  }
  // Serial.print(compensationAngle);
  // Serial.print(",");

  /*  based on the servo tests, 
    90 = 0 speed
    >90 = counterclockwise
    0-90 clockwise

    120 max speed <- this value is servo-dependent
    60 min speed 
  */
  // TODO: check for minimal speed that halts the engine
  // TODO: consider non-linear speed scale
  // TODO: move numbers to constants
  // TODO: move to constants
  int servoSpeed = map(compensationAngle, -180, 180, SERVO_ZERO_SPEED - SERVO_MAX_SPEED , SERVO_ZERO_SPEED + SERVO_MAX_SPEED); 
  if(servoSpeed > SERVO_ZERO_SPEED - SERVO_MIN_SPEED && servoSpeed < SERVO_ZERO_SPEED){
    servoSpeed = SERVO_ZERO_SPEED - SERVO_MIN_SPEED;
  }
  if(servoSpeed < SERVO_ZERO_SPEED + SERVO_MIN_SPEED && servoSpeed > SERVO_ZERO_SPEED){
    servoSpeed = SERVO_ZERO_SPEED + SERVO_MIN_SPEED;
  }

  /*
  Serial.print("north: ");
  Serial.print(angleCompass);
  Serial.print("\t dial: ");
  Serial.print(angleDial);
  Serial.print("\t diff: ");
  Serial.print(compensationAngle);
  Serial.print("\t speed: ");
  Serial.print(servoSpeed);
  */

  if(compassState.closed){
    servoSpeed = SERVO_ZERO_SPEED;
  }
  if(compassState.servoSpeed != servoSpeed){
    compassState.servoSpeed = servoSpeed;

    if(!DIABLE_MOTOR){
        servoMotor.write(compassState.servoSpeed);
    }

    //Serial.print("\t servoSpeed: ");
    //Serial.print(currentServoSpeed);  // Print the value to the serial monitor
  }
  if(REQUIRE_PLOOTER){
    plotCompassState (compassState);
  }else{
    printCompassState (compassState);
  }
  delay(DELAY);  
}