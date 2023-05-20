/*
*/

#define BLE_REVISION 2
#define CALIBRATE_COMPASS false // when set to true, use 'screen -L /dev/cu.usbmodem1101 9600'  in shell to save raw data from the mag
// CALIBRATE_COMPASS disables motor after motor reaches FIX_DIAL_POSITION dial position!
// #define FIX_DIAL_POSITION  0 // For calibration and static tests - the dial will be turned to this position and locked

#define IGNORE_HALL_SENSOR false
bool disableMotor = false;
#define REQUIRE_PLOTTER false


#define DEBUG_HALL false

#define DELAY 100 // usually 100 ?
/*
if delay 50, accelrometer doesn't have time to read

*/
#define COMPENSATE_COMPASS true // flag defines compensation for tilt. Bias and matrix are applied always, because otherwise it's garbage


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
#define ENCODER_HIGH        1023  // 950
#define ENCODER_PIN         A5    // angular encoder - analogue

// TODO: we can calculate this version using encoder for feedback
#define SERVO_PIN           D2    // servo - digital port
#define SERVO_ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
#define SERVO_MAX_SPEED     20    // 30 is prev value, max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
#define SERVO_MIN_SPEED     3     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
// TODO: auto-compensate min speed if no rotation happens
#define DIAL_ANGLE_SENSITIVITY 1  // angle difference where motor locks the engine
#define DIAL_ZERO           40   // correction to be applied to dial position
Servo servoMotor;  // Servo Object

#define gpsPort             Serial1 // 
#define GPS_HDOP_THRESHOLD        500 //  HDOP*100 to consider pointing to the target. See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
TinyGPSPlus gps;          // GPS object, reads through Serial1


#define REF_VOLTAGE   3.3 // reference voltage of the board
#define BATTERY_PIN   A7  // battery level reading pin
#define MIN_VOLTAGE   3.3 // minimum voltage for battery
#define MAX_VOLTAGE   4.2 // maximum voltage for battery
#define R1 5100.0  // resistance of R1 in the voltage divider circuit
#define R2 10000.0 //10000.0 // resistance of R2 in the voltage divider circuit
#define BATTERY_DANGER_THRESHOLD 10 //10% battery - dangerous level


struct CompassState{
  double lattitude = 0.0;
  double longitude = 0.0; // Current coordinates from GPS
  bool havePosition = false; // do we have gps reading or not
  bool closed = true;   // closed lid (hall sensor)
  int servoSpeed = SERVO_ZERO_SPEED; // current servo speed
  int heading = 0; // direction from north (angles)
  int dial = 0; // current dial position (angles)
  float batteryVoltage = 0; // current voltage
  int batteryLevel = 0; // battery level - %%

};
void plotHeadersCompassState() {
  if(REQUIRE_PLOTTER){
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

  /*
  Serial.print("Position: (");
  Serial.print(state.lattitude,6);
  Serial.print(",");
  Serial.print(state.longitude,6);
  Serial.print(")");
  
  Serial.print("\tClosed: ");
  Serial.print(state.closed);

  */
  
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
/*
29.881207,6.166102,9.720654
0.991923,0.027539,0.022088
0.027539,1.001931,-0.012021
0.022088,-0.012021,1.007619

*/

#if BLE_REVISION == 1
float magBias[3] = {29.881207,6.166102,9.720654};
float softIronMatrix[3][3] = {
  {0.991923,0.027539,0.022088},
  {0.027539,1.001931,-0.012021},
  {0.022088,-0.012021,1.007619}
};
#define COMPASS_PIVOT 180   // calibrated heading value when compass is aligned with north
#define COMPASS_DIRECTION 1 // 1 - clockwise increase, -1 - counterclockwise


#define COMPASS_CALIBRATIONS 1
/*
  this is a calibration matrix. 13 columns:
  [0] dial angle at which compass was calibrated
  [1] offset X
  [2] offset Y
  [3] offset Z
  [4] - [13] - soft iron matrix row by row
*/
const float calibrationMatrix[COMPASS_CALIBRATIONS][13] = {{
  0,  // doesn't matter in this case
  29.881207,6.166102,9.720654, // ossets
  0.991923,0.027539,0.022088,  // soft iron matrix
  0.027539,1.001931,-0.012021,
  0.022088,-0.012021,1.007619
}};

#else

float magBias[3] = {105.220763,-24.730319,-40.534515};
float softIronMatrix[3][3] = {
  {0.992524,-0.021443,-0.010875},
  {-0.021443,0.997302,-0.007623},
  {-0.010875,-0.007623,1.010909}
};

#define COMPASS_CALIBRATIONS 9
/*
  this is a calibration matrix. 13 columns:
  [0] dial angle at which compass was calibrated
  [1] offset X
  [2] offset Y
  [3] offset Z
  [4] - [13] - soft iron matrix row by row
*/
const float calibrationMatrix[COMPASS_CALIBRATIONS][13] = {
  {0,105.220763,-24.730319,-40.534515,0.992524,-0.021443,-0.010875,-0.021443,0.997302,-0.007623,-0.010875,-0.007623,1.010909},
  {45,117.150231,27.236267,-37.615467,0.993795,-0.023713,-0.000997,-0.023713,0.970534,-0.003384,-0.000997,-0.003384,1.037412},
  {90,52.543794,68.657375,-42.588304,0.999507,-0.017963,0.004898,-0.017963,0.980536,0.014455,0.004898,0.014455,1.020929},
  {135,-34.140303,68.650323,-51.260516,1.008759,-0.018910,0.012320,-0.018910,0.990208,0.012771,0.012320,0.012771,1.001800},
  {180,-103.034171,35.777372,-56.652512,0.996658,-0.016549,-0.000788,-0.016549,0.996306,-0.000955,-0.000788,-0.000955,1.007353},
  {215,-113.318053,-17.882824,-59.473321,1.002798,-0.017757,0.002987,-0.017757,1.002905,0.004721,0.002987,0.004721,0.994664},
  {270,-52.559935,-53.268727,-54.278888,0.991568,-0.009748,-0.005968,-0.009748,0.989802,-0.010635,-0.005968,-0.010635,1.019145},
  {315,27.619767,-57.106253,-46.685205,1.003785,-0.014268,0.003135,-0.014268,0.975811,0.012090,0.003135,0.012090,1.021298}
  
};

#define COMPASS_PIVOT 270 // calibrated heading value when compass is aligned with north
#define COMPASS_DIRECTION -1 // 1 - clockwise increase, -1 - counterclockwise

#endif

float currentCalibration[13];

/*
float magBias[3] = {28.06, 8.76, 6.15};
float magScale[3] = {1.00, 1.00, 1.02};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
*/


float alpha = 0.95; // filtration for potentiometer
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

  float angle = map(read,ENCODER_LOW,ENCODER_HIGH,0,359);

  if(angle < 0 || angle >= 360){
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

  angle = lpFilter(angle, oldAngle, alpha);   // filter values, but this introduces some inertia to the system
  oldAngle = angle;
  return angle;
}

float lpFilter(float value, float oldValue, float alp){
  return oldValue*(1.f-alp)+ alp*value;
}


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

float readCompass(float dialValue){
  // compass value compensated with accelerometer. will freak out when shaking.
  // TODO: use gyro ro introduce noise when shaking

  if (!IMU.accelerationAvailable() || !IMU.magneticFieldAvailable()) {
    return -1;
  }

  const float declinationAngle = 0;
  // this code generated with chatGPT
  float mx, my, mz, ax, ay, az;
  IMU.readMagneticField(mx, my, mz);

  if(CALIBRATE_COMPASS && disableMotor){
    // once motor is fixed - start printing data
    Serial.print(mx);
    Serial.print(",");
    Serial.print(my);
    Serial.print(",");
    Serial.print(mz);
    Serial.println();
  }

  interpolateCalibration( dialValue,currentCalibration,calibrationMatrix,COMPASS_CALIBRATIONS);
  calibrateMagReading (mx, my, mz, currentCalibration);
  //calibrateMagReading(mx, my, mz, magBias, softIronMatrix);

  IMU.readAcceleration(ax, ay, az); // this stuff works differently on rev1 and rev2 boards. Probably sensor orientation is off
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  /*Serial.print ("raw heading: ");  
  Serial.print ((int)heading(mx, my, declinationAngle));
  Serial.print ("...\t");*/
  
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);

  float m_x_comp = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch;
  float m_y_comp = my*cos_roll - mz*sin_roll;
  float m_z_comp = -mx*sin_pitch + my*cos_roll*sin_pitch + mz*cos_pitch*sin_roll;


  if(!COMPENSATE_COMPASS){
    return heading(mx, my, declinationAngle);
  }

  float calibrated_heading = heading(m_x_comp, m_y_comp, declinationAngle);
  calibrated_heading = (calibrated_heading - COMPASS_PIVOT)*COMPASS_DIRECTION;
  if(calibrated_heading<0){
    calibrated_heading+=360;
  }  
  if(calibrated_heading>=360){
    calibrated_heading-=360;
  }
  return calibrated_heading;
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

  // 3. Read angular position from encoder
  // TODO: compensate for initial position
  // TODO: figure out 360 degrees rotation range
  compassState.dial = readDialPosition() + DIAL_ZERO;
  
  while (compassState.dial >= 360){
    compassState.dial -= 360;
  }
  while (compassState.dial < 0){
    compassState.dial += 360;
  }


  // 4. Read magnetometer (with compensation for tilt from accelerometer)
  // TODO: compensate for upside down or other nonce related to initial installation
  // TODO: validate against true north
  int currentHeading = readCompass(compassState.dial);
  if(currentHeading >= 0){
    compassState.heading = currentHeading;
  }
  

  int compensationAngle = compassState.heading + compassState.dial;

  #ifdef FIX_DIAL_POSITION
    compensationAngle = FIX_DIAL_POSITION + compassState.dial;
  #endif

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

  if(-DIAL_ANGLE_SENSITIVITY < compensationAngle && compensationAngle < DIAL_ANGLE_SENSITIVITY){
    servoSpeed = SERVO_ZERO_SPEED;
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

    if(!disableMotor){
      servoMotor.write(compassState.servoSpeed);
    }else{
      servoMotor.write(SERVO_ZERO_SPEED);
    }

    //Serial.print("\t servoSpeed: ");
    //Serial.print(currentServoSpeed);  // Print the value to the serial monitor
  }
  if(CALIBRATE_COMPASS && disableMotor){

  }else{
    if(REQUIRE_PLOTTER){
      plotCompassState (compassState);
    }else{
      printCompassState (compassState);
    }
  }


  if(CALIBRATE_COMPASS && compassState.servoSpeed == SERVO_ZERO_SPEED ){
    disableMotor = true;
  }

  delay(DELAY);  
}