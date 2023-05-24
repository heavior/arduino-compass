/* DEBUG CONFIGURATION */

#define BLE_REVISION 2            // prod value: 2
// #define FIX_DIAL_POSITION  0 

#define IGNORE_HALL_SENSOR false  // prod value: false

bool disableMotor = false;        // prod value: false
bool spinMotor = false;           // prod value: false
// TODO: move to compassState
int spinSpeed = 0;
#define REQUIRE_PLOTTER false     // prod value: false

#define DEBUG_HALL false          // prod value: false

#define USE_BLUETOOTH true        // prod value: true

#define USE_DESTINATION true      // prod value: true
#define FIX_DIRECTION 90          // if !USE_DESTINATION -> use this as a direction to point at

#define DELAY 50                 // prod value: 100 
// if delay 50, accelrometer doesn't have time to read

#define COMPENSATE_COMPASS true   // prod value: true flag defines compensation for tilt. Bias and matrix are applied always, because otherwise it's garbage


// some useful locations
#define COORDINATES_MAN {40.786397, -119.206561}          // Burnin man - The Man - North from home
#define COORDINATES_LONG_BEACH {33.704752, -118.294033}   // Long Beach - South from home 
#define COORDINATES_OXNARD {34.197981, -119.242624}       // OXNARD shore - West from home
#define COORDINATES_SNOW_SUMMIT {34.222881, -116.892340}  // Snow summit - East from home

#define COORDINATES_NORTH COORDINATES_MAN
#define COORDINATES_SOUTH LONG_BEACH
#define COORDINATES_EAST COORDINATES_SNOW_SUMMIT
#define COORDINATES_WEST COORDINATES_OXNARD

#define COORDINATES_MILLER_PARK {34.183580992498726, -118.29623564524786} // Some point in Miller elementary

#define COORDINATES_CURROS {34.183580992498726, -118.29623564524786} // Some point in Miller elementary
#define COORDIANTES_HAPPYDAYSCAFE {34.15109680100193, -118.45051328404955}

double destination[2] = COORDINATES_NORTH;//{34.180800,-118.300850};        // lattitude, longtitude


/* END OF DEBUG CONFIGURATION */

#define BLUETOOTH_NAME "Jack Sparrow Compass"



#include <Arduino.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>

#if BLE_REVISION == 1
  #include <Arduino_LSM9DS1.h>
#else
  #include <Arduino_BMI270_BMM150.h>
#endif


#include "compass_utils.h"
#include "bluetooth_service.h"
#include "sparrow_music.h"
// TODO: format my headers as proper libraries

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

bool calibrateCompass = false;
// enables when calibration blutooth service is in use
// when using SerialMonitor - use 'screen -L /dev/cu.usbmodem1101 9600' in shell to save raw data from the mag
// then use Ellipsoid fit python to perform calibration

int calibrateCompassDial = 0;
int calibrateReadings = 0;
#define CALIBRATE_DIAL_MAX  359
#define CALIBRATE_DIAL_STEP 15
#define CALIBRATE_READINGS_FOR_DIAL 2000 // prod=2000 - how many readings needed to calibrate each angle
// one calibration will take about 40 minutes and will produce 48 000 results. That's too long for an uninterrupted flow
// TODO: move calibration control to the BT side to allow one calibration at a time

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
// TODO: change to A3 with new layout

// TODO: we can calculate this version using encoder for feedback
#define SERVO_PIN           D2    // servo - digital port
#define SERVO_ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
#define SERVO_MAX_SPEED     20    // 30 is prev value, max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
#define SERVO_MIN_SPEED     4     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
#define SERVO_SPIN_FAST_SPEED   (SERVO_ZERO_SPEED + SERVO_MAX_SPEED)
#define SERVO_SPIN_SLOW_SPEED   (SERVO_ZERO_SPEED - 2*SERVO_MIN_SPEED)

// TODO: auto-compensate min speed if no rotation happens
#define DIAL_ANGLE_SENSITIVITY 1  // angle difference where motor locks the engine
#define DIAL_ZERO           40   // correction to be applied to dial position
Servo servoMotor;  // Servo Object

#define gpsPort             Serial1 // 
#define GPS_HDOP_THRESHOLD        500 //  HDOP*100 to consider pointing to the target. See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
#define GPS_ACCURACY              2.5 // GPS accuracy of the hardware in meters
#define MIN_DISTANCE              (GPS_ACCURACY*2)            // distance when to consider destination reached
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
  double longtitude = 0.0; // Current coordinates from GPS
  bool havePosition = false; // do we have gps reading or not
  bool closed = true;   // closed lid (hall sensor)
  int servoSpeed = SERVO_ZERO_SPEED; // current servo speed
  int heading = 0; // direction from north (angles)
  int dial = 0; // current dial position (angles)
  float batteryVoltage = 0; // current voltage
  int batteryLevel = 0; // battery level - %%


  const double* destination = NULL;
  float direction = 0;
  float distance = 0;
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

  Serial.print("\tClosed: ");
  Serial.print(state.closed);

  Serial.print("\tPosition: (");
  Serial.print(state.lattitude,6);
  Serial.print(",");
  Serial.print(state.longtitude,6);
  Serial.print(")");

  if(destination){
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
/* 
  //float angle = ENCODER_SCALE * read;
  
  /*Serial.print("Encoder value  = ");
  Serial.print(read);
  Serial.print("\t\t angle:");
  Serial.print(angle);fmy
  Serial.print("\t\t min:");
  Serial.print(encoderMin);
  Serial.print("\t\t max:");
  Serial.println(encoderMax);*/

  angle += DIAL_ZERO;

  while (angle >= 360){
    angle -= 360;
  }
  while (angle < 0){
    angle += 360;
  }
  if(-10 < angle - oldAngle && angle - oldAngle < 10){ // if angles are close. Avoiding lp filter between 360 and 0 which will produce some random outcome
    angle = lpFilter(angle, oldAngle, alpha);   // filter values, but this introduces some inertia to the system
  }
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

void playTheme(){
  playSparrowTheme(SERVO_PIN,checkClosedLid); // play theme, but interrupt if lid is closed
}

void setup() {
  Serial.begin(9600); // non blocking - opening Serial port to connect to laptop for diagnostics
  Serial.println("Started");

  if(USE_BLUETOOTH){
    startBluetooth();
  }
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

bool readGps(){ // return true if there is a good position available now.
  while(gpsPort.available())//While there are characters to come from the GPS.
  {
    int data = gpsPort.read();
    gps.encode(data);
  }
  if (gps.location.isUpdated()){
    compassState.lattitude = gps.location.lat();
    compassState.longtitude = gps.location.lng();
    int precision = gps.hdop.value(); // Horizontal Dim. of Precision (100ths-i32)
    /* multiply by 100s:
      <1	Ideal	Highest possible confidence level to be used for applications demanding the highest possible precision at all times.
      1–2	Excellent	At this confidence level, positional measurements are considered accurate enough to meet all but the most sensitive applications.
      2–5	Good	Represents a level that marks the minimum appropriate for making accurate decisions. Positional measurements could be used to make reliable in-route navigation suggestions to the user.
      5–10	Moderate	Positional measurements could be used for calculations, but the fix quality could still be improved. A more open view of the sky is recommended.
      10–20	Fair	Represents a low confidence level. Positional measurements should be discarded or used only to indicate a very rough estimate of the current location.
      >20	Poor	At this level, measurements should be discarded.


      Interpretation: 
      
    */
    // 
    /*
     float metersPrecision = 1. * GPS_ACCURACY * precision/100; half of the precision circle
     if (precision > GPS_HDOP_THRESHOLD){ // animation: slow rotating. discard value, no position available }
     if (distance + metersPrecision <= MIN_DISTANCE) { // animation: pirate theme  - even furthest point of our confidence is close enough to celebrate }
     if (distance + metersPrecision > MIN_DISTANCE) {  // could be far from the point
        if(metersPrecision > distance){ 
            // we are close, but don't know where to go
            // sweep + spin motion 
          
        }
        else{
          // can identify the direction with some accuracy
          // sweep motion
          float sweepAngleRange = (2* Math.asin (metersPrecision/distance)) * 180/M_PI; // animation - movement within this angle
          if (sweepAngleRange < MIN_SWEEP_ANGLE_RANGE){
            // no animation - point straight, the confidence level is good enough
          }
        }
     }
    */
    // 
    compassState.havePosition = (precision < GPS_HDOP_THRESHOLD);
    // TODO: monitor the precision and try to unlock compass earlier. Maybe if the distance is exceeding precision, we can assume it works!
    
  }else{
    return false;
  }
  return compassState.havePosition;
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

  if(calibrateCompass && disableMotor){
    // once motor is fixed - start printing data
    writeCalibrationData(mx,my,mz,calibrateCompassDial?(360-calibrateCompassDial):0);
    calibrateReadings ++;

    Serial.print(mx);
    Serial.print(",");
    Serial.print(my);
    Serial.print(",");
    Serial.print(mz);
    Serial.println();
  }

  interpolateCalibration( dialValue,currentCalibration,calibrationMatrix,COMPASS_CALIBRATIONS);
  calibrateMagReading (mx, my, mz, currentCalibration);

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

void updateDirection(const double (&destination)[2]){
  compassState.destination = destination;
  compassState.distance = gps.distanceBetween(
    compassState.lattitude,
    compassState.longtitude,
    destination[0],
    destination[1]);
  compassState.direction = gps.courseTo(
    compassState.lattitude,
    compassState.longtitude,
    destination[0],
    destination[1]);
}

int getServoSpeed(int targetDial){
  int speed = SERVO_ZERO_SPEED;
  int compensationAngle = targetDial + compassState.dial;

  while (compensationAngle > 180){
    compensationAngle -= 360;
  }
  while (compensationAngle < -180){
    compensationAngle += 360;
  }

  // TODO: consider non-linear speed scale
  speed = map(compensationAngle, -180, 180, SERVO_ZERO_SPEED - SERVO_MAX_SPEED , SERVO_ZERO_SPEED + SERVO_MAX_SPEED); 
  if(speed > SERVO_ZERO_SPEED - SERVO_MIN_SPEED && speed < SERVO_ZERO_SPEED){
    speed = SERVO_ZERO_SPEED - SERVO_MIN_SPEED;
  }
  if(speed < SERVO_ZERO_SPEED + SERVO_MIN_SPEED && speed > SERVO_ZERO_SPEED){
    speed = SERVO_ZERO_SPEED + SERVO_MIN_SPEED;
  }

  if(-DIAL_ANGLE_SENSITIVITY < compensationAngle && compensationAngle < DIAL_ANGLE_SENSITIVITY){
    speed = SERVO_ZERO_SPEED;
  }
  return speed;
}
bool checkClosedLid(){
  int hallValue = analogRead(HALL_SENSOR_PIN);  // Read the value of the hall sensor
  compassState.closed = hallValue <= HALL_SENSOR_THRESHOLD;

  if(DEBUG_HALL){
    Serial.print("hall: ");
    Serial.print(hallValue);
    Serial.print(" ");
  }

  return compassState.closed;
}
void startCalibration(int targetCalibrationAngle){
  calibrateCompass = true;  // enable calibration mode
  disableMotor = false; // enabling motor to let it turn to target value
  calibrateCompassDial = targetCalibrationAngle;
  calibrateReadings = 0; // reset readings counter
}
void endCalibration(){
  calibrateCompass = false; // disable calibration mode
  disableMotor = false; // re-enable motor
  // Send angle = -1 at the end, once
  calibrateCompassDial = -1; // set calibration dial back to negative value to ensure restart next time
  writeCalibrationData(0,0,0,-1); // send special value to inform the receiver
}

void loop() {
  // reading angle from BT service. Negative angle means no calibration needed
  int calibrationCommand = checkBluetoothCalibrationAngle();
  if(calibrationCommand >= 0 && calibrationCommand != calibrateCompassDial){
    // new calibration value - starting or restarting calibration
    /* 
      Calibration steps
      1) Turn motor to target
      2) Lock motor
      3) Collect samples
      4) End calibration with sending angle=-1 reading
    */
    startCalibration(calibrationCommand);
  }

  if(calibrateCompass && (calibrateReadings >= CALIBRATE_READINGS_FOR_DIAL)){
    // enough readings were sent, so we ned calibration
    endCalibration();
  }

  // 1. Check if lid is closed or open:
  #if IGNORE_HALL_SENSOR
  compassState.closed = false;
  #else
  checkClosedLid();
  #endif


  if(!checkBattery()){
    //Serial.println("BATTERY LOW !!!");
  }
  
  // TODO: if state (close/open) changed - initiate wakeup or sleep sequence
  // TODO: use close-open to initiate bluetooth

  // 2. read GPS data
  readGps();
  
  // 3. Read angular position from encoder
  compassState.dial = readDialPosition();

  // 4. Read magnetometer (with compensation for tilt from accelerometer)
  int currentHeading = readCompass(compassState.dial);
  if(currentHeading >= 0){
    compassState.heading = currentHeading;
  }
  
  updateDirection(destination);
  
  if(compassState.distance < MIN_DISTANCE){
    if(!spinMotor){ // playing theme once
      playTheme();
    }
    spinMotor = true;
    spinSpeed = SERVO_SPIN_FAST_SPEED;
  } else if(!compassState.havePosition){
    spinMotor = true;
    spinSpeed = SERVO_SPIN_SLOW_SPEED;
  } else {
    spinMotor = false;
  }
  
  #if !USE_DESTINATION
    compassState.direction = FIX_DIRECTION;
    spinMotor = false;
  #endif

  int targetDial = compassState.heading - compassState.direction;

  #ifdef FIX_DIAL_POSITION
    targetDial = FIX_DIAL_POSITION;
  #endif

  if(calibrateCompass){
    spinMotor = false;
    targetDial = calibrateCompassDial;
  }
 
  int servoSpeed = getServoSpeed(targetDial);

  if(spinMotor){
    servoSpeed = spinSpeed;
  }
  if((compassState.closed && !calibrateCompass) || disableMotor){
    servoSpeed = SERVO_ZERO_SPEED;
  }

  if(compassState.servoSpeed != servoSpeed){
    compassState.servoSpeed = servoSpeed;
    servoMotor.write(compassState.servoSpeed);
  }
  
  if(calibrateCompass && disableMotor){

  }else{
    if(REQUIRE_PLOTTER){
      plotCompassState (compassState);
    }else{
      printCompassState (compassState);
    }
  }

  if(calibrateCompass && compassState.servoSpeed == SERVO_ZERO_SPEED ){
    // Disable motor once it reaches target compensation value
    // TODO: maybe add dial position check here
    disableMotor = true;
  }

  delay(DELAY);  
}