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

int calibrateCompassDial = -1;
// TODO: add command to stop calibration

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
  /*{0,105.220763,-24.730319,-40.534515,0.992524,-0.021443,-0.010875,-0.021443,0.997302,-0.007623,-0.010875,-0.007623,1.010909},
  {45,117.150231,27.236267,-37.615467,0.993795,-0.023713,-0.000997,-0.023713,0.970534,-0.003384,-0.000997,-0.003384,1.037412},
  {90,52.543794,68.657375,-42.588304,0.999507,-0.017963,0.004898,-0.017963,0.980536,0.014455,0.004898,0.014455,1.020929},
  {135,-34.140303,68.650323,-51.260516,1.008759,-0.018910,0.012320,-0.018910,0.990208,0.012771,0.012320,0.012771,1.001800},
  {180,-103.034171,35.777372,-56.652512,0.996658,-0.016549,-0.000788,-0.016549,0.996306,-0.000955,-0.000788,-0.000955,1.007353},
  {215,-113.318053,-17.882824,-59.473321,1.002798,-0.017757,0.002987,-0.017757,1.002905,0.004721,0.002987,0.004721,0.994664},
  {270,-52.559935,-53.268727,-54.278888,0.991568,-0.009748,-0.005968,-0.009748,0.989802,-0.010635,-0.005968,-0.010635,1.019145},
  {315,27.619767,-57.106253,-46.685205,1.003785,-0.014268,0.003135,-0.014268,0.975811,0.012090,0.003135,0.012090,1.021298}*/

  {0, 105.67776143575149, 21.20755519560987, -51.84271803537082, 0.9972568939796095, -0.03397724558306371, -0.0029936959044349425, -0.03397724558306369, 0.9902104609055149, 0.00541365832053093, -0.0029936959044349564, 0.005413658320530937, 1.0138869440950695},
  {120, -52.59487600799546, 63.86365026613032, -65.769762631303, 0.7194850902470941, -0.14716990079078485, -0.07582924893989489, -0.14716990079078482, 1.133035612579859, -0.018663054319372242, -0.0758292489398949, -0.018663054319372253, 1.2692215506027733},
  {135, -73.3429947192731, 52.462462586618955, -66.93271388931582, 0.26229512011859174, -0.17279655450040757, -0.007184309250283887, -0.17279655450040757, 1.8141178379043699, -0.09336624930577311, -0.00718430925028389, -0.09336624930577307, 2.248131796475313},
  {150, -100.45055403948385, 41.52385924865429, -69.42937224345708, 0.9109930764895028, -0.2546504424188301, -0.07325620042292019, -0.25465044241883006, 0.9633156456793662, -0.08047594035135447, -0.07325620042292018, -0.08047594035135444, 1.2477400259919356},
  {165, -125.00292409875917, 15.543412826738177, -74.14039553894928, 1.0413625288876078, -0.07639469600497897, -0.006590190926581286, -0.07639469600497893, 0.841558358132664, -0.007876487056254147, -0.006590190926581279, -0.007876487056254143, 1.1488492272639856},
  {180, -128.0413760383557, 6.3697801821313105, -74.21521389705497, 1.0762033030422313, -0.045771477494174775, 0.004104268232008591, -0.045771477494174775, 0.8355797198017783, 0.015072949196957217, 0.0041042682320085845, 0.015072949196957217, 1.1149246253230702},
  {195, -128.08405892007727, -11.894130605508975, -73.76482581801578, 1.002496476759533, 0.062311234194738024, 0.04307336673819493, 0.06231123419473795, 0.9212299926354588, -0.1180764467140841, 0.04307336673819489, -0.1180764467140841, 1.105119358625279},
  {210, -90.2104029215678, 7.801738442263601, -70.75529662654336, 1.3430145435440974, -0.16363448377582784, 0.0876236650777587, -0.1636344837758278, 0.69528509628583, 0.46790860113191546, 0.0876236650777587, 0.46790860113191546, 1.4473994673877},
  {225, -89.40630804195014, -46.95861883901371, -71.74618194776774, 0.7947560470524231, 0.16107808930709783, 0.012541627432976505, 0.16107808930709785, 1.068600375457223, -0.05298381221334089, 0.012541627432976507, -0.052983812213340906, 1.2177531528246164},
  {240, -58.565672498180156, -56.628297043582336, -67.03300939186967, 0.7410194462910925, 0.09019941944570414, 0.1091792786225368, 0.09019941944570413, 1.1308619738696208, 0.013727014267530514, 0.1091792786225368, 0.013727014267530537, 1.2211163895314219},
  {255, -32.175685711306826, -63.61048499034399, -67.20171688496801, 0.6559200555718201, 0.09409737989185292, -0.04099104291487185, 0.09409737989185289, 1.2234026002990603, 0.005972334087942875, -0.04099104291487185, 0.005972334087942876, 1.2627584789524513},
  {270, -27.0898535831267, -59.344873401202534, -66.4290714291877, 0.5582503574644094, 0.035637511180241936, 0.03322132877435657, 0.035637511180241936, 1.3152706183492182, -0.01905865642455058, 0.03322132877435658, -0.01905865642455066, 1.3666118498000241},
  {285, 9.990658298079024, -57.92689640586669, -63.039648265949296, 0.9062288839086614, -0.0860085435909597, -0.02617816258240473, -0.08600854359095968, 1.0033443395772317, -0.02555476251734414, -0.02617816258240472, -0.02555476251734417, 1.110363120252271},
  {300, -3.6163359732513967, -54.17453712788368, -63.637587246092295, 0.5683382075677171, -0.20053726448442605, -0.08457388578277322, -0.20053726448442602, 1.2991668853432812, -0.016342816868222393, -0.08457388578277324, -0.01634281686822239, 1.446676692712783},
  {315, 63.41508831369622, -42.59553615479749, -56.7401211612662, 0.8163559176993452, -0.14535826761386922, -0.01900426145762017, -0.1453582676138693, 1.028213445328146, 0.022011356803295183, -0.01900426145762015, 0.02201135680329518, 1.2228953646976997},
  {330, 87.53676871646059, -26.891415668871005, -55.353665467193, 0.9315397178758036, -0.08065833870775868, 0.03723049864026279, -0.08065833870775868, 1.0241072071907134, 0.013672480059813377, 0.037230498640262816, 0.013672480059813334, 1.0571879011174377},
  {345, 99.23727040211675, -14.460524506780175, -55.06299049285062, 1.0200901791699484, -0.08883953532668723, -0.04456737095868559, -0.08883953532668726, 0.9185653155248675, -0.08620752714738204, -0.04456737095868557, -0.08620752714738206, 1.0871368503903145}

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

  if(-10 < angle - oldAngle && angle - oldAngle < 10){ // Avoiding lp filter between 360 and 0 which will produce some random outcome
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
  // TODO: maybe use gyro to introduce noise when shaking

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
}
void endCalibration(){
  calibrateCompass = false; // disable calibration mode
  disableMotor = false; // re-enable motor
  calibrateCompassDial = -1; // set calibration dial back to negative value to ensure restart next time
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
  }else{
    if(calibrateCompass && !checkBluetooth()){
      // Stopping calibration when BT disconnected - suboptimal
      // TODO: maybe have a BT command for stopping calibration
      endCalibration();
    }
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
  // TODO: open/close sequences to control compass (next step, etc )
  // TODO: use close-open to initiate bluetooth
  // TODO: shutdown bluetooth after inactivity

  // 2. read GPS data
  readGps();
  
  // 3. Read angular position from encoder
  compassState.dial = readDialPosition();
  
  // 4. Read magnetometer (with compensation for tilt from accelerometer)
  int currentHeading = readCompass(compassState.dial);
  if(currentHeading >= 0){
    compassState.heading = currentHeading;
  }

  // Using this correction only when not calibrating the compass to ensure
  if(!calibrateCompass){ 
    compassState.dial += DIAL_ZERO;
    while(compassState.dial>=360){
      compassState.dial -= 360;
    }
    while(compassState.dial<0){
      compassState.dial += 360;
    }
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