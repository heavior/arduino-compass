/* DEBUG CONFIGURATION */

#define BLE_REVISION 2            // prod value: 2
// #define FIX_DIAL_POSITION  0 

#define IGNORE_HALL_SENSOR false  // prod value: false

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

const double destination[2] = COORDINATES_NORTH;//{34.180800,-118.300850};        // lattitude, longtitude


/* END OF DEBUG CONFIGURATION */

#define BLUETOOTH_NAME "Wooden Compass"



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
#include "BatteryLevelService.h"
#include "CompassState.h"
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
// TODO: add some reaction to low battery level
// TODO: find actual low voltage



CompassState compassState;

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

/*
  this is a calibration matrix. 13 columns:
  [0] dial angle at which compass was calibrated
  [1] offset X
  [2] offset Y
  [3] offset Z
  [4] - [13] - soft iron matrix row by row
*/
#define COMPASS_CALIBRATIONS 24
const float calibrationMatrix[COMPASS_CALIBRATIONS][13] = {
  {0, 35.2389347072011, 11.926297960551649, -24.40687593207286, 1.1435607685789355, -0.011980445734300337, 0.13333643757098027, -0.011980445734300337, 2.271722060230211, -0.10166208665790244, 0.13333643757098027, -0.10166208665790245, 0.40492691231777733},
  {15, 115.32590241179484, 23.968500992371236, -22.690892577418047, 1.0207038124486454, 0.1165465304399763, -0.014150118146213819, 0.11654653043997631, 0.8342264727119377, 0.0437994824583321, -0.014150118146213813, 0.04379948245833212, 1.1961470475551705},
  {30, 104.31585483324152, 37.424734151842095, -27.39104940930605, 0.9873134530448338, 0.18042890667647776, 0.04740503478619543, 0.1804289066764778, 0.8920243185356852, -0.08471688623981065, 0.04740503478619541, -0.08471688623981065, 1.1914591758337794},
  {45, 85.95523823709317, 48.474361135308314, -25.323021738608183, 0.8438576612075696, 0.2855246356600758, 0.0914083749851744, 0.28552463566007574, 1.0480201390969868, -0.0971951408036069, 0.09140837498517437, -0.09719514080360689, 1.2727097329744474},
  {60, 51.95432583368666, 61.478751321393915, -28.68697346058651, 0.6287106824455608, 0.16495231915873762, 0.005547056318590199, 0.16495231915873765, 1.2570039722171409, -0.0725347194881892, 0.005547056318590184, -0.07253471948818911, 1.315033652278322},
  {75, 45.848575802316184, 58.17197721749601, -25.758612500853083, 0.6236796383525005, 0.2183020316026321, -0.0631034062020118, 0.2183020316026321, 1.2357955761085626, -0.01037514059429809, -0.06310340620201178, -0.010375140594298072, 1.3894668079709644},
  {90, 10.23237335863182, 61.58429896064679, -29.62660853321496, 0.4128714405301831, 0.23258403746351725, -0.036661836077037685, 0.23258403746351725, 1.527001993223807, -0.08825987839502851, -0.036661836077037706, -0.08825987839502866, 1.7415558507593731},
  {105, -32.103244842790126, 62.01032448731433, -35.0892821110296, 0.6760941735847833, -0.11982046870480574, 0.06387295235232923, -0.11982046870480574, 1.1718240344311044, -0.017247595226444336, 0.06387295235232922, -0.017247595226444232, 1.2915664432919014},
  {120, -53.02544161279408, 55.775787304932294, -36.61123372333081, 0.7438716054208918, -0.1133547974169603, 0.01844866092767707, -0.11335479741696024, 1.0853557797367943, 0.01997491849691672, 0.018448660927677062, 0.019974918496916728, 1.259571495401699},
  {135, -87.83705802895982, 41.258511872611024, -37.72484072198952, 0.8557037792421246, -0.14329154973480665, 0.11675178253935195, -0.14329154973480668, 1.0719217827601852, 0.054953993330715226, 0.11675178253935196, 0.054953993330715226, 1.1364080775730239},
  {150, -104.68934066710804, 24.190543693696995, -42.17811753117079, 0.9476110981186541, -0.08274825809016301, -0.023150569490674663, -0.08274825809016298, 0.9558607705617738, -0.026026888733189243, -0.023150569490674667, -0.026026888733189257, 1.1138199015311354},
  {165, -112.54474454584324, 7.8469864739933675, -43.37041819978006, 1.009407276706587, -0.09442509863208497, 0.06693198228287894, -0.0944250986320849, 0.9291787364777011, 0.13754845540611751, 0.06693198228287892, 0.13754845540611754, 1.1033312679543195},
  {180, -24.116327012110375, 40.723935165034725, -34.69933859405637, 0.5317841902119793, -0.3988239388117227, -0.073077790172997, -0.39882393881172273, 1.3531302445936044, 0.011828007096490832, -0.07307779017299698, 0.011828007096490822, 1.7958755963571496},
  {195, -109.0821748136291, -21.56234995613705, -43.784602737645294, 0.9579896556144897, 0.02200444423478633, 0.0020929353854114542, 0.02200444423478628, 0.9961653001366431, 0.0048949614854398695, 0.0020929353854114486, 0.004894961485439871, 1.0484309778208005},
  {210, -92.0812057660295, 4.874937471342867, -44.15279381574041, 1.1692798965080693, -0.07901927886839008, -0.02393543545875311, -0.07901927886839007, 0.5927212440500926, -0.12340401899806314, -0.023935435458753106, -0.12340401899806314, 1.4831006879088475},
  {225, -98.90462845513612, -21.4813553546577, -40.79601689927032, 0.8792126778143328, 0.23898277492059242, -0.23542685519103793, 0.23898277492059242, 1.09679794770335, 0.07608797719021634, -0.23542685519103798, 0.07608797719021637, 1.1843429171822433},
  {240, -72.54959971614576, -49.54768341206112, -45.794248124370576, 0.7517122843507692, 0.1599027381629107, 0.03480868443351866, 0.1599027381629107, 1.0868424858484627, -0.10371244563135848, 0.03480868443351866, -0.10371244563135842, 1.2768845324732696},
  {255, -72.86234251248452, -36.46429513294253, -45.38559094108301, 0.6247870646684633, 0.32093762097343204, -0.04362176971893274, 0.32093762097343204, 1.2420542065668165, 0.044425795670053954, -0.043621769718932724, 0.04442579567005391, 1.493035566895775},
  {270, -16.097978586655284, -61.527429099925094, -36.0128124580443, 0.7500502678488368, 0.04822418551948506, -0.007536539508567837, 0.04822418551948505, 1.1074196566359464, 0.006002982192648483, -0.007536539508567837, 0.006002982192648472, 1.207413407707136},
  {285, 2.552944923662605, -59.99821356164157, -35.78174505980998, 0.5094455792024375, 0.07392987083317112, -0.00989469057834653, 0.07392987083317112, 1.3630790179575918, 0.013820976597582291, -0.00989469057834653, 0.013820976597582291, 1.4518506498169308},
  {300, 36.218098522338565, -55.75989973286457, -29.821037733485603, 0.5432679780218502, -0.22024814070887982, -0.12822164312815992, -0.2202481407088798, 1.3570947152513315, -0.056167771236167216, -0.1282216431281599, -0.05616777123616722, 1.4913797646973914},
  {315, 54.08181063976513, -49.62133650999933, -29.6797283136924, 0.6091762072980491, -0.2569575076003661, -0.048929519906150715, -0.2569575076003661, 1.2023935516404367, 0.0001214551003045268, -0.048929519906150715, 0.00012145510030454867, 1.5048196644921517},
  {330, 100.66547832628493, -30.29225948376169, -23.246042930215282, 0.8589569478195191, -0.21939132646966453, 0.13925018437487705, -0.21939132646966455, 1.0544636637330633, 0.17081792627706519, 0.13925018437487702, 0.1708179262770651, 1.231272469897699},
  {345, 115.1389802881891, -11.287900457392862, -22.483705893302712, 0.9995294829689106, -0.18471137254759812, -0.13655487096730823, -0.18471137254759812, 1.004645819206591, -0.16929654626273652, -0.13655487096730828, -0.16929654626273655, 1.0885179745729174}
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

  // calculate battery percentage
  float remainingCapacity = (compassState.batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0;
  compassState.batteryLevel = constrain(remainingCapacity, 0.0, 100.0);//to int

  BatteryLevelService.setBatteryLevel(compassState.batteryLevel);
  return compassState.batteryLevel > BATTERY_DANGER_THRESHOLD;
}

void playTheme(){
  playSparrowTheme(SERVO_PIN, checkClosedLid); // play theme, but interrupt if lid is closed
}

void setup() {
  Serial.begin(9600); // non blocking - opening Serial port to connect to laptop for diagnostics
  Serial.println("Started");

  if(USE_BLUETOOTH){
    startBluetooth();
    BatteryLevelService.begin();
    setupCompassStateBLEService();
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

  if(compassState.calibrate && compassState.disableMotor){
    // once motor is fixed - start printing data
    writeCalibrationData(mx,my,mz,compassState.calibrateTarget?(360-compassState.calibrateTarget):0);
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
  compassState.calibrate = true;  // enable calibration mode
  compassState.disableMotor = false; // enabling motor to let it turn to target value
  compassState.calibrateTarget = targetCalibrationAngle;
}
void endCalibration(){
  compassState.calibrate = false; // disable calibration mode
  compassState.disableMotor = false; // re-enable motor
  compassState.calibrateTarget = -1; // set calibration dial back to negative value to ensure restart next time
}

void loop() {
  // reading angle from BT service. Negative angle means no calibration needed
  int calibrationCommand = checkBluetoothCalibrationAngle();
  if(calibrationCommand >= 0 && calibrationCommand != compassState.calibrateTarget){
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
    if(compassState.calibrate && !checkBluetooth()){
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
  if(!compassState.calibrate){ 
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
    if(!compassState.spinMotor){ // playing theme once
      playTheme();
    }
    compassState.spinMotor = true;
    compassState.spinSpeed = SERVO_SPIN_FAST_SPEED;
  } else if(!compassState.havePosition){
    compassState.spinMotor = true;
    compassState.spinSpeed = SERVO_SPIN_SLOW_SPEED;
  } else {
    compassState.spinMotor = false;
  }
  
  #if !USE_DESTINATION
    compassState.direction = FIX_DIRECTION;
    compassState.spinMotor = false;
  #endif

  int targetDial = compassState.heading - compassState.direction;

  #ifdef FIX_DIAL_POSITION
    targetDial = FIX_DIAL_POSITION;
  #endif

  if(compassState.calibrate){
    compassState.spinMotor = false;
    targetDial = compassState.calibrateTarget;
  }
 
  int servoSpeed = getServoSpeed(targetDial);

  if(compassState.spinMotor){
    servoSpeed = compassState.spinSpeed;
  }
  if((compassState.closed && !compassState.calibrate) || compassState.disableMotor){
    servoSpeed = SERVO_ZERO_SPEED;
  }

  if(compassState.servoSpeed != servoSpeed){
    compassState.servoSpeed = servoSpeed;
    servoMotor.write(compassState.servoSpeed);
  }
  
  if(compassState.calibrate && compassState.servoSpeed == SERVO_ZERO_SPEED ){
    // Disable motor once it reaches target compensation value
    // TODO: maybe add dial position check here
    compassState.disableMotor = true;
  }

  printCompassState (compassState);
  updateCompassStateBLE (compassState);

  delay(DELAY);  
}