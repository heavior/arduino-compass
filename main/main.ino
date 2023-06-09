/* DEBUG CONFIGURATION */

#define BLE_REVISION 2            // prod value: 2
// #define FIX_DIAL_POSITION  0 

#include "compassData.pb.h"
/* To change the structure:
  1) in root directory, modify compassData.proto
  2) python3 ../nanopb/generator/nanopb_generator.py compassData.proto;mv compassData.pb.* main 
*/

compass_CompassConfig compassConfig { 
  .encoderZeroDialNorth = 45,   // prod: 45   // where does the arrow points when encoder is 0? this correction will be applied to dial position, value depends on the encoder magnet!
  .interpolateCalibrations = true, // prod value: true, if false - use closest calibration, if true - interpolate calibration values (needs good calibration)
  .useDestination = true,    // prod value: true, if false - ignore destination and GPS, point to fixDirection on the dial
  .useCompass = true,        // prod value: true, if false - ignore magnetometer, set fixDirection on the dial

  .fixDirection = 0,           // prod value: doesn't matter

  .delay = 50, // prod value=100, if delay 50, accelrometer doesn't always have time to read
  .ignoreHallSensor = false,  // prod value: false
  .debugHall = false,         // prod value: false
  .enableBluetooth = true,    // prod value: true
  .compensateCompassForTilt = true   // prod value: true flag defines compensation for tilt. Bias and matrix are applied always, because otherwise it's garbage
  };
/*
struct CompassConfig{
  //  Actual configuration
  int encoderZeroDialNorth = 45;     // prod: 45   // where does the arrow points when encoder is 0? this correction will be applied to dial position, value depends on the encoder magnet!
  // use 0 when debugging calibration - makes things easier

  // Debug parameters
  bool interpolateCalibrations = true; // prod value: true, if false - use closest calibration, if true - interpolate calibration values (needs good calibration)
  bool useDestination = true;    // prod value: true, if false - ignore destination and GPS, point to fixDirection on the dial
  bool useCompass = true;        // prod value: true, if false - ignore magnetometer, set fixDirection on the dial

  int fixDirection = 0;           // prod value: doesn't matter

  unsigned int delay = 50; // prod value=100, if delay 50, accelrometer doesn't always have time to read
  bool ignoreHallSensor = false;  // prod value: false
  bool debugHall = false;         // prod value: false
  bool enableBluetooth = true;    // prod value: true
  bool compensateCompassForTilt =  true;   // prod value: true flag defines compensation for tilt. Bias and matrix are applied always, because otherwise it's garbage
};
CompassConfig compassConfig;*/

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

/*
#define DESTINATIONS_COUNT 1
compass_MapPoint destinations[DESTINATIONS_COUNT] = {
//  {0, "Home", 20, true, COORDINATES_MILLER_PARK, true }, // fixed destination, visited = true
  {0, "The Man", 100, true, COORDINATES_MAN, false }
};
*/

#define DESTINATIONS_COUNT 23 // youtopia destinations
compass_MapPoint destinations[] = 
{
  { 0, "Home", 10, true, {32.651426442942004, -116.18317194896657}, true }, // fixed destination, visited = true
  { 1, "1) Body Dysturbia", 5, true, { 32.6520232, -116.185332 }, false },
  { 2, "2) Danger Beans Coffee", 5, true, { 32.652025, -116.1849642 }, false },
  { 3, "3) Dichroic Tesseract", 5, true, { 32.6521514, -116.1842883 }, false },
  { 4, "4) Nandily Tea Lounge", 5, true, { 32.6521695, -116.183945 }, false },
  { 5, "5) Baby Seal Club Bar", 5, true, { 32.6525797, -116.183826 }, false },
  { 6, "6) Ring Of Fire", 5, true, { 32.6529868, -116.183885 }, false },
  { 7, "7) The Meadow of Lambient Gladdering", 5, true, { 32.6532332, -116.1842593 }, false },
  { 8, "8) Art In Motion", 5, true, { 32.6533901, -116.184097 }, false },
  { 9, "9) Phoenix Syndrome", 5, true, { 32.6518185, -116.1837364 }, false },
  { 10, "10) Gathering of Unlimited Devotion", 5, true, { 32.6520127, -116.1834896 }, false },
  { 11, "11) Poultry In Motion", 5, true, { 32.6513217, -116.1824865 }, false },
  { 12, "12) Those Lights Out There", 5, true, { 32.6523605, -116.1820037 }, false },
  { 13, "13) Deep Sea Kingdom", 5, true, { 32.6517553, -116.1814673 }, false },
  { 14, "14) Satanic Drug Confessional", 5, true, { 32.6517959, -116.1810273 }, false },
  { 15, "15) Little Free Satanic Drug Thing Library", 5, true, { 32.651683, -116.1806948 }, false },
  { 16, "16) YOU", 5, true, { 32.6533813, -116.1820734 }, false },
  { 17, "17) Return of Toxic Unicorn", 5, true, { 32.6531735, -116.1819661 }, false },
  { 18, "18) Mr Tenterbator The Galactic Jelly Fish", 5, true, { 32.6536703, -116.1816335 }, false },
  { 19, "19) Jackalope Chaos Entity", 5, true, { 32.6535077, -116.1810113 }, false },
  { 20, "20) Barrel Lounge", 5, true, { 32.6530425, -116.1810488 }, false },
  { 21, "21) Who gives a Cluck", 5, true, { 32.6524192, -116.179697 }, false },
  { 22, "Temple", 5, true, { 32.6533506, -116.1841735 }, false }
};

/*

#define DESTINATIONS_COUNT 4
compass_MapPoint destinations[] = {
    { 0, "0) home", 0, true, { 34.1807809, -118.3008975 }, false },
    { 1, "1) colgin Ct", 0, true, { 34.1814535, -118.3002961 }, false },
    { 2, "2) Red Top Market & Kitchen", 0, true, { 34.1798233, -118.3010742 }, false },
    { 3, "3) Burbank Liquor & Food Market", 0, true, { 34.1804827, -118.3020486 }, false },
};*/


// destination: id, name, radius (meters), true, {lat,lon}, visited
compass_MapPoint* destination = &destinations[1];

/* END OF DEBUG CONFIGURATION */

#define BLUETOOTH_NAME "Wooden Compass"



#include <Arduino.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <pb_encode.h>

#if BLE_REVISION == 1
  #include <Arduino_LSM9DS1.h>
#else
  #include <Arduino_BMI270_BMM150.h>
#endif


#include "compass_utils.h"
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
#define SERVO_MIN_SPEED     3     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
#define SERVO_SPIN_FAST_SPEED   (SERVO_ZERO_SPEED + SERVO_MAX_SPEED)
#define SERVO_SPIN_SLOW_SPEED   (SERVO_ZERO_SPEED - 2*SERVO_MIN_SPEED)

// TODO: auto-compensate min speed if no rotation happens
#define DIAL_ANGLE_SENSITIVITY 2  // angle difference where motor locks the engine
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



CompassState compassState{
    .has_location= true,
    .location ={0,0}, /* Current coordinates from GPS */
    .havePosition=0, /* do we have gps reading or not */
    .closed=false, /* closed lid (based on hall sensor) */
    .servoSpeed=-1,
    .heading=-1, /* direction from north (degrees) */
    .dial=-1, /* current dial position (degrees) */
    .batteryVoltage=0,
    .batteryLevel=0, /* battery level - % */
    .has_destination=true,
    .destination={0,0},
    .direction=-1, /* direction to the destination (degrees) */
    .distance=-1, /* distance to destination (meters) */
    .disableMotor=false,
    .spinMotor=false,
    .spinSpeed=-1,
    .calibrate=false, /* are we in calibration state */
    .calibrateTarget=-1, /* encoder position for calibration */
    .currentCalibration_count=13
};

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
  {0, 113.68027072947622, 14.064053135526231, -44.39007398141654, 1.0027124831401617, -0.02268537881007984, 0.0024771971492191544, -0.022685378810079825, 0.9761447582364063, 0.00496493698119237, 0.002477197149219158, 0.004964936981192359, 1.0222363782316373},
  {15, 105.4894449414744, 28.750851534515938, -44.778087997022496, 1.0050664011679724, -0.016027403127613856, -0.013442661589566175, -0.016027403127613825, 0.9940222455470866, -0.019658251580080017, -0.013442661589566177, -0.019658251580080065, 1.0017771470461858},
  {30, 90.27691703314431, 42.67934991372452, -45.13708557197065, 1.0014644602888696, -0.01563453628854472, -0.00783476483893771, -0.015634536288544765, 0.9808319205422699, -0.025864602022571178, -0.007834764838937682, -0.02586460202257119, 1.0190551676583255},
  {45, 67.1834959976654, 53.03301724039426, -47.323123190575885, 1.0037523046565147, -0.020404726165525064, 0.013964421855366135, -0.020404726165525112, 0.9889010183898896, 0.024781535279895794, 0.013964421855366121, 0.024781535279895867, 1.0086959376490694},
  {60, 40.00213242604774, 61.54220763675254, -49.46180955935889, 0.9965843021462037, -0.016780508149607954, -0.0035112485061931952, -0.01678050814960798, 0.9806315142534335, -0.01163022604149039, -0.0035112485061931896, -0.01163022604149038, 1.0236927980612756},
  {75, 9.170071443336312, 65.41964271452983, -53.1767414540037, 0.9998989031475495, -0.015323560451904724, -0.006331409766199269, -0.015323560451904769, 0.9778417524481537, -0.01870357236615381, -0.006331409766199262, -0.01870357236615388, 1.0234110920735373},
  {90, -22.86971044701701, 64.65830189298863, -55.62073036286999, 0.9993353123652456, -0.010254177622772698, -0.000626313406307885, -0.01025417762277274, 0.9814460622673431, 0.005012548074823763, -0.0006263134063078621, 0.005012548074823746, 1.0197176466658602},
  {105, -52.9983437747491, 57.785191242882625, -58.71800378370409, 0.9915611591147083, -0.011782612979585806, 0.0035745011276197207, -0.011782612979585865, 0.9857243537467775, 0.00278436692205036, 0.003574501127619727, 0.0027843669220503626, 1.0232826444700904},
  {120, -80.48406298328722, 48.78870963530105, -60.604146049604964, 0.9932690490798706, -0.012834569824951103, 0.0012229916908350223, -0.012834569824951117, 0.9749837347985626, 0.0025166829638088705, 0.001222991690835021, 0.002516682963808871, 1.0327923327619115},
  {135, -102.48130835534027, 35.00637281662516, -62.78163663049053, 0.9820177526292468, -0.010730544266471909, 0.0029998982423880727, -0.010730544266471893, 0.9928423771401024, 0.00834731912740568, 0.002999898242388077, 0.008347319127405656, 1.0258538125971286},
  {150, -118.25370846141507, 19.981318364603794, -64.44987236333571, 0.9835309724353677, -0.011275318271599286, 0.0028073844061295684, -0.01127531827159933, 0.9893107657798387, 0.009064411509864112, 0.0028073844061295757, 0.009064411509864125, 1.0279564150417795},
  {165, -125.78768964482576, 2.8344542131786143, -65.65044517991218, 0.9830709352573865, -0.0083413780735145, 0.0056587749338371275, -0.008341378073514559, 0.995974560152159, 0.0037422175209434567, 0.005658774933837137, 0.0037422175209434476, 1.0214514854344332},
  {180, -125.71604174942912, -14.392206625473166, -65.932231329674, 0.9838397224583174, -0.007034683005114371, -0.001420593421148925, -0.007034683005114328, 0.9948575314550776, -2.520768580069581e-06, -0.0014205934211489284, -2.5207685800733145e-06, 1.02173338711403},
  {195, -117.19411035486398, -30.933397792535427, -65.37826719349695, 0.9975311882954417, -0.005424825937816687, -7.201152829280364e-05, -0.0054248259378166875, 1.0148844792665455, -0.015060008443052069, -7.20115282928007e-05, -0.015060008443052006, 0.9880246584810981},
  {210, -102.2676631159999, -44.434187162443706, -64.3537487945139, 0.9922551102182853, -0.010426411764957233, 0.0011050881951672185, -0.010426411764957205, 0.986830506411362, 0.004144919335270069, 0.0011050881951672072, 0.004144919335270086, 1.0213868821453411},
  {225, -80.73592111687381, -54.53589907640182, -63.022073549618085, 0.9882396987165638, -0.011857374473834064, 0.0018608830217514561, -0.011857374473834082, 0.9830560010362627, -0.0009183855820126616, 0.001860883021751461, -0.0009183855820126652, 1.0294947231971234},
  {240, -52.55327205316702, -62.74312190656143, -60.66564897275921, 0.9902045370316078, -0.004476278245883679, 0.0011081577473065847, -0.004476278245883677, 1.0268707970497413, -0.02033995948651258, 0.001108157747306588, -0.020339959486512578, 0.9838891699661827},
  {255, -24.126689116391148, -66.20798045373705, -58.46687089625445, 0.9974870724879553, -0.012149582318484444, 0.003859566575040832, -0.01214958231848443, 0.9777190048287381, 0.012122338805890899, 0.00385956657504084, 0.012122338805890866, 1.0256870641418654},
  {270, 8.719236809816982, -65.18677344675274, -55.86816450926917, 0.9988619623110431, -0.012415586996986222, -0.003931434498299776, -0.01241558699698624, 0.9775234308620747, -0.00043174612978680646, -0.0039314344982998, -0.0004317461297868002, 1.0243363333690276},
  {285, 37.71290307822991, -60.07931813792543, -53.54047722356499, 1.0118833549743103, -0.010657080525749183, 0.0036973321337608485, -0.010657080525749155, 0.9798331573604966, 0.011718356495301655, 0.003697332133760846, 0.011718356495301651, 1.0088665584310759},
  {300, 66.10356866931997, -50.15320067354163, -50.74251462602648, 0.997715410742149, -0.018829996162770583, 0.0025432414587117525, -0.018829996162770573, 0.980395232462622, 0.00372686152685443, 0.002543241458711742, 0.003726861526854425, 1.0227241491940107},
  {315, 88.42171334204656, -37.141946866699925, -47.77591089216518, 0.9995541886272624, -0.023400797926381906, 0.020749038576672894, -0.02340079792638192, 0.9988617284154712, 0.015359753640773821, 0.02074903857667297, 0.015359753640773795, 1.0028179434506492},
  {330, 105.08765469639064, -21.165609914832896, -45.8926274839026, 0.9830492118464791, -0.025766654047821403, -0.019296517575932197, -0.0257666540478214, 1.0025747081133132, -0.019144656901783644, -0.019296517575932207, -0.019144656901783654, 1.0160788282731497},
  {345, 113.19227299843816, -4.266111367447122, -44.651796589287336, 0.9879999201482579, -0.018126245129884602, 0.0049784873664251595, -0.018126245129884713, 0.9956721173120885, 0.006518881085469104, 0.004978487366425194, 0.0065188810854691515, 1.0169539420637315}
};

#define COMPASS_PIVOT 270 // calibrated heading value when compass is aligned with north
#define COMPASS_DIRECTION -1 // 1 - clockwise increase, -1 - counterclockwise

#endif

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

  float angle = map(read, ENCODER_LOW, ENCODER_HIGH, 0, 359);

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
  compassState.batteryVoltage = (float)((int)(10*batteryReading / 1023.0 * REF_VOLTAGE * (R1 + R2) / R2))/10;

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

  setNextDestination();
 
  if(compassConfig.enableBluetooth){

    if (!BLE.begin()) {
      Serial.println("Failed to initialize BLE");
      return;
    }

    BLE.setLocalName(BLUETOOTH_NAME);
    BLE.setDeviceName(BLUETOOTH_NAME);

    BatteryLevelService.begin();
    setupCompassStateBLEService();

    BLE.advertise();

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
    compassState.location.latitude = gps.location.lat();
    compassState.location.longitude = gps.location.lng();
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

float readCompass(float encoderValue){
  // compass value compensated with accelerometer. will freak out when shaking.
  // TODO: maybe use gyro to introduce noise when shaking

  if (!IMU.accelerationAvailable() || !IMU.magneticFieldAvailable()) {
    return -1;
  }

  const float declinationAngle = 0;
  // this code generated with chatGPT
  float mx, my, mz, ax, ay, az;
  IMU.readMagneticField(mx, my, mz);

  if(compassConfig.interpolateCalibrations){
    interpolateCalibration(encoderValue, compassState.currentCalibration, calibrationMatrix,COMPASS_CALIBRATIONS);
  }else{
    closestCalibration(encoderValue, compassState.currentCalibration, calibrationMatrix,COMPASS_CALIBRATIONS);
  }
  calibrateMagReading (mx, my, mz, compassState.currentCalibration);

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


  if(!compassConfig.compensateCompassForTilt){
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

float getDistanceToDestination(compass_MapPoint destination){
  return gps.distanceBetween(
    compassState.location.latitude,
    compassState.location.longitude,
    destination.coordinates.latitude,
    destination.coordinates.longitude);
}
void updateDirection(){
  compassState.distance = getDistanceToDestination(compassState.destination);
  compassState.direction = gps.courseTo(
    compassState.location.latitude,
    compassState.location.longitude,
    compassState.destination.coordinates.latitude,
    compassState.destination.coordinates.longitude);
}

int getCompensationAngle(int targetDial){
  int compensationAngle = targetDial + compassState.dial;

  while (compensationAngle > 180){
    compensationAngle -= 360;
  }
  while (compensationAngle < -180){
    compensationAngle += 360;
  }

  if(-DIAL_ANGLE_SENSITIVITY < compensationAngle && compensationAngle < DIAL_ANGLE_SENSITIVITY){
    compensationAngle = 0;
  }
  return compensationAngle;
}
int getServoSpeed(int compensationAngle){
  float speed = SERVO_ZERO_SPEED;

  // TODO: consider non-linear speed scale
  speed = mapFloat(compensationAngle, -180, 180, SERVO_ZERO_SPEED - SERVO_MAX_SPEED , SERVO_ZERO_SPEED + SERVO_MAX_SPEED); 
  
  if(speed > SERVO_ZERO_SPEED - SERVO_MIN_SPEED && speed < SERVO_ZERO_SPEED){
    speed = SERVO_ZERO_SPEED - SERVO_MIN_SPEED;
  }
  if(speed < SERVO_ZERO_SPEED + SERVO_MIN_SPEED && speed > SERVO_ZERO_SPEED){
    speed = SERVO_ZERO_SPEED + SERVO_MIN_SPEED;
  }
  return (int)speed;
}
bool checkClosedLid(){
  int hallValue = analogRead(HALL_SENSOR_PIN);  // Read the value of the hall sensor
  compassState.closed = hallValue <= HALL_SENSOR_THRESHOLD;

  if(compassConfig.debugHall){
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

void sendCalibrationDataIfNeeded(){
  if(!compassState.calibrate || !compassState.disableMotor){
    Serial.print("nope ");
    return;
  }
  if (!IMU.magneticFieldAvailable()) {
    Serial.print("no IMU ");
    return;
  }
  float mx, my, mz;
  IMU.readMagneticField(mx, my, mz);
  sendCalibrationData(mx, my, mz, compassState.calibrateTarget?(360-compassState.calibrateTarget):0);
}

void setVisitedDestination(uint32_t id){
  for (int i=0;i< DESTINATIONS_COUNT;i++){
    if(destinations[i].id == id){
      destinations[i].visited = true;
    }  
  }
}

void resetDestinations(){
  for (int i=1;i< DESTINATIONS_COUNT;i++){
      destinations[i].visited = false;
  }
}
compass_MapPoint* findClosestUnvistedDestination(){
  float minDistance = FLT_MAX;
  compass_MapPoint *closestPoint = NULL;
  for (int i=0;i< DESTINATIONS_COUNT;i++){
    if(destinations[i].visited){
      continue;
    }  
    float distance = getDistanceToDestination(destinations[i]);
    if(distance < minDistance){
      minDistance = distance;
      closestPoint = &(destinations[i]);
    }
  }
  return closestPoint;
}
void setNextDestination(){
  uint32_t oldId = compassState.destination.id;

  // Logic: pick next closest not visited destination
  compass_MapPoint *closestPoint = findClosestUnvistedDestination();

  if(!closestPoint){
    closestPoint = &(destinations[0]); // default destination is home
  }
  compassState.destination = *closestPoint;

  // no destinations left - go home


  //
}

void loop() {
  // reading angle from BT service. Negative angle means no calibration needed
  int calibrationCommand = checkCalibrationAngle();
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
    if(compassState.calibrate && (calibrationCommand == -1)){
      endCalibration();
    }
  }

  // 1. Check if lid is closed or open:
  if(compassConfig.ignoreHallSensor){
    compassState.closed = false;
  }else{
    bool oldClosedValue = compassState.closed;
    checkClosedLid();
    if (oldClosedValue != compassState.closed){
      // EVENT: lid just closed or opened
      if(!compassState.closed){
        setNextDestination(); // Finding closest destination to visit
      }
    }
  }


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
  int currentHeading = -1;
  if(!compassState.calibrate){ // do not read compass when calibration
    currentHeading = readCompass(compassState.dial);
  }
  
  if(currentHeading >= 0){
    compassState.heading = currentHeading;
  }

  // Using this correction only when not calibrating the compass to ensure
  if(!compassState.calibrate){ 
    compassState.dial += compassConfig.encoderZeroDialNorth;
    while(compassState.dial>=360){
      compassState.dial -= 360;
    }
    while(compassState.dial<0){
      compassState.dial += 360;
    }
  }
  
  updateDirection();
  
  if(!compassState.closed && (compassState.distance < compassState.destination.radius + MIN_DISTANCE)){
    setVisitedDestination(compassState.destination.id);
    if(!compassState.spinMotor){ // playing theme once
      playTheme();
    }
    compassState.spinMotor = true;
    compassState.spinSpeed = SERVO_SPIN_FAST_SPEED;
    // TODO: introduce some timer before setting next destination
    setNextDestination();
  } else if(!compassState.havePosition){
    compassState.spinMotor = true;
    compassState.spinSpeed = SERVO_SPIN_SLOW_SPEED;
  } else {
    compassState.spinMotor = false;
  }
  
  int targetDial = (compassConfig.useCompass?compassState.heading:compassConfig.fixDirection)
                  - (compassConfig.useDestination?compassState.direction:0);
                  // using config we can ignore magnetometer (compass orientation) and or direction to destination

  if(!compassConfig.useDestination){
    compassState.spinMotor = false;
  }

  if(compassState.calibrate){
    compassState.spinMotor = false;
    targetDial = compassState.calibrateTarget;
  }
 
  int compensationAngle = getCompensationAngle(targetDial);
  int servoSpeed = getServoSpeed(compensationAngle);


  if(compassState.calibrate){
    Serial.print("compensate: ");
    Serial.print(compensationAngle);

    compassState.disableMotor = !compensationAngle; // no need to compensate => disable motor
    
    // otherwise the calculated speed will be correct
    sendCalibrationDataIfNeeded(); // this will try to send calibration data
  }else{
    // some special animations here
    if(compassState.spinMotor){
      servoSpeed = compassState.spinSpeed;
    }

    if(compassState.closed && !compassState.calibrate){
      // closed lid - kill all movement except in calibration
      servoSpeed = SERVO_ZERO_SPEED;
    }
  }

  // finally, reset the speed if motor is expected to be disabled
  if(compassState.disableMotor){
    servoSpeed = SERVO_ZERO_SPEED;
  }

  if(compassState.servoSpeed != servoSpeed){
    compassState.servoSpeed = servoSpeed;
    servoMotor.write(compassState.servoSpeed);
  }
  

  printCompassState (compassState);
  updateCompassStateBLE (compassState);
  sendCompassConfigBLE (compassConfig); // TODO: change this to pull, not notify

  delay(compassConfig.delay);  
}