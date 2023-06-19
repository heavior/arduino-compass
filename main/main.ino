/* DEBUG CONFIGURATION */
// TODO: don't forget to set correct compilation target when switching boards

#ifdef TARGET_SEEED_XIAO_NRF52840_SENSE
  #define BOARD_REVISION 3            
#else
  #define BOARD_REVISION 2            // TODO: find how to distinguish rev1 and rev2
#endif


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
  .compensateCompassForTilt = true,   // prod value: true flag defines compensation for tilt. Bias and matrix are applied always, because otherwise it's garbage

    // sunrise 6:21 am 
  // sunset 19:30 pm 
  .sunriseTime = 620,
  .sunsetTime = 1930,
  .timeZoneOffset = -800 // PST timezone - +8 hours
  // TODO: need to read that from mobile and store permanently
  // TODO: actually, better read NMEA sentence GPZDA "Date and Time"  that has timezone

};

//#define OVERWRITE_MAP true

#define MOTOR_SPIN_FAST_SPEED 180
#define MOTOR_SPIN_SLOW_SPEED 60
// TODO: move slow and fast speed to compassConfig

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

unsigned int hardcodedDestinationsCount = 4;
// destination: id, name, radius (meters), true, {lat,lon}, visited
compass_MapPoint hardcodedDestinations[] = {
    { 0, "home", 0, true, { 34.1807809, -118.3008975 }, false },
    { 1, "colgin Ct", 0, true, { 34.1814535, -118.3002961 }, false },
    { 2, "Red Top Market & Kitchen", 0, true, { 34.1798233, -118.3010742 }, false },
    { 3, "Burbank Liquor & Food Market", 0, true, { 34.1804827, -118.3020486 }, false },
};

unsigned int destinationsCount = hardcodedDestinationsCount;
compass_MapPoint *destinations = hardcodedDestinations;

/* END OF DEBUG CONFIGURATION */

#define BLUETOOTH_NAME "Wooden Compass"



#include <Arduino.h>
#include <ArduinoBLE.h>
#include <math.h>
#include <TinyGPS++.h>
#include <pb_encode.h>

#include "storage.h"
#include "compass_utils.h"
#include "BatteryLevelService.h"
#include "CompassState.h"

#if BOARD_REVISION == 1   // Arduino nano ble sense rev1
  #define SERVO_PIN           D2    // servo - digital port
  #define ENCODER_PIN         A5    // angular encoder - analogue
  #define HALL_SENSOR_PIN     A6    // hall sensor - analogue
  #define BATTERY_PIN         A7  // battery level reading pin

  #define USE_SERVO true
  #include "Motor.h"
  Motor motor(ENCODER_PIN, SERVO_PIN);  // Create an instance of the Motor class

  #define COMPASS_PIVOT 180   // calibrated heading value when compass is aligned with north
  #define COMPASS_DIRECTION 1 // 1 - clockwise increase, -1 - counterclockwise

  #define R1 5100.0  // resistance of R1 in the voltage divider circuit
  #define R2 10000.0 //10000.0 // resistance of R2 in the voltage divider circuit

#elif BOARD_REVISION == 2 // Arduino nano ble sense rev2
  #define SERVO_PIN           D2    // servo - digital port
  #define ENCODER_PIN         A5    // angular encoder - analog
  #define HALL_SENSOR_PIN     A6    // hall sensor - analog
  #define BATTERY_PIN         A7  // battery level reading pin

  #include "Sensors.h"
  Sensors sensors();

  #define USE_SERVO true
  #include "Motor.h"
  Motor motor(ENCODER_PIN, SERVO_PIN);  // Create an instance of the Motor class

  #define COMPASS_PIVOT 270 // calibrated heading value when compass is aligned with north
  #define COMPASS_DIRECTION -1 // 1 - clockwise increase, -1 - counterclockwise

  #define R1 5100.0  // resistance of R1 in the voltage divider circuit
  #define R2 10000.0 //10000.0 // resistance of R2 in the voltage divider circuit

#elif BOARD_REVISION == 3 // Seedstudio XIAO ble sense
/*
A thread about reading battery levels: https://forum.seeedstudio.com/t/xiao-ble-sense-battery-level-and-charging-status/263248/23

So the solution becomes clear, replace nrfx_saadc_sample_convert() with nrfx_saadc_buffer_convert() in analogRead.
So we need to implement a non-blocking version of analogRead.

max analog value for XIAO is different from arduino, so need to use 4095 isntead of 1023
need to enable voltage read
*/
  #define BATTERY_PIN                     PIN_VBAT    // P0_31  battery level reading pin
  #define BATTERY_PIN_ENABLE              P0_14  // pin to allow readin battery level. Need to set to LOW to enable
  #define BATTERY_PIN_HIGH_CHARGE         P0_13  // pin to allow XIAO charge battery at 100ma. Need to set to LOW for charging
  #define BATTERY_PIN_CHARGING_STATUS     P0_17   // this is charging LED pin

  // uncomment to enable power switch control:
  #define POWER_SWITCH                    A0    // motor controller - analog port

  #define MOTOR_PIN1                      A1    // motor controller - analog port
  #define MOTOR_PIN2                      A2    // motor controller - analog port
  #define UV_LED_PIN                      D3    // UV LED for charging the disc
  #define HALL_SENSOR_PIN                 A4    // hass sensor - analog
  #define ENCODER_PIN                     A5    // angular encoder - analog
  // D6 (TX) - goes into RX on GPS
  // D7 (RX) - goes into TX on GPS
  #define MOTOR_PIN_SLEEP                 D8    // motor controller power - digital port
  #define MANGETOMETER_WIRE_PIN_SCL       D9       // For magnetometer
  #define MANGETOMETER_WIRE_PIN_SDA       D10      // For magnetometer

  #define UV_LED_TIME_CONTROL true
  #define R1 1000.0  // resistance of R1 in the voltage divider circuit inside XIAO
  #define R2 510.0 //10000.0 // resistance of R2 in the voltage divider circuit inside XIAO

  #include "Sensors.h"
  Sensors sensors(MANGETOMETER_WIRE_PIN_SCL, MANGETOMETER_WIRE_PIN_SDA);

  #define USE_SERVO false
  #include "Motor.h"
  Motor motor(ENCODER_PIN, MOTOR_PIN1, MOTOR_PIN2, MOTOR_PIN_SLEEP);  // Create an instance of the Motor class

  // TODO: find actual values
  #define COMPASS_PIVOT 270 // calibrated heading value when compass is aligned with north
  #define COMPASS_DIRECTION -1 // 1 - clockwise increase, -1 - counterclockwise

#endif
#define MOTOR_CALIBRATION false
// TODO: format my headers as proper libraries

/**
Components in use:
* Magnetometer to find magnetic north
* Gyroscope to compensate magnetometer for tilt
* GPS module to locate current position
* Angular encoder to check dial rotation
* motor to rotate the dial
* Hall sensor to identify closed lid and send everything to sleep if possible
* Blutooth to read an update. Turns on after first boot, and shuts down after two minutes without activity
*/

#define HALL_SENSOR_THRESHOLD   500   // value below that is a magnet


// TODO: auto-compensate min speed if no rotation happens
#define DIAL_ANGLE_SENSITIVITY 2  // angle difference where motor locks the engine

#define gpsPort                   Serial1 // 
#define GPS_HDOP_THRESHOLD        500 //  HDOP*100 to consider pointing to the target. See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
#define GPS_ACCURACY              2.5 // GPS accuracy of the hardware in meters
#define MIN_DISTANCE              (GPS_ACCURACY*2)            // distance when to consider destination reached
TinyGPSPlus gps;          // GPS object, reads through Serial1


#define REF_VOLTAGE   3.3 // reference voltage of the board
#define MIN_VOLTAGE   3.3 // minimum voltage for battery
#define MAX_VOLTAGE   4.2 // maximum voltage for battery

#define BATTERY_DANGER_THRESHOLD 10 //10% battery - dangerous level
// TODO: add some reaction to low battery level
// TODO: find actual low voltage



CompassState compassState{
    .has_location= true,
    .location ={0,0}, /* Current coordinates from GPS */
    .havePosition=0, /* do we have gps reading or not */
    .closed=false, /* closed lid (based on hall sensor) */
    .motorSpeed=-1,
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
    .currentCalibration_count=13,
};

#if BOARD_REVISION == 1

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

#endif


// TODO: do not check battery each loop (to save battery)
bool checkBattery(){ // return false if level is dangerous
  // measure battery voltage using analog input
  int batteryReading = 0;
  bool charging = false;
  #ifdef BATTERY_PIN_ENABLE
    digitalWrite(BATTERY_PIN_ENABLE, LOW); // connect pull-down resistors on XIAO to read battery level
    batteryReading = analogRead(BATTERY_PIN);
// Important: do not set to high during charging!
//    digitalWrite(BATTERY_PIN_ENABLE, HIGH);// disconnect pull-down resistors on XIAO to read battery level
  #else
    batteryReading = analogRead(BATTERY_PIN);
  #endif

  #ifdef BATTERY_PIN_CHARGING_STATUS
    charging = digitalRead(BATTERY_PIN_CHARGING_STATUS) == LOW;
  #endif

  float inputVoltage = REF_VOLTAGE * batteryReading / 1023;
  float batteryVoltage = inputVoltage * (R1 + R2) / R2;
  compassState.batteryVoltage = (float)((int)(10*batteryVoltage))/10; // cut second decimal digit 

/*  Serial.print("batterty pin reading: ");
  Serial.print(batteryReading);
  Serial.print(" votage: ");
  Serial.print(compassState.batteryVoltage);
  Serial.print(" charging?: ");
  Serial.println(charging);*/
  if(charging){
    compassState.batteryVoltage = -compassState.batteryVoltage;
  }

  if(compassState.batteryVoltage<.5){ // no battery
    return true;
  }

  // calculate battery percentage
  float remainingCapacity = (compassState.batteryVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100.0;
  compassState.batteryLevel = constrain(remainingCapacity, 0.0, 100.0);//to int

  BatteryLevelService.setBatteryLevel(compassState.batteryLevel);
  return compassState.batteryLevel > BATTERY_DANGER_THRESHOLD;
}

void playTheme(){
  motor.playTheme(checkClosedLid); // play theme, but interrupt if lid is closed
}
#define MAP_FILE MBED_FS_FILE_PREFIX "/map.csv"
void saveMap(){
  // Write the map points to a CSV file
  if(destinations){
    Serial.println("Saving map to memory");
    write_map_points(MAP_FILE, destinations, destinationsCount);
  }
}
void readMap(){
  // Create some map points

  // Read the map points back from the CSV file
  size_t num_points;
  compass_MapPoint* read_points = read_map_points(MAP_FILE, &num_points);

  if(!read_points)
  {
    Serial.println("No map in memory, using hardcoded:");
    destinations = hardcodedDestinations;
    destinationsCount = hardcodedDestinationsCount;
    saveMap();// Initialise file with hardcoded values
  }else{
    Serial.println("Reading map from memory:");
    if(destinations != hardcodedDestinations){
      // remove old points
      free(destinations);
    }
    destinations = read_points;
    destinationsCount = num_points;
  }

  // Print the read map points
  for (size_t i = 0; i < destinationsCount; ++i) {
    Serial.print(destinations[i].id);
    Serial.print(") ");
    Serial.print(destinations[i].name);

    Serial.print(" (");
    Serial.print(destinations[i].coordinates.latitude,6);
    Serial.print(",");
    Serial.print(destinations[i].coordinates.longitude,6);
    Serial.print(") r:");
    Serial.print(destinations[i].radius);
    Serial.print(" visited:");
    Serial.println(destinations[i].visited);
  }

  // Don't forget to free the memory
  //free(read_points);

}
void setup() {
  Serial.begin(9600); // non blocking - opening Serial port to connect to laptop for diagnostics
  delay(5000);
  Serial.println("Started");
  storage_begin();
  Serial.println("storage - done");
  #if OVERWRITE_MAP
    saveMap();// override file
  #endif
  readMap();

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
  gpsPort.begin(9600);
  if (!gpsPort) {  
    Serial.println("Failed to initialize GPS!");   
  }else{
    Serial.println("GPS ready");
  }
  
  if (!sensors.begin()) {
    Serial.println("Failed to initialize Sensors!");
  }else{
    Serial.println("Sensors ready");
  }

  #ifdef POWER_SWITCH // main switch transistor 
    // TODO: power it down when charging
    pinMode(POWER_SWITCH, OUTPUT);
    digitalWrite(POWER_SWITCH, HIGH); 
  #endif

  pinMode(ENCODER_PIN, INPUT);

  pinMode(BATTERY_PIN, INPUT);
  #ifdef BATTERY_PIN_ENABLE
    // battery things initialisation
    pinMode(BATTERY_PIN_ENABLE, OUTPUT);
  #endif
  #ifdef BATTERY_PIN_CHARGING_STATUS
    pinMode(BATTERY_PIN_CHARGING_STATUS, INPUT);// trying to read this pin to see if battery is charging
  #endif
  #ifdef BATTERY_PIN_HIGH_CHARGE // remove BATTERY_PIN_HIGH_CHARGE and it will not be trying to charge high
    pinMode(BATTERY_PIN_HIGH_CHARGE, OUTPUT);
    digitalWrite(BATTERY_PIN_HIGH_CHARGE, LOW); // setting XIAO to charge battery at high charge
  #endif

  motor.wakeUp();
  motor.calibrate();
  
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
    compassState.havePosition = true;//(precision < GPS_HDOP_THRESHOLD);// any position is good enough
    // TODO: monitor the precision and try to unlock compass earlier. Maybe if the distance is exceeding precision, we can assume it works!
    
  }else{
    return false;
  }
  return compassState.havePosition;
}

float readCompass(float encoderValue){
  // compass value compensated with accelerometer. will freak out when shaking.
  // TODO: maybe use gyro to introduce noise when shaking

  const float declinationAngle = 0;
  // this code generated with chatGPT
  float mx, my, mz, ax, ay, az;
  sensors.readMagneticField(mx, my, mz);

  if(compassConfig.interpolateCalibrations){
    interpolateCalibration(encoderValue, compassState.currentCalibration, calibrationMatrix,COMPASS_CALIBRATIONS);
  }else{
    closestCalibration(encoderValue, compassState.currentCalibration, calibrationMatrix,COMPASS_CALIBRATIONS);
  }
  calibrateMagReading (mx, my, mz, compassState.currentCalibration);

  sensors.readAcceleration(ax, ay, az); // this stuff works differently on rev1 and rev2 boards. Probably sensor orientation is off
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
  motor.wakeUp();
}
void endCalibration(){
  compassState.calibrate = false; // disable calibration mode
  compassState.disableMotor = false; // re-enable motor
  compassState.calibrateTarget = -1; // set calibration dial back to negative value to ensure restart next time
}

void sendCalibrationDataIfNeeded(){
  if(!compassState.calibrate || !compassState.disableMotor){
    Serial.print("Can't send calibration data while motor is running");
    return;
  }
  float mx, my, mz;
  sensors.readMagneticField(mx, my, mz);
  sendCalibrationData(mx, my, mz, compassState.calibrateTarget?(360-compassState.calibrateTarget):0);
}

void setVisitedDestination(uint32_t id){
  for (int i=0;i< destinationsCount;i++){
    if(destinations[i].id == id){
      destinations[i].visited = true;
    }  
  }
  saveMap(); // saving state to memory
}

void resetDestinations(){
  for (int i=1;i< destinationsCount;i++){
      destinations[i].visited = false;
  }
}
compass_MapPoint* findClosestUnvistedDestination(){
  float minDistance = FLT_MAX;
  compass_MapPoint *closestPoint = NULL;
  for (int i=0;i< destinationsCount;i++){
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

void disableUVLED(){
  #ifdef UV_LED_PIN
    digitalWrite(UV_LED_PIN, LOW);
  #endif
}
void enableUVLED(){
  #ifdef UV_LED_PIN

  bool needUV = true; // no time yet - keep LED working
  #if UV_LED_TIME_CONTROL
  if (gps.time.isUpdated()) {
    int hour = gps.time.hour();
    int minute = gps.time.minute();

    int time =  100*hour + minute + compassConfig.timeZoneOffset;
    if(time<0){
      time+=2400;
    }
    needUV = time < compassConfig.sunriseTime || time > compassConfig.sunsetTime;
  }
  #endif

  if(needUV){
    digitalWrite(UV_LED_PIN, HIGH);
  }
  #endif
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
        motor.wakeUp();
        disableUVLED();
        setNextDestination(); // Finding closest destination to visit
      }else{
        if(!compassState.calibrate){ // Just closed the lid - send motor to sleep
          motor.sleep(); // if compass is calibrating - do not send motor to sleep
        }
        enableUVLED();
      }
    }
  }

  // read battery
  if(!checkBattery()){
    //Serial.println("BATTERY LOW !!!");
  }

  // read temperature
  compassState.temperature = sensors.readTemperature();


  // TODO: if state (close/open) changed - initiate wakeup or sleep sequence
  // TODO: open/close sequences to control compass (next step, etc )
  // TODO: use close-open to initiate bluetooth
  // TODO: shutdown bluetooth after inactivity

  // 2. read GPS data
  readGps();
  
  // 3. Read angular position from encoder
  compassState.dial = motor.currentPosition();
  
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
    compassState.spinSpeed = MOTOR_SPIN_FAST_SPEED;
    // TODO: introduce some timer before setting next destination
    setNextDestination();
  } else if(!compassState.havePosition){
    compassState.spinMotor = true;
    compassState.spinSpeed = MOTOR_SPIN_SLOW_SPEED;
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
  int motorSpeed = compensationAngle;
  bool calibrateMotor = true;


  if(compassState.calibrate){
    Serial.print("compensate: ");
    Serial.print(compensationAngle);

    compassState.disableMotor = !compensationAngle; // no need to compensate => disable motor
    
    // otherwise the calculated speed will be correct
    sendCalibrationDataIfNeeded(); // this will try to send calibration data
  }else{
    // some special animations here
    if(compassState.spinMotor){
      motorSpeed = compassState.spinSpeed;
      calibrateMotor = false;
    }

    if(compassState.closed && !compassState.calibrate){
      // closed lid - kill all movement except in calibration
      motorSpeed = 0;
      calibrateMotor = false;
    }
  }

  // finally, reset the speed if motor is expected to be disabled
  if(compassState.disableMotor){
    motorSpeed = 0;
    calibrateMotor = false;
  }

//  if(compassState.motorSpeed != motor.mapSpeed(motorSpeed)){
  // TODO: 
    compassState.motorSpeed = motor.setSpeed(motorSpeed, MOTOR_CALIBRATION && calibrateMotor);
//  }
  

  //printCompassState (compassState);
  updateCompassStateBLE (compassState);
  sendCompassConfigBLE (compassConfig); // TODO: change this to pull, not notify

  delay(compassConfig.delay);  
}