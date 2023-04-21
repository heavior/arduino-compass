/**
<c> Evgeny Balashov + ChatGPT
Distributed under MIT Lisense (https://opensource.org/license/mit/)


Simple compass calibration utility. 
Created for Arduino Nano BLE Sense and embedded magnetometer, other platforms/magnetometers might require changes:
1) field range / gain calibration
2) API to read values
3) different indication mechanism (currently using embedded RGB on the board)
Some sensors/libraries might have better calibration tools embedded
If you adopt this tool for other boards or sensors - please, consider contributing back to this project


How to use: 
0. adjust calibrationTimeInSeconds and calibrationDelay if needed, default sampling will run for two minutes and produce 2400 samples
1. upload sketch to your controller
1.1 WARNING: be careful when uploading any sketches while compass is calibrating. For some reason this crashes my Mac
2. once powered up, it will perform calibration. You don't need connection to the host at this stage
2.1. during calibration the LED will be continously blue
2.2. while LED is continous, rotate your device in all possible directions
2.3. if serial monitor is connected, you will see progress and time left
4. after calibration, LED will change to green blinking state. 
4.1 connect serial monitor to read values
4.2 it will continouslty print calibration parameters, raw magnetometer readins, calibrated readings, raw north direction and calibrated north direction
5 compare calibrated north heading to some other compass (real or mobile app) in different direction to evaluate accuracy.
5.1 heading is not calibrated for tilt, so keep sensor horizontal
5.2 both magnetometer on your board, as well as your other compass are sensitive to each other, so keep them a bit apart
5.3 save calibration parameters
6 if curious, reset the device and calibrate again - compare calibration parameters
6.1 each calibration the numbers will be slightly different. longer calibration might or might not help get better outcomes
7. use final calibration parameters with calibrateMagReading function in your project
7.1 to make things better, apply tilt compensation to calibrated parameters
7.2 to make things even better, figure out declination angle for your location
7.3 magnetometers are noise, think if noise filtering makes sense for you
7.4 after final assembly, your soft iron matrix might change, so new calibration might improve accuracy

TODO: Make utility library (header) platform-agnostic
TODO: Firgure out Eigen library and implement soft iron matrix calculation
TODO: Move calibration to loop()
*/

#include <Arduino_LSM9DS1.h>
#include "compass_utils.h"
#include "led_indicators.h"



LED indicatorLed(LED_BUILTIN);

constexpr int calibrationTimeInSeconds = 120;
constexpr int calibrationDelay = 50;
constexpr float calibrationRadius = 0.1; //not used yet

float fieldRange = 4.0;  // not supported on arduino nano ble sense embedded magnetometer
// note: 
// 1) some magnetometers use gain instead of field range 
// 2) some magnetometers don't have the setting at all
// so, check out if you need it

float magBias[3] = {0, 0, 0};
float magScale[3] = {1, 1, 1};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

/*
19:31:37.541 -> Offset (Bias): 25.44, 1.61, 8.97
19:31:37.541 -> Scale: 0.96, 1.01, 1.03

19:33:51.577 -> Offset (Bias): 26.46, 2.41, 9.68
19:33:51.577 -> Scale: 1.01, 0.96, 1.03

19:36:03.012 -> Offset (Bias): 26.37, 0.88, 9.25
19:36:03.012 -> Scale: 0.99, 0.98, 1.03

20:55:06.135 -> Offset (Bias): 28.06, 8.76, 6.15
20:55:06.135 -> Scale: 1.00, 0.99, 1.01

21:29:11.246 -> Offset (Bias): -5.19, 0.59, 19.69
21:29:11.246 -> Scale: 0.83, 1.19, 1.05

21:34:15.241 -> Offset (Bias): 14.75, -3.80, 14.34
21:34:15.241 -> Scale: 0.97, 0.96, 1.05

22:43:44.150 -> Offset (Bias): 29.83, 8.45, 6.16
22:43:44.150 -> Scale: 0.99, 1.00, 1.02

22:46:45.904 -> Offset (Bias): 29.61, 8.59, 6.15
22:46:45.904 -> Scale: 0.99, 0.99, 1.02
*/

void setup() {
  indicatorLed.on(COLOR_RED);
  Serial.begin(115200);
  //while (!Serial);// Serial port is not blocking

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Rotate the magnetometer in all directions for calibration...");
  
  calibrate();
  indicatorLed.off();
  printCalibrationData();
}
void calibrate(){
  //fieldRange = calibrateMagFieldRange(calibrationTimeInSeconds*1000/calibrationDelay, calibrationDelay);
  //IMU.setMagneticFieldRange(fieldRange); // Set the magnetometer scale to ±16 Gauss

  indicatorLed.on(COLOR_BLUE);
  calibrateMag(magBias, magScale, softIronMatrix, calibrationTimeInSeconds*1000/calibrationDelay, calibrationDelay, calibrationRadius);
  indicatorLed.off();
}
void printCalibrationData(){
  Serial.println("Calibration results:");
  Serial.print("Field Range: "); Serial.println(fieldRange); 
  Serial.print("Offset (Bias): "); Serial.print(magBias[0]); Serial.print(", "); Serial.print(magBias[1]); Serial.print(", "); Serial.println(magBias[2]);
  Serial.print("Scale: "); Serial.print(magScale[0]); Serial.print(", "); Serial.print(magScale[1]); Serial.print(", "); Serial.println(magScale[2]);
  Serial.println("Soft Iron Matrix:");
  for (int i = 0; i < 3; i++) {
    Serial.print("  "); Serial.print(softIronMatrix[i][0]); Serial.print(", "); Serial.print(softIronMatrix[i][1]); Serial.print(", "); Serial.println(softIronMatrix[i][2]);
  }
}


void loop() {
  float x, y, z, rawHeading;
  printCalibrationData();
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    Serial.print("Raw magnetometer data (uT): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);

    rawHeading = heading(x, y, 0);
    calibrateMagReading(x,y,z,magBias,magScale, softIronMatrix);
    Serial.print("Calibrated magnetometer data (uT): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
    
    Serial.print("Heading north. raw: ");
    Serial.print(rawHeading);
    
    Serial.print("\t calibrated: ");
    Serial.println(heading(x, y, 0));

    Serial.println("=========================================================================================");
  }
  indicatorLed.blink(COLOR_GREEN,1000);
}
