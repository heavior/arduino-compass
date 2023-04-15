#include <Arduino_LSM9DS1.h>
#include "compass_utils.h"
#include "led_indicators.h"

LED indicatorLed(LED_BUILTIN);
constexpr int calibrationSampleCount = 2000;
constexpr int calibrationDelay = 50;
constexpr float calibrationRadius = 0.1;

float fieldRange = 4.0; 
// note: 
// 1) some magnetometers use gain instead of field range 
// 2) some magnetometers don't have the setting at all
// so, check out if you need it

// When calibrating, the LED will be blue
// Once completed, it will blink green
// when green is blinking it will print calibration settings, oringial and calibrated readings from the magnetometer

float magBias[3] = {0, 0, 0};
float magScale[3] = {1, 1, 1};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

/*

19:31:37.541 -> Field Range: 4.00
19:31:37.541 -> Offset (Bias): 25.44, 1.61, 8.97
19:31:37.541 -> Scale: 0.96, 1.01, 1.03



19:33:51.577 -> Field Range: 4.00
19:33:51.577 -> Offset (Bias): 26.46, 2.41, 9.68
19:33:51.577 -> Scale: 1.01, 0.96, 1.03


19:36:03.012 -> Field Range: 4.00
19:36:03.012 -> Offset (Bias): 26.37, 0.88, 9.25
19:36:03.012 -> Scale: 0.99, 0.98, 1.03


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
  //fieldRange = calibrateMagFieldRange(calibrationSampleCount, calibrationDelay);
  //IMU.setMagneticFieldRange(fieldRange); // Set the magnetometer scale to ±16 Gauss

  indicatorLed.on(COLOR_BLUE);
  calibrateMag(magBias, magScale, softIronMatrix, calibrationSampleCount, calibrationDelay, calibrationRadius);
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
  float x, y, z;
  printCalibrationData();
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    Serial.print("Raw magnetometer data (uT): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);

    Serial.print("Raw north direction (degrees)");
    Serial.println("TODO");


    calibrateMagReading(x,y,z,magBias,magScale, NULL);
    Serial.print("Calibrated magnetometer data (uT): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
    Serial.print("Calibrated magnetometer data (uT): ");

    Serial.print("Calibrated north direction (degrees)");
    Serial.println("TODO");
  }
  indicatorLed.blink(COLOR_GREEN,500);
}
