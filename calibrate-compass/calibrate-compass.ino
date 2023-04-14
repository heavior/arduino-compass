#include <Arduino_LSM9DS1.h>
#include "compass_utils.h"

constexpr int calibrationSampleCount = 2000;
constexpr int calibrationDelay = 50;
constexpr float calibrationRadius = 0.1;

float magBias[3] = {0, 0, 0};
float magScale[3] = {1, 1, 1};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.println("Rotate the magnetometer in all directions for calibration...");
  

  float fieldRange = calibrateMagFieldRange(calibrationSampleCount, calibrationDelay);
  Serial.print("Field Range: "); Serial.println(fieldRange); 
  //IMU.setMagneticFieldRange(fieldRange); // Set the magnetometer scale to ±16 Gauss

  calibrateMag(magBias, magScale, softIronMatrix, calibrationSampleCount, calibrationDelay, calibrationRadius);

  Serial.println("Calibration complete:");
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

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);

    calibrateMagReading(x,y,z,magBias,magScale, NULL);
    Serial.print("Magnetometer (uT): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);
  }

  delay(100);
}
