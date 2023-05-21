#ifndef COMPASS_UTILS_H
#define COMPASS_UTILS_H

#include <Arduino.h>
#if BLE_REVISION == 1
  #include <Arduino_LSM9DS1.h>
#else
  #include <Arduino_BMI270_BMM150.h>
#endif


#include <float.h>
#include <math.h>

// Can't compile eigen, so will leave it out of equation for now
//#include <eigen.h>
//#include <Eigen>
//#include <Eigen/Dense>
//using namespace Eigen;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void interpolateCalibration(float dialValue, float (&targetCalibraion)[13], const float (*matrix)[13], int numberOfRows){
  int targetRow = -1;
  int maxRow = 0;

  if(numberOfRows == 1){
    targetRow = 0;
  }
  while(maxRow<numberOfRows && matrix[maxRow][0]<=dialValue){ maxRow++; }
  if(maxRow == 0){  // dialValue = 0 or less than the first calibration
    // return first calibration
    targetRow = 0;
  }
  if(maxRow<numberOfRows && matrix[maxRow][0]==dialValue){ // exact match - special case
    targetRow = maxRow;
  }

  if(targetRow>=0){
  /*  Serial.print("calibration: ");
    Serial.print(targetRow);
    Serial.print("=");
    Serial.print(matrix[targetRow][0], 0);
    Serial.print("\tangle: ");
    Serial.print(matrix[targetRow][0], 0);
    Serial.print("\t");*/
    for(int i=0;i<13;i++){
      targetCalibraion[i] = matrix[targetRow][i];
    }
    return;
  }

  int i1 = maxRow - 1;
  float startAngle = matrix[i1][0];
  int i2 = maxRow;
  float endAngle = 0; 
  if(maxRow == numberOfRows){
    i2 = 0;
    endAngle = 360; // first calibration also works for 360 degree value
  }else{
    endAngle = matrix[i2][0];
  }

  for(int i=0;i<13;i++){
    // here we interpolate between calibrationMatrix[i1] and calibrationMatrix[i2]
    targetCalibraion[i] = mapFloat(dialValue, startAngle, endAngle, matrix[i1][i], matrix[i2][i]);
  }

  targetCalibraion[0] = mapFloat(dialValue, startAngle, endAngle,  startAngle, endAngle);
/*
  Serial.print("calibration: ");
  Serial.print(i2);
  Serial.print("-");
  Serial.print(endAngle, 0);
  Serial.print("\tangle: ");
  Serial.print(targetCalibraion[0], 0);
  Serial.print("\t");
*/
}

void calibrateMagReading(float &x, float &y, float &z,const float *calibration) {

  /*
    calibration - this is a calibration matrix. 13 columns:
      [0] dial angle at which compass was calibrated
      [1] offset X
      [2] offset Y
      [3] offset Z
      [4] - [13] - soft iron matrix row by row
  */

    // Apply bias
    x = (x - calibration[1]);
    y = (y - calibration[2]);
    z = (z - calibration[3]);

    // Apply soft iron calibration and scale
    x = x * calibration[4] + y * calibration[5] + z * calibration[6];
    y = x * calibration[7] + y * calibration[8] + z * calibration[9];
    z = x * calibration[10] + y * calibration[11] + z * calibration[12];
}

void calibrateMagReading(float &x, float &y, float &z,float *magBias, float softIronMatrix[3][3]) {
    // Apply bias
    x = (x - magBias[0]);
    y = (y - magBias[1]);
    z = (z - magBias[2]);

    if(softIronMatrix != NULL){
        // Apply soft iron calibration and scale
        x = x * softIronMatrix[0][0] + y * softIronMatrix[0][1] + z * softIronMatrix[0][2];
        y = x * softIronMatrix[1][0] + y * softIronMatrix[1][1] + z * softIronMatrix[1][2];
        z = x * softIronMatrix[2][0] + y * softIronMatrix[2][1] + z * softIronMatrix[2][2];
    }
}

void calibrateMagReading(float &x, float &y, float &z,float *magBias, float *magScale, float softIronMatrix[3][3]) {
    // Apply bias and scale
    x = (x - magBias[0]) * magScale[0];
    y = (y - magBias[1]) * magScale[1];
    z = (z - magBias[2]) * magScale[2];

    if(softIronMatrix != NULL){
        // Apply soft iron calibration
        x = x * softIronMatrix[0][0] + y * softIronMatrix[0][1] + z * softIronMatrix[0][2];
        y = x * softIronMatrix[1][0] + y * softIronMatrix[1][1] + z * softIronMatrix[1][2];
        z = x * softIronMatrix[2][0] + y * softIronMatrix[2][1] + z * softIronMatrix[2][2];
    }
}


float heading (float x, float y, float declinationAngle){
  float heading = atan2(y, x);
  heading += declinationAngle;
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  return heading * 180 / PI;
}

/**
it's a good idea to set the field range before calibrating other parameters like offset (bias) and scale factors. The calibration process calculates these parameters based on the raw magnetometer data. Since the raw data depends on the selected field range, it's important to choose the appropriate range before performing calibration.

The field range setting determines the sensitivity of the magnetometer and affects the raw output values. If the field range is not set correctly before calibration, the calculated calibration parameters may not provide accurate and consistent results when applied to the magnetometer data.

So the recommended order of operations is:

1. Determine the best field range for your application (e.g., by using the testing approach described earlier).
2. Set the selected field range using IMU.setMagneticFieldRange(rangeInGauss).
3. Perform the calibration process to compute the offset (bias), scale factors, and potentially the soft iron matrix.
4. Apply the calibration parameters to the raw magnetometer data during the normal operation of your application.
*/
float calibrateMagFieldRange(int sampleCount, int delayTime) {
  float maxField = 0;

  for (int i = 0; i < sampleCount; i++) {
    float x, y, z;
    while (!IMU.magneticFieldAvailable());
    IMU.readMagneticField(x, y, z);
    float fieldMagnitude = sqrt(x * x + y * y + z * z);
    maxField = max(maxField, fieldMagnitude);
    if(i*100 % sampleCount == 0){
      Serial.print(i*100 / sampleCount);
      Serial.println("%");
    }
    delay(delayTime);
  }


  if (maxField <= 4000) {
    return 4.0;// Best field range: ±4 Gauss
  } else if (maxField <= 8000) {
    return 8.0;// Best field range: ±8 Gauss
  } else if (maxField <= 12000) {
    return 12.0;// Best field range: ±12 Gauss
  } else {
    return 16.0;// Best field range: ±16 Gauss
  }
}
/**
This function collects magnetometer samples and computes the minimum, maximum, 
and average values for each axis. It calculates the bias as the midpoint between 
the minimum and maximum values and determines the scale factors by comparing 
the range for each axis against the average range.

Please note that this is a simplified calibration process and may not provide accurate results. 
For better calibration, you may consider more advanced techniques like the ellipsoid fitting method. 
The code above also does not include the soft iron matrix calculation, as this requires a more sophisticated algorithm.
*/
void calibrateMag(float *magBias, float *magScale, float softIronMatrix[3][3], int sampleCount, int delayTime, float calibrationRadius) {
  float magMin[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
  float magMax[3] = {FLT_MIN, FLT_MIN, FLT_MIN};
  float magSum[3] = {0, 0, 0};

  for (int i = 0; i < sampleCount; i++) {
    float x, y, z;
    while (!IMU.magneticFieldAvailable());
    IMU.readMagneticField(x, y, z);

    magSum[0] += x;
    magSum[1] += y;
    magSum[2] += z;

    magMin[0] = min(magMin[0], x);
    magMin[1] = min(magMin[1], y);
    magMin[2] = min(magMin[2], z);

    magMax[0] = max(magMax[0], x);
    magMax[1] = max(magMax[1], y);
    magMax[2] = max(magMax[2], z);

    if(i*100 % sampleCount == 0){
      Serial.print(i*100 / sampleCount);
      Serial.print("%, time left: ");
      Serial.print((sampleCount-i)*delayTime/1000);
      Serial.println("s");
    }
    
    delay(delayTime);
  }

  // Calculate bias (offset)
  magBias[0] = (magMax[0] + magMin[0]) / 2.0;
  magBias[1] = (magMax[1] + magMin[1]) / 2.0;
  magBias[2] = (magMax[2] + magMin[2]) / 2.0;

  // Calculate scale
  float magRange[3];
  magRange[0] = magMax[0] - magMin[0];
  magRange[1] = magMax[1] - magMin[1];
  magRange[2] = magMax[2] - magMin[2];

  float avgRange = (magRange[0] + magRange[1] + magRange[2]) / 3.0;

  magScale[0] = avgRange / magRange[0];
  magScale[1] = avgRange / magRange[1];
  magScale[2] = avgRange / magRange[2];

}


/*
void calibrateMag2(float *magBias, float *magScale, float softIronMatrix[3][3], int sampleCount, int delayTime, float calibrationRadius) {
  float magSum[3] = {0, 0, 0};
  MatrixXf samples(sampleCount, 3);

  // Collect magnetometer samples
  for (int i = 0; i < sampleCount; i++) {
    float x, y, z;
    while (!IMU.magneticFieldAvailable());
    IMU.readMagneticField(x, y, z);

    samples(i, 0) = x;
    samples(i, 1) = y;
    samples(i, 2) = z;

    delay(delayTime);
  }

  // Calculate the mean value of the samples
  Vector3f mean = samples.colwise().mean();

  // Center the samples
  samples.rowwise() -= mean.transpose();

  // Compute the covariance matrix
  Matrix3f cov = (samples.adjoint() * samples) / float(sampleCount - 1);

  // Compute the eigenvalues and eigenvectors of the covariance matrix
  SelfAdjointEigenSolver<Matrix3f> eigensolver(cov);
  Matrix3f eigenvectors = eigensolver.eigenvectors();
  Vector3f eigenvalues = eigensolver.eigenvalues();

  // Find the scaling factors
  magScale[0] = calibrationRadius / sqrt(eigenvalues(0));
  magScale[1] = calibrationRadius / sqrt(eigenvalues(1));
  magScale[2] = calibrationRadius / sqrt(eigenvalues(2));

  // Calculate the soft iron correction matrix
  Matrix3f softIronMat = eigenvectors * Matrix3f::Identity();
  softIronMat(0, 0) *= magScale[0];
  softIronMat(1, 1) *= magScale[1];
  softIronMat(2, 2) *= magScale[2];
  softIronMat = eigenvectors.transpose() * softIronMat;

  // Copy the soft iron matrix to the output array
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      softIronMatrix[i][j] = softIronMat(i, j);
    }
  }

  // Calculate the bias
  Vector3f bias = -1.0 * softIronMat * mean;
  magBias[0] = bias(0);
  magBias[1] = bias(1);
  magBias[2] = bias(2);
}
*/

#endif // COMPASS_UTILS_H