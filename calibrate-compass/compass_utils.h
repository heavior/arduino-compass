#ifndef COMPASS_UTILS_H
#define COMPASS_UTILS_H

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>

#include <float.h>

// Can't compile eigen, so will leave it out of equation for now
//#include <eigen.h>
//#include <Eigen>
//#include <Eigen/Dense>
//using namespace Eigen;


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
      Serial.println("%");
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