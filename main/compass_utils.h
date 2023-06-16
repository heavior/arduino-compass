#ifndef COMPASS_UTILS_H
#define COMPASS_UTILS_H

#include <Arduino.h>

#include <float.h>
#include <math.h>

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void closestCalibration(float encoderValue, float (&targetCalibraion)[13], const float (*matrix)[13], int numberOfRows){
  int targetRow = -1;
  int maxRow = 0;

  if(numberOfRows == 1){
    targetRow = 0;
  }
  while(maxRow<numberOfRows && matrix[maxRow][0]<=encoderValue){ 
    maxRow++; 
  }
  if(maxRow == 0){  // encoderValue = 0 or less than the first calibration
    // return first calibration
    targetRow = 0;
  }
  if(maxRow<numberOfRows && matrix[maxRow][0]==encoderValue){ // exact match - special case
    targetRow = maxRow;
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

  if(encoderValue - startAngle > endAngle - encoderValue){
    targetRow = i2;
  }else{
    targetRow = i1;
  }

  for(int i=0;i<13;i++){
    targetCalibraion[i] = matrix[targetRow][i];
  }
}

void interpolateCalibration(float encoderValue, float (&targetCalibraion)[13], const float (*matrix)[13], int numberOfRows){
  int targetRow = -1;
  int maxRow = 0;

  if(numberOfRows == 1){
    targetRow = 0;
  }
  while(maxRow<numberOfRows && matrix[maxRow][0]<=encoderValue){ maxRow++; }
  if(maxRow == 0){  // encoderValue = 0 or less than the first calibration
    // return first calibration
    targetRow = 0;
  }
  if(maxRow<numberOfRows && matrix[maxRow][0]==encoderValue){ // exact match - special case
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
    targetCalibraion[i] = mapFloat(encoderValue, startAngle, endAngle, matrix[i1][i], matrix[i2][i]);
  }

  //targetCalibraion[0] = mapFloat(encoderValue, startAngle, endAngle,  startAngle, endAngle);
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

#endif // COMPASS_UTILS_H