#ifndef Motor_h
#define Motor_h

#include "sparrow_music.h"


#define DEBUG_MOTOR_CALIBRATION true


#if USE_SERVO
  #include <Servo.h>
  #define ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
  #define MAX_SPEED     90    // 30 is prev value, max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
  #define MIN_SPEED     0     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
#else
  #define ZERO_SPEED    0    
  #define MAX_SPEED     255  
  #define MIN_SPEED     0 
#endif

#define MIN_CALIBRATION_TIME  500 // need at least half a second to make a calibration decision
#define CALIBRATION_INCREASE  1 // need at least half a second to make a calibration decision
#define CALIBRATION_DECREASE  4 // need at least half a second to make a calibration decision

#define MEASURE_INTERVAL      50     // RPM Measurement interval in milliseconds
#define MEASURE_TIME_ROUGH    500    // Total time for RPM measurement in milliseconds
#define MEASURE_TIME_PRECISE  1000 // Longer measurement for precision
#define MOTOR_REACTION_TIME   300 // How much time do we give motor to slow down or speed up
#define RECALIBRATE_STEP      10
#define MAX_CALIBRATION_ATTEMPTS  5

// TODO: find later what works
#define MAX_RPM   120  // 120 // 2 dial turns per second = 720 degrees per second
#define MIN_RPM   .5   // .5/60 dial turns per second = 3 degrees per second

class Motor {
  public:

#if USE_SERVO
    Motor(int encPin, int servoPin);
#else
    Motor(int encPin, int pin1, int pin2, int pinPower);
#endif

    void setInitialCalibration(int minSpeedForward,int maxSpeedForward,int minSpeedBackward,int maxSpeedBackward);
    void sleep();
    void wakeUp();
    int setSpeed(int value);
    void stopMotor();
    int setSpeed(int compensationAngle, bool calibrate);
    void playTheme(bool (*interrupt)());
    int mapSpeed(int compensationAngle);
    float currentPosition();
    void calibrate();
    void calibrate2();

  private:
    void sendSpeedCommand(int speed);
    int calibrateForSpeed(int minSpeed, int maxSpeed, float minRPM, float maxRPM, float targetRPM, bool precise);
    int calibrateForSpeed2(int minSpeed, int maxSpeed, float minRPM, float maxRPM, float targetRPM, bool precise);
    
    int recalibrate(int initialSpeed, float targetRPM);
    void recalibrateMinSpeeds();
    float measureRPM(int speed, bool precise);
    float measureRPM(bool precise);

    int lastCompensationAngle;
    unsigned long lastCalibrationTime;
    int spinDirection;

    int minSpeedForward = ZERO_SPEED + MIN_SPEED;
    int maxSpeedForward = ZERO_SPEED + MAX_SPEED;
    int minSpeedBackward = ZERO_SPEED - MIN_SPEED;
    int maxSpeedBackward = ZERO_SPEED - MAX_SPEED;
    bool reverseDirection = false;
    int encoderPin;

#if USE_SERVO
    int servoPin;
    Servo motorServo;
#else
    int motorPin1;
    int motorPin2;
    int motorPinPower;
#endif

};

void Motor::setInitialCalibration(int minSpeedForward,int maxSpeedForward,int minSpeedBackward,int maxSpeedBackward){
  this->minSpeedForward = minSpeedForward;
  this->maxSpeedForward = maxSpeedForward;
  this->minSpeedBackward = minSpeedBackward;
  this->maxSpeedBackward = maxSpeedBackward;
}

void Motor::calibrate(){
  int startSpeed = ZERO_SPEED + MAX_SPEED;
  int endSpeed = ZERO_SPEED - MAX_SPEED;
  float startRPM = measureRPM(startSpeed, true); 
  float endRPM = measureRPM(endSpeed, true); 

  // TODO: optimise each following calibration by using previous values

  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating max forward speed");
  #endif
  maxSpeedForward = calibrateForSpeed(startSpeed, endSpeed, startRPM, endRPM, MAX_RPM, false); // do not use precise calibration - not afraid of stall, and don't need accuracy
  
  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating max backward speed");
  #endif
  maxSpeedBackward = calibrateForSpeed(startSpeed, endSpeed, startRPM, endRPM, -MAX_RPM, false); // do not use precise calibration - not afraid of stall, and don't need accuracy

  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating min forward speed");
  #endif
  minSpeedForward = calibrateForSpeed(startSpeed, endSpeed, startRPM, endRPM, MIN_RPM, true); // use precise calibration - afraid of stall, need accuracy, so stop between all measures

  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating min backward speed");
  #endif
  minSpeedBackward = calibrateForSpeed(startSpeed, endSpeed, startRPM, endRPM, -MIN_RPM, true); // use precise calibration - afraid of stall, need accuracy, so stop between all measures

  #if DEBUG_MOTOR_CALIBRATION
    Serial.print("calibrated motor. min forward: ");
    Serial.print(minSpeedForward);
    Serial.print(" max forward: ");
    Serial.print(maxSpeedForward);
    Serial.print(" min backward: ");
    Serial.print(minSpeedBackward);
    Serial.print(" max backward: ");
    Serial.println(maxSpeedBackward);
  #endif

}
void Motor::calibrate2(){
  // calibration logic:
  // 1. We have target min and max rpm for both directions
  // 2. Do calibration for foward and backward - min and max (4 calibrations total):
  // 3. This logic allows working with poorly calibrated servo motors
  int minSpeed = ZERO_SPEED + MIN_SPEED;
  int maxSpeed = ZERO_SPEED + MAX_SPEED;
  float minRPM = 0; //measureRPM(minSpeed, false);  // just assume stalling at min speed, not so important for max speed calculation
  float maxRPM = measureRPM(maxSpeed, false); 

  reverseDirection = (maxRPM < 0);
  float sign = reverseDirection?-1:1; 

  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating max forward speed");
  #endif
  maxSpeedForward = calibrateForSpeed(minSpeed, maxSpeed, minRPM, maxRPM, sign*MAX_RPM, false); // do not use precise calibration - not afraid of stall, and don't need accuracy
  minSpeedForward = calibrateForSpeed(minSpeed, maxSpeed, minRPM, maxRPM, sign*MIN_RPM, true); // do not use precise calibration - not afraid of stall, and don't need accuracy

  minSpeed = ZERO_SPEED - MIN_SPEED;
  maxSpeed = ZERO_SPEED - MAX_SPEED;
  minRPM = 0;// measureRPM(minSpeed, false); // just assume stalling at min speed, not so important for max speed calculation
  maxRPM = measureRPM(maxSpeed, false); 

  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating max backward speed");
  #endif
  maxSpeedBackward = calibrateForSpeed(maxSpeed, minSpeed, maxRPM, minRPM,  -sign*MAX_RPM, false); // do not use precise calibration - not afraid of stall, and don't need accuracy
  minSpeedBackward = calibrateForSpeed(maxSpeed, minSpeed, maxRPM, minRPM,  -sign*MIN_RPM, true); // do not use precise calibration - not afraid of stall, and don't need accuracy;

  if(reverseDirection){
    #if DEBUG_MOTOR_CALIBRATION
      Serial.println("reversed direction motor, swapping boundaries ");
    #endif
    int temp;
    
    temp = minSpeedBackward;
    minSpeedBackward = minSpeedForward;
    minSpeedForward = temp;

    temp = maxSpeedBackward;
    maxSpeedBackward = maxSpeedForward;
    maxSpeedForward = temp;
  }

/*
  #if DEBUG_MOTOR_CALIBRATION
    Serial.println("Calibrating minimal speed for accuracy");
  #endif
  recalibrateMinSpeeds();
*/
  #if DEBUG_MOTOR_CALIBRATION
    Serial.print("calibrated motor. min forward: ");
    Serial.print(minSpeedForward);
    Serial.print(" max forward: ");
    Serial.print(maxSpeedForward);
    Serial.print(" min backward: ");
    Serial.print(minSpeedBackward);
    Serial.print(" max backward: ");
    Serial.println(maxSpeedBackward);
  #endif
}
void Motor::recalibrateMinSpeeds(){
  float sign = 1;//reverseDirection?-1:1; 
  minSpeedForward = recalibrate(minSpeedForward, sign*MIN_RPM);
  minSpeedBackward = recalibrate(minSpeedBackward, -sign*MIN_RPM);
}
int Motor::recalibrate(int speed, float targetRPM){

  #if DEBUG_MOTOR_CALIBRATION
    Serial.print("recalibrate speed:");
    Serial.print(speed);

    Serial.print(" targetRPM:");
    Serial.println(targetRPM);
  #endif

  float sign = (reverseDirection?-1:1); // if direction is reversed, then reducing speed will lead to increased rpm, so need to change the sifn for recalibration step
  float minLimitSpeed, maxLimitSpeed;
  if(!reverseDirection){
    minLimitSpeed = (targetRPM>0)? ZERO_SPEED: (ZERO_SPEED - MAX_SPEED);
    maxLimitSpeed = (targetRPM>0)? (ZERO_SPEED + MAX_SPEED): ZERO_SPEED;
  }else{
    minLimitSpeed = (targetRPM>0)? (ZERO_SPEED + MAX_SPEED): ZERO_SPEED;
    maxLimitSpeed = (targetRPM>0)? ZERO_SPEED: (ZERO_SPEED - MAX_SPEED);
  }

 
// Motor could be stuck, need precise calibration based on old value  
  for(int attempt = 0;attempt< MAX_CALIBRATION_ATTEMPTS;attempt++){
    #if DEBUG_MOTOR_CALIBRATION
      Serial.println("recalibrate - measure boundaries. min:");
    #endif
    int minSpeed = speed;
    float minRPM;
    do { 
      minSpeed -= sign*RECALIBRATE_STEP;
      minRPM = measureRPM(minSpeed, true);

      Serial.print(minSpeed);
      Serial.print("-");
      Serial.print(minRPM);
      Serial.print(", ");
    } while( minRPM > targetRPM && minSpeed > minLimitSpeed);     // stepping to expand boundaries
    if(minSpeed < minLimitSpeed){
      minSpeed = minLimitSpeed;
    }

    #if DEBUG_MOTOR_CALIBRATION
      Serial.println("recalibrate - measure boundaries. max:");
    #endif
    int maxSpeed = speed; 
    float maxRPM;
    do {
      maxSpeed += sign*RECALIBRATE_STEP;
      maxRPM = measureRPM(maxSpeed, true);

      Serial.print(maxSpeed);
      Serial.print("-");
      Serial.print(maxRPM);
      Serial.print(", ");
    } while( maxRPM < targetRPM && maxSpeed > maxLimitSpeed);  // stepping to expand boundaries
     if(maxSpeed > maxLimitSpeed){
      maxSpeed = maxLimitSpeed;
    }

    #if DEBUG_MOTOR_CALIBRATION
      Serial.println("calibrating:");
    #endif
    speed = calibrateForSpeed(minSpeed - RECALIBRATE_STEP, maxSpeed + RECALIBRATE_STEP, minRPM, maxRPM, targetRPM, true); // presice calibration now
    // run presize calibration
    float finalRPM = measureRPM(speed, true);
    if(finalRPM < targetRPM){
      #if DEBUG_MOTOR_CALIBRATION
        Serial.print("RECALIBRATION ");
        Serial.print(attempt);
        Serial.print(" FAILED @ ");
        Serial.print(speed);
        Serial.print(" rpm: ");
        Serial.print(finalRPM);
        Serial.print(" vs target: ");
        Serial.println(targetRPM);
      #endif
    }else{
        Serial.print("RECALIBRATION ");
        Serial.print(attempt);
        Serial.print(" SUCCESS @ ");
        Serial.print(speed);
        Serial.print(" rpm: ");
        Serial.print(finalRPM);
        Serial.print(" vs target: ");
        Serial.println(targetRPM);
      return speed;
    }
  }
}


int Motor::calibrateForSpeed(int startSpeed, int endSpeed, float startRPM, float endRPM, float targetRPM, bool precise){
  while(abs(startSpeed-endSpeed)>1){
    int currentSpeed = (startSpeed+endSpeed)/2;
    float currentRPM = measureRPM(currentSpeed, precise);

    #if DEBUG_MOTOR_CALIBRATION
      Serial.print("calibrateForSpeed. startSpeed: ");
      Serial.print(startSpeed);
      Serial.print(" endSpeed: ");
      Serial.print(endSpeed);

      Serial.print(" startRPM: ");
      Serial.print(startRPM);

      Serial.print(" endRPM: ");
      Serial.print(endRPM);

      Serial.print(" targetRPM: ");
      Serial.print(targetRPM);

      Serial.print(" currentSpeed: ");
      Serial.print(currentSpeed);

      Serial.print(" currentRPM: ");
      Serial.println(currentRPM);
    #endif

    // Now, we have startRPM , currentRPM, endRMP and need to see where targetRPM falls
    float currentProportion = (currentRPM-startRPM)/(endRPM-startRPM);
    if(currentProportion >= 1){
      // currentrRPM is outside of the end - moving the end
      endRPM = currentRPM;
      endSpeed = currentSpeed;
    }else if(currentProportion <= 0){
      // currentrRPM is behind the start - moving the start
      startRPM = currentRPM;
      startSpeed = currentSpeed;
    }else if( (targetRPM-startRPM)/(endRPM-startRPM) > currentProportion ){
      //target is between current and end
      startRPM = currentRPM;
      startSpeed = currentSpeed;
    }else{
      //target is between start and current
      endRPM = currentRPM;
      endSpeed = currentSpeed;
    }

  }

  int result = abs(endRPM)>abs(startRPM)?endSpeed:startSpeed;
  #if DEBUG_MOTOR_CALIBRATION
    Serial.print("calibration result - speed: ");
    Serial.println(result);
  #endif
  // if we got here, our interval got too small, so return upper boundary of this interval, is it is more likely to have some movement
  return result;
}

int Motor::calibrateForSpeed2(int minSpeed, int maxSpeed, float minRPM, float maxRPM, float targetRPM, bool precise){
  float sign = (minRPM > maxRPM)?-1:0;
  while(maxSpeed - minSpeed > 1){
    // find new target point - proportionally
    int currentSpeed = 0;
    currentSpeed = (minSpeed + maxSpeed)/2;

    if(precise){
      stopMotor();
    }
    float currentRPM = measureRPM(currentSpeed, precise);
    #if DEBUG_MOTOR_CALIBRATION
      Serial.print("calibrateForSpeed. minSpeed: ");
      Serial.print(minSpeed);
      Serial.print(" maxSpeed: ");
      Serial.print(maxSpeed);

      Serial.print(" minRPM: ");
      Serial.print(minRPM);

      Serial.print(" maxRPM: ");
      Serial.print(maxRPM);

      Serial.print(" targetRPM: ");
      Serial.print(targetRPM);

      Serial.print(" currentSpeed: ");
      Serial.print(currentSpeed);

      Serial.print(" currentRPM: ");
      Serial.print(currentRPM);
    #endif


    /*if(currentRPM == targetRPM){
      // exact match, no need to dig deeper
      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" MATCH!");
      #endif
        
      return currentSpeed;
    }*/
    if(currentRPM < minRPM){
      minSpeed = currentSpeed;
      minRPM = currentRPM;
      
      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" CORRECTING MIN RPM");
      #endif
    }else if(currentRPM > maxRPM){
      maxRPM = currentRPM;
      maxSpeed = currentSpeed;
      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" CORRECTING MAX RPM");
      #endif
    }else if(abs(currentRPM - minRPM) < 1){ // flat line on the left
      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" FLAT LEFT!");
      #endif
      minSpeed = currentSpeed;
      minRPM = currentRPM;
    }else if(abs(maxRPM - currentRPM) < 1){ // flat line on the right
      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" FLAT RIGHT!");
      #endif
      maxSpeed = currentSpeed; // shifting right boundary
      maxRPM = currentRPM;
    }else if(maxSpeed*currentRPM > maxSpeed*targetRPM){

      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" MOVING RIGHT BOUNDARY!");
      #endif

      // actually moving right boundary
      maxSpeed = currentSpeed;
      maxRPM = currentRPM;
    }
    else if(maxSpeed*currentRPM < maxSpeed*targetRPM){
      #if DEBUG_MOTOR_CALIBRATION
        Serial.println(" MOVING LEFT BOUNDARY!");
      #endif

      // actually moving left boundary
      minSpeed = currentSpeed;
      minRPM = currentRPM;
    } 
  }

  int result = abs(maxRPM)>abs(minRPM)?maxSpeed:minSpeed;
  #if DEBUG_MOTOR_CALIBRATION
    Serial.print("calibration result - speed: ");
    Serial.println(result);
  #endif
  // if we got here, our interval got too small, so return upper boundary of this interval, is it is more likely to have some movement
  return result;
}
float Motor::measureRPM(int speed, bool precise){
  if(precise){
    stopMotor();
  }
  sendSpeedCommand(speed);
  float result = measureRPM(precise);
  return result;
}


float Motor::measureRPM(bool precise){
  unsigned long lastMillis;
  unsigned long startMillis;
  int lastPosition = 0;
  int revolutions = 0;
  int totalAngleChange = 0;

  delay(MOTOR_REACTION_TIME); // giving it time to accelerate
  int startPosition = currentPosition();
  lastPosition = startPosition;
  lastMillis = millis();
  startMillis = lastMillis;
  delay(MEASURE_INTERVAL);

  int measureTime = precise?MEASURE_TIME_PRECISE:MEASURE_TIME_ROUGH;

  while(lastMillis - startMillis < measureTime){
    int currPosition = currentPosition();
    int delta = currPosition - lastPosition;
    /*Serial.print("position: ");
    Serial.print(currPosition);
    Serial.print(" delta: ");
    Serial.print(delta);*/
    // If the change in position is more than 180 degrees, the rotation is likely in the other direction
    if(delta > 180) {
      delta -= 360;
      revolutions--;
    } else if(delta < -180) {
      delta += 360;
      revolutions++;
    }
    /*Serial.print("->");
    Serial.print(delta);

    Serial.print(" revolutions:");
    Serial.print(revolutions);*/

    totalAngleChange += delta;

    /*Serial.print(" angle:");
    Serial.println(totalAngleChange);*/


    /*if(currPosition < lastPosition) { // Assuming a complete revolution
      revolutions++;
    }*/
    lastPosition = currPosition;
    lastMillis = millis();
    delay(MEASURE_INTERVAL);
  }

  // After the 0.5 second time window, calculate and print RPM, then stop the program
  
  //totalAngleChange += revolutions * 360; // include complete revolutions into totalAngleChange
  double rpm = (totalAngleChange / 360.0) * (60 * 1000 / (float)(lastMillis - startMillis));
  return rpm;

}


// float alpha = 0.95; // filtration for potentiometer
//float encoderMin=10000;
//float encoderMax=-1;
float Motor::currentPosition(){ // reading encoder and returning angle
  //static float oldAngle = 0;
  int read = analogRead(ENCODER_PIN);

/*  if(read < encoderMin){
    encoderMin = read;
  }

  if(read > encoderMax){
    encoderMax = read;
  }*/

  float angle = mapFloat(read, 0, 1023, 0, 359);

  if(angle < 0 || angle >= 360){
    angle = 0;
  }

  /*if(-10 < angle - oldAngle && angle - oldAngle < 10){ // Avoiding lp filter between 360 and 0 which will produce some random outcome
    angle = lpFilter(angle, oldAngle, alpha);   // filter values, but this introduces some inertia to the system
  }
  oldAngle = angle;*/
  return angle;
}

float lpFilter(float value, float oldValue, float alp){
  return oldValue*(1.f-alp)+ alp*value;
}



int Motor::mapSpeed(int compensationAngle){
  if(compensationAngle == 0){
    return ZERO_SPEED;
  }

  compensationAngle = -compensationAngle; // our goal is to reduce compensation angle to zero, so we are going to proved speed opposite to the angle

  int speed = ZERO_SPEED;
  if(compensationAngle > 0){
    speed = map(compensationAngle, 1, 180, minSpeedForward, maxSpeedForward); 
  }else if(compensationAngle < 0){
    speed = map(-compensationAngle, 1, 180, minSpeedBackward, maxSpeedBackward); 
  }
  return speed;
}

int Motor::setSpeed(int compensationAngle) {
  int speed = mapSpeed(compensationAngle);
  sendSpeedCommand (speed);
  return speed;
}

void Motor::stopMotor(){
  // TODO: instead of just waiting - confirm through encoder readings, might be faster
  sendSpeedCommand(ZERO_SPEED);
  delay(MOTOR_REACTION_TIME);
}


#if USE_SERVO
  Motor::Motor(int encPin, int servPin) {
    encoderPin = encPin;
    servoPin = servPin;
  }

  void Motor::sleep() {
    setSpeed(0);
    motorServo.detach();
  }

  void Motor::wakeUp() {
    motorServo.attach(servoPin);
  }

  
  void Motor::sendSpeedCommand(int speed) {
    motorServo.write(speed);
  }
 
  int Motor::setSpeed(int compensationAngle, bool calibrate) {
    if(calibrate){
      Serial.println("calibration is not implemented for Servo");
      // TODO: if we ever go back to Servo again - Servo might be not calibrated around the center, so would be nice to implement it
    }
    return setSpeed(compensationAngle);
  }

  void Motor::playTheme(bool (*interrupt)()){
    playSparrowTheme(servoPin, servoPin, interrupt);
  }

#else
  Motor::Motor(int encPin, int pin1, int pin2, int pinPower){
    encoderPin = encPin;
    motorPin1 = pin1;
    motorPin2 = pin2;
    motorPinPower = pinPower;
  }

  void Motor::sleep() {
    digitalWrite(motorPinPower, LOW);
  }

  void Motor::wakeUp() {

    pinMode(motorPin1,OUTPUT);
    pinMode(motorPin2,OUTPUT);
    pinMode(motorPinPower,OUTPUT);
    digitalWrite(motorPinPower, HIGH);
  }


  void Motor::sendSpeedCommand(int speed) {
    if (speed > 0) {
      analogWrite(motorPin2, 0);
      analogWrite(motorPin1, speed);
    } else if (speed < 0) {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, -speed);
    } else {
      analogWrite(motorPin1, 0);
      analogWrite(motorPin2, 0);
    }
  }

  int Motor::setSpeed(int compensationAngle, bool calibrate) {
    // TODO: add clock here maybe
    if(calibrate){
      // TODO: add clock here maybe
      bool canMakeCalibrationCorrection = lastCalibrationTime && compensationAngle && lastCompensationAngle && (millis()  - lastCalibrationTime > MIN_CALIBRATION_TIME);
      if(canMakeCalibrationCorrection){ // Check that we have data to start calibration
        /*int sign = compensationAngle > 0?1:-1;
        if(sign*lastCompensationAngle < sign*compensationAngle){
          // wrong direction
          // TODO: maybe we should not try to detect direction
          Serial.print("change direction");
          direction = -direction;
        }*/
        if(lastCompensationAngle == compensationAngle){ 
          // no changes in the angle - maybe increase min speed
          // TODO: when to decrease min speed?
          // TODO: this should be visible over BT
          // minSpeed+=CALIBRATION_INCREASE;
          Serial.println("Motor stuck ");
          recalibrateMinSpeeds();
          /*Serial.print("Motor calibration ");
          Serial.print(compensationAngle);
          Serial.print(" - increase min speed: ");
          Serial.println(minSpeed);*/
          lastCompensationAngle = 0; // skip next calibration
          lastCalibrationTime = 0;
        }
        if(lastCompensationAngle * compensationAngle<0){
          Serial.println("MOTOR TOO FAST!");
          /*minSpeed-=CALIBRATION_DECREASE;
          Serial.print("Motor calibration ");
          Serial.print(compensationAngle);
          Serial.print(" - decrease min speed: ");
          Serial.println(minSpeed);
          */
          lastCompensationAngle = 0; // skip next calibration
          lastCalibrationTime = 0;
        }
        if(lastCompensationAngle){
          lastCompensationAngle = compensationAngle;
        }
      }else{
        if(!lastCalibrationTime){
          lastCalibrationTime = millis();
        }
        lastCompensationAngle = compensationAngle;
      }
    }else{
      lastCompensationAngle = 0; // no calibration - forget old angle value
      lastCalibrationTime = 0;
    }
    int speed = setSpeed(compensationAngle);
    return speed;
  }

  void Motor::playTheme(bool (*interrupt)()){
    setSpeed(0);
    // Back and forth didn't work very well
    playSparrowTheme(motorPin1, motorPin1, interrupt);
  }

#endif

#endif
