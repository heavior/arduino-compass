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

#define MEASURE_INTERVAL      50     // RPM Measurement interval in milliseconds
#define MEASURE_TIME_ROUGH    500    // Total time for RPM measurement in milliseconds
#define MEASURE_TIME_PRECISE  1000   // Longer measurement for precision
#define MOTOR_REACTION_TIME   500    // How much time do we give motor to slow down or speed up
#define RECALIBRATE_STEP      10
#define MAX_CALIBRATION_ATTEMPTS  5
#define TRICKLE_DELAY         50     // how long is motor boost
#define TRY_TRICKLE           false

// TODO: find later what works
#define MAX_RPM   120  // 120 // 2 dial turns per second = 720 degrees per second
#define MIN_RPM   1   // .5/60 dial turns per second = 3 degrees per second

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
    void calibrate();


    float currentPosition();

  private:
    void sendSpeedCommand(int speed, bool trickle);
    void sendSpeedCommand(int speed);
    int calibrateForSpeed(int minSpeed, int maxSpeed, float minRPM, float maxRPM, float targetRPM, bool precise);
    
    void recalibrateMinSpeeds();
    float measureRPM(int speed, bool precise);
    float measureRPM(bool precise);

    int lastCompensationAngle;
    unsigned long lastCalibrationTime;
    int spinDirection;

    int minSpeedForward;
    int maxSpeedForward;
    int minSpeedBackward;
    int maxSpeedBackward;
    int encoderPin;
    int latestSpeed;

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
  // TODO: check initial calibration
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

  recalibrateMinSpeeds();

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
  float confirmRPM = abs(measureRPM(minSpeedForward, true));
  while(confirmRPM < MIN_RPM){     // trying calibration again
    Serial.println("incorrect minSpeedForward calibration ");
    minSpeedForward = calibrateForSpeed(minSpeedForward, maxSpeedForward, 0, MAX_RPM, MIN_RPM, true); 
    confirmRPM = abs(measureRPM(minSpeedForward, true));
  }

  confirmRPM = abs(measureRPM(minSpeedBackward, true));
  if(confirmRPM<MIN_RPM){
    Serial.println("incorrect minSpeedBackward calibration ");
    minSpeedBackward = calibrateForSpeed(minSpeedBackward, maxSpeedBackward, 0, -MAX_RPM, -MIN_RPM, true); 
    // trying calibration once again
  }
}

void Motor::sendSpeedCommand(int speed, bool trickle){
  #if TRY_TRICKLE
    if(trickle && latestSpeed!=speed){ // short boost 
      int trickleSpeed = speed*maxSpeedForward > 0 ? maxSpeedForward:maxSpeedBackward;
      sendSpeedCommand(trickleSpeed);
      Serial.print("trickle " );
      Serial.print(trickleSpeed);
      Serial.print("for" );
      Serial.println(speed);
      
      delay(TRICKLE_DELAY);
    }
  #endif
  
  latestSpeed = speed;
  sendSpeedCommand(speed);
  
}

float roundToLowestAbsoluteValue(float number) {
    return  round(fabs(number)) * (number >= 0 ? 1 : -1);
}
int Motor::calibrateForSpeed(int startSpeed, int endSpeed, float startRPM, float endRPM, float targetRPM, bool precise){
  while(abs(startSpeed-endSpeed)>1){
    int currentSpeed = (startSpeed+endSpeed)/2;
    float currentRPM = measureRPM(currentSpeed, precise);
    if(!precise){ // Rounding value when looking for max RPM to have a
      currentRPM = roundToLowestAbsoluteValue(currentRPM);
    }

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

  int result = abs(endRPM)>=abs(startRPM)?endSpeed:startSpeed;
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
  sendSpeedCommand(speed, true);
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

//  Serial.print("dial position: ");
//  Serial.print(angle);


/*  Serial.print("encoder value:");
  Serial.print(read);
  Serial.print(" angle:");
  Serial.println(angle);*/

  return angle;
}

float lpFilter(float value, float oldValue, float alp){
  return oldValue*(1.f-alp)+ alp*value;
}



int Motor::mapSpeed(int compensationAngle){
  if(compensationAngle == 0){
    return ZERO_SPEED;
  }

//Serial.print(" map speed: ");
//Serial.print(compensationAngle);
  compensationAngle = -compensationAngle; // our goal is to reduce compensation angle to zero, so we are going to proved speed opposite to the angle

  int speed = ZERO_SPEED;
  if(compensationAngle > 0){
    speed = map(compensationAngle, 1, 180, minSpeedForward, maxSpeedForward); 
  }else if(compensationAngle < 0){
    speed = map(-compensationAngle, 1, 180, minSpeedBackward, maxSpeedBackward); 
  }

//Serial.print(" to: ");
//Serial.println(speed);

  return speed;
}

int Motor::setSpeed(int compensationAngle) {
  int speed = mapSpeed(compensationAngle);
  sendSpeedCommand (speed, true);
  return speed;
}

void Motor::stopMotor(){
  // TODO: instead of just waiting - confirm through encoder readings, might be faster
  sendSpeedCommand(ZERO_SPEED, false);
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
    //trickle(speed);
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
          Serial.println("Motor stuck ");
          recalibrateMinSpeeds();
          lastCompensationAngle = 0; // skip next calibration
          lastCalibrationTime = 0;
        }
        if(lastCompensationAngle * compensationAngle<0){
          Serial.println("MOTOR TOO FAST!");
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
