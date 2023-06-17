#ifndef Motor_h
#define Motor_h

#include "sparrow_music.h"

#if USE_SERVO
  #include <Servo.h>
  #define ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
  #define MAX_SPEED     20    // 30 is prev value, max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
  #define MIN_SPEED     3     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
#else
  #define ZERO_SPEED    0    
  #define MAX_SPEED     255  
  #define MIN_SPEED     32 
#endif

#define MIN_CALIBRATION_TIME  500 // need at least half a second to make a calibration decision
#define CALIBRATION_INCREASE  1 // need at least half a second to make a calibration decision
#define CALIBRATION_DECREASE  4 // need at least half a second to make a calibration decision


class Motor {
  public:

#if USE_SERVO
    Motor(int pin);
#else
    Motor(int pin1, int pin2, int pinPower);
#endif

    void sleep();
    void wakeUp();
    int setSpeed(int value);
    int setSpeed(int compensationAngle, bool calibrate);
    void playTheme(bool (*interrupt)());
    int mapSpeed(int compensationAngle);

  private:
    int lastCompensationAngle;
    unsigned long lastCalibrationTime;
    int spinDirection;
    int minSpeed = MIN_SPEED;
#if USE_SERVO
    int motorPin;
    Servo motorServo;
#else
    int motorPin1;
    int motorPin2;
    int motorPinPower;
#endif

};

int Motor::mapSpeed(int compensationAngle){
  if(compensationAngle == 0){
    return ZERO_SPEED;
  }
  //compensationAngle 
  bool reverse = compensationAngle < 0;
  if(reverse){
    compensationAngle = -compensationAngle;
  }
  int speed = ZERO_SPEED;

  // TODO: consider non-linear speed scale
  speed = map(compensationAngle, 0, 180, minSpeed , MAX_SPEED); 

  speed *= reverse?-1:1;
  speed += ZERO_SPEED;
  return (int)speed;
}

#if USE_SERVO
  Motor::Motor(int pin) {
    motorPin = pin;
  }

  void Motor::sleep() {
    setSpeed(0);
    motorServo.detach();
  }

  void Motor::wakeUp() {
    motorServo.attach(motorPin);
  }

  int Motor::setSpeed(int value) {
    lastValue = value;
    int speed = mapSpeed(value);
    motorServo.write(speed);
    return speed;
  }

  int Motor::setSpeed(int compensationAngle, bool calibrate) {
    if(calibrate){
      Serial.println("calibration is not implemented for Servo");
      // TODO: if we ever go back to Servo again - Servo might be not calibrated around the center, so would be nice to implement it
    }
    return setSpeed(compensationAngle);
  }

  void Motor::playTheme(bool (*interrupt)()){
    playSparrowTheme(motorPin, motorPin, interrupt);
  }

#else
  Motor::Motor(int pin1, int pin2, int pinPower){
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

  int Motor::setSpeed(int compensationAngle) {
    int speed = mapSpeed(compensationAngle);
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
    return speed;
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
          minSpeed+=CALIBRATION_INCREASE;

          Serial.print("Motor calibration ");
          Serial.print(compensationAngle);
          Serial.print(" - increase min speed: ");
          Serial.println(minSpeed);
          lastCompensationAngle = 0; // skip next calibration
          lastCalibrationTime = 0;
        }
        if(lastCompensationAngle * compensationAngle<0){
          minSpeed-=CALIBRATION_DECREASE;
          Serial.print("Motor calibration ");
          Serial.print(compensationAngle);
          Serial.print(" - decrease min speed: ");
          Serial.println(minSpeed);
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
    // TODO: switch to back and forth
    playSparrowTheme(motorPin1, motorPin2, interrupt);
  }

#endif

#endif
