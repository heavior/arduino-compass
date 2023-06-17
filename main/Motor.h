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
  #define MIN_SPEED     48 
#endif


class Motor {
  public:

#if USE_SERVO
    Motor(int pin);
#else
    Motor(int pin1, int pin2, int pinPower);
#endif

    void sleep();
    void wakeUp();
    int setSpeed(int compensationAngle);
    void playTheme(bool (*interrupt)());
    int mapSpeed(int compensationAngle);

  private:

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
  speed = map(compensationAngle, 0, 180, MIN_SPEED , MAX_SPEED); 

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

  void Motor::setSpeed(int compensationAngle) {
    int speed = mapSpeed(compensationAngle);
    motorServo.write(speed);
    return speed;
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

  int Motor::setSpeed(int value) {
    int speed = mapSpeed(value);
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

  void Motor::playTheme(bool (*interrupt)()){
    setSpeed(0);
    // TODO: switch to back and forth
    playSparrowTheme(motorPin1, motorPin2, interrupt);
  }

#endif

#endif
