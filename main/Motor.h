#ifndef Motor_h
#define Motor_h

#include "compassData.pb.h"

#if USE_SERVO
  #include <Servo.h>
  #define ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
  #define MAX_SPEED     20    // 30 is prev value, max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
  #define MIN_SPEED     3     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
#else
  #define ZERO_SPEED    0    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
  #define MAX_SPEED     255    // 30 is prev value, max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
  #define MIN_SPEED     -255     // min speed where servo becomes unresponsive or doesn't have enough power. somewhere around 8 we start getting consistent results
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
    void setSpeed(int compensationAngle);
    void playTheme(bool (*interrupt)());

  private:
    int mapSpeed(int compensationAngle);
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
  float speed = ZERO_SPEED; //using float here for sensitivity around zero

  // TODO: consider non-linear speed scale
  speed = mapFloat(compensationAngle, -180, 180, ZERO_SPEED - MAX_SPEED , ZERO_SPEED + MAX_SPEED); 
  
  if(speed > ZERO_SPEED - MIN_SPEED && speed < ZERO_SPEED){
    speed = ZERO_SPEED - MIN_SPEED;
  }
  if(speed < ZERO_SPEED + MIN_SPEED && speed > ZERO_SPEED){
    speed = ZERO_SPEED + MIN_SPEED;
  }
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
    motorServo.write(mapSpeed(compensationAngle));
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
    digitalWrite(motorPinPower, HIGH);
  }

  void Motor::setSpeed(int value) {
    value = mapSpeed(value);
    if (value > 0) {
      digitalWrite(motorPin1, LOW);
      analogWrite(motorPin2, value);
    } else if (value < 0) {
      digitalWrite(motorPin1, LOW);
      analogWrite(motorPin2, -value);
    } else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
    }
  }

  void Motor::playTheme(bool (*interrupt)()){
    setSpeed(0);
    // TODO: switch to back and forth
    playSparrowTheme(motorPin1, motorPin2, interrupt);
  }

#endif

#endif
