/*
*/

#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>

#include "compass_utils.h"
/**
Components in use:
* Magnetometer to find magnetic north
* Gyroscope to compensate magnetometer for tilt
* GPS module to locate current position
* Angular encoder to check dial rotation
* Servo motor to rotate the dial
* Hall sensor to identify closed lid and send everything to sleep if possible
* Blutooth to read an update. Turns on after first boot, and shuts down after two minutes without activity
*/


#define HALL_SENSOR_PIN         A2    // hass sensor - analogue
#define HALL_SENSOR_THRESHOLD   500   // value below that is a magnet

// TODO: 
// 1. research better conversion to degrees, this one doesn't really work.
// 2. find difference between north and arrow
// 3. test servo rotating speeds, find nice values
// 4. send right speed and direction to servo
// 5. stop when angle is correct?
#define ENCODER_LOW         0     // 50      
#define ENCODER_HIGH        1024  // 950
#define ENCODER_PIN         A1    // angular encoder - analogue

// TODO: we can calculate this version using encoder for feedback
#define SERVO_PIN           D9    // servo - digital port
#define SERVO_ZERO_SPEED    90    // 90 is a still position in Servo.h to stop servo motor. Full range supported by servo.h is 0 to 180
#define SERVO_MAX_SPEED     30    // max speed where servo stops accelerating. Note: can go beyond that value, or below to slow it down
#define SERVO_MIN_SPEED     10    // min speed where servo becomes unresponsive or doesn't have enough power
Servo servoMotor;  // Servo Object

#define gpsPort             Serial1 // 
#define GPS_HDOP_THRESHOLD        500 //  HDOP*100 to consider pointing to the target. See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
TinyGPSPlus gps;          // GPS object, reads through Serial1
double myLattitude = 0, myLongitude = 0; // Current coordinates from GPS
bool havePosition = false; // do we have gps reading or not


// TODO: do the calibration again after assembly, and iron matrix
// TODO: move to the library
float magBias[3] = {0, 0, 0};
float magScale[3] = {1.00, 1.00, 1.00};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
/*
float magBias[3] = {28.06, 8.76, 6.15};
float magScale[3] = {1.00, 1.00, 1.02};
float softIronMatrix[3][3] = {
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};
*/


float alpha = 0.7; // filtration for potentiometer
float encoderMin=10000;
float encoderMax=-1;

float readDialPosition(){ // функция получения угла с потенциометра/энкодера
  static float oldAngle = 0;
  float read = analogRead(ENCODER_PIN);

  if(read < encoderMin){
    encoderMin = read;
  }

  if(read > encoderMax){
    encoderMax = read;
  }

  float angle = map(read,ENCODER_LOW,ENCODER_HIGH,0,360);
  if(angle < 0 || angle > 360){
    angle = 0;
  }
/*  if(angle > 360){
    angle = 0;
  }*/
  //float angle = ENCODER_SCALE * read;
  
  /*Serial.print("Encoder value  = ");
  Serial.print(read);
  Serial.print("\t\t angle:");
  Serial.print(angle);
  Serial.print("\t\t min:");
  Serial.print(encoderMin);
  Serial.print("\t\t max:");
  Serial.println(encoderMax);*/

  angle = lpFilter(angle, oldAngle, alpha);   // фильтруем показания с потенциометра, если надо
  oldAngle = angle;
  return angle;
}

float lpFilter(float value, float oldValue, float alp){
  return oldValue*(1.f-alp)+ alp*value;
}



void setup() {
  Serial.begin(9600); // non blocking - opening Serial port to connect to laptop for diagnostics
  Serial.println("Started");

  pinMode(ENCODER_PIN, INPUT);

  servoMotor.attach(SERVO_PIN);
  
  gpsPort.begin(9600);
  while (!gpsPort) {   };
  Serial.println("GPS ready");
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //TODO: Calibrate magnetometer here
  IMU.setContinuousMode(); // continous reading for 
  /*
      // Set the magnetometer calibration values
      magnetometer.setMagGain(MAGGAIN_4GAUSS);  Generally, a gain level of 1.3 to 1.9 Gauss is a good starting point for using a magnetometer sensor as a compass.
      magnetometer.setMagDataRate(HMC5883L_DATARATE_15HZ);
      magnetometer.setMagBias(0, 0);  // adjust these values as needed
      magnetometer.setMagOffset(0, 0); // adjust these values as needed
      magnetometer.setMagCalibration(magOffsetX, magOffsetY, magOffsetZ, magScaleX, magScaleY, magScaleZ, magSoftIron);

  */
}
void readGps(){

  while(gpsPort.available())//While there are characters to come from the GPS.
  {
    int data = gpsPort.read();
    gps.encode(data);
  }
  if (gps.location.isUpdated()){
    myLattitude = gps.location.lat();
    myLongitude = gps.location.lng();
    int precision = gps.hdop.value(); // Horizontal Dim. of Precision (100ths-i32)
    havePosition = (precision < GPS_HDOP_THRESHOLD);

    /*Serial.print(precision);
    Serial.print("\t");
    Serial.print(myLattitude,6);
    Serial.print("\t");
    Serial.println(myLongitude,6);*/

  }
}

float readCompass(){
  // compass value compensated with accelerometer. will freak out when shaking.
  // TODO: use gyro ro introduce noise when shaking

  if (!IMU.accelerationAvailable() || !IMU.magneticFieldAvailable()) {
    return -1;
  }

  const float declinationAngle = 0;
  // this code generated with chatGPT
  float mx, my, mz, ax, ay, az;
  IMU.readMagneticField(mx, my, mz);
  IMU.readAcceleration(ax, ay, az);

  calibrateMagReading(mx,my,mz,magBias,magScale, softIronMatrix);
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  float m_x_comp = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch;
  float m_y_comp = my*cos_roll - mz*sin_roll;
  float m_z_comp = -mx*sin_pitch + my*cos_roll*sin_pitch + mz*cos_pitch*sin_roll;

  return heading(m_x_comp, m_y_comp, declinationAngle);
}

float angleCompass = 0;
int currentServoSpeed = -1;
void loop() {
  // 1. Check if lid is closed or open:
  int hallValue = analogRead(HALL_SENSOR_PIN);  // Read the value of the hall sensor
  bool motorOn = hallValue > HALL_SENSOR_THRESHOLD;
  // TODO: if state (close/open) changed - initiate wakeup or sleep sequence

  Serial.print("Hall sensor: ");
  Serial.print(hallValue);  // Print the value to the serial monitor
  Serial.print("\t motorOn: ");
  Serial.print(motorOn);  // Print the value to the serial monitor

  // 2. read GPS data
  readGps();

  // 3. Read magnetometer (with compensation for tilt from accelerometer)
  // TODO: compensate for upside down or other nonce related to initial installation
  // TODO: validate against true north
  int newCompass = readCompass();
  if(newCompass >= 0){
    angleCompass = newCompass;
  }
  
  // 4. Read angular position from encoder
  // TODO: compensate for initial position
  // TODO: figure out 360 degrees rotation range
  int angleDial = readDialPosition();

  int compensationAngle = angleCompass + angleDial;
  while (compensationAngle > 180){
    compensationAngle -= 360;
  }
  while (compensationAngle < -180){
    compensationAngle += 360;
  }

  /*  based on the servo tests, 
    90 = 0 speed
    >90 = counterclockwise
    0-90 clockwise

    120 max speed <- this value is servo-dependent
    60 min speed 
  */
  // TODO: check for minimal speed that halts the engine
  // TODO: consider non-linear speed scale
  // TODO: move numbers to constants
  // TODO: move to constants
  int servoSpeed = map(compensationAngle, -180, 180, SERVO_ZERO_SPEED - SERVO_MAX_SPEED , SERVO_ZERO_SPEED + SERVO_MAX_SPEED); 
  if(servoSpeed > SERVO_ZERO_SPEED - SERVO_MIN_SPEED && servoSpeed < SERVO_ZERO_SPEED){
    servoSpeed = SERVO_ZERO_SPEED - SERVO_MIN_SPEED;
  }
  if(servoSpeed < SERVO_ZERO_SPEED + SERVO_MIN_SPEED && servoSpeed > SERVO_ZERO_SPEED){
    servoSpeed = SERVO_ZERO_SPEED + SERVO_MIN_SPEED;
  }

  /*
  Serial.print("north: ");
  Serial.print(angleCompass);
  Serial.print("\t dial: ");
  Serial.print(angleDial);
  Serial.print("\t diff: ");
  Serial.print(compensationAngle);
  Serial.print("\t speed: ");
  Serial.println(servoSpeed);
  */

  if(!motorOn){
    servoSpeed = SERVO_ZERO_SPEED;
  }
  if(currentServoSpeed != servoSpeed){
    currentServoSpeed = servoSpeed;
    servoMotor.write(currentServoSpeed);

    Serial.print("\t servoSpeed: ");
    Serial.print(currentServoSpeed);  // Print the value to the serial monitor
  }
  Serial.println();
  delay(50);  
}