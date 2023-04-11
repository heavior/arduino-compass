/*
*/

#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>
/**
Components in use:
* Magnetometer to find magnetic north
* Gyroscope to compensate magnetometer for tilt
* GPS module to locate current position
* Angular encoder to check disc rotation
* Servo motor to rotate the disk
* Hall sensor to identify closed lid and send everything to sleep if possible
* Blutooth to read an update. Turns on after first boot, and shuts down after two minutes without activity

*/

// TODO: 1. research better conversion to degrees, this one doesn't really work.
// 2. find difference between north and arrow
// 3. test servo rotating speeds, find nice values
// 4. send right speed and direction to servo
// 5. stop when angle is correct?xzs
#define ENCODER_SCALE       270.f/1024.f  // макрос перевода угла потенциометра или энкодера из АЦП попугаев [0:1024] в градусы [0:270]
#define ENCODER_PIN         A1    // angular encoder - analogue

#define HALL_SENSOR_PIN     A2    // hass sensor - analogue
#define SERVO_PIN           D9    // servo - digital port

Servo servoMotor;  // Servo Object

#define gpsPort             Serial1 // 
#define GPS_HDOP_THRESHOLD        500 //  HDOP*100 to consider pointing to the target. See https://en.wikipedia.org/wiki/Dilution_of_precision_(navigation)
TinyGPSPlus gps;          // GPS object, reads through Serial1
double myLattitude = 0, myLongitude = 0; // Current coordinates from GPS
bool havePosition = false; // do we have gps reading or not


float alpha = 0.9; // filtration for potentiometer
float readDiscPosition(){ // функция получения угла с потенциометра/энкодера
  static float oldAngle = 0;
  float read = analogRead(ENCODER_PIN);
  float angle = ENCODER_SCALE * read;
  
 /* Serial.print("Encoder value  = ");
  Serial.print(read);
  Serial.print("\t\t angle:");
  Serial.println(angle);*/

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

  pinMode(ENCODER_PIN,INPUT);

  servoMotor.attach(SERVO_PIN);
  
  gpsPort.begin(9600);
  while (!gpsPort) {   };
  Serial.println("GPS ready");
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  //TODO: Calibrate magnetometer here
  /*
      // Set the magnetometer calibration values
      magnetometer.setMagGain(HMC5883L_GAIN_1_3);
      magnetometer.setMagDataRate(HMC5883L_DATARATE_15HZ);
      magnetometer.setMagMeasurementMode(HMC5883L_CONTINOUS);
      magnetometer.setMagBias(0, 0);  // adjust these values as needed
      magnetometer.setMagOffset(0, 0); // adjust these values as needed
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

    Serial.print(precision);
    Serial.print("\t");
    Serial.print(myLattitude,6);
    Serial.print("\t");
    Serial.println(myLongitude,6);

  }
}

float readCompass(){
  // compass value compensated with accelerometer. will freak out when shaking.
  // TODO: use gyro ro introduce noise when shaking

  if (!IMU.accelerationAvailable() || !IMU.magneticFieldAvailable()) {
    return -1;
  }

  // this code generated with chatGPT
  float mx, my, mz, ax, ay, az;
  IMU.readMagneticField(mx, my, mz);
  IMU.readAcceleration(ax, ay, az);
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay*ay + az*az));
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  float m_x_comp = mx*cos_pitch + my*sin_roll*sin_pitch + mz*cos_roll*sin_pitch;
  float m_y_comp = my*cos_roll - mz*sin_roll;
  float m_z_comp = -mx*sin_pitch + my*cos_roll*sin_pitch + mz*cos_pitch*sin_roll;
  float heading = atan2(-m_y_comp, m_x_comp);

  // Convert the heading to degrees
  float heading_degrees = heading * 180.0 / PI;
  if (heading_degrees < 0) {
    heading_degrees += 360;
  }

  return heading_degrees;
}

void loop() {
  // 1. Check if lid is closed or open:
  int hallValue = analogRead(HALL_SENSOR_PIN);  // Read the value of the hall sensor

  // TODO: check threshold, if state (close/open) changed - initiate wakeup or sleep sequence
  // Serial.println(hallValue);  // Print the value to the serial monitor

  // 2. read GPS data
  readGps();


  // 3. Read magnetometer (with compensation for tilt from accelerometer)
  // TODO: compensate for upside down or other nonce related to initial installation
  // TODO: validate against true north
  // TODO: look into magnetometer setup
  float angleCompass = readCompass();

  // 4. Read angular position from encoder
  // TODO: compensate for initial position
  // TODO: figure out 360 degrees rotation range
  float angleWheel = readDiscPosition();


  float compensationAngle = angleCompass - angleWheel;
  if (compensationAngle < -360){
    compensationAngle += 360;
  }
  if (compensationAngle > 360){
    compensationAngle -= 360;
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
  float servoSpeed = compensationAngle*30/27+90; 

  Serial.print(angleCompass);
  Serial.print("\t");
  Serial.print(angleWheel);
  Serial.print("\t");
  Serial.print(compensationAngle);
  Serial.print("\t");
  Serial.println(servoSpeed);

  //servoMotor.write(servoSpeed);

  //Serial.print(angleCompass);
  //Serial.print("\t");
  //Serial.println(angleWheel);
  delay(100);
  
}