/*
*/

#include <Arduino_LSM9DS1.h>
#include <math.h>
#include <Servo.h>
#include <TinyGPS++.h>
// TODO: 1. research better conversion to degrees, this one doesn't really work.
// 2. find difference between north and arrow
// 3. test servo rotating speeds, find nice values
// 4. send right speed and direction to servo
// 5. stop when angle is correct?xzs
#define ENCODER_SCALE   270.f/1024.f  // макрос перевода угла потенциометра или энкодера из АЦП попугаев [0:1024] в градусы [0:270]
#define ENCODER_PIN     A1    // пин потенциометра или энкодера
#define HALL_SENSOR_PIN     A2    // пин потенциометра или энкодера


TinyGPSPlus gps;// This is the GPS object that will pretty much do all the grunt work with the NMEA data

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
float alpha = 1.0; // filtration for potentiometer


float getAngleDirection(float x, float y){
  float deltax=-34,deltay=-1.5; 
  x=x+deltax;
  y=y+deltay;
  float angle=atan2(y,x);
  angle = angle * 180.0 / M_PI;
  if (angle<0){
    angle=angle+360;
  }
  return angle;
}

float getDirectionToPoint(float myX, float myY, float pointX, float pointY){

}


float getAngleWheel(){ // функция получения угла с потенциометра/энкодера
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

  pinMode(ENCODER_PIN,INPUT);
  myservo.attach(9);
  Serial.begin(9600);
 // while (!Serial);
  Serial.println("Started");
  Serial1.begin(9600);
  while (!Serial1) {   };
  Serial.println("GPS ready");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
}

void loop() {
   
  int hallValue = analogRead(HALL_SENSOR_PIN);  // Read the value of the hall sensor
  Serial.println(hallValue);  // Print the value to the serial monitor
  

 // delay(1000);
 // Serial.println("reading..");
  while(Serial1.available())//While there are characters to come from the GPS.
  {
    int data = Serial1.read();
    //Serial.print(data);
    gps.encode(data);
  }
  /*if (gps.location.isUpdated())
  {
     //Get the latest info from the gps object which it derived from the data sent by the GPS unit
    Serial.println("Satellite Count:"); 
    Serial.println(gps.satellites.value());
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.println("Longitude");
    Serial.println(gps.location.lng(), 6); 
    Serial.println("Speed MPH:");
    Serial.println(gps.speed.mph());
    Serial.println("Altiude Feet:");
    Serial.println(gps.altitude.feet());    
    Serial.println("Course:");
    Serial.println(gps.course.deg());
    Serial.println("");
  }
  return;*/
  float x, y, z, angleCompass, angleWheel;

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    angleCompass = getAngleDirection(x, y);
    angleWheel = getAngleWheel();
    float h = angleCompass-angleWheel;
    //Serial.print(h);
    //Serial.print("\t");
    h=h/270*30+90;
    myservo.write(h);


    /* 
      90 = 0 speed
      >90= counterclockwise
      0-90 clockwise
      120 max speed
60 min speed  
      
    
    int newPos = angleCompass+80;
    if(newPos - pos > 3 || newPos - pos < -3){
      pos = newPos;
      myservo.write(pos);
    }
    */

   //Serial.print(angleCompass);
   //Serial.print("\t");
   //Serial.println(angleWheel);
    delay(100);
  }
}