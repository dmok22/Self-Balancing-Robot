#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>

long lastTime;
float angle;
float gry_angle,acc_angle;         // units degrees (filtered tilt angle)


void setup()
{

  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  lastTime = micros();
  Serial.println(lastTime);
}



void combine(){
    Accelerator();
    gyroscope();
    float k = 0.5;
    angle = k*gry_angle + (1-k)*acc_angle;
    char buffer[50];
    sprintf(buffer, "%.2f, %.2f, %.2f", angle, acc_angle, gry_angle);
    Serial.println(buffer);
}


void gyroscope(){

    float gyroX, gyroY, gyroZ;
    long lastInterval;

    long currentTime = micros();
    //Serial.println(lastTime);
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    //above is time setup

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Gyroscope integration for yaw (tilt angle around z-axis)
    float dt = lastInterval / 1000000.0;                               // Convert microseconds to seconds
    gry_angle = angle + ((gyroX) * dt); // Drift-corrected integration
}

void Accelerator(){
  float x, y, z;
  IMU.readAcceleration(x, y, z);

  if(y >= 0){
      y = 100*y;
      }
  if(y < 0){
      y = 100*y;
      }

  if(z >= 0){
      z = 100*z;
      }
  if(z < 0){
      z = 100*z;
      }
    
    acc_angle = atan2f(y, z);
    acc_angle = acc_angle * (180.0f / M_PI); 
  //Serial.println(thetaDegrees);
}


void loop() {
  char userInput;
  float x, y, z;
   userInput = Serial.read();               // read user input
      
   if(userInput == 'g'){              //uncomment when testing with python       
   combine();
   } //uncommnet when using with python

}