/*
  Arduino BMI270_BMM150 - Simple Accelerometer

  This example reads the acceleration values from the BMI270_BMM150
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include "Arduino_BMI270_BMM150.h"
#include <stdio.h>
#include <math.h>

float x, y, z;
int degreesX = 0;
int degreesY = 0;

void setup() {
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
}

void loop() {
  char userInput;
  float x, y, z;
   userInput = Serial.read();               // read user input
      
   if(userInput == 'g'){              //uncomment when testing with python       
    Accelerator();
   } //uncommnet when using with python

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
    
  float thetaRadians = atan2f(y, z);
  float thetaDegrees = thetaRadians * (180.0f / M_PI); 
  Serial.println(thetaDegrees);
}

  

