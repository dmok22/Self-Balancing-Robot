#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>


long lastTime;
long lastInterval;
int i = 1;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ,                                     // units dps (degrees per second)
    gyroDriftX, gyroDriftY, gyroDriftZ,                      // units dps
    gyroRoll, gyroPitch, gyroYaw,                            // units degrees (expect major drift)
    gyroCorrectedRoll, gyroCorrectedPitch, gyroCorrectedYaw, // units degrees (expect minor drift)
    accRoll, accPitch, accYaw,                               // units degrees (roll and pitch noisy, yaw not possible)
    tiltAngle;


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
}

/**
   Read accel and gyro data.
   returns true if value is 'new' and false if IMU is returning old cached data
*/
bool readIMU()
{
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable())
    {
        IMU.readAcceleration(accelX, accelY, accelZ);
        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        return true;
    }
    return false;
}

void loop() {
  char userInput;
  float x, y, z;
   userInput = Serial.read();               // read user input
      
   if(userInput == 'g'){              //uncomment when testing with python       
    gyroscope();
   } //uncommnet when using with python

}

/**
   I'm expecting, over time, the Arduino_LSM6DS3.h will add functions to do most of this,
   but as of 1.0.0 this was missing.
*/

void gyroscope(){

    long currentTime = micros();
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    //above is time setup

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Gyroscope integration for yaw (tilt angle around z-axis)
    float dt = lastInterval / 1000000.0;                               // Convert microseconds to seconds
    long tiltAngle = tiltAngle + ((gyroX) * dt); // Drift-corrected integration
    Serial.println(tiltAngle);
}

/*
void doCalculations()
{
    long last_angle;

    float lastFrequency = (float)1000000.0 / lastInterval; // Frequency in Hz

    // Gyroscope integration for yaw (tilt angle around z-axis)
    float dt = lastInterval / 1000000.0;                               // Convert microseconds to seconds
    gyroYaw = gyroYaw + (gyroZ * dt);                                  // Integrating angular velocity
    gyroCorrectedYaw = gyroCorrectedYaw + ((gyroX - gyroDriftX) * dt); // Drift-corrected integration
    tiltAngle = gyroCorrectedYaw;
    if (i == 1){
      last_angle = tiltAngle;
      i = 0; 
      Serial.println("wrong");
    }
    long calculated_angle = last_angle - tiltAngle;
    Serial.println(calculated_angle);
    last_angle = tiltAngle;                                    // Final tilt angle
}
*/
