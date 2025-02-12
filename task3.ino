#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>

long lastTime;
float tiltAngle;

void setup()
{

  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Started");

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  lastTime = micros();
  Serial.println(lastTime);
}

void loop()
{
  char userInput;
  float x, y, z;
  userInput = Serial.read(); // read user input

  if (userInput == 'g')
  { // uncomment when testing with python
    gyroscope();
  } // uncommnet when using with python
}

void gyroscope()
{
  float gyroX, gyroY, gyroZ;
  long lastInterval;

  long currentTime = micros();
  // Serial.println(lastTime);
  lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
  lastTime = currentTime;

  // above is time setup

  IMU.readGyroscope(gyroX, gyroY, gyroZ);

  // Gyroscope integration for yaw (tilt angle around z-axis)
  float dt = lastInterval / 1000000.0;  // Convert microseconds to seconds
  tiltAngle = tiltAngle + ((gyroX)*dt); // Drift-corrected integration
  Serial.println(tiltAngle);
}
