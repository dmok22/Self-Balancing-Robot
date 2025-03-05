#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>


long lastTime;
float angle;
float gry_angle,acc_angle;         // units degrees (filtered tilt angle)

const int INPUT_B1= 3;  // Motor A - Input 1 (PWM)
const int INPUT_B2 = 5;  // Motor A - Input 2 (PWM)
const int INPUT_A1 = 6;  // Motor B - Input 1 (PWM)
const int INPUT_A2 = 9;  // Motor B - Input 2 (PWM)

float setpoint = 0.0;  // Desired tilt angle (upright)
float PDI_signal;

float integral = 0;
float derivative;
float angle_error, previousError = 0;



void setup() {
  
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  lastTime = micros();
  Serial.println(lastTime);

  pinMode(INPUT_A1, OUTPUT);
  pinMode(INPUT_A2, OUTPUT);
  pinMode(INPUT_B1, OUTPUT);
  pinMode(INPUT_B2, OUTPUT);
}

void loop() {

  float limited_angle;
  float power_precentage = 0;
  int pwm_output = 0;

  //----------------
  if(userInput == "f25")
  //--------------
  combine();
  //Serial.println(angle);
  PID(angle);
  Serial.println(PDI_signal);
  moveMotors(PDI_signal);
}


void combine(){
    Accelerator();
    gyroscope();
    float k = 0.5;
    angle = k*gry_angle + (1-k)*acc_angle;
    char buffer[50];
    //sprintf(buffer, "%.2f, %.2f, %.2f", angle, acc_angle, gry_angle);
    //Serial.println(buffer);
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


void moveMotors(float controlSignal) {
    int pwmValue = constrain(abs(controlSignal), 0, 255); // Convert to PWMrange
    Serial.println(pwmValue);

    if (controlSignal > 0) {  // Move Forward
      analogWrite(INPUT_A1, pwmValue); //max 255, min 0
      analogWrite(INPUT_A2, 0);

      analogWrite(INPUT_B1, pwmValue); //max 255, min 0
      analogWrite(INPUT_B2, 0);
    } else {  // Move Backward
      analogWrite(INPUT_A1, 0); //max 255, min 0
      analogWrite(INPUT_A2, pwmValue);

      analogWrite(INPUT_B1, 0); //max 255, min 0
      analogWrite(INPUT_B2, pwmValue);
    }
}

void PID(float angle){
  //------------------------------------------
  float Kp = 3;   // Proportional Gain
  float Ki = 0;    // Integral Gain
  float Kd = 5;    // Derivative Gain
  //------------------------------------------

  angle_error = setpoint - angle;
  integral += angle_error;
  derivative = angle_error - previousError;
  PDI_signal  = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
}
