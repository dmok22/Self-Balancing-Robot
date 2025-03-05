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

float Kp = 0;   // Proportional Gain
float Ki = 0;    // Integral Gain
float Kd = 0;    // Derivative Gain



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

  updatePID();
  //--------------
  combine();
  //Serial.println(angle);
  PID(angle);
  //Serial.println(PDI_signal);
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
    int pwmValue = 35 + pow(10, (abs(controlSignal) / 43)); // Convert to PWMrange
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
  //String userInput = Serial.readStringUntil('\n');  // Read full command
  //userInput.trim();  // Remove any extra whitespace
  /*
  if(userInput == "kp"){
    while (Serial.available() == 0) {
    }
    userInput = Serial.readStringUntil('\n');
     Kp = Serial.parseFloat();
    Serial.println("kp: " + userInput);
  }
  if(userInput == "ki"){
    while (Serial.available() == 0) {
    }
    userInput = Serial.readStringUntil('\n');
     Ki = Serial.parseFloat();
    Serial.println("ki: " + userInput);
  }
  if(userInput == "kd"){
    while (Serial.available() == 0) {
    }
    userInput = Serial.readStringUntil('\n');
     Kd = Serial.parseFloat();
    Serial.println("kd: " + userInput);
  }
  */
  /*
  float Kp = 3;   // Proportional Gain
  float Ki = 0;    // Integral Gain
  float Kd = 5;    // Derivative Gain
  */
  //------------------------------------------

  angle_error = setpoint - angle;
  integral += angle_error;
  derivative = angle_error - previousError;
  PDI_signal  = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
  PDI_signal = constrain(PDI_signal, -100, 100);
}



void updatePID() {
    if (Serial.available()) {  // Check if data is available
        String userInput = Serial.readStringUntil('\n');  // Read full command
        userInput.trim();  // Remove whitespace

        if (userInput.equals("kp")) {
            Serial.print("Kp: ");
            Serial.println(Kp);

            Serial.print("Ki: ");
            Serial.println(Ki);

            Serial.print("Kd: ");
            Serial.println(Kd);
            while (Serial.available() == 0);  // Wait for input
            Kp = Serial.parseFloat();
        }
        else if (userInput.equals("ki")) {
            Serial.print("Kp: ");
            Serial.println(Kp);

            Serial.print("Ki: ");
            Serial.println(Ki);

            Serial.print("Kd: ");
            Serial.println(Kd);
            Serial.print("Enter new Ki: ");
            while (Serial.available() == 0);
            Ki = Serial.parseFloat();
        }
        else if (userInput.equals("kd")) {
            Serial.print("Kp: ");
            Serial.println(Kp);

            Serial.print("Ki: ");
            Serial.println(Ki);

            Serial.print("Kd: ");
            Serial.println(Kd);
            Serial.print("Enter new Kd: ");
            while (Serial.available() == 0);
            Kd = Serial.parseFloat();
        }
        else {
            Serial.println("Invalid command! Use: kp, ki, or kd.");
        }
    }
}

