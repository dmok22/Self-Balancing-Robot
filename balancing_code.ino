#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include <ArduinoBLE.h>

// **BLE Service & Characteristic**
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, 50, false);


float dt;
//float PID_dt
long lastTime;
long lastTime_PID;

float angle;
float gry_angle,acc_angle;         // units degrees (filtered tilt angle)
float gyroX, gyroY, gyroZ;

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

  setupBLE();
  
  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");

  lastTime = micros();
  lastTime_PID = micros();

  //Serial.println(lastTime);
  //setpoint = setpoint + 0.75;

  pinMode(INPUT_A1, OUTPUT);
  pinMode(INPUT_A2, OUTPUT);
  pinMode(INPUT_B1, OUTPUT);
  pinMode(INPUT_B2, OUTPUT);
}

void loop() {

  float limited_angle;
  float power_precentage = 0;
  int pwm_output = 0;

  handleBLECommands(); 
  //--------------
  combine();
  //Serial.println(angle);
  PID(angle);
  //Serial.println(PDI_signal);
  moveMotors(PDI_signal);
}




void handleBLECommands() {
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("🔗 Connected to: ");
    //Serial.println(central.address());

    // Check for BLE updates **without blocking**
    if (customCharacteristic.written()) {
      int length = customCharacteristic.valueLength();
      char buffer[length + 1];
      memcpy(buffer, customCharacteristic.value(), length);
      buffer[length] = '\0';

      String receivedCommand = String(buffer);

      processCommand(receivedCommand);
      respondToBLE(receivedCommand);
    }
  }
}

// **Process BLE Commands**
void processCommand(String cmd) {
  if (cmd.startsWith("kp=")) {
    Kp = cmd.substring(3).toFloat();
    Serial.print("✅ Kp Updated: "); Serial.println(Kp);
    respondToBLE("Kp=" + String(Kp)); // Send back the updated value
  } else if (cmd.startsWith("ki=")) {
    Ki = cmd.substring(3).toFloat();
    Serial.print("✅ Ki Updated: "); Serial.println(Ki);
    respondToBLE("Ki=" + String(Ki)); // Send back the updated value
  } else if (cmd.startsWith("kd=")) {
    Kd = cmd.substring(3).toFloat();
    Serial.print("✅ Kd Updated: "); Serial.println(Kd);
    respondToBLE("Kd=" + String(Kd)); // Send back the updated value
  } else if (cmd == "s") {  // Show PID values
    String pidValues = "Kp=" + String(Kp) + " | Ki=" + String(Ki) + " | Kd=" + String(Kd);
    Serial.println("🔍 " + pidValues);
    respondToBLE(pidValues); // Send PID values over BLE
  } else {
    Serial.println("❌ Unknown command");
  }
}

// **Send Acknowledgment via BLE**
void respondToBLE(String cmd) {
  //String response = "PID:" + PDI_signal;
  //customCharacteristic.writeValue(response.c_str());
}



void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("❌ BLE Initialization Failed!");
    while (1);
  }
  BLE.setLocalName("CJJ");//"NanoBLE"
  BLE.setDeviceName("CJJ");//"NanoBLE"
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  BLE.advertise();

  Serial.println("✅ BLE Ready - Waiting for commands...");
}

//---------------------------------------------------------------------------------------------------------


void combine(){
    Accelerator();
    gyroscope();
    float k = 0.8;//0.5
    angle = k*gry_angle + (1-k)*acc_angle;
    
    char buffer[50];
    //sprintf(buffer, "%.2f, %.2f, %.2f", angle, acc_angle, gry_angle);
    if(abs(angle)<0.7){
      angle = 0;
    }

    //Serial.println(angle);
}


void gyroscope(){

    //float gyroX, gyroY, gyroZ;
    long lastInterval;

    long currentTime = micros();
    //Serial.println(lastTime);
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    //above is time setup

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Gyroscope integration for yaw (tilt angle around z-axis)
    dt = lastInterval / 1000000.0;                               // Convert microseconds to seconds
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
    int pwmValue;
    //int pwmValue = 35 + pow(10, (abs(controlSignal) / 43)); // Convert to PWMrange
    //Serial.println(pwmValue);
    if (controlSignal != 0){
      pwmValue = abs(controlSignal) + 30;
    }
    else {pwmValue = 0;}
    pwmValue = constrain(pwmValue, 30, 255);
    
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
  double now_PID = micros();
  float dt_PID = (now_PID - lastTime_PID) / 1000000.0;  // seconds

  angle_error = setpoint - angle;

  //integral = integral + angle_error*dt_PID;
  integral = integral + angle_error;
  integral = constrain(integral, -255, 255);

  //derivative = (angle_error - previousError)/dt_PID;
  derivative = gyroX;
  derivative = constrain(derivative, -255, 255);

  PDI_signal  = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
  PDI_signal = constrain(PDI_signal, -255, 255);
  
  previousError = angle_error;
  PID_lastTime = now_PID;
  //Serial.println(dt);
}