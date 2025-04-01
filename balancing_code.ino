#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include <ArduinoBLE.h>

// **BLE Service & Characteristic**
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, 50, false);

/*
float kalmanAngle = 0.0;
float bias = 0.0;
float P[2][2] = { {1, 0}, {0, 1} };
*/

float dt;
//float PID_dt
long lastTime;
long lastTime_PID;

float angle;
float angle_filter_gry;
float gry_angle,acc_angle;         // units degrees (filtered tilt angle)
float gyroX, gyroY, gyroZ;
float gryoX_drift;

bool isCalibrated = false;
const int CALIBRATION_SAMPLES = 1000;

const int INPUT_B1= 3;  // Motor A - Input 1 (PWM)
const int INPUT_B2 = 5;  // Motor A - Input 2 (PWM)
const int INPUT_A1 = 6;  // Motor B - Input 1 (PWM)
const int INPUT_A2 = 9;  // Motor B - Input 2 (PWM)

float setpoint = 0;  // Desired tilt angle (upright)

float PDI_signal;

float integral = 0;
float derivative;
float angle_error, previousError = 0;

float Kp = 0;   // Proportional Gain
float Ki = 0;    // Integral Gain
float Kd = 0;    // Derivative Gain
float sp = -1.15;  //additional setpoint adjustment due to center of gravity

int motor_difference;


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

  lastTime = millis();
  lastTime_PID = millis();

  delay(1000);
  calibrateTarget();

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
  //kalmanCombine();
  //Serial.println(angle);
  PID(angle);
  //Serial.println(PDI_signal);
  moveMotors(PDI_signal);
}


//------------------------------------------------------------------------------------------------------
void calibrateTarget()
{
  float sumAngle = 0.0;
  float sumGyroX = 0.0;
  Serial.println("Calibrating... Keep robot still and upright");

  // Take multiple readings and average them
  for (int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    combine(); // Get current angle reading
    sumAngle += angle;
    sumGyroX += gyroX;
    delay(10); // Small delay between readings
  }

  setpoint = sumAngle / CALIBRATION_SAMPLES;
  gryoX_drift = sumGyroX / CALIBRATION_SAMPLES;
  isCalibrated = true;
  Serial.print("Calibration complete. Target angle: ");
  Serial.println(setpoint);
  //respondToBLE("Calibration Done");
}
//------------------------------------------------------------------------------------------------------



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
    //Serial.print("✅ Kp Updated: "); Serial.println(Kp);
    respondToBLE("Kp=" + String(Kp)); // Send back the updated value
  } else if (cmd.startsWith("ki=")) {
    Ki = cmd.substring(3).toFloat();
    //Serial.print("✅ Ki Updated: "); Serial.println(Ki);
    respondToBLE("Ki=" + String(Ki)); // Send back the updated value
  } else if (cmd.startsWith("kd=")) {
    Kd = cmd.substring(3).toFloat();
    //Serial.print("✅ Kd Updated: "); Serial.println(Kd);
    respondToBLE("Kd=" + String(Kd)); // Send back the updated value
  } else if (cmd.startsWith("md=")) {
    motor_difference = cmd.substring(3).toFloat();
    //Serial.print("✅ sp Updated: "); Serial.println(sp);
    respondToBLE("motor_difference=" + String(motor_difference)); // Send back the updated value
  }
  else if (cmd.startsWith("sp=")) {
    sp = cmd.substring(3).toFloat();
    //Serial.print("✅ sp Updated: "); Serial.println(sp);
    respondToBLE("sp=" + String(sp)); // Send back the updated value
  } 
  else {
    //Serial.println("❌ Unknown command");
  }
}

void respondToBLE(String response) {
  customCharacteristic.writeValue(response.c_str());
}



void setupBLE() {
  if (!BLE.begin()) {
    //Serial.println("❌ BLE Initialization Failed!");
    while (1);
  }
  BLE.setLocalName("CJJ");//"NanoBLE"
  BLE.setDeviceName("CJJ");//"NanoBLE"
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  BLE.advertise();

  //Serial.println("✅ BLE Ready - Waiting for commands...");
}

//---------------------------------------------------------------------------------------------------------

void combine() {
    Accelerator();
    gyroscope();

    // --- 1. Low-pass filter for accelerometer angle ---
    static float acc_angle_filtered = 0;
    float alpha = 0.5; // smoothing factor (lower = smoother)
    acc_angle_filtered = alpha * acc_angle + (1 - alpha) * acc_angle_filtered;

    // --- 2. Complementary filter with motion gating ---
    float k = 0.85;

    angle_filter_gry = k * gry_angle + (1 - k) * acc_angle_filtered;

    if (abs(gyroX) < 2.0) {
        // Robot is relatively still — trust both sensors
        angle = k * gry_angle + (1 - k) * acc_angle_filtered;
    } else {
        // Robot is moving fast — trust only gyro to avoid false spikes
        angle = gry_angle;
    }
    //Serial.println(angle);
}




void gyroscope(){

    //float gyroX, gyroY, gyroZ;
    long lastInterval;

    long currentTime = millis();
    //Serial.println(lastTime);
    lastInterval = currentTime - lastTime; // expecting this to be ~104Hz +- 4%
    lastTime = currentTime;

    //above is time setup

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    // Gyroscope integration for yaw (tilt angle around z-axis)
    dt = lastInterval / 1000.0;                               // Convert microseconds to seconds
    gry_angle = angle + ((-gyroX) * dt); // Drift-corrected integration
}

void Accelerator(){
  float x, y, z;
  IMU.readAcceleration(x, y, z);
    
  acc_angle = atan2f(100*y, 100*z);
  acc_angle = acc_angle * (180.0f / M_PI); 
  //Serial.println(thetaDegrees);
}



void moveMotors(float controlSignal) {
    int left_pwmValue;
    int right_pwmValue;
    //int pwmValue = 35 + pow(10, (abs(controlSignal) / 43)); // Convert to PWMrange
    //Serial.println(pwmValue);

    // for getting the initial pwm 
    if (abs(controlSignal) > 0){
      left_pwmValue = abs(controlSignal) + 0;
      right_pwmValue = abs(controlSignal) + 0 + motor_difference;
    }
    else {
      left_pwmValue = 0;
      right_pwmValue = 0;
    }

    // contraint 
    right_pwmValue = constrain(right_pwmValue, 0, 255);
    left_pwmValue = constrain(left_pwmValue, 0, 255);


   // char buffer[50];
    //sprintf(buffer, "%.2f, %d", angle, pwmValue);
    //Serial.println(buffer);
    
    if (controlSignal > 0) {  // Move Forward
      analogWrite(INPUT_A1, left_pwmValue); //max 255, min 0
      analogWrite(INPUT_A2, 0);

      analogWrite(INPUT_B1, right_pwmValue); //max 255, min 0
      analogWrite(INPUT_B2, 0);
    } else {  // Move Backward
      analogWrite(INPUT_A1, 0); //max 255, min 0
      analogWrite(INPUT_A2, left_pwmValue);

      analogWrite(INPUT_B1, 0); //max 255, min 0
      analogWrite(INPUT_B2, right_pwmValue);
    }
}

void PID(float angle){
  double now_PID = millis();
  float dt_PID = (now_PID - lastTime_PID) / 1000.0;  // seconds

  angle_error = angle - (setpoint + sp);

  integral = integral + angle_error*dt_PID;
  //integral = integral + angle_error;
  integral = constrain(integral, -255, 255);

  //derivative = (angle_error - previousError)/dt_PID;
  derivative = -gyroX + gryoX_drift; // 0.49 is to counter the drift 
  derivative = constrain(derivative, -255, 255);

  PDI_signal  = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
  PDI_signal = constrain(PDI_signal, -255, 255);
  
  previousError = angle_error;
  lastTime_PID = now_PID;
  //Serial.println(angle-setpoint);

  //char buffer[50];
  //sprintf(buffer, "%.2f, -255, 255", angle);
  //Serial.println(buffer);
}
