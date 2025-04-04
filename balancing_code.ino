#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include <ArduinoBLE.h>

// BLE Service & Characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, 50, false);

float dt;
long lastTime;
long lastTime_PID;

float angle;
float angle_filter_gry;
float gry_angle, acc_angle;
float gyroX, gyroY, gyroZ;
float gryoX_drift;

bool isCalibrated = false;
const int CALIBRATION_SAMPLES = 1000;

const int INPUT_B1 = 3;
const int INPUT_B2 = 5;
const int INPUT_A1 = 6;
const int INPUT_A2 = 9;

float setpoint = 0;
float PDI_signal;
float integral = 0;
float derivative;
float angle_error, previousError = 0;

float Kp = 17.5;
float Ki = 65.8;
float Kd = 1.37;
float sp = -0.6;

int motor_difference;
float turning_coeff = 0.0;
float moving_coeff = 0.0;
float moving_mt = 0.0;
int mode = 0;

// Flashing state for turn signals
int left_signal_state = 0;
int right_signal_state = 0;
bool left_led_on = false;
bool right_led_on = false;
unsigned long last_blink_time = 0;
const unsigned long blink_interval = 500;

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

  pinMode(INPUT_A1, OUTPUT);
  pinMode(INPUT_A2, OUTPUT);
  pinMode(INPUT_B1, OUTPUT);
  pinMode(INPUT_B2, OUTPUT);

  pinMode(A0, OUTPUT);  // Left signal
  pinMode(A1, OUTPUT);  // Right signal
  pinMode(A3, OUTPUT);  // Headlight
}

void loop() {
  handleBLECommands();
  combine();
  PID(angle);
  moveMotors(PDI_signal);

  // Flashing logic for turn signals
  unsigned long current_time = millis();
  if (current_time - last_blink_time >= blink_interval) {
    last_blink_time = current_time;

    if (left_signal_state) {
      left_led_on = !left_led_on;
      digitalWrite(A0, left_led_on ? HIGH : LOW);
    }

    if (right_signal_state) {
      right_led_on = !right_led_on;
      digitalWrite(A1, right_led_on ? HIGH : LOW);
    }
  }
}

//------------------------------------------------------------------------------------------------------
void calibrateTarget() {
  float sumAngle = 0.0;
  float sumGyroX = 0.0;
  Serial.println("Calibrating... Keep robot still and upright");

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    combine();
    sumAngle += angle;
    sumGyroX += gyroX;
    delay(10);
  }

  setpoint = sumAngle / CALIBRATION_SAMPLES;
  gryoX_drift = sumGyroX / CALIBRATION_SAMPLES;
  isCalibrated = true;
  Serial.print("Calibration complete. Target angle: ");
  Serial.println(setpoint);
}

void handleBLECommands() {
  BLEDevice central = BLE.central();

  if (central) {
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

void processCommand(String cmd) {
  if (cmd.startsWith("kp=")) {
    Kp = cmd.substring(3).toFloat();
    respondToBLE("Kp=" + String(Kp));
  } else if (cmd.startsWith("ki=")) {
    Ki = cmd.substring(3).toFloat();
    respondToBLE("Ki=" + String(Ki));
  } else if (cmd.startsWith("kd=")) {
    Kd = cmd.substring(3).toFloat();
    respondToBLE("Kd=" + String(Kd));
  } else if (cmd.startsWith("md=")) {
    motor_difference = cmd.substring(3).toFloat();
    respondToBLE("motor_difference=" + String(motor_difference));
  } else if (cmd.startsWith("sp=")) {
    sp = cmd.substring(3).toFloat();
    respondToBLE("sp=" + String(sp));
  } else if (cmd.startsWith("mode=")) {
    mode = cmd.substring(5).toInt();
    respondToBLE("mode=" + String(mode));
  } else if (cmd.startsWith("x=")) {
    int commaIndex = cmd.indexOf(',');
    if (commaIndex > 0) {
      String xStr = cmd.substring(2, commaIndex);
      String yStr = cmd.substring(cmd.indexOf("y=") + 2);
      turning_coeff = xStr.toFloat();
      moving_coeff = yStr.toFloat();
    }
  } else if (cmd.startsWith("left_signal=")) {
    left_signal_state = cmd.substring(12).toInt();
    if (left_signal_state == 0) {
      digitalWrite(A0, LOW);
      left_led_on = false;
    }
    respondToBLE("left_signal=" + String(left_signal_state));
  } else if (cmd.startsWith("right_signal=")) {
    right_signal_state = cmd.substring(13).toInt();
    if (right_signal_state == 0) {
      digitalWrite(A1, LOW);
      right_led_on = false;
    }
    respondToBLE("right_signal=" + String(right_signal_state));
  } else if (cmd.startsWith("headlight=")) {
    int val = cmd.substring(10).toInt();
    digitalWrite(A3, val ? HIGH : LOW);
    respondToBLE("headlight=" + String(val));
  }
}

void respondToBLE(String response) {
  customCharacteristic.writeValue(response.c_str());
}

void setupBLE() {
  if (!BLE.begin()) {
    while (1);
  }
  BLE.setLocalName("CJJ");
  BLE.setDeviceName("CJJ");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  BLE.advertise();
}

//---------------------------------------------------------------------------------------------------------

void combine() {
  Accelerator();
  gyroscope();

  static float acc_angle_filtered = 0;
  float alpha = 0.5;
  acc_angle_filtered = alpha * acc_angle + (1 - alpha) * acc_angle_filtered;

  float k = 0.85;
  angle_filter_gry = k * gry_angle + (1 - k) * acc_angle_filtered;

  if (abs(gyroX) < 2.0) {
    angle = k * gry_angle + (1 - k) * acc_angle_filtered;
  } else {
    angle = gry_angle;
  }
}

void gyroscope() {
  long lastInterval;
  long currentTime = millis();
  lastInterval = currentTime - lastTime;
  lastTime = currentTime;

  IMU.readGyroscope(gyroX, gyroY, gyroZ);

  dt = lastInterval / 1000.0;
  gry_angle = angle + ((-gyroX) * dt);
}

void Accelerator() {
  float x, y, z;
  IMU.readAcceleration(x, y, z);
  acc_angle = atan2f(100 * y, 100 * z);
  acc_angle = acc_angle * (180.0f / M_PI);
}

void moveMotors(float controlSignal) {
  int left_pwmValue;
  int right_pwmValue;

  if (mode == 0) {
    moving_mt = 0.05;
    if (turning_coeff > 0) {
    right_pwmValue *= (1 - abs(turning_coeff) * 0.01);
    left_pwmValue *= (1 - abs(turning_coeff) * 0.005);
  } else {
    left_pwmValue *= (1 - abs(turning_coeff) * 0.01);
    right_pwmValue *= (1 - abs(turning_coeff) * 0.005);
  }
  } else {
    moving_mt *= 0.1;
    if (turning_coeff > 0) {
    right_pwmValue *= (1 - abs(turning_coeff) * 0.01);
  } else {
    left_pwmValue *= (1 - abs(turning_coeff) * 0.01);
  }
  }

  if (abs(controlSignal) > 0) {
    left_pwmValue = abs(controlSignal);
    right_pwmValue = abs(controlSignal) + motor_difference;
  } else {
    left_pwmValue = 0;
    right_pwmValue = 0;
  }
  
  right_pwmValue = constrain(right_pwmValue, 0, 255);
  left_pwmValue = constrain(left_pwmValue, 0, 255);

  if (controlSignal > 0) {
    analogWrite(INPUT_A1, left_pwmValue);
    analogWrite(INPUT_A2, 0);
    analogWrite(INPUT_B1, right_pwmValue);
    analogWrite(INPUT_B2, 0);
  } else {
    analogWrite(INPUT_A1, 0);
    analogWrite(INPUT_A2, left_pwmValue);
    analogWrite(INPUT_B1, 0);
    analogWrite(INPUT_B2, right_pwmValue);
  }
}

void PID(float angle) {
  double now_PID = millis();
  float dt_PID = (now_PID - lastTime_PID) / 1000.0;

  Serial.print("cjj: ");
  Serial.println(moving_coeff);
  angle_error = angle - (setpoint + sp + moving_mt * moving_coeff);

  integral += angle_error * dt_PID;
  integral = constrain(integral, -255, 255);

  derivative = -gyroX + gryoX_drift;
  derivative = constrain(derivative, -255, 255);

  PDI_signal = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
  PDI_signal = constrain(PDI_signal, -255, 255);

  previousError = angle_error;
  lastTime_PID = now_PID;
}
