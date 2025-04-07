#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include <ArduinoBLE.h>

// BLE Setup
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, 50, false);

// Time
float dt;
long lastTime;
long lastTime_PID;

// IMU
float angle;
float angle_filter_gry;
float gry_angle, acc_angle;
float gyroX, gyroY, gyroZ;
float gryoX_drift;

bool isCalibrated = false;
const int CALIBRATION_SAMPLES = 1000;

// Pins
const int INPUT_B1 = 3;
const int INPUT_B2 = 5;
const int INPUT_A1 = 6;
const int INPUT_A2 = 9;
const int SONAR_TRIG = 7;
const int SONAR_ECHO = 8;
const int SERVO_PIN = 4;

// PID
float setpoint = 0;
float PDI_signal;
float previous_PDI = 0;
float integral = 0;
float derivative;
float angle_error, previousError = 0;

float Kp = 17.5;
float Ki = 66;
float Kd = 1.37;
float sp = 0;

int motor_difference;
float turning_coeff = 0.0;
float moving_coeff = 0.0;
float moving_mt = 0.0;
int mode = 0;

float lt_trigger = 0.0;
float rt_trigger = 0.0;
String single_wheel = "";

int left_signal_state = 0;
int right_signal_state = 0;
bool left_led_on = false;
bool right_led_on = false;
unsigned long last_blink_time = 0;
const unsigned long blink_interval = 500;

bool sonar_enabled = false;

void setup() {
  setupBLE();

  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized.");

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
  pinMode(A2, OUTPUT);  // Brake light
  pinMode(A3, OUTPUT);  // Headlight

  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(SONAR_ECHO, INPUT);
  pinMode(SERVO_PIN, OUTPUT);
}

void loop() {
  handleBLECommands();
  combine();
  PID(angle);
  moveMotors(PDI_signal);

  if (sonar_enabled) {
    float dist = getSonarDistance();
    if (dist >= 0) {
      Serial.print("📡 Distance: ");
      Serial.print(dist);
      Serial.println(" cm");
    } else {
      Serial.println("⚠️ Sonar timeout or error.");
    }
  }

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

float getSonarDistance() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

  long duration = pulseIn(SONAR_ECHO, HIGH, 20000);  // 20ms timeout
  if (duration == 0) {
    return -1;  // Timeout
  }

  float distance_cm = duration * 0.0343 / 2.0;
  return distance_cm;
}

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
    }
  }
}

void processCommand(String cmd) {
  if (cmd.startsWith("sonar=")) {
    sonar_enabled = cmd.substring(6).toInt() == 1;
    Serial.print("📡 Sonar is now ");
    Serial.println(sonar_enabled ? "ENABLED" : "DISABLED");

    // Use shorter BLE response to reduce buffer issues
    delay(10);
    respondToBLE(sonar_enabled ? "sonar=1" : "sonar=0");
  }

  // (Other BLE commands stay unchanged...)
  else if (cmd == "forward") {
    setpoint += 5;
    respondToBLE("Setpoint increased: " + String(setpoint));
  } else if (cmd == "stop") {
    setpoint = 0;
    respondToBLE("Setpoint reset: " + String(setpoint));
  } else if (cmd == "left") {
    turning_coeff -= 0.1;
    respondToBLE("Turning left: " + String(turning_coeff));
  } else if (cmd == "right") {
    turning_coeff += 0.1;
    respondToBLE("Turning right: " + String(turning_coeff));
  }

  // ... keep all your other commands here like kp=, ki=, x=, etc.
}

void respondToBLE(String response) {
  customCharacteristic.writeValue(response.c_str());
}

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("CJJ");
  BLE.setDeviceName("CJJ");
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);
  BLE.advertise();
}

void combine() {
  Accelerator();
  gyroscope();

  static float acc_angle_filtered = 0;
  float alpha = 0.5;
  acc_angle_filtered = alpha * acc_angle + (1 - alpha) * acc_angle_filtered;

  float k = 0.75;
  angle_filter_gry = k * gry_angle + (1 - k) * acc_angle_filtered;

  if (abs(gyroX) < 2.0) {
    angle = angle_filter_gry;
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
  } else {
    moving_mt = 0.1;
  }

  if (abs(controlSignal) > 30 && abs(derivative) >= 15) {
    left_pwmValue = abs(controlSignal) + 25;
    right_pwmValue = abs(controlSignal) + 25 + motor_difference;
  } else if (abs(derivative) < 15 || abs(controlSignal) < 30) {
    left_pwmValue = abs(controlSignal) + 25;
    right_pwmValue = abs(controlSignal) + 25 + motor_difference;
  } else {
    left_pwmValue = 0;
    right_pwmValue = 0;
  }

  if (mode == 0) {
    if (turning_coeff > 0) {
      right_pwmValue *= (1 - abs(turning_coeff) * 0.005);
    } else {
      left_pwmValue *= (1 - abs(turning_coeff) * 0.005);
    }
  } else {
    if (turning_coeff > 0) {
      right_pwmValue *= (1 - abs(turning_coeff) * 0.01);
    } else {
      left_pwmValue *= (1 - abs(turning_coeff) * 0.01);
    }
  }

  if (lt_trigger > 50.0) left_pwmValue = 0;
  if (rt_trigger > 50.0) right_pwmValue = 0;

  if (single_wheel == "left") right_pwmValue = 0;
  else if (single_wheel == "right") left_pwmValue = 0;

  if (abs(controlSignal) < 1) {
    right_pwmValue = 0;
    left_pwmValue = 0;
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

  angle_error = angle - (setpoint + moving_mt * moving_coeff);
  if (abs(angle_error) <= 0.1) angle_error = 0;

  if (abs(angle_error) > 0.5) {
    integral += angle_error * dt_PID;
  }

  integral = constrain(integral, -255, 255);
  derivative = -gyroX + gryoX_drift;
  derivative = constrain(derivative, -255, 255);

  PDI_signal = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
  PDI_signal = constrain(PDI_signal, -255, 255);

  float decel_threshold = 15.0;
  if (abs(previous_PDI) - abs(PDI_signal) > decel_threshold) {
    digitalWrite(A2, HIGH);
  } else {
    digitalWrite(A2, LOW);
  }

  previous_PDI = PDI_signal;
  previousError = angle_error;
  lastTime_PID = now_PID;
}
