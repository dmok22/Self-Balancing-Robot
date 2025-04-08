#include "Arduino_BMI270_BMM150.h"
#include <Wire.h>
#include <ArduinoBLE.h>
#include <Servo.h>

// BLE setup
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
  "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, 50, false);

// Motor & sensor pins
const int INPUT_B1 = 3;
const int INPUT_B2 = 5;
const int INPUT_A1 = 6;
const int INPUT_A2 = 9;
const int SONAR_TRIG = 7;
const int SONAR_ECHO = 8;
const int SERVO_PIN = 4;

// State variables
float dt;
long lastTime;
long lastTime_PID;
float angle, gry_angle, acc_angle, angle_filter_gry;
float gyroX, gyroY, gyroZ;
float gryoX_drift;
bool isCalibrated = false;
const int CALIBRATION_SAMPLES = 1000;

float setpoint = 0;
float PDI_signal, previous_PDI = 0;
float integral = 0, derivative;
float angle_error, previousError = 0;
float Kp = 17.5, Ki = 66, Kd = 1.37;
float sp = 0;
int motor_difference;
float turning_coeff = 0.0, moving_coeff = 0.0, moving_mt = 0.0;
int mode = 0;
float lt_trigger = 0.0, rt_trigger = 0.0;
String single_wheel = "";

// Signals
int left_signal_state = 0;
int right_signal_state = 0;
bool left_led_on = false;
bool right_led_on = false;
unsigned long last_blink_time = 0;
const unsigned long blink_interval = 500;

// Sonar
Servo sonarServo;
enum SonarState {
  IDLE, TO_CENTER, WAIT_BEFORE_LEFT, TO_LEFT, READ_LEFT,
  TO_RIGHT, READ_RIGHT, RETURN_CENTER, SEND_RESULTS, DONE
};
SonarState sonarState = IDLE;
unsigned long sonarTimer = 0;
float sonarLeft = 0, sonarRight = 0, sonarForward = 0;

void setup() {
  setupBLE();

  Serial.begin(9600);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  lastTime = millis();
  lastTime_PID = millis();
  delay(1000);

  pinMode(INPUT_A1, OUTPUT);
  pinMode(INPUT_A2, OUTPUT);
  pinMode(INPUT_B1, OUTPUT);
  pinMode(INPUT_B2, OUTPUT);

  pinMode(A0, OUTPUT);  // left signal
  pinMode(A1, OUTPUT);  // right signal
  pinMode(A2, OUTPUT);  // brake
  pinMode(A3, OUTPUT);  // headlight

  pinMode(SONAR_TRIG, OUTPUT);
  pinMode(SONAR_ECHO, INPUT);
  sonarServo.attach(SERVO_PIN);
  sonarServo.write(90);

  calibrateTarget();
}

void loop() {
  handleBLECommands();
  combine();
  handleSonarSweep();
  PID(angle);
  moveMotors(PDI_signal);

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

void calibrateTarget() {
  float sumAngle = 0.0, sumGyroX = 0.0;
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

void handleSonarSweep() {
  switch (sonarState) {
    case IDLE:
      return;

    case TO_CENTER:
      sonarServo.write(90);
      sonarTimer = millis();
      sonarState = WAIT_BEFORE_LEFT;
      break;

    case WAIT_BEFORE_LEFT:
      if (millis() - sonarTimer >= 400) {
        sonarServo.write(60);
        sonarTimer = millis();
        sonarState = READ_LEFT;
      }
      break;

    case READ_LEFT:
      if (millis() - sonarTimer >= 400) {
        sonarLeft = getSonarDistance();
        sonarServo.write(120);
        sonarTimer = millis();
        sonarState = READ_RIGHT;
      }
      break;

    case READ_RIGHT:
      if (millis() - sonarTimer >= 400) {
        sonarRight = getSonarDistance();
        sonarServo.write(90);
        sonarTimer = millis();
        sonarState = RETURN_CENTER;
      }
      break;

    case RETURN_CENTER:
      if (millis() - sonarTimer >= 400) {
        sonarForward = getSonarDistance();
        sonarTimer = millis();
        sonarState = SEND_RESULTS;
      }
      break;

    case SEND_RESULTS:
      respondToBLE("sonar_l=" + String(sonarLeft));
      delay(100);
      respondToBLE("sonar_r=" + String(sonarRight));
      delay(100);
      respondToBLE("sonar_f=" + String(sonarForward));
      delay(100);
      sonarState = DONE;
      break;

    case DONE:
      sonarState = IDLE;
      break;
  }
}

float getSonarDistance() {
  digitalWrite(SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG, LOW);

  long duration = pulseIn(SONAR_ECHO, HIGH, 20000);
  return duration == 0 ? -1 : (duration * 0.0343 / 2.0);
}

void handleBLECommands() {
  BLEDevice central = BLE.central();
  if (central) {
    if (customCharacteristic.written()) {
      int length = customCharacteristic.valueLength();
      char buffer[length + 1];
      memcpy(buffer, customCharacteristic.value(), length);
      buffer[length] = '\0';
      String cmd = String(buffer);
      processCommand(cmd);
      respondToBLE(cmd);
    }
  }
}

void processCommand(String cmd) {
  if (cmd == "forward") {
    setpoint += 5;
  } else if (cmd == "stop") {
    setpoint = 0;
  } else if (cmd == "left") {
    turning_coeff -= 0.1;
  } else if (cmd == "right") {
    turning_coeff += 0.1;
  } else if (cmd.startsWith("kp=")) {
    Kp = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("ki=")) {
    Ki = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("kd=")) {
    Kd = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("md=")) {
    motor_difference = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("sp=")) {
    sp = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("mode=")) {
    mode = cmd.substring(5).toInt();
  } else if (cmd.startsWith("x=")) {
    int commaIndex = cmd.indexOf(',');
    String xStr = cmd.substring(2, commaIndex);
    String yStr = cmd.substring(cmd.indexOf("y=") + 2);
    turning_coeff = xStr.toFloat();
    moving_coeff = yStr.toFloat();
  } else if (cmd.startsWith("lt=")) {
    lt_trigger = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("rt=")) {
    rt_trigger = cmd.substring(3).toFloat();
  } else if (cmd.startsWith("scan=")) {
    int doScan = cmd.substring(5).toInt();
    if (doScan == 1 && sonarState == IDLE) {
      sonarState = TO_CENTER;
      sonarTimer = millis();
      Serial.println("📡 Starting sonar scan...");
    }
  } else if (cmd.startsWith("single=")) {
    single_wheel = cmd.substring(7);
  } else if (cmd.startsWith("left_signal=")) {
    left_signal_state = cmd.substring(12).toInt();
    if (left_signal_state == 0) {
      digitalWrite(A0, LOW);
      left_led_on = false;
    }
  } else if (cmd.startsWith("right_signal=")) {
    right_signal_state = cmd.substring(13).toInt();
    if (right_signal_state == 0) {
      digitalWrite(A1, LOW);
      right_led_on = false;
    }
  } else if (cmd.startsWith("headlight=")) {
    int val = cmd.substring(10).toInt();
    digitalWrite(A3, val ? HIGH : LOW);
  }
}

void respondToBLE(String response) {
  customCharacteristic.writeValue(response.c_str());
}

void setupBLE() {
  if (!BLE.begin()) while (1);
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

  if (abs(gyroX) < 2.0) angle = angle_filter_gry;
  else angle = gry_angle;
}

void gyroscope() {
  long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  IMU.readGyroscope(gyroX, gyroY, gyroZ);
  gry_angle = angle + (-gyroX * dt);
}

void Accelerator() {
  float x, y, z;
  IMU.readAcceleration(x, y, z);
  acc_angle = atan2f(100 * y, 100 * z) * (180.0f / M_PI);
}

void moveMotors(float controlSignal) {
  int left_pwm = abs(controlSignal) + 25;
  int right_pwm = left_pwm + motor_difference;

  if (mode == 0) moving_mt = 0.05;
  else moving_mt = 0.1;

  if (turning_coeff > 0) right_pwm *= (1 - abs(turning_coeff) * 0.01);
  else left_pwm *= (1 - abs(turning_coeff) * 0.01);

  if (lt_trigger > 50) left_pwm = 0;
  if (rt_trigger > 50) right_pwm = 0;
  if (single_wheel == "left") right_pwm = 0;
  if (single_wheel == "right") left_pwm = 0;
  if (abs(controlSignal) < 1) left_pwm = right_pwm = 0;

  left_pwm = constrain(left_pwm, 0, 255);
  right_pwm = constrain(right_pwm, 0, 255);

  if (controlSignal > 0) {
    analogWrite(INPUT_A1, left_pwm); analogWrite(INPUT_A2, 0);
    analogWrite(INPUT_B1, right_pwm); analogWrite(INPUT_B2, 0);
  } else {
    analogWrite(INPUT_A1, 0); analogWrite(INPUT_A2, left_pwm);
    analogWrite(INPUT_B1, 0); analogWrite(INPUT_B2, right_pwm);
  }
}

void PID(float angle) {
  double now_PID = millis();
  float dt_PID = (now_PID - lastTime_PID) / 1000.0;

  angle_error = angle - (setpoint + moving_mt * moving_coeff);
  if (abs(angle_error) <= 0.1) angle_error = 0;
  if (abs(angle_error) > 0.5) integral += angle_error * dt_PID;

  integral = constrain(integral, -255, 255);
  derivative = -gyroX + gryoX_drift;
  derivative = constrain(derivative, -255, 255);

  PDI_signal = (Kp * angle_error) + (Ki * integral) + (Kd * derivative);
  PDI_signal = constrain(PDI_signal, -255, 255);

  if (abs(previous_PDI) - abs(PDI_signal) > 15.0) digitalWrite(A2, HIGH);
  else digitalWrite(A2, LOW);

  previous_PDI = PDI_signal;
  previousError = angle_error;
  lastTime_PID = now_PID;
}
