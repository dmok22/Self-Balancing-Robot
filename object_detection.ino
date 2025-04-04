#include <Servo.h>

// Pin definitions
const int trigPin = 9;   // Ultrasonic sensor trigger pin
const int echoPin = 10;  // Ultrasonic sensor echo pin
const int servoPin = 11; // Servo motor control pin

// Constants
const int SERVO_LEFT = 0;     // Left position (degrees)
const int SERVO_CENTER = 90;  // Center position (degrees)
const int SERVO_RIGHT = 180;  // Right position (degrees)
const int SAFE_DISTANCE = 20; // Safe distance in centimeters

// Create servo object
Servo sensorServo;

void setup()
{
    // Initialize serial communication
    Serial.begin(9600);

    // Configure pins
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Attach servo to pin
    sensorServo.attach(servoPin);

    // Center the servo at startup
    sensorServo.write(SERVO_CENTER);
    delay(1000);
}

void loop()
{
    // Check distance at current position
    int distance = getDistance();

    // If obstacle detected
    if (distance < SAFE_DISTANCE)
    {
        // Look left
        sensorServo.write(SERVO_LEFT);
        delay(1000);
        int leftDistance = getDistance();

        // Look right
        sensorServo.write(SERVO_RIGHT);
        delay(1000);
        int rightDistance = getDistance();

        // Return to center
        sensorServo.write(SERVO_CENTER);
        delay(500);

        // Choose direction with more space
        if (leftDistance > rightDistance && leftDistance > SAFE_DISTANCE)
        {
            // Turn left (add your motor control code here)
            Serial.println("Turning left");
        }
        else if (rightDistance > leftDistance && rightDistance > SAFE_DISTANCE)
        {
            // Turn right (add your motor control code here)
            Serial.println("Turning right");
        }
        else
        {
            // No clear path, reverse (add your motor control code here)
            Serial.println("Reversing");
        }
    }
    else
    {
        // Path is clear, move forward (add your motor control code here)
        Serial.println("Moving forward");
    }

    delay(100); // Small delay before next check
}

// Function to measure distance using ultrasonic sensor
int getDistance()
{
    // Clear trigger pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Send 10μs pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read echo pulse duration
    long duration = pulseIn(echoPin, HIGH);

    // Calculate distance in centimeters
    // Speed of sound = 343m/s = 34300cm/s
    // Distance = (Time × Speed) ÷ 2
    int distance = duration * 0.034 / 2;

    return distance;
}
