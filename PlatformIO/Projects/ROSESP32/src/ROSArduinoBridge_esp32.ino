#include <Arduino.h>
#include <ESP32Encoder.h>

// Motor pins
#define RIGHT_MOTOR_FORWARD 22
#define RIGHT_MOTOR_BACKWARD 23
#define LEFT_MOTOR_FORWARD 3
#define LEFT_MOTOR_BACKWARD 2


#define RIGHT_MOTOR_ENABLE 21
#define LEFT_MOTOR_ENABLE 15

#define LEFT_MOTOR_PWM 13
#define RIGHT_MOTOR_PWM 12

// Encoder pins
#define LEFT_ENC_PIN_A 32
#define LEFT_ENC_PIN_B 33
#define RIGHT_ENC_PIN_A 25
#define RIGHT_ENC_PIN_B 26

// PID parameters
float kp = 0.5;
float ki = 0.1;
float kd = 0.01;

// Motor control
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

float desiredSpeed = 100.0; // RPM deseado
float leftSpeed = 0.0;
float rightSpeed = 0.0;
float leftErrorSum = 0;
float rightErrorSum = 0;
float lastLeftError = 0;
float lastRightError = 0;

unsigned long lastTime = 0;

void setup() {
  // Motor pins configuration
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  // Set up encoders
  leftEncoder.attachHalfQuad(LEFT_ENC_PIN_A, LEFT_ENC_PIN_B);
  rightEncoder.attachHalfQuad(RIGHT_ENC_PIN_A, RIGHT_ENC_PIN_B);
  leftEncoder.clearCount();
  rightEncoder.clearCount();

  // Enable motors
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);

  // Set initial PWM to 0
  ledcSetup(0, 5000, 8); // Channel 0 for left motor
  ledcAttachPin(LEFT_MOTOR_PWM, 0);
  ledcSetup(1, 5000, 8); // Channel 1 for right motor
  ledcAttachPin(RIGHT_MOTOR_PWM, 1);

  Serial.begin(115200);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Calculate speed from encoders
  long leftCount = leftEncoder.getCount();
  long rightCount = rightEncoder.getCount();

  // Assuming the encoder gives pulses per revolution
  float leftSpeed = (leftCount / 390.0) * 60.0 / dt; // RPM
  float rightSpeed = (rightCount / 390.0) * 60.0 / dt; // RPM

  leftEncoder.clearCount();
  rightEncoder.clearCount();

  // PID control for left motor
  float leftError = desiredSpeed - leftSpeed;
  leftErrorSum += leftError * dt;
  float leftDeltaError = (leftError - lastLeftError) / dt;
  lastLeftError = leftError;
  float leftOutput = kp * leftError + ki * leftErrorSum + kd * leftDeltaError;
  controlMotor(LEFT_MOTOR_FORWARD, LEFT_MOTOR_BACKWARD, 0, leftOutput);

  // PID control for right motor
  float rightError = desiredSpeed - rightSpeed;
  rightErrorSum += rightError * dt;
  float rightDeltaError = (rightError - lastRightError) / dt;
  lastRightError = rightError;
  float rightOutput = kp * rightError + ki * rightErrorSum + kd * rightDeltaError;
  controlMotor(RIGHT_MOTOR_FORWARD, RIGHT_MOTOR_BACKWARD, 1, rightOutput);

  // Debug output
  Serial.print("Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" RPM, Right Speed: ");
  Serial.print(rightSpeed);
  Serial.println(" RPM");
}

void controlMotor(int forwardPin, int backwardPin, int pwmChannel, float speed) {
  if (speed > 0) {
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
  } else if (speed < 0) {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
    speed = -speed;
  } else {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
  }
  ledcWrite(pwmChannel, constrain(speed, 0, 255));
}