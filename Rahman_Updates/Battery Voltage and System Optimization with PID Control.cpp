#include <Servo.h>

#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5
#define L_S A0
#define R_S A1
#define trigPin 12
#define echoPin 13
#define S0 11
#define S1 4
#define S2 3
#define S3 2
#define colorOut A2
#define servoPin A5
#define BATTERY_PIN A3  // Battery voltage analog input

Servo scanServo;
int servoPos = 50;
bool sweepingRight = true;
unsigned long lastServoMoveTime = 0;

int forwardSpeed = 150;
int turnSpeed = 120;
int approachThreshold = 15;  // cm
int scanThreshold = 2.8;     // cm

bool ultrasonicEnabled = true;
bool approaching = false;
String detectedColor = "UNKNOWN";

// Voltage divider resistor values for battery voltage measurement
const float R1 = 30000.0; // 30k ohms
const float R2 = 10000.0; // 10k ohms

// PID parameters
float batterySetpoint = 7.4;  // Nominal battery voltage (adjust for your battery)
float kp = 20.0;
float ki = 0.1;
float kd = 5.0;

float integral = 0;
float previousError = 0;
unsigned long lastPIDTime = 0;

float pidOutput = 0;  // PID correction output

void setup() {
  Serial.begin(9600);

  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  pinMode(BATTERY_PIN, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  scanServo.attach(servoPin);
  scanServo.write(50);
  delay(300);
  lastPIDTime = millis();
}

void loop() {
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = ultrasonicEnabled ? getDistance() : 100;

  // Read battery voltage and update PID output
  float voltage = getBatteryVoltage();
  pidOutput = calculatePID(voltage);

  // Servo scanning 
  if (millis() - lastServoMoveTime >= 100) {
    if (sweepingRight) {
      servoPos += 20;
      if (servoPos >= 60) sweepingRight = false;
    } else {
      servoPos -= 20;
      if (servoPos <= 40) sweepingRight = true;
    }
    scanServo.write(servoPos);
    lastServoMoveTime = millis();
  }

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    approaching = true;
    Serial.println("Obstacle detected at " + String(distance) + "cm");
    Stop();
    delay(300);
  }

  if (approaching) {
    while (true) {
      distance = getDistance();
      if (distance <= scanThreshold || distance <= 0) break;

      leftIR = digitalRead(L_S);
      rightIR = digitalRead(R_S);

      if (leftIR == 1 || rightIR == 1) {
        Serial.println("Lost line during approach! Aborting.");
        Stop();
        delay(300);
        ultrasonicEnabled = true;
        approaching = false;
        return;
      }

      int slowApproachSpeed = 120;
      int pwmSlowA = adjustPWM(slowApproachSpeed, pidOutput);
      int pwmSlowB = adjustPWM(slowApproachSpeed, pidOutput);
      analogWrite(enA, pwmSlowA);
      analogWrite(enB, pwmSlowB);
      forward();
      delay(100);
      Stop();
      delay(100);
    }

    Stop();
    delay(300);
    readColor();
    scanWithServo();
    handleDetectedColor(detectedColor);
    delay(500);
    ultrasonicEnabled = true;
    approaching = false;
    return;
  }

  // Regular line-following
  leftIR = digitalRead(L_S);
  rightIR = digitalRead(R_S);

  Serial.print("L_IR=");
  Serial.print(leftIR);
  Serial.print(" | R_IR=");
  Serial.print(rightIR);
  Serial.print(" | Distance=");
  Serial.println(distance);
  Serial.print(" | Battery V=");
  Serial.println(voltage, 2);

  if ((leftIR == 0) && (rightIR == 0)) {
    int pwmA = adjustPWM(forwardSpeed, pidOutput);
    int pwmB = adjustPWM(forwardSpeed, pidOutput);
    analogWrite(enA, pwmA);
    analogWrite(enB, pwmB);
    forward();
  } else if ((leftIR == 0) && (rightIR == 1)) {
    int pwmTurn = adjustPWM(turnSpeed, pidOutput);
    analogWrite(enA, pwmTurn);
    analogWrite(enB, pwmTurn);
    turnRight();
    delay(50);
    delay(50);
  } else if ((leftIR == 1) && (rightIR == 0)) {
    int pwmTurn = adjustPWM(turnSpeed, pidOutput);
    analogWrite(enA, pwmTurn);
    analogWrite(enB, pwmTurn);
    delay(50);
    turnLeft();
    delay(50);
  } else {
    Stop();
  }

  delay(50); // stability delay
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  int distance = duration * 0.034 / 2;
  return distance;
}

float getBatteryVoltage() {
  int sensorValue = analogRead(BATTERY_PIN);
  float voltage = sensorValue * (5.0 / 1023.0) * ((R1 + R2) / R2);
  return voltage;
}

float calculatePID(float currentVoltage) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0;
  if (dt <= 0) dt = 0.01;

  float error = batterySetpoint - currentVoltage;
  integral += error * dt;
  float derivative = (error - previousError) / dt;

  float output = kp * error + ki * integral + kd * derivative;

  previousError = error;
  lastPIDTime = now;

  return output;
}

int adjustPWM(int basePWM, float pidCorrection) {
  int adjustedPWM = basePWM + (int)pidCorrection;
  if (adjustedPWM > 255) adjustedPWM = 255;
  if (adjustedPWM < 0) adjustedPWM = 0;
  return adjustedPWM;
}

void readColor() {
  unsigned int red, green, blue;

  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  red = pulseIn(colorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  green = pulseIn(colorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blue = pulseIn(colorOut, LOW);

  if (red < green && red < blue) detectedColor = "RED";
  else if (green < red && green < blue) detectedColor = "GREEN";
  else if (blue < red && blue < green) detectedColor = "BLUE";
  else detectedColor = "UNKNOWN";

  Serial.print("R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.println(blue);
  Serial.println("Detected Color: " + detectedColor);
}

void scanWithServo() {
  scanServo.write(50);
  delay(300);
  scanServo.write(100);
  Serial.println("Scanning left...");
  delay(500);
  scanServo.write(50);
  delay(300);
  scanServo.write(0);
  Serial.println("Scanning right...");
  delay(500);
  scanServo.write(50);
  Serial.println("Returning to center...");
  delay(500);
}

void driveForward(int cm, int speed = 70) {
  int t = cm * 30;
  float voltage = getBatteryVoltage();
  pidOutput = calculatePID(voltage);
  int pwmA = adjustPWM(speed, pidOutput);
  int pwmB = adjustPWM(speed, pidOutput);
  analogWrite(enA, pwmA);
  analogWrite(enB, pwmB);
  forward();
  delay(t);
  Stop();
  delay(200);
}

void driveBackwards(int cm) {
  int estimatedTime = cm * 30;
  float voltage = getBatteryVoltage();
  pidOutput = calculatePID(voltage);
  int pwmA = adjustPWM(forwardSpeed, pidOutput);
  int pwmB = adjustPWM(forwardSpeed, pidOutput);
  analogWrite(enA, pwmA);
  analogWrite(enB, pwmB);
  backward();
  delay(estimatedTime);
  Stop();
  delay(200);
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnRight() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void driveCurveLeft(int cm) {
  int t = cm * 30;
  float voltage = getBatteryVoltage();
  pidOutput = calculatePID(voltage);
  int pwmA = adjustPWM(forwardSpeed / 2, pidOutput);
  int pwmB = adjustPWM(forwardSpeed, pidOutput);
  analogWrite(enA, pwmA);  // Left motor slower
  analogWrite(enB, pwmB);  // Right motor normal
  forward();
  delay(t);
  Stop();
  delay(200);
}

void rotate180() {
  float voltage = getBatteryVoltage();
  pidOutput = calculatePID(voltage);
  int pwmA = adjustPWM(forwardSpeed, pidOutput);
  int pwmB = adjustPWM(forwardSpeed, pidOutput);
  analogWrite(enA, pwmA);
  analogWrite(enB, pwmB);
  turnLeft();  // Rotate in place
  delay(800); // wait till rotate 180° is Done
  Stop();
  delay(200);
}

void handleDetectedColor(String color) {
  if (color == "RED") {
    turnRight();
    delay(1000);
    {
      float voltage = getBatteryVoltage();
      pidOutput = calculatePID(voltage);
      int pwmA = adjustPWM(forwardSpeed, pidOutput);
      int pwmB = adjustPWM(forwardSpeed, pidOutput);
      analogWrite(enA, pwmA);
      analogWrite(enB, pwmB);
    }
    forward();
    delay(400); // 20 cm
    Stop();
    delay(300);

    turnLeft();
    delay(500); // recentering
    Stop();
    delay(300);

    forward();
    delay(1000); // move 50 cm forward
    Stop();
    delay(300);

    turnLeft();
    delay(500); // scan for black line again
    Stop();
    delay(300);
  }

  else if (color == "GREEN") {
    driveForward(40, 100);
    Stop();
    delay(300);
    driveBackwards(60);
  }

else if (color == "BLUE") {
    driveBackwards(30);
    driveCurveLeft(30);
    rotate180();
    Serial.println("BLUE maneuver complete → Parked");
  while (true);  // Stay parked
}
  else {
    Serial.println("Obstacle is " + color + " → Stopping");
    Stop();
  }
}
