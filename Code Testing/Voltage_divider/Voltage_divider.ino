#include <Servo.h>

// Motor and sensor pin definitions
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
#define batteryPin A3  // Voltage monitor pin

Servo scanServo;
int servoPos = 50;
bool sweepingRight = true;
unsigned long lastServoMoveTime = 0;

int forwardSpeed = 70;
int turnSpeed = 50;
int approachThreshold = 15;
int scanThreshold = 2.8;

bool ultrasonicEnabled = true;
bool approaching = false;
String detectedColor = "UNKNOWN";

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
  pinMode(batteryPin, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  scanServo.attach(servoPin);
  scanServo.write(50);
  delay(300);
}

void loop() {
  adaptiveSystemControl();

  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = ultrasonicEnabled ? getDistance() : 100;

  if (millis() - lastServoMoveTime >= 100) {
    servoPos += sweepingRight ? 20 : -20;
    if (servoPos >= 60) sweepingRight = false;
    if (servoPos <= 40) sweepingRight = true;
    scanServo.write(servoPos);
    lastServoMoveTime = millis();
  }

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    approaching = true;
    Serial.println("Obstacle detected at " + String(distance) + "cm");
    Stop(); delay(300);
  }

  if (approaching) {
    while (true) {
      distance = getDistance();
      if (distance <= scanThreshold || distance <= 0) break;

      leftIR = digitalRead(L_S);
      rightIR = digitalRead(R_S);
      if (leftIR == 1 || rightIR == 1) {
        Serial.println("Lost line during approach! Aborting.");
        Stop(); delay(300);
        ultrasonicEnabled = true; approaching = false;
        return;
      }

      analogWrite(enA, 120);
      analogWrite(enB, 120);
      forward(); delay(100); Stop(); delay(100);
    }

    Stop(); delay(300);
    readColor();
    scanWithServo();
    handleDetectedColor(detectedColor);
    delay(500);
    ultrasonicEnabled = true; approaching = false;
    return;
  }

  leftIR = digitalRead(L_S);
  rightIR = digitalRead(R_S);
  Serial.print("L_IR="); Serial.print(leftIR);
  Serial.print(" | R_IR="); Serial.print(rightIR);
  Serial.print(" | Distance="); Serial.println(getDistance());

  if ((leftIR == 0) && (rightIR == 0)) {
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
  } else if ((leftIR == 0) && (rightIR == 1)) {
    analogWrite(enA, turnSpeed);
    analogWrite(enB, turnSpeed);
    turnRight(); delay(100);
  } else if ((leftIR == 1) && (rightIR == 0)) {
    analogWrite(enA, turnSpeed);
    analogWrite(enB, turnSpeed);
    turnLeft(); delay(100);
  } else {
    Stop();
  }

  delay(50);
}

float readBatteryVoltage() {
  int raw = analogRead(batteryPin);
  float voltage = (raw / 1023.0) * 5.0;
  float batteryVoltage = voltage * (11.1 / 5.0); // Example for 3S LiPo
  return batteryVoltage;
}

void adjustParametersByVoltage(float voltage) {
  if (voltage >= 7.2) {
    forwardSpeed = 80; turnSpeed = 60;
  } else if (voltage >= 6.5) {
    forwardSpeed = 70; turnSpeed = 50;
  } else {
    forwardSpeed = 60; turnSpeed = 40;
  }
}

void adaptiveSystemControl() {
  float voltage = readBatteryVoltage();
  adjustParametersByVoltage(voltage);
}

int getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  int distance = duration * 0.034 / 2;
  return distance;
}

void readColor() {
  unsigned long red = 0, green = 0, blue = 0;
  int samples = 5;
  for (int i = 0; i < samples; i++) {
    digitalWrite(S2, LOW); digitalWrite(S3, LOW);
    red += pulseIn(colorOut, LOW);
    digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
    green += pulseIn(colorOut, LOW);
    digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
    blue += pulseIn(colorOut, LOW);
  }
  red /= samples; green /= samples; blue /= samples;
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
  scanServo.write(50); delay(300);
  scanServo.write(100); Serial.println("Scanning left..."); delay(500);
  scanServo.write(50); delay(300);
  scanServo.write(0); Serial.println("Scanning right..."); delay(500);
  scanServo.write(50); Serial.println("Returning to center..."); delay(500);
}

void driveForward(int cm, int speed = forwardSpeed) {
  int t = cm * 30;
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  forward(); delay(t); Stop(); delay(200);
}

void driveBackwards(int cm) {
  int t = cm * 30;
  analogWrite(enA, forwardSpeed);
  analogWrite(enB, forwardSpeed);
  backward(); delay(t); Stop(); delay(200);
}

void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void driveCurveLeft(int cm) {
  int t = cm * 30;
  analogWrite(enA, forwardSpeed / 2);
  analogWrite(enB, forwardSpeed);
  forward(); delay(t); Stop(); delay(200);
}

void rotate180() {
  analogWrite(enA, forwardSpeed);
  analogWrite(enB, forwardSpeed);
  turnLeft(); delay(800); Stop(); delay(200);
}

void handleDetectedColor(String color) {
  if (color == "RED") {
    turnRight(); delay(1000);
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward(); delay(400); Stop(); delay(300);
    turnLeft(); delay(500); Stop(); delay(300);
    forward(); delay(1000); Stop(); delay(300);
    turnLeft(); delay(500); Stop(); delay(300);
  } else if (color == "GREEN") {
    driveForward(40, 100); Stop(); delay(300);
    driveBackwards(60);
  } else if (color == "BLUE") {
    driveBackwards(30);
    driveCurveLeft(30);
    rotate180();
    Serial.println("BLUE maneuver complete → Parked");
    while (true);
  } else {
    Serial.println("Obstacle is " + color + " → Stopping");
    Stop();
  }
}
