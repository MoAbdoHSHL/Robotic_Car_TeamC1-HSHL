#include <Servo.h>

// Motor Pins
const int enA = 10, in1 = 9, in2 = 8, in3 = 7, in4 = 6, enB = 5;

// IR Sensors
const int leftIRSensor = A0;
const int rightIRSensor = A1;

// Ultrasonic Sensor
const int trigPin = 12;
const int echoPin = 13;

// Color Sensor
const int S0 = 11, S1 = 4, S2 = 3, S3 = 2;
const int colorOut = A2;

// Servo
const int servoPin = A5;
Servo scanServo;

// Control Parameters
int forwardSpeed = 75;
int turnSpeed = 80;
int approachThreshold = 20;
int scanThreshold = 3;

// State Variables
bool approaching = false;
bool ultrasonicEnabled = true;
String detectedColor = "UNKNOWN";

// Setup
void setup() {
  Serial.begin(9600);
  configurePins();
  scanServo.attach(servoPin);
  scanServo.write(50);
  delay(300);
}

// Main Loop
void loop() {
  int leftIR = digitalRead(leftIRSensor);
  int rightIR = digitalRead(rightIRSensor);
  int distance = ultrasonicEnabled ? getDistance() : 100;

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    handleApproach();
  }

  if (!approaching) {
    followLine(leftIR, rightIR, distance);
  }
}

// Configuration
void configurePins() {
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(leftIRSensor, INPUT); pinMode(rightIRSensor, INPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);
}

// Line Following Logic
void followLine(int leftIR, int rightIR, int distance) {
  Serial.print("L_IR="); Serial.print(leftIR);
  Serial.print(" | R_IR="); Serial.print(rightIR);
  Serial.print(" | Distance="); Serial.println(distance);

  if (leftIR == 0 && rightIR == 0) {
    moveForward(forwardSpeed);
  } else if (leftIR == 0 && rightIR == 1) {
    attemptCorrection(turnRight);
  } else if (leftIR == 1 && rightIR == 0) {
    attemptCorrection(turnLeft);
  } else {
    Stop(); delay(100);
  }
}

// Handle Object Approach
void handleApproach() {
  approaching = true;
  Serial.println("Obstacle detected → Stopping to scan");
  Stop(); delay(300);

  while (true) {
    int distance = getDistance();
    if (distance <= scanThreshold || distance <= 0) break;

    if (digitalRead(leftIRSensor) == 1 || digitalRead(rightIRSensor) == 1) {
      Serial.println("Lost line during approach. Aborting.");
      Stop(); delay(300);
      ultrasonicEnabled = true;
      approaching = false;
      return;
    }

    moveForward(120);
    delay(50);
    Stop(); delay(50);
  }

  Stop(); delay(300);
  readColor();
  handleDetectedColor(detectedColor);
  delay(500);
  ultrasonicEnabled = true;
  approaching = false;
}

// Movement Helpers
void moveForward(int speed) {
  analogWrite(enA, speed); analogWrite(enB, speed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void moveBackward(int speed = forwardSpeed, int duration = 500) {
  analogWrite(enA, speed); analogWrite(enB, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(duration);
  Stop(); delay(200);
}

void driveForward(int cm, int speed = forwardSpeed) {
  moveForward(speed);
  delay(cm * 30);
  Stop(); delay(200);
}

void driveCurveLeft(int cm) {
  analogWrite(enA, forwardSpeed / 2);
  analogWrite(enB, forwardSpeed);
  moveForward(forwardSpeed);

}

void rotate180() {
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  turnLeft(); delay(900);
  Stop(); delay(200);
}

// Try to Recover Line
void attemptCorrection(void (*turnFunc)()) {
  Stop(); delay(100);
  for (int i = 0; i < 3; i++) {
    analogWrite(enA, turnSpeed); analogWrite(enB, turnSpeed);
    delay(120);
    turnFunc(); delay(120);
    Stop(); delay(100);
    if (digitalRead(leftIRSensor) == 0 && digitalRead(rightIRSensor) == 0) break;
  }
}

// Ultrasonic Distance
int getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration == 0 ? -1 : duration * 0.034 / 2;
}

// Color Detection
void readColor() {
  unsigned int red, green, blue;

  digitalWrite(S2, LOW); digitalWrite(S3, LOW); red = pulseIn(colorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); green = pulseIn(colorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); blue = pulseIn(colorOut, LOW);

  if (red < green && red < blue) detectedColor = "RED";
  else if (green < red && green < blue) detectedColor = "GREEN";
  else if (blue < red && blue < green) detectedColor = "BLUE";
  else detectedColor = "UNKNOWN";

  Serial.print("R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.println(blue);
  Serial.println("Detected Color: " + detectedColor);
}

// Servo Scanning
void scanWithServo() {
  scanServo.write(50); delay(300);
  scanServo.write(100); Serial.println("Scanning left..."); delay(500);
  scanServo.write(0); Serial.println("Scanning right..."); delay(500);
  scanServo.write(50); Serial.println("Returning to center..."); delay(500);
}

// Handle Color-Based Action
void handleDetectedColor(String color) {
  if (color == "RED") {
    moveBackward(); scanWithServo(); rotate180();
    Serial.println("RED → Reversed and turned");
  } else if (color == "GREEN") {
    driveForward(40, 100); delay(300);
    moveBackward(forwardSpeed, 1800);
    Serial.println("GREEN → Forward then back");
  } else if (color == "BLUE") {
    moveBackward(forwardSpeed, 900);
    driveCurveLeft(30);
    rotate180();
    Serial.println("BLUE → Parked");
    while (true);  // halt
  } else {
    Serial.println("UNKNOWN color → Stopping");
    Stop();
  }
}
