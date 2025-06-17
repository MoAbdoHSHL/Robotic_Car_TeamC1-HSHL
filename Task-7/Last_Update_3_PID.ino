// PID-based Line Follower with Obstacle & Color Detection + Ramp-Up Acceleration
#include <Servo.h>

// Motor pins
#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5

// Sensor & control pins
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

Servo scanServo;

// Motor speed control
int forwardSpeed = 90;
int baseSpeed = 80;
int maxSpeed = 150;

// Obstacle & approach config
int approachThreshold = 20;
int scanThreshold = 3;

// State tracking
bool ultrasonicEnabled = true;
bool approaching = false;
bool debug = true;
bool hasStarted = false;
String detectedColor = "UNKNOWN";

// PID control variables
float Kp = 40.0; // TRY 20 && 30  Abdo
//car turns too slow or doesn’t correct much → Increase Kp
//car oscillates (zig-zags side to side) → Decrease Kp 

float Ki = 0.0;  // Try 0.5 when drifts off and does not return on lane again
float Kd = 25.0; // Try 15  //dampen the overshoot range (5 : 20) better
float lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(R_S, INPUT); pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(colorOut, INPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);
  scanServo.attach(servoPin); scanServo.write(50);
  delay(300);
}

void loop() {
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = ultrasonicEnabled ? getDistance() : 100;

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    approaching = true;
    if (debug) Serial.println("Obstacle detected at " + String(distance) + "cm");
    Stop(); delay(300);
  }

  if (approaching) {
    while (true) {
      distance = getDistance();
      if (distance <= scanThreshold || distance <= 0) break;
      leftIR = digitalRead(L_S); rightIR = digitalRead(R_S);
      if (leftIR == 1 || rightIR == 1) {
        if (debug) Serial.println("Lost line during approach! Aborting.");
        Stop(); delay(300); ultrasonicEnabled = true; approaching = false; return;
      }
      analogWrite(enA, 120); analogWrite(enB, 120);
      forward(); delay(50); Stop(); delay(50);
    }
    Stop(); delay(300);
    readColor(); handleDetectedColor(detectedColor);
    delay(500); ultrasonicEnabled = true; approaching = false; return;
  }

  lineFollowPID();
}

void lineFollowPID() {
  int left = digitalRead(L_S);
  int right = digitalRead(R_S);

  int error = 0;
  if (left == 0 && right == 1) error = -1;
  else if (left == 1 && right == 0) error = 1;
  else if (left == 1 && right == 1) error = 0;

  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftMotorSpeed = constrain(baseSpeed - correction, 0, maxSpeed);
  int rightMotorSpeed = constrain(baseSpeed + correction, 0, maxSpeed);

  if (!hasStarted) {
    rampUpToSpeed(min(leftMotorSpeed, rightMotorSpeed));
    hasStarted = true;
  } else {
    analogWrite(enA, leftMotorSpeed);
    analogWrite(enB, rightMotorSpeed);
    forward();
  }

  lastError = error;

  if (debug) {
    Serial.print("Left IR: "); Serial.print(left);
    Serial.print(" | Right IR: "); Serial.print(right);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | Correction: "); Serial.println(correction);
  }
}

void rampUpToSpeed(int targetSpeed) {
  for (int s = 0; s <= targetSpeed; s += 5) {
    analogWrite(enA, s);
    analogWrite(enB, s);
    forward();
    delay(15);
  }
}

void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

int getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}

void readColor() {
  unsigned int red, green, blue;
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); red = pulseIn(colorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); green = pulseIn(colorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); blue = pulseIn(colorOut, LOW);

  if (red < green && red < blue) detectedColor = "RED";
  else if (green < red && green < blue) detectedColor = "GREEN";
  else if (blue < red && blue < green) detectedColor = "BLUE";
  else detectedColor = "UNKNOWN";

  if (debug) {
    Serial.print("R="); Serial.print(red);
    Serial.print(" G="); Serial.print(green);
    Serial.print(" B="); Serial.println(blue);
    Serial.println("Detected Color: " + detectedColor);
  }
}

void handleDetectedColor(String color) {
  if (color == "RED") {
    driveBackwards(5); scanWithServo(); rotate180();
    if (debug) Serial.println("RED maneuver complete → Resuming line following");
  } else if (color == "GREEN") {
    driveForward(40, 100); Stop(); delay(300); driveBackwards(40);
  } else if (color == "BLUE") {
    driveBackwards(5); scanWithServo(); rotate90(); driveBackwards(30);
    Stop(); if (debug) Serial.println("BLUE maneuver complete → Parked");
    while (true);
  } else {
    if (debug) Serial.println("Obstacle is " + color + " → Stopping");
    Stop();
  }
}

void driveForward(int cm, int speed = forwardSpeed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  forward(); delay(t); Stop(); delay(200);
}

void driveBackwards(int cm) {
  int t = cm * 30;
  analogWrite(enA, forwardSpeed); analogWrite(enB, forwardSpeed);
  backward(); delay(t); Stop(); delay(200);
}

void rotate180() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(900); Stop(); delay(200);
}

void rotate90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(450); Stop(); delay(200);
}

void scanWithServo() {
  scanServo.write(50); delay(300);
  scanServo.write(100); if (debug) Serial.println("Scanning left..."); delay(500);
  scanServo.write(0);   if (debug) Serial.println("Scanning right..."); delay(500);
  scanServo.write(50);  if (debug) Serial.println("Returning to center..."); delay(500);
}
