#include <Servo.h> 
#define enA 10
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enB 5
#define L_S A4
#define R_S A5
#define trigPin 12
#define echoPin 13
#define S0 A2
#define S1 A1
#define S2 0
#define S3 1
#define colorOut A0
#define servoPin 4
#define SERVO_CENTER 70

Servo scanServo;

int forwardSpeed = 70;
int turnSpeed = 80;
int curveDelay = 150;
int approachThreshold = 20;
int scanThreshold = 3;
int approachSpeed = 80;
bool ultrasonicEnabled = true;
bool approaching = false;
String detectedColor = "UNKNOWN";

void forward();
void backward();
void turnLeft();
void turnRight();
void Stop();
void curveLeft();
void curveRight();
void approachToObstacle();
void readColor();
void handleDetectedColor(String color);
void driveForward(int cm, int speed = 70);
void driveBackwards(int cm, int speed = 70);
void rotate90();
void rotateRight90();
void approachRightToLane();
int getDistance();

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
  scanServo.attach(servoPin);
  scanServo.write(SERVO_CENTER);
  delay(300);
}

void loop() {
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = getDistance();

  Serial.print("L_IR="); Serial.print(leftIR);
  Serial.print(" | R_IR="); Serial.print(rightIR);
  Serial.print(" | Distance="); Serial.println(distance);

  if (!approaching && distance > 0 && distance <= approachThreshold && leftIR == 0 && rightIR == 0) {
    approaching = true;
    Serial.println("Obstacle detected at " + String(distance) + "cm");
    Stop(); delay(300);
  }

  if (approaching) {
    approachToObstacle();
    return;
  }

  if (distance > 0 && distance <= approachThreshold) {
    Stop();
    Serial.println("Obstacle ahead! Stopping.");
    delay(500);
    return;
  }

  if (leftIR == 0 && rightIR == 0) {
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
  } else if (leftIR == 1 && rightIR == 0) {
    Stop(); delay(30);
    curveLeft();
  } else if (leftIR == 0 && rightIR == 1) {
    curveRight();
    Stop(); delay(30);
  } else {
    Stop();
    delay(100);
  }
}

void approachToObstacle() {
  int l, r, d;
  unsigned long stepTime = 30;
  while (true) {
    d = getDistance();
    l = digitalRead(L_S); r = digitalRead(R_S);
    if (l == 1 && r == 1) {
      Serial.println("Lost line during approach! Aborting.");
      Stop(); delay(300); ultrasonicEnabled = true; approaching = false; return;
    }
    if (d <= scanThreshold || d <= 0) break;
    if (l == 0 && r == 0) {
      analogWrite(enA, approachSpeed); analogWrite(enB, approachSpeed);
      forward(); delay(stepTime); Stop();
    } else if (l == 1 && r == 0) {
      analogWrite(enA, 0); analogWrite(enB, approachSpeed);
      digitalWrite(in1, LOW); digitalWrite(in2, LOW);
      digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
      delay(stepTime); Stop();
    } else if (l == 0 && r == 1) {
      analogWrite(enA, approachSpeed); analogWrite(enB, 0);
      digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
      digitalWrite(in3, LOW); digitalWrite(in4, LOW);
      delay(stepTime); Stop();
    }
    delay(10);
  }
  Stop(); delay(300); readColor(); handleDetectedColor(detectedColor);
  delay(500); approaching = false;
}

void readColor() {
  unsigned int red, green, blue;
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); red = pulseIn(colorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); green = pulseIn(colorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); blue = pulseIn(colorOut, LOW);
  if (red < green && red < blue) detectedColor = "RED";
  else if (green < red && green < blue) detectedColor = "GREEN";
  else detectedColor = "OTHER";
  Serial.print("R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.println(blue);
  Serial.println("Detected Color: " + detectedColor);
}

void handleDetectedColor(String color) {
  int colorScenarioSpeed = 80;
  if (color == "RED") {
    driveBackwards(12, colorScenarioSpeed);
    scanServo.write(180); delay(400);
    int leftDist = getDistance();
    scanServo.write(SERVO_CENTER); delay(200);
    if (leftDist > 20) {
      ultrasonicEnabled = false;
      rotate90(); delay(50);
      driveForward(30, colorScenarioSpeed); delay(200);
      rotateRight90(); delay(200);
      driveForward(20, colorScenarioSpeed);
      approachRightToLane();
      ultrasonicEnabled = true;
      Serial.println("RED: Path left is clear, performed left-right bypass.");
    } else {
      Serial.println("RED: Left not clear, only backed up and stopped.");
    }
    scanServo.write(SERVO_CENTER);
    Stop();
    Serial.println("RED scenario complete.");
  } else if (color == "GREEN") {
    Serial.println("GREEN scenario: Forward 40cm, then reverse 40cm on lane, ignore ultrasonic for 5s.");
    driveForward(20, 150);
    Stop(); delay(300);
    driveBackwards(20, 70);
    Stop(); delay(300);
    ultrasonicEnabled = false;
    unsigned long ignoreStart = millis();
    Serial.println("Ultrasonic disabled for 5 seconds...");
    while (millis() - ignoreStart < 5000) {
      int l = digitalRead(L_S);
      int r = digitalRead(R_S);
      if (l == 0 && r == 0) {
        analogWrite(enA, forwardSpeed);
        analogWrite(enB, forwardSpeed);
        forward();
      } else if (l == 1 && r == 0) {
        Stop(); delay(30);
        curveLeft();
      } else if (l == 0 && r == 1) {
        curveRight();
        Stop(); delay(30);
      } else {
        Stop();
        delay(100);
      }
      delay(20);
    }
    ultrasonicEnabled = true;
    Serial.println("Ultrasonic re-enabled, GREEN scenario complete: Lane reacquired.");
    Stop();
  } else {
    Serial.println("OTHER COLOR scenario: Back 10cm, scan right/left, park if right is clear.");
    driveBackwards(10, colorScenarioSpeed);
    delay(200);
    scanServo.write(0); delay(400);
    int rightDist = getDistance();
    scanServo.write(180); delay(400);
    int leftDist = getDistance();
    scanServo.write(SERVO_CENTER); delay(200);
    Serial.print("Scan distances: Right="); Serial.print(rightDist); Serial.print(", Left="); Serial.println(leftDist);
    if (rightDist > 20) {
      rotateRight90(); delay(200);
      Stop();
      Serial.println("Parked to the right.");
      while (1) { Stop(); }
    } else {
      Serial.println("Right not clear, not parking.");
    }
  }
}

void approachRightToLane() {
  int l, r;
  int leftSpeed = 65;
  int rightSpeed = 40;
  while (true) {
    l = digitalRead(L_S);
    r = digitalRead(R_S);
    if (l == 1 || r == 1) {
      Stop();
      delay(100);
      Serial.println("ApproachRightToLane: Black detected, resuming normal lane following.");
      break;
    }
    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    delay(120); Stop(); delay(5);
  }
  Stop();
}

void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void turnRight() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void Stop() {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void curveLeft() {
  analogWrite(enA, turnSpeed);
  
  turnLeft();
  delay(curveDelay);
  Stop();
}

void curveRight() {
  
  analogWrite(enB, turnSpeed);
  turnRight();
  delay(curveDelay);
  Stop();
}

void driveForward(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  forward();
  delay(t);
  Stop(); delay(200);
}

void driveBackwards(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  backward();
  delay(t);
  Stop(); delay(200);
}

void rotate90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(500);
  Stop(); delay(200);
}

void rotateRight90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(500);
  Stop(); delay(200);
}

int getDistance() {
  if (!ultrasonicEnabled) return 1000;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}

