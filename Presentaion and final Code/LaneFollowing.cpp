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
#define servoPin 4
#define S0 A2
#define S1 A1
#define S2 2
#define S3 3
#define sensorOut A0
#define OE 11

float distance = 10;
bool avoidingObstacle = false;
unsigned long reengageStartTime = 0;
bool reengageInProgress = false;
int speedA = 58;
int speedB = 58;
Servo scanServo;
String detectedColor = "OTHER";
bool ultrasonicEnabled = true;
#define SERVO_CENTER 70
int turnSpeed = 60;
int curveDelay = 150;
int forwardSpeed = 70;


void readColor() {
  unsigned int red, green, blue;
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); red = pulseIn(sensorOut, LOW);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); green = pulseIn(sensorOut, LOW);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); blue = pulseIn(sensorOut, LOW);
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
    delay(200);
    driveBackwards(12, colorScenarioSpeed);
    scanServo.write(180); delay(400);
    int leftDist = getDistance();
    scanServo.write(SERVO_CENTER); delay(200);
    if (leftDist > 20) {
      ultrasonicEnabled = false;
      rotate90(); delay(50);
      driveForward(20, colorScenarioSpeed); delay(200);
      rotateRight90(); delay(200);
      driveForward(20, colorScenarioSpeed);
      approachRightToLane();
      ultrasonicEnabled = true;
      Serial.println("RED: Path left is clear, performed left-right bypass.");
    } else {
      Serial.println("RED: Left not clear, only backed up and stopped.");
    }
    scanServo.write(SERVO_CENTER);
    stopMotors();
    Serial.println("RED scenario complete.");
  } else if (color == "GREEN") {
    delay(200);
    driveForward(20, 150);
    stopMotors(); delay(300);
    driveBackwards(40, 70);
    stopMotors(); delay(300);
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
        stopMotors(); delay(30);
        curveLeft();
      } else if (l == 0 && r == 1) {
        curveRight();
        stopMotors(); delay(30);
      } else {
        stopMotors();
        delay(100);
      }
      delay(20);
    }
    ultrasonicEnabled = true;
    Serial.println("Ultrasonic re-enabled, GREEN scenario complete: Lane reacquired.");
    stopMotors();
  } else {
    delay(200);
    driveBackwards(10, colorScenarioSpeed);
    delay(200);
    scanServo.write(0); delay(400);
    int rightDist = getDistance();
    scanServo.write(180); delay(400);
    int leftDist = getDistance();
    scanServo.write(SERVO_CENTER); delay(200);
    Serial.print("Scan distances: Right="); Serial.print(rightDist); Serial.print(", Left="); Serial.println(leftDist);
    if (rightDist > 40) {
      rotateRight90(); delay(200);
      stopMotors();
      Serial.println("Parked to the right.");
      while (1) { stopMotors(); }
    } else {
      Serial.println("Right not clear, not parking.");
    }
  }
}

void driveForward(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(t);
  stopMotors(); delay(200);
}

void driveBackwards(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(t);
  stopMotors(); delay(200);
}

void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void curveLeft() {
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(curveDelay);
  stopMotors();
}

void curveRight() {
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(curveDelay);
  stopMotors();
}

void rotate90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(500);
  stopMotors(); delay(200);
}

void rotateRight90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(500);
  stopMotors(); delay(200);
}

void approachRightToLane() {
  int l, r;
  int leftSpeed = 65;
  int rightSpeed = 40;
  while (true) {
    l = digitalRead(L_S);
    r = digitalRead(R_S);
    if (l == 1 || r == 1) {
      stopMotors();
      delay(100);
      Serial.println("ApproachRightToLane: Black detected, resuming normal lane following.");
      break;
    }
    analogWrite(enA, leftSpeed);
    analogWrite(enB, rightSpeed);
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
    delay(120); stopMotors(); delay(5);
  }
  stopMotors();
}

int getDistance() {
  if (!ultrasonicEnabled) return 1000;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}



void setup() {
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(L_S, INPUT); pinMode(R_S, INPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OE, OUTPUT); pinMode(sensorOut, INPUT);
  digitalWrite(OE, LOW);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  scanServo.attach(servoPin);
  scanServo.write(SERVO_CENTER);
  Serial.begin(9600);
  Serial.println("Robot with Color Detection Started");
}

void loop() {
  distance = getDistance();
  if (distance > 0 && distance <= 6) {
    stopMotors();
    delay(500);
    readColor();
    handleDetectedColor(detectedColor);
    delay(500);
    return;
  }
  int left = digitalRead(L_S);
  int right = digitalRead(R_S);
  Serial.print("Left IR: "); Serial.print(left);
  Serial.print(" | Right IR: "); Serial.println(right);
  if (left == 0 && right == 0) {
    moveForward(speedA, speedB);
  }
  else if (left == 0 && right == 1) {
    leftForward(80);
    rightBackward(67);
    
  }
  else if (left == 1 && right == 0) {
    leftBackward(67);
    rightForward(80);
    
  }
  else {
    stopMotors();
  }
}


void moveForward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void moveBackward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void leftForward(int speedVal) {
  analogWrite(enA, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void leftBackward(int speedVal) {
  analogWrite(enA, speedVal);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void rightForward(int speedVal) {
  analogWrite(enB, speedVal);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void rightBackward(int speedVal) {
  analogWrite(enB, speedVal);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveForward1(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void handleObstacle() {
  stopMotors(); delay(300);
  moveBackward(80, 80); delay(500); stopMotors(); delay(300);
  leftBackward(180); rightForward(180); delay(180); stopMotors(); delay(300);
  moveForward(65, 65); delay(1600); stopMotors(); delay(300);
  leftForward(200); rightBackward(200); delay(280); stopMotors(); delay(300);
  moveForward1(80); delay(300);
  waitForLineDetection();
  avoidingObstacle = false;
}

void waitForLineDetection() {
  Serial.println("Searching for line...");
  while (true) {
    int l = digitalRead(L_S);
    int r = digitalRead(R_S);
    if (l == 1 || r == 1) {
      Serial.println("Line detected!");
      break;
    } else {
      moveForward1(50);
    }
    delay(10);
  }
}