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

#define SERVO_CENTER 70

float distance = 10;
bool ultrasonicEnabled = true;
int speedA = 62;
int speedB = 62;
int turnSpeed = 60;
int curveDelay = 150;
int forwardSpeed = 70;

Servo scanServo;
String detectedColor = "OTHER";

// --- ENUM and FSM variables ---
enum ColorCode { RED, GREEN, OTHER };
ColorCode currentColor;
bool colorActionInProgress = false;
unsigned long actionStartTime = 0;
int colorState = 0;

// --- Function declarations ---
ColorCode getColorCode(String color);
void handleDetectedColor();
void readColor();

// --- Setup and Loop ---
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
  if (distance > 0 && distance <= 6 && !colorActionInProgress) {
    stopMotors();
    readColor();
    currentColor = getColorCode(detectedColor);
    colorActionInProgress = true;
    actionStartTime = millis();
    colorState = 0;
  }

  if (colorActionInProgress) {
    handleDetectedColor();
    return;
  }

  int left = digitalRead(L_S);
  int right = digitalRead(R_S);
  if (left == 0 && right == 0) {
    moveForward(speedA, speedB);
  } else if (left == 0 && right == 1) {
    leftForward(80); rightBackward(60);
  } else if (left == 1 && right == 0) {
    leftBackward(60); rightForward(80);
  } else {
    stopMotors();
  }
}

// --- Color Detection and Handling ---
ColorCode getColorCode(String color) {
  if (color == "RED") return RED;
  if (color == "GREEN") return GREEN;
  return OTHER;
}

void handleDetectedColor() {
  static int rightDist = 0, leftDist = 0;

  switch (currentColor) {
    case RED:
      switch (colorState) {
        case 0:
          driveBackwards(12, 80);
          scanServo.write(180);
          actionStartTime = millis();
          colorState = 1;
          break;
        case 1:
          if (millis() - actionStartTime >= 400) {
            leftDist = getDistance();
            scanServo.write(SERVO_CENTER);
            if (leftDist > 20) {
              ultrasonicEnabled = false;
              rotate90();
              driveForward(20, 80);
              rotateRight90();
              driveForward(20, 80);
              approachRightToLane();
              ultrasonicEnabled = true;
            }
            stopMotors();
            colorActionInProgress = false;
          }
          break;
      }
      break;

    case GREEN:
      switch (colorState) {
        case 0:
          driveForward(20, 150);
          stopMotors();
          actionStartTime = millis();
          colorState = 1;
          break;
        case 1:
          if (millis() - actionStartTime >= 300) {
            driveBackwards(40, 70);
            stopMotors();
            actionStartTime = millis();
            colorState = 2;
          }
          break;
        case 2:
          if (millis() - actionStartTime >= 300) {
            ultrasonicEnabled = false;
            actionStartTime = millis();
            colorState = 3;
          }
          break;
        case 3:
          if (millis() - actionStartTime < 5000) {
            int l = digitalRead(L_S);
            int r = digitalRead(R_S);
            if (l == 0 && r == 0) {
              analogWrite(enA, forwardSpeed); analogWrite(enB, forwardSpeed);
              forward();
            } else if (l == 1 && r == 0) {
              stopMotors(); curveLeft();
            } else if (l == 0 && r == 1) {
              curveRight(); stopMotors();
            } else {
              stopMotors();
            }
          } else {
            ultrasonicEnabled = true;
            stopMotors();
            colorActionInProgress = false;
          }
          break;
      }
      break;

    case OTHER:
      switch (colorState) {
        case 0:
          driveBackwards(10, 80);
          scanServo.write(0);
          actionStartTime = millis();
          colorState = 1;
          break;
        case 1:
          if (millis() - actionStartTime >= 400) {
            rightDist = getDistance();
            scanServo.write(180);
            actionStartTime = millis();
            colorState = 2;
          }
          break;
        case 2:
          if (millis() - actionStartTime >= 400) {
            leftDist = getDistance();
            scanServo.write(SERVO_CENTER);
            if (rightDist > 40) {
              rotateRight90();
              while (1) stopMotors();
            }
            colorActionInProgress = false;
          }
          break;
      }
      break;
  }
}

// --- Sensor Reading ---
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

int getDistance() {
  if (!ultrasonicEnabled) return 1000;
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}

// --- Movement ---
void stopMotors() {
  analogWrite(enA, 0); analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

void forward() {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void driveForward(int cm, int speed) {
  int t = cm * 30;
  analogWrite(enA, speed); analogWrite(enB, speed);
  forward();
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

void curveLeft() {
  analogWrite(enA, turnSpeed); analogWrite(enB, turnSpeed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(curveDelay); stopMotors();
}

void curveRight() {
  analogWrite(enA, turnSpeed); analogWrite(enB, turnSpeed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(curveDelay); stopMotors();
}

void rotate90() {
  analogWrite(enA, 100); analogWrite(enB, 100);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  delay(500); stopMotors(); delay(200);
}

void rotateRight90() {
  analogWrite(enA, 100); analogWrite(enB, 100);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  delay(500); stopMotors(); delay(200);
}

void approachRightToLane() {
  int l, r;
  while (true) {
    l = digitalRead(L_S);
    r = digitalRead(R_S);
    if (l == 1 || r == 1) {
      stopMotors(); break;
    }
    analogWrite(enA, 65); analogWrite(enB, 40);
    forward();
    delay(120); stopMotors(); delay(5);
  }
}

void moveForward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1); analogWrite(enB, speedVal2);
  forward();
}

void leftForward(int speedVal) {
  analogWrite(enA, speedVal); digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
}

void leftBackward(int speedVal) {
  analogWrite(enA, speedVal); digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
}

void rightForward(int speedVal) {
  analogWrite(enB, speedVal); digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void rightBackward(int speedVal) {
  analogWrite(enB, speedVal); digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}
