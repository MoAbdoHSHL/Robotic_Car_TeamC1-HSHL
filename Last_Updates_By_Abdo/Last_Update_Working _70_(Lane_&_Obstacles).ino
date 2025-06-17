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

Servo scanServo;
int forwardSpeed = 80;
int turnSpeed = 65;
int approachThreshold = 20;
int scanThreshold = 3;
bool ultrasonicEnabled = true;
bool approaching = false;
String detectedColor = "UNKNOWN";

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
    approaching = true; Serial.println("Obstacle detected at " + String(distance) + "cm"); Stop(); delay(300);
  }

  if (approaching) {
    while (true) {
      distance = getDistance();
      if (distance <= scanThreshold || distance <= 0) break;
      leftIR = digitalRead(L_S); rightIR = digitalRead(R_S);
      if (leftIR == 1 || rightIR == 1) {
        Serial.println("Lost line during approach! Aborting.");
        Stop(); delay(300); ultrasonicEnabled = true; approaching = false; return;
      }
      analogWrite(enA, 120); analogWrite(enB, 120); forward(); delay(50); Stop(); delay(50);
    }
    Stop(); delay(300); readColor(); handleDetectedColor(detectedColor);
    delay(500); ultrasonicEnabled = true; approaching = false; return;
  }

  leftIR = digitalRead(L_S); rightIR = digitalRead(R_S);
  Serial.print("L_IR="); Serial.print(leftIR);
  Serial.print(" | R_IR="); Serial.print(rightIR);
  Serial.print(" | Distance="); Serial.println(distance);

  if (leftIR == 0 && rightIR == 0) {
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
  } else if (leftIR == 1 && rightIR == 0) {
    curveLeft();
  } else if (leftIR == 0 && rightIR == 1) {
    curveRight();
  } else if (leftIR == 1 && rightIR == 1) {
    Stop(); delay(100);
    driveBackwards(5);
    int l = digitalRead(L_S), r = digitalRead(R_S);
    if (l == 0 || r == 0) return;
    rotate180();
  } else {
    Stop(); delay(100);
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
  analogWrite(enA, turnSpeed / 2);
  analogWrite(enB, turnSpeed);
  forward();
}


void curveRight() {
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed / 2);
  forward();
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
  Serial.print("R="); Serial.print(red);
  Serial.print(" G="); Serial.print(green);
  Serial.print(" B="); Serial.println(blue);
  Serial.println("Detected Color: " + detectedColor);
}

void scanWithServo() {
  scanServo.write(50); delay(300); scanServo.write(100);
  Serial.println("Scanning left..."); delay(500);
  scanServo.write(50); delay(300); scanServo.write(0);
  Serial.println("Scanning right..."); delay(500);
  scanServo.write(50); Serial.println("Returning to center..."); delay(500);
}

void handleDetectedColor(String color) {
  if (color == "RED") {
    driveBackwards(5);
    scanWithServo();
    rotate180();
    Serial.println("RED maneuver complete → Resuming line following");
  } else if (color == "GREEN") {
    driveForward(40, 100); Stop(); delay(300); driveBackwards(40);
  } else if (color == "BLUE") {
    driveBackwards(5);
    scanWithServo();
    rotate90();
    driveBackwards(30);
    Stop();
    Serial.println("BLUE maneuver complete → Parked");
    while (true);
  } else {
    Serial.println("Obstacle is " + color + " → Stopping");
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
  turnLeft(); delay(900); Stop(); delay(200);
}

void rotate90() {
  int rotateSpeed = 100;
  analogWrite(enA, rotateSpeed); analogWrite(enB, rotateSpeed);
  turnLeft(); delay(450); Stop(); delay(200);
}
