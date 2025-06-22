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
#define S2 0
#define S3 1
#define sensorOut A0
//#define OE 11 

float distance = 10;
bool avoidingObstacle = false;
unsigned long reengageStartTime = 0;
bool reengageInProgress = false;
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
int speedA = 58;
int speedB = 58;
int add = 10;
int sub = 15;

void setup() {
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(L_S, INPUT); pinMode(R_S, INPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  
  //pinMode(OE, OUTPUT); 
  pinMode(sensorOut, INPUT);

 // digitalWrite(OE, LOW);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  Serial.begin(9600);
  Serial.println("Robot with Color Detection Started");
}

void loop() {
  distance = readUltrasonic();
  if (distance > 0 && distance <= 6) {
    stopMotors();
    delay(500);
    String detectedColor = performColorDetection();
    Serial.print("Final Color Decision: ");
    Serial.println(detectedColor);
    if (detectedColor == "RED") {
      Serial.println("RED detected - Performing manual overtaking");
      avoidingObstacle = true;
      handleObstacle();
    }
    else if (detectedColor == "BLUE") {
      Serial.println("BLUE detected - Pushing through obstacle");
      pushThroughObstacle();
    }
    else {
      Serial.println("Unknown color - Default overtaking");
      avoidingObstacle = true;
      handleObstacle();
    }
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
    rightBackward(60);
  }
  else if (left == 1 && right == 0) {
    leftBackward(60);
    rightForward(80);
  }
  else {
    stopMotors();
  }
}

String performColorDetection() {
  Serial.println("Starting color detection - taking 10 readings...");
  int redCount = 0, greenCount = 0, blueCount = 0, unknownCount = 0;
  for (int i = 0; i < 10; i++) {
    String color = readSingleColor();
    Serial.print("Reading "); Serial.print(i + 1); Serial.print(": "); Serial.println(color);
    if (color == "RED") redCount++;
    else if (color == "GREEN") greenCount++;
    else if (color == "BLUE") blueCount++;
    else unknownCount++;
    delay(200);
  }
  Serial.print("Results - Red: "); Serial.print(redCount);
  Serial.print(", Green: "); Serial.print(greenCount);
  Serial.print(", Blue: "); Serial.print(blueCount);
  Serial.print(", Unknown: "); Serial.println(unknownCount);
  if (redCount >= greenCount && redCount >= blueCount && redCount >= 3) return "RED";
  else if (greenCount >= redCount && greenCount >= blueCount && greenCount >= 3) return "GREEN";
  else if (blueCount >= redCount && blueCount >= greenCount && blueCount >= 3) return "BLUE";
  else return "UNKNOWN";
}

String readSingleColor() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  delay(50);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  delay(50);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);
  delay(50);
  return detectColor();
}

String detectColor() {
  int totalSignal = redFrequency + greenFrequency + blueFrequency;
  if (totalSignal > 3000) return "NO OBJECT";
  if (totalSignal < 100) return "SENSOR ERROR";
  int redGreenDiff = abs(redFrequency - greenFrequency);
  int redBlueDiff = abs(redFrequency - blueFrequency);
  int greenBlueDiff = abs(greenFrequency - blueFrequency);
  int minDifference = 15, greenMinDiff = 10;
  if (redGreenDiff < 8 && redBlueDiff < 8 && greenBlueDiff < 8) return "WHITE/GRAY";
  int minFreq = min(redFrequency, min(greenFrequency, blueFrequency));
  if (greenFrequency == minFreq) {
    if ((redGreenDiff > greenMinDiff || greenBlueDiff > greenMinDiff)) {
      if (greenFrequency < (redFrequency * 0.9) || greenFrequency < (blueFrequency * 0.9)) return "GREEN";
    }
    if (greenFrequency < redFrequency && greenFrequency < blueFrequency) {
      float greenAdvantage = ((float)(redFrequency + blueFrequency) / 2) / greenFrequency;
      if (greenAdvantage > 1.05) return "GREEN";
    }
  }
  if (redFrequency == minFreq && redFrequency < greenFrequency && redFrequency < blueFrequency) {
    if (redGreenDiff > minDifference && redBlueDiff > minDifference) {
      if (redFrequency < (greenFrequency * 0.8) && redFrequency < (blueFrequency * 0.8)) return "RED";
    }
  }
  if (blueFrequency == minFreq && blueFrequency < redFrequency && blueFrequency < greenFrequency) {
    if (redBlueDiff > minDifference && greenBlueDiff > minDifference) {
      if (blueFrequency < (redFrequency * 0.8) && blueFrequency < (greenFrequency * 0.8)) return "BLUE";
    }
  }
  int maxDiff = max(redGreenDiff, max(redBlueDiff, greenBlueDiff));
  if (maxDiff > 20) {
    float redRatio = (float)redFrequency / totalSignal;
    float greenRatio = (float)greenFrequency / totalSignal;
    float blueRatio = (float)blueFrequency / totalSignal;
    if (greenRatio < redRatio && greenRatio < blueRatio) {
      if ((redRatio - greenRatio) > 0.05 && (blueRatio - greenRatio) > 0.05) return "GREEN";
    }
    else if (redRatio < greenRatio && redRatio < blueRatio) {
      if ((greenRatio - redRatio) > 0.05 && (blueRatio - redRatio) > 0.05) return "RED";
    }
    else if (blueRatio < redRatio && blueRatio < greenRatio) {
      if ((redRatio - blueRatio) > 0.05 && (greenRatio - blueRatio) > 0.05) return "BLUE";
    }
  }
  return "UNKNOWN";
}

void pushThroughObstacle() {
  Serial.println("Pushing blue obstacle forward...");
  moveForward(100, 100);
  delay(600);
  stopMotors();
  delay(300);
  Serial.println("Returning to original position...");
  moveBackward(100, 100);
  delay(600);
  stopMotors();
  delay(300);
  Serial.println("Obstacle cleared! Resuming line following");
}

void stopForever() {
  Serial.println("STOP SIGNAL DETECTED - ROBOT STOPPED PERMANENTLY");
  while (true) {
    stopMotors();
    delay(1000);
  }
}

// *** FIXED DIRECTION MOTOR CONTROL FUNCTIONS (SWAP LOGIC) ***
void moveForward(int speedVal1, int speedVal2) {
  analogWrite(enA, speedVal1);
  analogWrite(enB, speedVal2);
  // SWAP LOGIC: Reverse the HIGH/LOW for both motors to match your wiring
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

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

long readUltrasonic() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  if (duration == 0) return -1;
  long distance = duration * 0.034 / 2;
  if (distance > 300 || distance <= 0) return -1;
  return distance;
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

void moveForward1(int speedVal) {
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
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