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

int forwardSpeed = 70;
int turnSpeed = 80;
int curveDelay = 100;  // duration in ms for correction turn
int approachThreshold = 20;  // obstacle distance in cm

void setup() {
  Serial.begin(9600);
  pinMode(R_S, INPUT); pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
}

void loop() {
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);
  int distance = getDistance();

  Serial.print("L_IR="); Serial.print(leftIR);
  Serial.print(" | R_IR="); Serial.print(rightIR);
  Serial.print(" | Distance="); Serial.println(distance);

  // Obstacle detected
  if (distance > 0 && distance <= approachThreshold) {
    Stop();
    Serial.println("Obstacle ahead! Stopping.");
    delay(500);
    return;
  }

  // Both sensors on white: go forward
  if (leftIR == 0 && rightIR == 0) {
    analogWrite(enA, forwardSpeed);
    analogWrite(enB, forwardSpeed);
    forward();
    
else if ((leftIR == 0) && (rightIR == 1)) {
  // Turn right gently until line is found again
  analogWrite(enA, turnSpeed / 2);  // slow left
  analogWrite(enB, turnSpeed);      // normal right
  forward();
  while (digitalRead(R_S) == 1 && digitalRead(L_S) == 0) {
    // wait until right IR detects line again
  }
  Stop();
  delay(50);
} 
else if ((leftIR == 1) && (rightIR == 0)) {
  // Turn left gently until line is found again
  analogWrite(enA, turnSpeed);      // normal left
  analogWrite(enB, turnSpeed / 2);  // slow right
  forward();
  while (digitalRead(L_S) == 1 && digitalRead(R_S) == 0) {
    // wait until left IR detects line again
  }
  Stop();
  delay(50);
}

}

// Movement functions
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

// Curve left slightly
void curveLeft() {
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
  turnLeft();
  delay(curveDelay);
  Stop();
}

// Curve right slightly
void curveRight() {
  analogWrite(enA, turnSpeed);
  analogWrite(enB, turnSpeed);
  turnRight();
  delay(curveDelay);
  Stop();
}

// Ultrasonic distance in cm
int getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000);
  return duration * 0.034 / 2;
}
