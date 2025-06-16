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

int forwardSpeed = 100;      // Increased speed for better response
int turnSpeedLeft = 60;      // For turning left: reduce left motor speed
int turnSpeedRight = 60;     // For turning right: reduce right motor speed
int approachThreshold = 20;  // obstacle distance in cm

void setup() {
  Serial.begin(9600);
  pinMode(R_S, INPUT);
  pinMode(L_S, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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

  // Line following logic
  if (leftIR == 0 && rightIR == 0) {
    moveForward(forwardSpeed, forwardSpeed);  // Go straight
  } else if (leftIR == 1 && rightIR == 0) {
    moveForward(turnSpeedLeft, forwardSpeed);  // Turn left gently
  } else if (leftIR == 0 && rightIR == 1) {
    moveForward(forwardSpeed, turnSpeedRight); // Turn right gently
  } else {
    Stop();  // Both black or undefined
  }
}

// Smooth motor control
void moveForward(int speedA, int speedB) {
  analogWrite(enA, speedA);
  analogWrite(enB, speedB);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void Stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

// Ultrasonic distance in cm
int getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); // timeout 20ms
  return duration * 0.034 / 2;
}
