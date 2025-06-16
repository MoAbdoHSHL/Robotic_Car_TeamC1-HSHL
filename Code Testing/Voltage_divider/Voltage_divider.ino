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

float distanceFromObject = 5000;

int baseSpeed = 100;
int turnSpeed = 70;

void setup() {
  Serial.begin(9600);
  
  pinMode(L_S, INPUT);
  pinMode(R_S, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void activateTriggerPin() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

float measureDistance() {
  activateTriggerPin();
  long travelTime = pulseIn(echoPin, HIGH, 30000);
  return travelTime * 0.034 / 2;  // Distance in cm
}

void motorAForward(int speed) {
  analogWrite(enA, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorBForward(int speed) {
  analogWrite(enB, speed);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void motorABackward(int speed) {
  analogWrite(enA, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void motorBBackward(int speed) {
  analogWrite(enB, speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void forward(int speed) {
  motorAForward(speed);
  motorBForward(speed);
}

void backward(int speed) {
  motorABackward(speed);
  motorBBackward(speed);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void turn180() {
  motorABackward(baseSpeed);
  motorBForward(baseSpeed);
  delay(1000);
  stopMotors();
}

void decelerate() {
  for (int i = baseSpeed; i > 0; i -= 30) {
    forward(i);
    delay(100);
  }
}

void avoidObject() {
  distanceFromObject = measureDistance();
  if (distanceFromObject <= 20.0) {
    Serial.println("Obstacle Found");
    stopMotors();
    delay(10);
  }
}

void loop() {
  int leftIR = digitalRead(L_S);
  int rightIR = digitalRead(R_S);

  avoidObject();

  if (leftIR == HIGH && rightIR == HIGH) {
    forward(baseSpeed);  // on black line
  } else if (leftIR == LOW && rightIR == HIGH) {
    motorABackward(150);
    motorBForward(baseSpeed + turnSpeed);
  } else if (leftIR == HIGH && rightIR == LOW) {
    motorAForward(baseSpeed + turnSpeed);
    motorBBackward(150);
  } else {
    backward(baseSpeed);
  }
}
