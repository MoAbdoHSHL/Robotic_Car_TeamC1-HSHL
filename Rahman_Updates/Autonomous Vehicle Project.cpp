#include <IRremote.h>
#include <Servo.h>

// Motor 1 (Left Motor)
const int enA = 10;
const int in1 = 7;
const int in2 = 6;

// Motor 2 (Right Motor)
const int enB = 9;
const int in3 = 12;
const int in4 = 13;

// Ultrasonic sensor
const int echo = 4;
const int trigger = 3;

// IR receiver
const int IR_RECEIVE_PIN = 2;

// Servo
Servo servo_11;
int servoPos = 90;  // Start at center position (90°)

// Speed and distance
int speed = 150;  // Default speed value
long duration;
float distance_F;

// Movement state tracking
enum MovementState { STOPPED, FORWARD, BACKWARD, LEFT, RIGHT };
MovementState currentState = STOPPED;
bool obstacleDetected = false;

void setup() {
  Serial.begin(9600);

  // Motor pins setup
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Ultrasonic sensor setup
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);

  // Servo setup
  servo_11.attach(11, 500, 2500);  // Attach servo to pin 11
  servo_11.write(90);  // Center the servo
  delay(1000); // Wait for the servo to stabilize

  // IR receiver setup
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // Start IR receiver

  // Initialize distance sensor
  distance_F = Ultrasonic_read();
  delay(500);

  Serial.println("Ready for IR control with ultrasonic safety");
}

void loop() {
  // Read Ultrasonic Distance
  distance_F = Ultrasonic_read();
  Serial.print("Distance: ");
  Serial.print(distance_F);
  Serial.println(" cm");

  // Safety check - stop if object is too close (100cm)
  if (distance_F > 0 && distance_F < 100) {
    if (!obstacleDetected) {
      Stop();
      obstacleDetected = true;
      Serial.println("Obstacle detected! Stopped.");
    }
    delay(100);  // Short delay to prevent rapid checks
    return;  // Skip the rest of the loop
  } else if (obstacleDetected) {
    obstacleDetected = false;
    Serial.println("Obstacle cleared. Resuming movement.");
    resumeMovement();
  }

  // Check if IR data is received
  if (IrReceiver.decode()) {
    int command = IrReceiver.decodedIRData.command;
    Serial.print("Received IR command: ");
    Serial.println(command);

    switch (command) {
      case 1:  // vol+ -> forward
        currentState = FORWARD;
        forward();
        break;
      case 4:  // |<< -> turn right
        currentState = RIGHT;
        turnRight();
        break;
      case 5:  // >|| -> stop
        currentState = STOPPED;
        Stop();
        break;
      case 6:  // >>| -> turn left
        currentState = LEFT;
        turnLeft();
        break;
      case 8:  // down arrow -> slow down
        speedDown();
        break;
      case 9:  // vol- -> backward
        currentState = BACKWARD;
        backward();
        break;
      case 10: // up arrow -> speed up
        speedUp();
        break;
    }

    IrReceiver.resume();  // ready to receive next signal
  }

  // Sweep the servo
  sweepServo();
}

void resumeMovement() {
  switch (currentState) {
    case FORWARD:
      forward();
      break;
    case BACKWARD:
      backward();
      break;
    case LEFT:
      turnLeft();
      break;
    case RIGHT:
      turnRight();
      break;
    case STOPPED:
      Stop();
      break;
  }
}

// --- Ultrasonic Sensor Function ---
long Ultrasonic_read() {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  long time = pulseIn(echo, HIGH, 30000);  // Max wait time = 30ms (for safety)
  if (time == 0) {
    return 999;  // No echo received, assume no obstacle
  }
  return time / 29 / 2;  // distance in cm
}

// --- Movement Functions ---
void forward() {
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Moving Forward");
}

void backward() {
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Moving Backward");
}

void turnLeft() {
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println("Turning Left");
}

void turnRight() {
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println("Turning Right");
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  Serial.println("Stopped");
}

void speedUp() {
  speed += 10;
  if (speed > 255) speed = 255;
  Serial.print("Speed increased to: ");
  Serial.println(speed);
  // Resume movement with new speed
  if (currentState != STOPPED) {
    resumeMovement();
  }
}

void speedDown() {
  speed -= 10;
  if (speed < 0) speed = 0;
  Serial.print("Speed decreased to: ");
  Serial.println(speed);
  // Resume movement with new speed
  if (currentState != STOPPED) {
    resumeMovement();
  }
}


// --- Servo Sweep Function ---
void sweepServo() {
  // Left sweep: 90° → 0°
  for (servoPos = 90; servoPos >= 0; servoPos--) {
    servo_11.write(servoPos);
    delay(15);
  }

  // Right sweep: 0° → 180°
  for (servoPos = 0; servoPos <= 180; servoPos++) {
    servo_11.write(servoPos);
    delay(15);
  }

  // Return to center: 180° → 90°
  for (servoPos = 180; servoPos >= 90; servoPos--) {
    servo_11.write(servoPos);
    delay(15);
  }

  delay(500);  // Pause at center
}

