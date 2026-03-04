#include <Servo.h>

/*
 * Bayes Filter for Object Detection - Lab 07
 * Simple implementation for finding an object with ultrasonic sensor
 *
 * This code demonstrates the basic concept of using a Bayes filter
 * to track the probability of an object being at each angle
 */

// ===== CONFIGURATION =====
Servo servo;
const int SERVO_PIN = A0;

const int TRIG_PIN = 12;
const int ECHO_PIN = 11;

const float CLOSE_THRESHOLD = 30.0;   // Distance in cm - closer than this = likely object
const float FAR_THRESHOLD   = 100.0;  // Distance in cm - farther than this = likely wall

const int NUM_ANGLES = 8;   // number of scan positions you want

// IMPORTANT: float step (not int math)
const float ANGLE_STEP = 180.0f / (float)NUM_ANGLES;

// ===== BAYES FILTER ARRAYS =====
float probabilities[NUM_ANGLES];  // Probability that object is at each angle (0.0 to 1.0)
float distances[NUM_ANGLES];      // Most recent distance measurement at each angle

// ===== MOTOR PINS =====
const int enA = 9;
const int enB = 10;
const int in1 = 6;
const int in2 = 7;
const int in3 = 4;
const int in4 = 5;

// ===== STATE MACHINE =====
enum RobotState {
  SCANNING,
  TURNING,
  DRIVING,
  STOPPED
};

RobotState currentState = SCANNING;

const float CONFIDENCE_THRESHOLD = 0.4f;
const float STOP_DISTANCE = 5.0f;

const int MOTOR_SPEED = 140;
const int TURN_SPEED  = 120;

int bestIndex = 0;
float maxProb = 0.0f;

// ===== HELPERS =====
void stopMotors();
void driveForward(int speedVal);
void driveBackward(int speedVal);
void rotateLeft(int speedVal);
void rotateRight(int speedVal);

float getDistance();
float calculateLikelihood(float distance);

void performScan();
void updateBayesFilter();
int  findBestIndex(float &maxProb);
void printResults(int bestAngleIndex);
void normalizeProbabilities();

void handleScanning();
void handleTurning();
void handleDriving();

// Shift probability distribution by bins to keep robot-frame angles consistent after turning
void shiftProbabilities(int shift) {
  float temp[NUM_ANGLES];
  for (int i = 0; i < NUM_ANGLES; i++) {
    int newIndex = (i + shift) % NUM_ANGLES;
    if (newIndex < 0) newIndex += NUM_ANGLES;
    temp[newIndex] = probabilities[i];
  }
  for (int i = 0; i < NUM_ANGLES; i++) {
    probabilities[i] = temp[i];
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servo.attach(SERVO_PIN);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initialize all probabilities to uniform prior
  for (int i = 0; i < NUM_ANGLES; i++) {
    probabilities[i] = 1.0f / (float)NUM_ANGLES;
    distances[i] = 0.0f;
  }

  Serial.println("Bayes Filter Object Detection - Lab 07");
  Serial.println("Scanning environment...");
}

// ===== MAIN LOOP =====
void loop() {
  switch (currentState) {
    case SCANNING:
      handleScanning();
      break;

    case TURNING:
      handleTurning();
      break;

    case DRIVING:
      handleDriving();
      break;

    case STOPPED:
      stopMotors();
      break;
  }
}

// ===== STATE HANDLERS =====
void handleScanning() {
  performScan();
  updateBayesFilter();

  bestIndex = findBestIndex(maxProb);
  printResults(bestIndex);

  if (maxProb >= CONFIDENCE_THRESHOLD) {
    // Use FLOAT angles consistently
    float targetAngle = bestIndex * ANGLE_STEP;
    float error = targetAngle - 90.0f;

    if (abs(error) < 5.0f) {
      Serial.println("CONFIDENT & ALIGNED — switching to DRIVING");
      currentState = DRIVING;
    } else {
      Serial.println("CONFIDENT but MISALIGNED — switching to TURNING");
      currentState = TURNING;
    }
  }
}

void handleTurning() {
  const int   CENTER_INDEX = NUM_ANGLES / 2;  // forward bin (e.g., 4 when NUM_ANGLES=8)
  const float CENTER_ANGLE = 90.0f;           // servo forward
  const int   TOL_BINS     = 0;               // set to 1 if you want more tolerance

  // How many bins away from forward is the best direction?
  int errorIndex = bestIndex - CENTER_INDEX;

  if (abs(errorIndex) <= TOL_BINS) {
    stopMotors();
    Serial.println("Aligned with object — switching to DRIVING");
    currentState = DRIVING;
    return;
  }

  // Turn toward the best bin, then shift beliefs to account for the robot rotation.
  int shift = 0;

  if (errorIndex < 0) {
    // Object is left of forward -> turn left
    rotateLeft(TURN_SPEED);
    delay(60);   // pulse; tune 40–120ms
    shift = +1;  // turning left makes object appear more to the right in robot frame
  } else {
    // Object is right of forward -> turn right
    rotateRight(TURN_SPEED);
    delay(60);
    shift = -1;  // turning right makes object appear more to the left
  }

  stopMotors();

  // Keep probability bins aligned with robot frame after the turn
  shiftProbabilities(shift);

  // Quick center re-check to maintain confidence (fast update)
  servo.write(CENTER_ANGLE);
  delay(120);

  float dist = getDistance();
  float likelihood = calculateLikelihood(dist);

  // Update only the center bin with the new measurement
  probabilities[CENTER_INDEX] *= likelihood;
  normalizeProbabilities();

  bestIndex = findBestIndex(maxProb);

  if (maxProb < CONFIDENCE_THRESHOLD) {
    Serial.println("Lost confidence — back to SCANNING");
    currentState = SCANNING;
  }
}

void handleDriving() {
  servo.write(90);   // keep sensor forward
  delay(50);

  float dist = getDistance();

  if (dist <= STOP_DISTANCE) {
    stopMotors();
    Serial.println("Reached object — STOPPED");
    currentState = STOPPED;
    return;
  }

  // Slow down as approaching
  int dynamicSpeed = map((long)dist, (long)STOP_DISTANCE, 50L, 80, MOTOR_SPEED);
  dynamicSpeed = constrain(dynamicSpeed, 80, MOTOR_SPEED);

  driveForward(dynamicSpeed);
}

// ===== PROBABILITY NORMALIZATION =====
void normalizeProbabilities() {
  float total = 0.0f;
  for (int i = 0; i < NUM_ANGLES; i++) total += probabilities[i];

  // guard against divide-by-zero
  if (total <= 0.0f) {
    for (int i = 0; i < NUM_ANGLES; i++) probabilities[i] = 1.0f / (float)NUM_ANGLES;
    return;
  }

  for (int i = 0; i < NUM_ANGLES; i++) probabilities[i] /= total;
}

// ===== SCAN FUNCTION =====
void performScan() {
  Serial.println("\n--- New Scan ---");
  for (int i = 0; i < NUM_ANGLES; i++) {
    // Float angles that match your scan output
    float angle = i * ANGLE_STEP;

    servo.write(angle);
    delay(250);

    distances[i] = getDistance();

    Serial.print("Angle ");
    Serial.print(angle, 1);  // 1 decimal place
    Serial.print("°: ");
    Serial.print(distances[i], 1);
    Serial.println(" cm");

    delay(50);
  }
}

// ===== BAYES FILTER UPDATE =====
void updateBayesFilter() {
  float totalProb = 0.0f;

  for (int i = 0; i < NUM_ANGLES; i++) {
    float distance = distances[i];
    float likelihood = calculateLikelihood(distance);
    probabilities[i] = likelihood * probabilities[i];
    totalProb += probabilities[i];
  }

  if (totalProb <= 0.0f) {
    for (int i = 0; i < NUM_ANGLES; i++) probabilities[i] = 1.0f / (float)NUM_ANGLES;
    return;
  }

  for (int i = 0; i < NUM_ANGLES; i++) {
    probabilities[i] /= totalProb;
  }
}

// ===== LIKELIHOOD FUNCTION =====
float calculateLikelihood(float distance) {
  if (distance < CLOSE_THRESHOLD) {
    return 0.9f;
  } else if (distance < FAR_THRESHOLD) {
    return 0.3f;
  } else {
    return 0.1f;
  }
}

// ===== FIND BEST ANGLE =====
int findBestIndex(float &maxProbOut) {
  int best = 0;
  maxProbOut = probabilities[0];

  for (int i = 1; i < NUM_ANGLES; i++) {
    if (probabilities[i] > maxProbOut) {
      maxProbOut = probabilities[i];
      best = i;
    }
  }
  return best;
}

// ===== VISUALIZATION =====
void printResults(int bestAngleIndex) {
  Serial.println("\n=== PROBABILITIES ===");
  for (int i = 0; i < NUM_ANGLES; i++) {
    float angle = i * ANGLE_STEP;   // FIXED: float angle matches scan

    Serial.print("Angle ");
    Serial.print(angle, 1);
    Serial.print("°: ");
    Serial.print(probabilities[i] * 100.0f, 1);
    Serial.print("%");

    Serial.print(" [");
    int bars = (int)(probabilities[i] * 20.0f);
    for (int j = 0; j < bars; j++) Serial.print("=");
    Serial.println("]");
  }

  float bestAngleDegrees = bestAngleIndex * ANGLE_STEP;  // FIXED
  Serial.print("\n>>> OBJECT MOST LIKELY AT: ");
  Serial.print(bestAngleDegrees, 1);
  Serial.print("° (");
  Serial.print(probabilities[bestAngleIndex] * 100.0f, 1);
  Serial.println("% confidence)");
}

// ===== ULTRASONIC SENSOR =====
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout

  if (duration == 0) return 999.0f;

  float distance = (float)duration * 0.034f / 2.0f;
  return distance;
}

// ===== MOTOR CONTROL (FIXED DIRECTION) =====
void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void driveForward(int speedVal) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void driveBackward(int speedVal) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void rotateLeft(int speedVal) {
  // Right wheel backward, left wheel forward (your wiring assumption)
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void rotateRight(int speedVal) {
  // Right wheel forward, left wheel backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}