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

const float CLOSE_THRESHOLD = 30.0;  // Distance in cm - closer than this = likely object
const float FAR_THRESHOLD = 100.0;   // Distance in cm - farther than this = likely wall

const int NUM_ANGLES = 8;   // or 16, or however many scan positions you want


// ===== BAYES FILTER ARRAYS =====
float probabilities[NUM_ANGLES];  // Probability that object is at each angle (0.0 to 1.0)
float distances[NUM_ANGLES];      // Most recent distance measurement at each angle

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

const float ANGLE_STEP = 180.0 / NUM_ANGLES;
const float CONFIDENCE_THRESHOLD = 0.4;
const float STOP_DISTANCE = 5.0;

const int MOTOR_SPEED = 140;
const int TURN_SPEED = 120;

int bestIndex = 0;
float maxProb = 0;


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

  // Initialize all probabilities to uniform prior (equal probability everywhere)
  for (int i = 0; i < NUM_ANGLES; i++) {
    probabilities[i] = 1.0 / NUM_ANGLES;  // Each angle starts at ~6.25% probability
    distances[i] = 0;
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

void handleScanning() {

  performScan();
  updateBayesFilter();

  bestIndex = findBestIndex(maxProb);
  printResults(bestIndex);

  if (maxProb >= CONFIDENCE_THRESHOLD) {

    float targetAngle = bestIndex * ANGLE_STEP;
    float error = targetAngle - 90.0;

    if (abs(error) < 5) {
      Serial.println("CONFIDENT & ALIGNED — switching to DRIVING");
      currentState = DRIVING;
    } 
    else {
      Serial.println("CONFIDENT but MISALIGNED — switching to TURNING");
      currentState = TURNING;
    }
  }
}

void handleTurning() {

  float targetAngle = bestIndex * ANGLE_STEP;
  float centerAngle = 90.0;

  float error = targetAngle - centerAngle;

  if (abs(error) < 5) {
    stopMotors();
    Serial.println("Aligned with object — switching to DRIVING");
    currentState = DRIVING;
    return;
  }

  if (error < 0) {
    rotateLeft(TURN_SPEED);
  } else {
    rotateRight(TURN_SPEED);
  }

  // Small delay to allow motion
  delay(50);

  stopMotors();

  // Re-scan center area only (fast update)
  servo.write(centerAngle);
  delay(150);

  float dist = getDistance();
  float likelihood = calculateLikelihood(dist);

  probabilities[NUM_ANGLES / 2] *= likelihood;
  normalizeProbabilities();

  bestIndex = findBestIndex(maxProb);

  if (maxProb < CONFIDENCE_THRESHOLD) {
    Serial.println("Lost confidence — back to SCANNING");
    currentState = SCANNING;
  }
}

void handleDriving() {

  servo.write(90);   // Keep sensor forward
  delay(50);

  float dist = getDistance();

  if (dist <= STOP_DISTANCE) {
    stopMotors();
    Serial.println("Reached object — STOPPED");
    currentState = STOPPED;
    return;
  }

  // Slow down as approaching
  int dynamicSpeed = map(dist, STOP_DISTANCE, 50, 80, MOTOR_SPEED);
  dynamicSpeed = constrain(dynamicSpeed, 80, MOTOR_SPEED);

  driveForward(dynamicSpeed);
}

void normalizeProbabilities() {
  float total = 0;
  for (int i = 0; i < NUM_ANGLES; i++) {
    total += probabilities[i];
  }
  for (int i = 0; i < NUM_ANGLES; i++) {
    probabilities[i] /= total;
  }
}


// ===== SCAN FUNCTION =====
void performScan() {
 Serial.println("\n--- New Scan ---");
  for (int i = 0; i < NUM_ANGLES; i++) {
   // Calculate angle for this index
   const float ANGLE_STEP = 180.0 / NUM_ANGLES;
   float angle = i * ANGLE_STEP;
  
   // Move servo to this angle (if using servo)
   servo.write(angle);
   delay(250);  // Wait for servo to reach position
  
   // Measure distance
   distances[i] = getDistance();
  
   Serial.print("Angle ");
   Serial.print(angle);
   Serial.print("°: ");
   Serial.print(distances[i]);
   Serial.println(" cm");
  
   delay(50);  // Small delay between readings
 }
}


// ===== BAYES FILTER UPDATE =====
void updateBayesFilter() {
 float totalProb = 0;
  // Update each angle's probability based on measurement
 for (int i = 0; i < NUM_ANGLES; i++) {
   float distance = distances[i];
  
   // Calculate likelihood: P(measurement | object at this angle)
   float likelihood = calculateLikelihood(distance);
  
   // Bayes update: posterior = likelihood × prior
   probabilities[i] = likelihood * probabilities[i];
  
   totalProb += probabilities[i];
 }
  // Normalize probabilities so they sum to 1.0
 for (int i = 0; i < NUM_ANGLES; i++) {
   probabilities[i] = probabilities[i] / totalProb;
 }
}


// ===== LIKELIHOOD FUNCTION =====
// This is the key: how likely is this measurement if the object is here?
float calculateLikelihood(float distance) {
 if (distance < CLOSE_THRESHOLD) {
   // Very close reading → high likelihood of object being here
   return 0.9;
  
 } else if (distance < FAR_THRESHOLD) {
   // Medium distance → lower likelihood
   return 0.3;
  
 } else {
   // Far reading → very low likelihood (probably wall)
   return 0.1;
 }
}


// ===== FIND BEST ANGLE =====
int findBestIndex(float &maxProb) {
  int bestIndex = 0;
  maxProb = probabilities[0];

  for (int i = 1; i < NUM_ANGLES; i++) {
    if (probabilities[i] > maxProb) {
      maxProb = probabilities[i];
      bestIndex = i;
    }
  }
  return bestIndex;
}


// ===== VISUALIZATION =====
void printResults(int bestAngle) {
 Serial.println("\n=== PROBABILITIES ===");
  for (int i = 0; i < NUM_ANGLES; i++) {
   float angle = i * (180 / NUM_ANGLES);
  
   Serial.print("Angle ");
   Serial.print(angle);
   Serial.print("°: ");
   Serial.print(probabilities[i] * 100);
   Serial.print("%");
  
   // Visual bar graph
   Serial.print(" [");
   int bars = probabilities[i] * 20;  // Scale to 20 characters
   for (int j = 0; j < bars; j++) {
     Serial.print("=");
   }
   Serial.println("]");
 }
  float bestAngleDegrees = bestAngle * (180 / NUM_ANGLES);
 Serial.print("\n>>> OBJECT MOST LIKELY AT: ");
 Serial.print(bestAngleDegrees);
 Serial.print("° (");
 Serial.print(probabilities[bestAngle] * 100);
 Serial.println("% confidence)");
}


// ===== ULTRASONIC SENSOR =====
float getDistance() {
 // Send ultrasonic pulse
 digitalWrite(TRIG_PIN, LOW);
 delayMicroseconds(2);
 digitalWrite(TRIG_PIN, HIGH);
 delayMicroseconds(10);
 digitalWrite(TRIG_PIN, LOW);
  // Read echo
 long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // 30ms timeout
  if (duration == 0) {
   return 999;  // No echo = very far
 }
  // Convert to distance in cm
 float distance = duration * 0.034 / 2;
  return distance;
}


// ===== MOTOR CONTROL (FIXED DIRECTION) =====

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void driveForward(int speedVal) {
  // FLIPPED
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
  // Left wheel backward, right wheel forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void rotateRight(int speedVal) {
  // Left wheel forward, right wheel backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void approachObject(int bestIndex) {

  float targetAngle = bestIndex * ANGLE_STEP;
  float centerAngle = 90.0;   // assume forward = 90°

  Serial.println("Rotating toward object...");

  // Rotate until centered
  if (targetAngle < centerAngle - 5) {
    rotateLeft(MOTOR_SPEED);
  }
  else if (targetAngle > centerAngle + 5) {
    rotateRight(MOTOR_SPEED);
  }
  else {
    stopMotors();
    Serial.println("Facing object. Driving forward...");

    while (true) {
      float dist = getDistance();

      if (dist <= STOP_DISTANCE) {
        stopMotors();
        Serial.println("Reached object!");
        break;
      }

      driveForward(MOTOR_SPEED);
    }
  }
}


// ===== ADVANCED: MOVEMENT PREDICTION (OPTIONAL) =====
// If your robot moves between scans, you can shift probabilities
void predictMovement(int robotTurnedDegrees) {
 // Convert degrees to index shift
 int indexShift = robotTurnedDegrees / (180 / NUM_ANGLES);
  // Create temporary array
 float tempProbs[NUM_ANGLES];
  // Shift probabilities based on movement
 for (int i = 0; i < NUM_ANGLES; i++) {
   int newIndex = (i + indexShift) % NUM_ANGLES;
   tempProbs[newIndex] = probabilities[i];
 }
  // Copy back
 for (int i = 0; i < NUM_ANGLES; i++) {
   probabilities[i] = tempProbs[i];
 }
}
