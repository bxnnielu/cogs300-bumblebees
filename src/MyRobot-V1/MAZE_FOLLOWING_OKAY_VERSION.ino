// REVISED MAZE-FOLLOWER

/*
  ROBOT MAZE LOGIC — SIDE + FRONT ULTRASONIC

  STATE 1: WALL FOLLOW
    Conditions:
      - robot starts here
      - front clear: no wall detected OR front > 20 cm
      - side wall detected: side > 0 AND side <= 60 cm
    Action:
      - wall following

  STATE 2: LEFT PIVOT
    Conditions:
      - front wall detected: front > 0 AND front <= 20 cm
      - only if right-pivot condition is NOT active
    Action:
      - stop about 15 cm from wall
      - pivot left 90 degrees
      - resume wall follow

  STATE 3: RIGHT PIVOT
    Conditions:
      - side wall missing: side == 0 OR side > 60 cm
      - THIS TAKES PRIORITY OVER FRONT SENSOR
    Action:
      - go forward about 2 cm
      - pivot right 90 degrees
      - resume wall follow

  COMMANDS:
    w = start wall-follow mode
    s = stop robot completely
*/

bool wallFollowMode = true;

// ------------------ Wall-follow parameters ------------------
const float WALL_SET_POINT = 16.0;   // desired side distance
const float WALL_KP = 6.5;

const int BASE_SPEED = 160;
const int MAX_SPEED  = 230;
const int MIN_SPEED  = 70;
const int DEAD_BAND  = 1;

// ------------------ Required thresholds ------------------
const int WALL_DETECTED_CM   = 60;
const int FRONT_THRESHOLD_CM = 20;
const int FRONT_STOP_CM      = 15;

// ------------------ Timed calibration ------------------
// tune these on your robot
const int FORWARD_2CM_MS    = 120;
const int PIVOT_LEFT_90_MS  = 430;
const int PIVOT_RIGHT_90_MS = 430;

const int TURN_SPEED  = 240;
const int CREEP_SPEED = 110;

// ------------------ State machine ------------------
enum WFState { WF_FOLLOW, WF_LEFT_PIVOT, WF_RIGHT_PIVOT };
WFState wfState = WF_FOLLOW;

// ------------------ Motor pins ------------------
int enA = 9;    // Right motor PWM
int enB = 10;   // Left motor PWM

int in1 = 6;    // Right motor direction
int in2 = 7;

int in3 = 4;    // Left motor direction
int in4 = 5;

// ------------------ Ultrasonic pins ------------------
int trigSide  = A2;
int echoSide  = A3;

int trigFront = 12;
int echoFront = 11;

// ------------------ Sensor readings ------------------
long sideCm  = 0;
long frontCm = 0;

// ------------------ Ultrasonic read ------------------
long readUltrasonicCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long d = pulseIn(echo, HIGH, 30000);
  if (d == 0) return 0;

  long cm = (d / 2) / 29.1;
  if (cm < 0) cm = 0;
  return cm;
}

// ------------------ Helpers ------------------
int clampPWM(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void forward(int rightSpd, int leftSpd) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, clampPWM(rightSpd));
  analogWrite(enB, clampPWM(leftSpd));
}

// right wheel only
void rightWheelForwardOnly(int spd) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, clampPWM(spd));

  analogWrite(enB, 0);
}

// left wheel only
void leftWheelForwardOnly(int spd) {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, clampPWM(spd));

  analogWrite(enA, 0);
}

// pivot LEFT = move RIGHT wheel only
void pivotLeft(int spd) {
  rightWheelForwardOnly(spd);
}

// pivot RIGHT = move LEFT wheel only
void pivotRight(int spd) {
  leftWheelForwardOnly(spd);
}

void driveForwardTimed(int msTime, int rightSpd, int leftSpd) {
  forward(rightSpd, leftSpd);
  delay(msTime);
  stopMotors();
}

void pivotLeft90() {
  pivotLeft(TURN_SPEED);
  delay(PIVOT_LEFT_90_MS);
  stopMotors();
}

void pivotRight90() {
  pivotRight(TURN_SPEED);
  delay(PIVOT_RIGHT_90_MS);
  stopMotors();
}

// move slowly until front sensor says robot is 15 cm away
void moveUntilFrontStopDistance() {
  while (true) {
    frontCm = readUltrasonicCM(trigFront, echoFront);

    if (frontCm > 0 && frontCm <= FRONT_STOP_CM) {
      stopMotors();
      return;
    }

    forward(CREEP_SPEED, CREEP_SPEED);
    delay(20);
  }
}

// ------------------ Wall-follow controller ------------------
void wallFollowControl() {
  if (sideCm == 0 || sideCm > WALL_DETECTED_CM) {
    stopMotors();
    return;
  }

  float error = (float)sideCm - WALL_SET_POINT;
  if (abs(error) < DEAD_BAND) {
    error = 0;
  }

  float correction = WALL_KP * error;

  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  leftSpeed  = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  forward(rightSpeed, leftSpeed);
}

// ------------------ Main maze logic ------------------
void wallFollowWithCorners() {
  sideCm  = readUltrasonicCM(trigSide, echoSide);
  frontCm = readUltrasonicCM(trigFront, echoFront);

  bool sideWallMissing   = (sideCm == 0) || (sideCm > WALL_DETECTED_CM);
  bool sideWallDetected  = (sideCm > 0) && (sideCm <= WALL_DETECTED_CM);
  bool frontWallDetected = (frontCm > 0) && (frontCm <= FRONT_THRESHOLD_CM);
  bool frontClear        = (frontCm == 0) || (frontCm > FRONT_THRESHOLD_CM);

  switch (wfState) {
    case WF_FOLLOW:
      // RIGHT PIVOT takes priority over front
      if (sideWallMissing) {
        wfState = WF_RIGHT_PIVOT;
        return;
      }

      // LEFT PIVOT
      if (frontWallDetected) {
        wfState = WF_LEFT_PIVOT;
        return;
      }

      // WALL FOLLOW
      if (frontClear && sideWallDetected) {
        wallFollowControl();
      } else {
        stopMotors();
      }
      return;

    case WF_LEFT_PIVOT:
      // stop 15 cm in front of wall
      stopMotors();
      delay(50);

      if (frontCm == 0 || frontCm > FRONT_STOP_CM) {
        moveUntilFrontStopDistance();
      }

      stopMotors();
      delay(100);

      // turn 90 degrees left
      pivotLeft90();
      delay(100);

      // resume wall following
      wfState = WF_FOLLOW;
      return;

    case WF_RIGHT_PIVOT:
      // go forward about 2 cm
      driveForwardTimed(FORWARD_2CM_MS, BASE_SPEED, BASE_SPEED);
      delay(100);

      // turn 90 degrees right
      pivotRight90();
      delay(100);

      // resume wall following
      wfState = WF_FOLLOW;
      return;
  }
}

// ------------------ Setup ------------------
void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  Serial.begin(9600);

  wallFollowMode = true;
  wfState = WF_FOLLOW;

  stopMotors();
}

// ------------------ Loop ------------------
void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'w') {
      wallFollowMode = true;
      wfState = WF_FOLLOW;
      stopMotors();
    }

    if (command == 's') {
      wallFollowMode = false;
      stopMotors();
    }
  }

  if (wallFollowMode) {
    wallFollowWithCorners();
  } else {
    stopMotors();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    Serial.print("SIDE(cm): ");
    Serial.print(sideCm);
    Serial.print(" | FRONT(cm): ");
    Serial.print(frontCm);
    Serial.print(" | state: ");
    Serial.println((int)wfState);
  }
}