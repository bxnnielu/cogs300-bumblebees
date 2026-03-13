/*
  ROBOT MAZE LOGIC — SIDE + FRONT ULTRASONIC
  RIGHT WALL FOLLOWING

  STATES
  ------
  1) WALL FOLLOW
     Condition:
       - front clear: no wall detected OR front > 20 cm
       - side wall detected: side > 0 AND side <= 60 cm
     Action:
       - wall follow along right wall

  2) LEFT PIVOT
     Condition:
       - front wall detected: front > 0 AND front <= 20 cm
       - side wall detected:  side > 0 AND side <= 60 cm
     Action:
       - stop wall following immediately
       - move forward until front distance is about 15 cm
       - pivot left 90 degrees
       - resume wall follow

  3) RIGHT PIVOT
     Condition:
       - side wall missing: side == 0 OR side > 60 cm
       - this takes priority over front sensor
     Action:
       - move forward a little (~2 cm, time-based)
       - pivot right 90 degrees
       - resume wall follow

  COMMANDS
  --------
    'w' = start wall-follow mode
    's' = stop robot completely
*/

bool wallFollowMode = true;

// ------------------ Wall-follow tuning ------------------
const float WALL_SET_POINT = 14.0;   // desired right-wall distance (cm)
const float WALL_KP = 4;

const int BASE_SPEED = 160;
const int MAX_SPEED  = 230;
const int MIN_SPEED  = 70;
const int DEAD_BAND  = 1;

// ------------------ Thresholds ------------------
const int WALL_DETECTED_CM   = 60;   // side wall exists if <= 60 cm
const int FRONT_THRESHOLD_CM = 20;   // trigger LEFT_PIVOT if front <= 20 cm
const int FRONT_STOP_CM      = 15;   // in LEFT_PIVOT, stop when front <= 15 cm

// ------------------ Timed calibration ------------------
// tune these on your robot
const int FORWARD_2CM_MS    = 120;   // approx time to move forward 2 cm
const int PIVOT_LEFT_90_MS  = 430;   // approx left pivot 90 deg
const int PIVOT_RIGHT_90_MS = 430;   // approx right pivot 90 deg

const int TURN_SPEED  = 240;
const int CREEP_SPEED = 110;

// ------------------ State machine ------------------
enum WFState {
  WF_FOLLOW,
  WF_LEFT_PIVOT,
  WF_RIGHT_PIVOT
};

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

  long d = pulseIn(echo, HIGH, 30000);   // 30 ms timeout
  if (d == 0) {
    return 0; // no echo
  }

  long cm = (d / 2) / 29.1;
  if (cm < 0) {
    cm = 0;
  }
  return cm;
}

// ------------------ Helpers ------------------
int clampPWM(int v) {
  if (v < 0) {
    return 0;
  }
  if (v > 255) {
    return 255;
  }
  return v;
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void forward(int rightSpd, int leftSpd) {
  // Right motor forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Left motor forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, clampPWM(rightSpd));
  analogWrite(enB, clampPWM(leftSpd));
}

// Right wheel only forward, left stopped
void rightWheelForwardOnly(int spd) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, clampPWM(spd));

  analogWrite(enB, 0);
}

// Left wheel only forward, right stopped
void leftWheelForwardOnly(int spd) {
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, clampPWM(spd));

  analogWrite(enA, 0);
}

// Pivot left = move right wheel only
void pivotLeft(int spd) {
  rightWheelForwardOnly(spd);
}

// Pivot right = move left wheel only
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

// In LEFT_PIVOT state: move slowly until about 15 cm from front wall
void moveUntilFrontStopDistance() {
  while (true) {
    frontCm = readUltrasonicCM(trigFront, echoFront);

    if (frontCm > 0 && frontCm <= FRONT_STOP_CM) {
      stopMotors();
      return;
    }

    forward(CREEP_SPEED, CREEP_SPEED);
    delay(20);
    stopMotors();
    delay(10);
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
      // RIGHT PIVOT takes priority over everything else
      if (sideWallMissing) {
        wfState = WF_RIGHT_PIVOT;
        return;
      }

      // LEFT PIVOT only when wall in front AND wall on side
      if (frontWallDetected && sideWallDetected) {
        wfState = WF_LEFT_PIVOT;
        return;
      }

      // Normal wall follow
      if (frontClear && sideWallDetected) {
        wallFollowControl();
      } else {
        stopMotors();
      }
      return;

    case WF_LEFT_PIVOT:
      // Stop wall-following immediately, then move up to 15 cm from wall
      stopMotors();
      delay(50);

      if (frontCm == 0 || frontCm > FRONT_STOP_CM) {
        moveUntilFrontStopDistance();
      }

      stopMotors();
      delay(80);

      // Then pivot left
      pivotLeft90();
      delay(100);

      // Resume wall follow
      wfState = WF_FOLLOW;
      return;

    case WF_RIGHT_PIVOT:
      // Move forward a little first
      driveForwardTimed(FORWARD_2CM_MS, BASE_SPEED, BASE_SPEED);
      delay(80);

      // Then pivot right
      pivotRight90();
      delay(100);

      // Resume wall follow
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

  Serial.print("SIDE: ");
  Serial.print(sideCm);
  Serial.print(" cm");

  Serial.print(" | FRONT: ");
  Serial.print(frontCm);
  Serial.print(" cm");

  Serial.print(" | STATE: ");

  if (wfState == WF_FOLLOW) {
    Serial.print("FOLLOW");
  }
  else if (wfState == WF_LEFT_PIVOT) {
    Serial.print("LEFT_PIVOT");
  }
  else if (wfState == WF_RIGHT_PIVOT) {
    Serial.print("RIGHT_PIVOT");
  }

  Serial.println();
}
}
