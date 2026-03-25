const int BASE_SPEED = 140;
const int TURN_SPEED = 140;

const int WALL_DETECTED_CM   = 45;
const int FRONT_THRESHOLD_CM = 20;

// 

int trigSide  = A2;
int echoSide  = A3;

int trigFront = 12;
int echoFront = 11;

int enA = 9;
int enB = 10;

int in1 = 6;
int in2 = 7;

int in3 = 4;
int in4 = 5;

//

const float WALL_SET_POINT = 18.0;
const float WALL_KP = 4;

const int MIN_SPEED = 80;
const int MAX_SPEED = 200;
const int DEAD_BAND = 1;

// ------------------ Robot states ------------------
enum RobotState {
  ROBOT_MAZE,
  ROBOT_OBJECT_DETECTION
};

RobotState robotState = ROBOT_MAZE;

// ------------------ Wall-follow states ------------------
enum WFState {
  WF_FOLLOW,
  WF_PREP_RIGHT,
  WF_TURN_RIGHT,
  WF_PREP_LEFT,
  WF_CREEP_FORWARD,
  WF_TURN_LEFT
};

WFState wfState = WF_FOLLOW;

// ------------------ Motion states ------------------
enum MotionState {
  MOTION_IDLE,
  MOTION_FORWARD,
  MOTION_PIVOT_LEFT,
  MOTION_PIVOT_RIGHT
};

MotionState motionState = MOTION_IDLE;

// ------------------ Timing ------------------
unsigned long motionStartTime = 0;
unsigned long lastSensorTime = 0;

// ------------------ Constants ------------------
const int SENSOR_INTERVAL = 40; // ms

const int FORWARD_2CM_MS    = 160;
const int PIVOT_LEFT_90_MS  = 400;
const int PIVOT_RIGHT_90_MS = 400;

const int FRONT_STOP_CM = 20;

// ------------------ Sensor values ------------------
long sideCm = 0;
long frontCm = 0;

// ------------------ NON-BLOCKING ultrasonic ------------------

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

  stopMotors();
}

long readUltrasonicCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long d = pulseIn(echo, HIGH, 15000); // reduced timeout

  if (d == 0) return 0;
  return (d / 2) / 29.1;
}

void updateSensors() {
  if (millis() - lastSensorTime < SENSOR_INTERVAL) return;

  lastSensorTime = millis();

  sideCm  = readUltrasonicCM(trigSide, echoSide);
  frontCm = readUltrasonicCM(trigFront, echoFront);
}

// ------------------ Motion control ------------------
void setMotion(MotionState newState) {
  motionState = newState;
  motionStartTime = millis();
}

bool motionDone(int duration) {
  return millis() - motionStartTime >= duration;
}

void updateMotion() {
  switch (motionState) {

    case MOTION_FORWARD:
      forward(BASE_SPEED, BASE_SPEED);
      break;

    case MOTION_PIVOT_LEFT:
      pivotLeft(TURN_SPEED);
      break;

    case MOTION_PIVOT_RIGHT:
      pivotRight(TURN_SPEED);
      break;

    case MOTION_IDLE:
      stopMotors();
      break;
  }
}

// ------------------ Wall-follow logic ------------------
void mazeController() {

  bool rightOpen = (sideCm == 0) || (sideCm > WALL_DETECTED_CM);
  bool frontOpen = (frontCm == 0) || (frontCm > FRONT_THRESHOLD_CM);

  switch (wfState) {

    case WF_FOLLOW:

      // PRIORITY: right turn
      if (rightOpen) {
        wfState = WF_PREP_RIGHT;
        setMotion(MOTION_FORWARD);
        return;
      }

      // front blocked → left turn
      if (!frontOpen) {
        wfState = WF_PREP_LEFT;
        setMotion(MOTION_IDLE);
        return;
      }

      // normal wall follow
      wallFollowControl();
      return;

    // ---------- RIGHT TURN ----------
    case WF_PREP_RIGHT:

      if (motionDone(FORWARD_2CM_MS)) {
        setMotion(MOTION_PIVOT_RIGHT);
        wfState = WF_TURN_RIGHT;
      }
      return;

    case WF_TURN_RIGHT:

      if (motionDone(PIVOT_RIGHT_90_MS)) {
        setMotion(MOTION_IDLE);
        wfState = WF_FOLLOW;
      }
      return;

    // ---------- LEFT TURN ----------
    case WF_PREP_LEFT:
      setMotion(MOTION_FORWARD);
      wfState = WF_CREEP_FORWARD;
      return;

    case WF_CREEP_FORWARD:

      // move until close to wall
      if (frontCm > 0 && frontCm <= FRONT_STOP_CM) {
        setMotion(MOTION_PIVOT_LEFT);
        wfState = WF_TURN_LEFT;
      }
      return;

    case WF_TURN_LEFT:

      if (motionDone(PIVOT_LEFT_90_MS)) {
        setMotion(MOTION_IDLE);
        wfState = WF_FOLLOW;
      }
      return;
  }
}

// ------------------ LOOP ------------------
void loop() {

  if (Serial.available()) {
  char cmd = Serial.read();

  if (cmd == 's') {
    stopMotors();

    while (true) {
      // do nothing forever
    }
  }
}

  updateSensors();   // always running
  updateMotion();    // always running

  if (robotState == ROBOT_MAZE) {
    mazeController();
  }

  // -------- exit condition --------
  if (frontCm > 200 && frontCm < 300) {
    stopMotors();
    robotState = ROBOT_OBJECT_DETECTION;
    Serial.println("MAZE COMPLETE");
  }

  // -------- telemetry --------
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 100) {
    lastPrint = millis();

    Serial.print("SIDE: ");
    Serial.print(sideCm);
    Serial.print(" | FRONT: ");
    Serial.print(frontCm);
    Serial.print(" | STATE: ");
    Serial.print(wfState);
    Serial.print(" | MOTION: ");
    Serial.println(motionState);
  }
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void forward(int rightSpd, int leftSpd) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  analogWrite(enA, rightSpd);
  analogWrite(enB, leftSpd);
}

void pivotLeft(int spd) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, spd);

  analogWrite(enB, 0);
}

void pivotRight(int spd) {
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, spd);

  analogWrite(enA, 0);
}

void wallFollowControl() {

  // If no wall detected, just go forward (or you could stop)
  if (sideCm == 0 || sideCm > WALL_DETECTED_CM) {
    forward(BASE_SPEED, BASE_SPEED);
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