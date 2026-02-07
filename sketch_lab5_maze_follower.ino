/*
  WALL FOLLOWING + CORNER HANDLING (SIDE + FRONT ULTRASONIC) — Arduino Uno

  UPDATE: Corner turning is now PIVOT-STYLE:
    - Only ONE wheel moves during corner turns (the other wheel is stopped).
    - Inside corner (left pivot):  RIGHT wheel moves, LEFT wheel stopped
    - Outside corner (right pivot when following right wall): LEFT wheel moves, RIGHT wheel stopped

  SIDE SENSOR  (wall distance):  trigSide=13, echoSide=12
  FRONT SENSOR (inside corner):  trigFront=A0, echoFront=A1
*/

bool wallFollowMode = false;

// ------------------ Wall-follow parameters ------------------
const float WALL_SET_POINT = 12.0;   // cm desired SIDE distance
const float WALL_KP = 6.5;

const int BASE_SPEED = 160;
const int MAX_SPEED  = 230;
const int MIN_SPEED  = 70;

const int DEAD_BAND  = 1;

// ------------------ State machine ----------------------------
enum WFState { WF_FOLLOW, WF_OUTSIDE_CORNER_PIVOT, WF_INSIDE_CORNER_PIVOT };
WFState wfState = WF_FOLLOW;

// Outside corner (side sensor)
const int WALL_LOST_CM  = 55;
const int WALL_FOUND_CM = 30;

// Outside-corner stop condition
const int SIDE_REACQUIRE_COUNT  = 3;
const int OUTSIDE_PIVOT_MAX_MS  = 2000;

unsigned long outsidePivotStart = 0;
int sideFoundCount = 0;

// Inside corner (front sensor)
const int FRONT_START_PIVOT_CM  = 12;   // start pivot when front <= 12
const int FRONT_STOP_TURN_CM    = 30;   // stop pivot when front >= 30 cm
const int FRONT_CLEAR_COUNT     = 3;
const int INSIDE_PIVOT_MAX_MS   = 2500;

unsigned long insidePivotStart = 0;
int frontClearCount = 0;

// Turning speed (pivot wheel speed)
const int TURN_SPEED = 240;

// Follow which wall?
const bool FOLLOW_RIGHT_WALL = true;

// ------------------ Manual (optional) -------------------------
bool spinMode = false;   // keep naming, but now it will "pivot"
char spinDir  = 'l';

// ------------------ Motor pins -------------------------
int enA = 9;    // Right motor PWM
int enB = 10;   // Left motor PWM

int in1 = 6;    // Right motor direction
int in2 = 7;

int in3 = 4;    // Left motor direction
int in4 = 5;

// ------------------ Encoder pins ------------------------
int encoderLeft  = 2;
int encoderRight = 3;

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

const int TICKS_PER_REV = 20;
const float WHEEL_DIAMETER = 0.026;

// ------------------ Ultrasonic pins ----------------------
int trigSide  = 13;
int echoSide  = 12;

int trigFront = A0;
int echoFront = A1;

// Sensor readings
long sideCm  = 0;
long frontCm = 0;

// ------------------ Encoder ISRs -------------------------
void leftEncoderISR()  { leftTicks++; }
void rightEncoderISR() { rightTicks++; }

// ------------------ Ultrasonic read ----------------------
long readUltrasonicCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(5);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long d = pulseIn(echo, HIGH, 30000); // timeout
  if (d == 0) return 0;               // no echo

  long cm = (d / 2) / 29.1;
  if (cm < 0) cm = 0;
  return cm;
}

// ------------------ Helpers -------------------------
float ticksToDistance(long ticks) {
  float rotations = (float)ticks / (float)TICKS_PER_REV;
  float circumference = PI * WHEEL_DIAMETER;
  return rotations * circumference;
}

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
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);  // right forward
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);  // left forward
  analogWrite(enA, clampPWM(rightSpd));
  analogWrite(enB, clampPWM(leftSpd));
}

// Drive ONLY right wheel forward; left wheel stopped
void rightWheelForwardOnly(int spd) {
  // Right forward
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  analogWrite(enA, clampPWM(spd));

  // Left stop
  analogWrite(enB, 0);
}

// Drive ONLY left wheel forward; right wheel stopped
void leftWheelForwardOnly(int spd) {
  // Left forward
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  analogWrite(enB, clampPWM(spd));

  // Right stop
  analogWrite(enA, 0);
}

// Pivot LEFT = move RIGHT wheel, stop LEFT wheel
void pivotLeft(int spd) {
  rightWheelForwardOnly(spd);
}

// Pivot RIGHT = move LEFT wheel, stop RIGHT wheel
void pivotRight(int spd) {
  leftWheelForwardOnly(spd);
}

// Turn toward wall (for reacquire)
void pivotTowardWall(int spd) {
  if (FOLLOW_RIGHT_WALL) pivotRight(spd);
  else                   pivotLeft(spd);
}

// ------------------ Sensor history (ring buffers) ------------------
const int HIST_LEN = 25;

long frontHist[HIST_LEN];
long sideHist[HIST_LEN];
int histHead = 0;
int histCount = 0;

void pushHistory(long frontVal, long sideVal) {
  frontHist[histHead] = frontVal;
  sideHist[histHead]  = sideVal;

  histHead = (histHead + 1) % HIST_LEN;
  if (histCount < HIST_LEN) histCount++;
}

long histGet(const long *arr, int iBack) {
  if (iBack < 0) iBack = 0;
  if (histCount <= 0) return 0;
  if (iBack >= histCount) iBack = histCount - 1;
  int idx = histHead - 1 - iBack;
  while (idx < 0) idx += HIST_LEN;
  return arr[idx];
}

bool lastNAtLeast(const long *arr, int N, long threshold, bool ignoreZero) {
  if (histCount == 0) return false;
  int checked = 0;
  for (int i = 0; i < histCount && checked < N; i++) {
    long v = histGet(arr, i);
    if (ignoreZero && v == 0) continue;
    checked++;
    if (v < threshold) return false;
  }
  return (checked >= N);
}

bool lastNAtMost(const long *arr, int N, long threshold, bool ignoreZero) {
  if (histCount == 0) return false;
  int checked = 0;
  for (int i = 0; i < histCount && checked < N; i++) {
    long v = histGet(arr, i);
    if (ignoreZero && v == 0) continue;
    checked++;
    if (v > threshold) return false;
  }
  return (checked >= N);
}

// ------------------ Wall-follow controller -------------------------
void wallFollowControl() {
  if (sideCm == 0) return;

  float error = (float)sideCm - WALL_SET_POINT;
  if (abs(error) < DEAD_BAND) error = 0;

  float correction = WALL_KP * error;

  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

  forward(rightSpeed, leftSpeed);
}

// ------------------ Main wall-follow with corners -----------------
void wallFollowWithCorners() {
  // OUTSIDE CORNER START: side wall missing => start pivot toward wall
  bool sideMissing = (sideCm == 0) || (sideCm > WALL_LOST_CM);
  if (wfState == WF_FOLLOW && sideMissing) {
    wfState = WF_OUTSIDE_CORNER_PIVOT;
    outsidePivotStart = millis();
    sideFoundCount = 0;
  }

  // INSIDE CORNER START: front <= 12 => start pivot left
  bool frontClose = (frontCm > 0 && frontCm <= FRONT_START_PIVOT_CM);
  if (wfState == WF_FOLLOW && frontClose) {
    wfState = WF_INSIDE_CORNER_PIVOT;
    insidePivotStart = millis();
    frontClearCount = 0;
  }

  // OUTSIDE CORNER PIVOT: pivot toward wall until side is found consistently
  if (wfState == WF_OUTSIDE_CORNER_PIVOT) {
    pivotTowardWall(TURN_SPEED);

    // keep sampling side while turning
    sideCm = readUltrasonicCM(trigSide, echoSide);
    pushHistory(frontCm, sideCm);

    if (sideCm > 0 && sideCm <= WALL_FOUND_CM) sideFoundCount++;
    else sideFoundCount = 0;

    bool foundByHistory = lastNAtMost(sideHist, SIDE_REACQUIRE_COUNT, WALL_FOUND_CM, true);

    bool found = (sideFoundCount >= SIDE_REACQUIRE_COUNT) || foundByHistory;
    bool timedOut = (millis() - outsidePivotStart) > OUTSIDE_PIVOT_MAX_MS;

    if (found || timedOut) wfState = WF_FOLLOW;
    return;
  }

  // INSIDE CORNER PIVOT: pivot left until front has been >=30cm for N reads
  if (wfState == WF_INSIDE_CORNER_PIVOT) {
    pivotLeft(TURN_SPEED);

    // keep sampling front while turning
    frontCm = readUltrasonicCM(trigFront, echoFront);
    pushHistory(frontCm, sideCm);

    if (frontCm >= FRONT_STOP_TURN_CM) frontClearCount++;
    else frontClearCount = 0;

    bool clearedByHistory = lastNAtLeast(frontHist, FRONT_CLEAR_COUNT, FRONT_STOP_TURN_CM, true);

    bool cleared = (frontClearCount >= FRONT_CLEAR_COUNT) || clearedByHistory;
    bool timedOut = (millis() - insidePivotStart) > INSIDE_PIVOT_MAX_MS;

    if (cleared || timedOut) wfState = WF_FOLLOW;
    return;
  }

  // Default: normal wall follow
  wallFollowControl();
}

// ------------------ Setup -------------------------
void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(encoderLeft, INPUT_PULLUP);
  pinMode(encoderRight, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeft), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), rightEncoderISR, RISING);

  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  Serial.begin(9600);

  for (int i = 0; i < HIST_LEN; i++) {
    frontHist[i] = 0;
    sideHist[i]  = 0;
  }
}

// ------------------ Loop -------------------------
void loop() {
  sideCm  = readUltrasonicCM(trigSide,  echoSide);
  frontCm = readUltrasonicCM(trigFront, echoFront);

  pushHistory(frontCm, sideCm);

  // Commands
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'w') {
      wallFollowMode = true;
      wfState = WF_FOLLOW;
      spinMode = false;
    }

    if (command == 'm') {
      wallFollowMode = false;
      wfState = WF_FOLLOW;
      spinMode = false;
      stopMotors();
    }

    // manual pivot override (optional)
    if (command == 'q') { spinMode = true;  spinDir = 'l'; }
    if (command == 'e') { spinMode = true;  spinDir = 'r'; }
    if (command == 'x') { spinMode = false; }

    if (command == 's') {
      spinMode = false;
      stopMotors();
    }
  }

  // Priority control
  if (spinMode) {
    if (spinDir == 'l') pivotLeft(TURN_SPEED);
    else                pivotRight(TURN_SPEED);
  }
  else if (wallFollowMode) {
    wallFollowWithCorners();
  }
  else {
    stopMotors();
  }

  // Encoder telemetry (optional)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();

    noInterrupts();
    long lTicks = leftTicks;
    long rTicks = rightTicks;
    interrupts();

    float leftDist  = ticksToDistance(lTicks);
    float rightDist = ticksToDistance(rTicks);
    float avgDist   = (leftDist + rightDist) / 2.0;

    Serial.print("SIDE(cm): "); Serial.print(sideCm);
    Serial.print(" | FRONT(cm): "); Serial.print(frontCm);
    Serial.print(" | state: "); Serial.print((int)wfState);

    Serial.print(" | L ticks: "); Serial.print(lTicks);
    Serial.print(" | R ticks: "); Serial.print(rTicks);
    Serial.print(" | Dist(m): "); Serial.println(avgDist, 3);
  }
}


