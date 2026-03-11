/*
  BOUNCING WALL FOLLOW (SIDE ULTRASONIC ONLY) — Arduino Uno

  Behavior:
    - Uses ONLY the side ultrasonic sensor
    - "Too close"  -> steer away from wall
    - "Too far"    -> steer toward wall
    - "In band"    -> drive straight
    - "Wall lost"  -> keep turning toward wall until wall is seen again

  Assumes:
    - Side ultrasonic is mounted facing the wall
    - FOLLOW_RIGHT_WALL = true means wall is on robot's right side
*/

bool wallFollowMode = false;

// ------------------ Wall-follow parameters ------------------
const bool FOLLOW_RIGHT_WALL = true;

// Desired wall region instead of exact setpoint
const int WALL_TARGET_CM = 12;
const int WALL_BAND_CM   = 3;     // acceptable +/- band

const int TOO_CLOSE_CM = WALL_TARGET_CM - WALL_BAND_CM;   // 9 cm
const int TOO_FAR_CM   = WALL_TARGET_CM + WALL_BAND_CM;   // 15 cm

// If side reading is missing or very large, assume wall disappeared
const int WALL_LOST_CM = 45;

// Speeds
const int STRAIGHT_SPEED = 165;
const int TURN_FAST      = 185;
const int TURN_SLOW      = 95;

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
int trigSide  = A2;
int echoSide  = A3;

// Sensor reading
long sideCm = 0;

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

  long d = pulseIn(echo, HIGH, 30000);
  if (d == 0) return 0;

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
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);   // right forward
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);   // left forward

  analogWrite(enA, clampPWM(rightSpd));
  analogWrite(enB, clampPWM(leftSpd));
}

// ------------------ Steering helpers ------------------
// For right-wall following:
//   turn toward wall  = steer right
//   turn away from wall = steer left
//
// For left-wall following, reversed.

void steerTowardWall() {
  if (FOLLOW_RIGHT_WALL) {
    // steer right: left wheel faster, right wheel slower
    forward(TURN_SLOW, TURN_FAST);
  } else {
    // steer left
    forward(TURN_FAST, TURN_SLOW);
  }
}

void steerAwayFromWall() {
  if (FOLLOW_RIGHT_WALL) {
    // steer left: right wheel faster, left wheel slower
    forward(TURN_FAST, TURN_SLOW);
  } else {
    // steer right
    forward(TURN_SLOW, TURN_FAST);
  }
}

void driveStraight() {
  forward(STRAIGHT_SPEED, STRAIGHT_SPEED);
}

// ------------------ Optional manual mode ------------------
bool spinMode = false;
char spinDir  = 'l';

void pivotLeft(int spd) {
  // Right wheel forward, left stopped
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  analogWrite(enA, clampPWM(spd));
  analogWrite(enB, 0);
}

void pivotRight(int spd) {
  // Left wheel forward, right stopped
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  analogWrite(enB, clampPWM(spd));
  analogWrite(enA, 0);
}

// ------------------ Bouncing wall-follow ------------------
void wallFollowBounce() {
  // If wall is lost, keep turning toward the wall until reacquired
  if (sideCm == 0 || sideCm > WALL_LOST_CM) {
    steerTowardWall();
    return;
  }

  // Too close to wall -> move away
  if (sideCm < TOO_CLOSE_CM) {
    steerAwayFromWall();
    return;
  }

  // Too far from wall -> move toward wall
  if (sideCm > TOO_FAR_CM) {
    steerTowardWall();
    return;
  }

  // Within acceptable band -> straight
  driveStraight();
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

  Serial.begin(9600);
}

// ------------------ Loop -------------------------
void loop() {
  sideCm = readUltrasonicCM(trigSide, echoSide);

  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'w') {
      wallFollowMode = true;
      spinMode = false;
    }

    if (command == 'm') {
      wallFollowMode = false;
      spinMode = false;
      stopMotors();
    }

    // Optional manual pivots
    if (command == 'q') { spinMode = true;  spinDir = 'l'; }
    if (command == 'e') { spinMode = true;  spinDir = 'r'; }
    if (command == 'x') { spinMode = false; }

    if (command == 's') {
      wallFollowMode = false;
      spinMode = false;
      stopMotors();
    }
  }

  if (spinMode) {
    if (spinDir == 'l') pivotLeft(180);
    else                pivotRight(180);
  }
  else if (wallFollowMode) {
    wallFollowBounce();
  }
  else {
    stopMotors();
  }

  // Telemetry
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
    Serial.print(" | mode: "); Serial.print(wallFollowMode ? "WALL" : "STOP");

    if (sideCm == 0 || sideCm > WALL_LOST_CM) {
      Serial.print(" | action: REACQUIRE");
    } else if (sideCm < TOO_CLOSE_CM) {
      Serial.print(" | action: AWAY");
    } else if (sideCm > TOO_FAR_CM) {
      Serial.print(" | action: TOWARD");
    } else {
      Serial.print(" | action: STRAIGHT");
    }

    Serial.print(" | L ticks: "); Serial.print(lTicks);
    Serial.print(" | R ticks: "); Serial.print(rTicks);
    Serial.print(" | Dist(m): "); Serial.println(avgDist, 3);
  }
}