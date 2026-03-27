// ---------------- Motor Pins ----------------
int enA = 9;    // Right motor PWM
int enB = 10;   // Left motor PWM

int in1 = 6;    // Right motor direction
int in2 = 7;

int in3 = 5;    // Left motor direction
int in4 = 4;

// ---------------- IR Sensors ----------------
int rightIR = 3;
int leftIR  = 2;

// ---------------- Speed ----------------
int baseSpeed = 80;
int turnSpeed = 150;
int searchSpeed = 120;

bool lineFollowMode = false;

// Remember last known direction
// -1 = left, 1 = right, 0 = straight
int lastDirection = 0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  Serial.begin(9600);
  Serial.println("Type 'f' to start, 's' to stop.");
}

void loop() {

  // -------- Serial Commands --------
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'f') {
      lineFollowMode = true;
      Serial.println("Line following ON");
    }
    else if (cmd == 's') {
      lineFollowMode = false;
      stopMotors();
      Serial.println("STOPPED");
    }
  }

  if (!lineFollowMode) return;

  // -------- Sensor Read --------
  int leftVal  = digitalRead(leftIR);
  int rightVal = digitalRead(rightIR);

  // 0 = line, 1 = no line

  if (leftVal == 0 && rightVal == 0) {
    // On track
    setMotor(baseSpeed, baseSpeed);
    lastDirection = 0;
  }
  else if (leftVal == 0 && rightVal == 1) {
    // Drifted right → turn left
    setMotor(turnSpeed, 0);
    lastDirection = -1;
  }
  else if (leftVal == 1 && rightVal == 0) {
    // Drifted left → turn right
    setMotor(0, turnSpeed);
    lastDirection = 1;
  }
  else {
    // -------- LOST LINE → SEARCH --------
    searchForLine();
  }

  delay(10);
}

// ---------------- Motor Control ----------------
void setMotor(int rightSpeed, int leftSpeed) {

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  analogWrite(enA, rightSpeed);
  analogWrite(enB, leftSpeed);
}

void stopMotors() {
  setMotor(0, 0);
}


// ---------------- Recovery Behavior ----------------
void searchForLine() {

  if (lastDirection == -1) {
    // Last seen on left → keep turning left
    setMotor(searchSpeed, 0);
  }
  else if (lastDirection == 1) {
    // Last seen on right → keep turning right
    setMotor(0, searchSpeed);
  }
  else {
    // No idea → spin slowly
    setMotor(searchSpeed, -searchSpeed);
  }
}