// ---------------- Motor Pins ----------------
int enA = 9;    // Right motor PWM
int enB = 10;   // Left motor PWM

int in1 = 7;    // Right motor direction
int in2 = 6;

int in3 = 4;    // Left motor direction
int in4 = 5;

// ---------------- IR Sensors ----------------
int leftIR   = 2;
int rightIR  = 3;
int centerIR = 8;

// ---------------- Speed Settings ----------------
int baseSpeed   = 120;  // speed for going straight
int turnSpeed   = 120;  // turning speed
int searchSpeed = 120;  // spin speed while searching

// ---------------- Control Variables ----------------
bool lineFollowMode = false;
int lastDirection = 0;  // -1 = left, 1 = right, 0 = straight

// ===================================================
void setup() {
  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Sensor pins
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(centerIR, INPUT);

  Serial.begin(9600);
  Serial.println("Type 'f' to start line following, 's' to stop.");
}

// ===================================================
void loop() {
  // --- Command via Serial Monitor ---
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'f') {
      lineFollowMode = true;
      Serial.println("Line Following ENABLED");
    }
    else if (cmd == 's') {
      lineFollowMode = false;
      stopMotors();
      Serial.println("STOPPED");
    }
  }

  if (!lineFollowMode) return;

  // --- Read Sensors ---
  int leftVal   = digitalRead(leftIR);
  int rightVal  = digitalRead(rightIR);
  int centerVal = digitalRead(centerIR);

  // For debugging in Serial Monitor
  Serial.print("L:");
  Serial.print(leftVal);
  Serial.print(" C:");
  Serial.print(centerVal);
  Serial.print(" R:");
  Serial.println(rightVal);

  // --- LINE FOLLOWING LOGIC (white line on black) ---
  if (centerVal == 0) {
    // Center sensor on white line → go straight
    setMotor(baseSpeed, baseSpeed);
    lastDirection = 0;
  }
  else if (rightVal == 0) {
    // White under right sensor → steer right
    setMotor(turnSpeed, 0);
    lastDirection = 1;
  }
  else if (leftVal == 0) {
    // White under left sensor → steer left
    setMotor(0, turnSpeed);
    lastDirection = -1;
  }
  else {
    // All sensors on dark → lost line
    searchForLine();
  }

  delay(10);
}

// ===================================================
// ---------------- Motor Control --------------------
void setMotor(int rightSpeed, int leftSpeed) {
  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (rightSpeed < 0) {
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
  } else if (leftSpeed < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  analogWrite(enA, abs(rightSpeed));
  analogWrite(enB, abs(leftSpeed));
}


// ---------------------------------------------------
void stopMotors() {
  setMotor(0, 0);
}

// ---------------------------------------------------
// -------------- Lost‑line Recovery -----------------
void searchForLine() {
  stopMotors();
  delay(100);
  Serial.println("Line lost — scanning...");

  if (lastDirection == -1) {
    // Last seen to the left
    setMotor(searchSpeed, 0);
  }
  else if (lastDirection == 1) {
    // Last seen to the right
    setMotor(0, searchSpeed);
  }
  else {
    // Unknown — spin in place
    setMotor(searchSpeed, -searchSpeed);
  }

  delay(400);
  stopMotors();
}

