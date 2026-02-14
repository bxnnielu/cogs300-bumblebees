// ---------------- Motor Pins ----------------
int enA = 9;    // Right motor PWM
int enB = 10;   // Left motor PWM

int in1 = 6;    // Right motor direction
int in2 = 7;

int in3 = 4;    // Left motor direction
int in4 = 5;

// ---------------- IR Sensors ----------------
int rightIR = 3;   // Right IR sensor
int leftIR  = 2;   // Left IR sensor

// ---------------- Speed ----------------
int baseSpeed = 80;
int turnSpeed = 180;

bool lineFollowMode = false;

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
  Serial.println("Type 'f' to start line following, 's' to stop.");
}

void loop() {

  // -------- Serial Commands --------
  if (Serial.available() > 0) {
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

  if (!lineFollowMode) {
    return;
  }

  // -------- Sensor Read --------
  int leftVal  = digitalRead(leftIR);
  int rightVal = digitalRead(rightIR);

  // NEW RULE:
  // 0 = line
  // 1 = not line

  if (leftVal == 0 && rightVal == 0) {
    forward(baseSpeed);
  }
  else if (leftVal == 0 && rightVal == 1) {
    turnLeft(turnSpeed);
  }
  else if (leftVal == 1 && rightVal == 0) {
    turnRight(turnSpeed);
  }
  else {
    stopMotors(); // both 1 = lost line
  }

  delay(20);
}

// ---------------- Motor Functions ----------------
void forward(int speedVal) {
  // Right motor forward (SWAPPED)
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Left motor forward (SWAPPED)
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void turnLeft(int speedVal) {
  // Right motor forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Left motor stop
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, speedVal);
  analogWrite(enB, 0);
}

void turnRight(int speedVal) {
  // Left motor forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  // Right motor stop
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, speedVal);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
