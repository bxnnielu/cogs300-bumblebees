// SIMPLE WALL FOLLOWING!
// Type "w" to start wall following 
// Type "s" to stop wall following 

// ------------------ Motor pins ------------------
int enA = 9;    // Right motor PWM
int enB = 10;   // Left motor PWM

int in1 = 6;    // Right motor direction
int in2 = 7;

int in3 = 4;    // Left motor direction
int in4 = 5;

// ------------------ Ultrasonic pins ------------------
int trigSide = A2;
int echoSide = A3;

int trigFront = 12;
int echoFront = 11;

// ------------------ Sensor reading ------------------
long sideCm = 0;

// ------------------ Wall follow settings ------------------
const int TARGET_DIST = 12;
const int TOLERANCE   = 2;
const int WALL_LOST   = 40;

const int BASE_SPEED  = 120;
const int TURN_SPEED  = 90;

// ------------------ Mode control ------------------
bool wallFollowMode = false;


// =================================================
// MOTOR FUNCTIONS
// =================================================

void forward(int rightSpd, int leftSpd) {

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, rightSpd);
  analogWrite(enB, leftSpd);
}

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void turnLeftSoft() {
  forward(BASE_SPEED + 20, TURN_SPEED);
}

void turnRightSoft() {
  forward(TURN_SPEED, BASE_SPEED + 20);
}


// =================================================
// ULTRASONIC FUNCTION
// =================================================

long readUltrasonicCM(int trigPin, int echoPin) {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return 999;
  }

  return duration * 0.034 / 2;
}


// =================================================
// WALL FOLLOW LOGIC
// =================================================

void wallFollow() {

  sideCm = readUltrasonicCM(trigSide, echoSide);

  Serial.print("Side distance: ");
  Serial.println(sideCm);

  if (sideCm > WALL_LOST) {
    Serial.println("Wall lost -> search right");
    turnRightSoft();
  }

  else if (sideCm < TARGET_DIST - TOLERANCE) {
    Serial.println("Too close -> steer left");
    turnLeftSoft();
  }

  else if (sideCm > TARGET_DIST + TOLERANCE) {
    Serial.println("Too far -> steer right");
    turnRightSoft();
  }

  else {
    Serial.println("Good distance -> forward");
    forward(BASE_SPEED, BASE_SPEED);
  }
}


// =================================================
// SERIAL COMMAND HANDLER
// =================================================

void checkCommands() {

  if (Serial.available()) {

    char cmd = Serial.read();

    if (cmd == 'w') {
      wallFollowMode = true;
      Serial.println("Wall following ON");
    }

    if (cmd == 's') {
      wallFollowMode = false;
      stopMotors();
      Serial.println("STOPPED");
    }
  }
}


// =================================================
// SETUP
// =================================================

void setup() {

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(trigSide, OUTPUT);
  pinMode(echoSide, INPUT);

  Serial.begin(9600);
}


// =================================================
// LOOP
// =================================================

void loop() {

  checkCommands();

  if (wallFollowMode) {
    wallFollow();
  }

  delay(80);
}