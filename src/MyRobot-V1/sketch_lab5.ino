bool distanceHold = false;

bool wallFollowMode = false;

// ---- Wall-following parameters ----
const float WALL_SET_POINT = 20.0;  // cm
const float WALL_KP = 5.0;

const int BASE_SPEED = 160;

// ---- P-controller parameters ----
const float SET_POINT_CM = 25.0;   // desired distance
const float KP = 12;              // proportional gain (tune this!)

const int MAX_SPEED = 230;          // safety cap
const int DEAD_BAND = 2;            // cm tolerance (prevents jitter)

// -----------------------------

char driveMode = 's';   // 'f', 'b', 's'
char turnMode  = 'n';   // 'l', 'r', 'n'
int speed = 200;          // 0–255

bool spinMode = false;   // true = turning in place
char spinDir  = 'l';     // 'l' or 'r'


int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int enB = 10;

int in1 = 6;   // Direction control pin 1 for Motor A - Right wheel
int in2 = 7;   // Direction control pin 2 for Motor A - Right wheel

int in3 = 4; // Direction control pin 3 for Motor B - Left wheel
int in4 = 5; // Direction control pin 4 for Motor B - Left wheel

int encoderLeft = 2; // Left wheel's encoder wheel sensor
int encoderRight = 3; // Right wheel's encoder wheel sensor

volatile long leftTicks  = 0;
volatile long rightTicks = 0;

const int TICKS_PER_REV = 20;     // CHANGE to match your encoder
const float WHEEL_DIAMETER = 0.026; // meters (2.6 cm)


int trigPin = 13;    // Trigger
int echoPin = 12;    // Echo

int pos = 0;
long duration, cm, inches;

void leftEncoderISR() {
  leftTicks++;
}

void rightEncoderISR() {
  rightTicks++;
}



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

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);
}

float ticksToDistance(long ticks) {
  float rotations = (float)ticks / TICKS_PER_REV;
  float circumference = PI * WHEEL_DIAMETER;
  return rotations * circumference;
}


void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  if (distanceHold) {
  float error = cm - SET_POINT_CM;

  if (abs(error) < DEAD_BAND) error = 0;

  float control = KP * error;

  if (control > 0) driveMode = 'f';
  else if (control < 0) driveMode = 'b';
  else driveMode = 's';

  speed = constrain(abs(control), 0, MAX_SPEED);
  turnMode = 'n';
  spinMode = false;
  }

  
  
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'p') distanceHold = true;   // enable P-control
    if (command == 'o') distanceHold = false;  // disable P-control

    if (command == 'w') {
      wallFollowMode = true;
      distanceHold = false;
      spinMode = false;
    }

    if (command == 'm') {
      wallFollowMode = false;
    }

    // ---- Spin in place (highest priority) ----
    if (command == 'q') {
      spinMode = true;
      spinDir = 'l';
    }

    if (command == 'e') {
      spinMode = true;
      spinDir = 'r';
    }

    if (command == 'x') {
      spinMode = false;
    }

    // ---- Drive commands ----
    if (command == 'f' || command == 'b') {
      speed = 200;
      driveMode = command;
      // speed = Serial.parseInt();
      // speed = constrain(speed, 0, 255);
    }

    if (command == 's') {
      driveMode = 's';
      speed = 0;
      turnMode = 'n';
      spinMode = false;
    }

    // ---- Turning while driving ----
    if (command == 'l') turnMode = 'l';
    if (command == 'r') turnMode = 'r';
    if (command == 'n') turnMode = 'n';
  }

  if (spinMode) {
    turnInPlace();
  }
  else if (wallFollowMode) {
    wallFollowControl();
  }
  else {
    applyMotion();
  }

  
  // ---- Encoder reading (simple) ----
  // int leftVal  = digitalRead(encoderLeft);
  // int rightVal = digitalRead(encoderRight);

  // Serial.print(leftVal);
  // Serial.print(',');
  // Serial.println(rightVal);

  // delay(5);   // ~200 Hz


  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 200) {   // 5 Hz telemetry
    lastPrint = millis();

    noInterrupts();
    long lTicks = leftTicks;
    long rTicks = rightTicks;
    interrupts();

    float leftDist  = ticksToDistance(lTicks);
    float rightDist = ticksToDistance(rTicks);
    float avgDist   = (leftDist + rightDist) / 2.0;

    Serial.print("L ticks: ");
    Serial.print(lTicks);
    Serial.print(" | R ticks: ");
    Serial.print(rTicks);

    Serial.print(" | Distance (m): ");
    Serial.println(avgDist, 3);
  }

}

void turnInPlace() {
  int spinSpeed = 200;  // you can tune this

  if (spinDir == 'l') {
    // Left wheel backward, right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, spinSpeed);

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, spinSpeed);
  }

  if (spinDir == 'r') {
    // Left wheel forward, right wheel backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, spinSpeed);

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, spinSpeed);
  }
}


void applyMotion() {

  // Highest priority: spin in place
  if (spinMode) {
    turnInPlace();
    return;
  }

  if (driveMode == 's') {
    stop();
    return;
  }

  int leftSpeed  = speed;
  int rightSpeed = speed;

  if (turnMode == 'l') leftSpeed = speed / 4;
  if (turnMode == 'r') rightSpeed = speed / 4;

  if (driveMode == 'f') {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  if (driveMode == 'b') {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  analogWrite(enA, rightSpeed);
  analogWrite(enB, leftSpeed);
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void wallFollowControl() {
  // cm already contains SIDE distance when sensor is rotated
  float error = cm - WALL_SET_POINT;

  if (abs(error) < DEAD_BAND) error = 0;

  float correction = WALL_KP * error;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  // Always drive forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  analogWrite(enA, rightSpeed);
  analogWrite(enB, leftSpeed);
}

// --------------------- SCRAP WORK BELOW  ------------------

// int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
// int enB = 10;

// int in1 = 2;   // Direction control pin 1 for Motor A
// int in2 = 3;   // Direction control pin 2 for Motor A

// int in3 = 4; // Direction control pin 3 for Motor B
// int in4 = 5; // Direction control pin 4 for Motor B

// void setup() {
//     // Set motor control pins as outputs
//     pinMode(enA, OUTPUT);
//     pinMode(enB, OUTPUT);
//     pinMode(in1, OUTPUT);
//     pinMode(in2, OUTPUT);
//     pinMode(in3, OUTPUT);
//     pinMode(in4, OUTPUT);

//     Serial.begin(9600);
// }

// void loop() {
//   if (Serial.available()) {
//     char command = Serial.read();   // read 'f', 'b', or 's'

//     if (command == 'f' || command == 'b') {
//       int speed = Serial.parseInt();  // read speed (0–255)
//       speed = constrain(speed, 0, 255);

//       if (command == 'f') forward(speed);
//       if (command == 'b') backward(speed);
//     }

//     if (command == 's') {
//       stop();
//     }
//   }
// }

// void forward(int speed) {
//   digitalWrite(in1, LOW);
//   digitalWrite(in2, HIGH);
//   analogWrite(enA, speed);

//   digitalWrite(in3, LOW);
//   digitalWrite(in4, HIGH);
//   analogWrite(enB, speed);
// }

// void backward(int speed) {
//   digitalWrite(in1, HIGH);
//   digitalWrite(in2, LOW);
//   analogWrite(enA, speed);

//   digitalWrite(in3, HIGH);
//   digitalWrite(in4, LOW);
//   analogWrite(enB, speed);
// }

// void stop() {
//   analogWrite(enA, 0);
//   analogWrite(enB, 0);
// }

// void turnRight() {
//     // Right wheel doesn't turn
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, LOW);
//     analogWrite(enA, 0);

//     // Left wheel turns 
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, HIGH);
//     analogWrite(enB, 255);
// }

// void turnLeft() {
//     // Left wheel doesn't turn
//     digitalWrite(in3, LOW);
//     digitalWrite(in4, LOW);
//     analogWrite(enB, 0);

//     // Right wheel turns 
//     digitalWrite(in1, LOW);
//     digitalWrite(in2, HIGH);
//     analogWrite(enA, 255);

// }

// void turnInPlace() {

// }
