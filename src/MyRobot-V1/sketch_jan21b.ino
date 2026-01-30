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

  Serial.begin(9600);
}

float ticksToDistance(long ticks) {
  float rotations = (float)ticks / TICKS_PER_REV;
  float circumference = PI * WHEEL_DIAMETER;
  return rotations * circumference;
}


void loop() {
  if (Serial.available()) {
    char command = Serial.read();

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

  applyMotion();

  
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
