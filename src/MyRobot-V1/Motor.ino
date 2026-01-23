/**
 * @file Motor.ino
 * @brief Simple H-bridge motor control helpers.
 *
 * Provides a minimal interface for driving a DC motor using
 * digital GPIO pins (e.g. Arduino-style platforms).
 *
 * The motor direction is controlled via two input pins,
 * while a separate enable pin turns the motor on or off.
 *
 * @author Paul Bucci
 * @date 2026
 */


/**
 * @brief Drives a DC motor in a fixed direction using an H-bridge.
 *
 * @param in1 GPIO pin connected to motor driver input 1 (direction control)
 * @param in2 GPIO pin connected to motor driver input 2 (direction control)
 * @param enA GPIO pin connected to motor driver enable pin (motor on/off)
 */
// void drive(int in1, int in2, int enA) {
//     digitalWrite(in1, LOW);   // Direction control: IN1
//     digitalWrite(in2, HIGH);  // Direction control: IN2 (sets rotation direction)
//     digitalWrite(enA, HIGH);  // Enable motor driver
// }

// void stop(int in1, int in2, int enA) {
//     digitalWrite(in1, LOW);   // Direction control: IN1
//     digitalWrite(in2, HIGH);  // Direction control: IN2 (sets rotation direction)
//     digitalWrite(enA, LOW);   // Disable motor driver
// }

char driveMode = 's';   // 'f', 'b', 's'
char turnMode  = 'n';   // 'l', 'r', 'n'
int speed = 0;          // 0–255

bool spinMode = false;   // true = turning in place
char spinDir  = 'l';     // 'l' or 'r'


int enA = 9;   // Enable pin for Motor A — must be a PWM-capable pin
int enB = 10;

int in1 = 2;   // Direction control pin 1 for Motor A - Right wheel
int in2 = 3;   // Direction control pin 2 for Motor A - Right wheel

int in3 = 4; // Direction control pin 3 for Motor B - Left wheel
int in4 = 5; // Direction control pin 4 for Motor B - Left wheel

void setup() {
    // Set motor control pins as outputs
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    Serial.begin(9600);
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
      driveMode = command;
      speed = Serial.parseInt();
      speed = constrain(speed, 0, 255);
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
}

void turnInPlace() {
  int spinSpeed = 100;  // you can tune this

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

  if (turnMode == 'l') leftSpeed = speed / 2;
  if (turnMode == 'r') rightSpeed = speed / 2;

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
