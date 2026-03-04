#include <Servo.h>

Servo servo;

// --- CHANGE THIS IF NEEDED ---
const int SERVO_PIN = A0;

// Start guess (change if you want)
int currentAngle = 90;

// Safety clamp (most servos are safe 0–180, but your mount might not be)
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;

// Step for + / - commands
const int NUDGE_STEP = 1;

void moveToAngle(int angle) {
  angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
  currentAngle = angle;
  servo.write(currentAngle);

  Serial.print("Moved to: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
}

void printHelp() {
  Serial.println("\n--- Servo Center Finder ---");
  Serial.println("Type an angle (0 to 180) and press Enter.");
  Serial.println("Examples: 90  |  88  |  102");
  Serial.println("Or use:");
  Serial.println("  +   (nudge +1 degree)");
  Serial.println("  -   (nudge -1 degree)");
  Serial.println("  ++  (nudge +5 degrees)");
  Serial.println("  --  (nudge -5 degrees)");
  Serial.println("  h   (help)");
  Serial.println("---------------------------\n");
}

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; } // helps on some boards; harmless otherwise

  servo.attach(SERVO_PIN);
  printHelp();
  moveToAngle(currentAngle);
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) return;

  if (cmd == "h" || cmd == "help") {
    printHelp();
    return;
  }

  if (cmd == "+") {
    moveToAngle(currentAngle + NUDGE_STEP);
    return;
  }

  if (cmd == "-") {
    moveToAngle(currentAngle - NUDGE_STEP);
    return;
  }

  if (cmd == "++") {
    moveToAngle(currentAngle + 5);
    return;
  }

  if (cmd == "--") {
    moveToAngle(currentAngle - 5);
    return;
  }

  // Otherwise, try parsing as an integer angle
  bool isNumber = true;
  for (unsigned int i = 0; i < cmd.length(); i++) {
    char c = cmd.charAt(i);
    if (!(c == '-' || (c >= '0' && c <= '9'))) {
      isNumber = false;
      break;
    }
  }

  if (!isNumber) {
    Serial.println("Unrecognized input. Type an angle (e.g., 90) or + / - / h.");
    return;
  }

  int angle = cmd.toInt();
  moveToAngle(angle);
}