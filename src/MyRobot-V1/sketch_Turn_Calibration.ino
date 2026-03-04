const int enA = 9;
const int enB = 10;
const int in1 = 6;
const int in2 = 7;
const int in3 = 4;
const int in4 = 5;

enum Dir { LEFT, RIGHT };
Dir dir = LEFT;

int turnSpeed = 120;

// Option A settings
int pulseMs  = 120;   // each pulse duration
int settleMs = 40;   // pause after each pulse

unsigned long pulseCountLeft  = 0;
unsigned long pulseCountRight = 0;

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

// Keep consistent with your wiring assumptions
void rotateRight(int speedVal) {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);  digitalWrite(in4, HIGH);
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void rotateLeft(int speedVal) {
  digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  analogWrite(enA, speedVal);
  analogWrite(enB, speedVal);
}

void doOnePulse() {
  if (dir == LEFT) rotateLeft(turnSpeed);
  else rotateRight(turnSpeed);

  delay(pulseMs);
  stopMotors();
  delay(settleMs);

  if (dir == LEFT) pulseCountLeft++;
  else pulseCountRight++;
}

void doNPulses(int n) {
  if (n <= 0) {
    Serial.println("Enter a positive integer (e.g., 5).");
    return;
  }

  Serial.print("Doing ");
  Serial.print(n);
  Serial.print(" pulses ");
  Serial.print((dir == LEFT) ? "LEFT" : "RIGHT");
  Serial.print(" | pulseMs=");
  Serial.print(pulseMs);
  Serial.print(" | speed=");
  Serial.println(turnSpeed);

  for (int i = 0; i < n; i++) {
    doOnePulse();
  }

  Serial.print("Done. Count now (");
  Serial.print((dir == LEFT) ? "LEFT" : "RIGHT");
  Serial.print(") = ");
  Serial.println((dir == LEFT) ? pulseCountLeft : pulseCountRight);
}

void printStatus() {
  Serial.println("\n--- STATUS ---");
  Serial.print("Direction: "); Serial.println((dir == LEFT) ? "LEFT" : "RIGHT");
  Serial.print("turnSpeed (PWM): "); Serial.println(turnSpeed);
  Serial.print("pulseMs: "); Serial.println(pulseMs);
  Serial.print("settleMs: "); Serial.println(settleMs);
  Serial.print("LEFT pulses: "); Serial.println(pulseCountLeft);
  Serial.print("RIGHT pulses: "); Serial.println(pulseCountRight);
  Serial.println("-------------\n");
}

void printHelp() {
  Serial.println("\n--- Turn Calibration (Enter N pulses) ---");
  Serial.println("Commands:");
  Serial.println("  l              -> direction LEFT");
  Serial.println("  r              -> direction RIGHT");
  Serial.println("  <number>       -> do that many pulses (e.g., 5)");
  Serial.println("  speed <0-255>  -> set turning PWM speed");
  Serial.println("  pulse <ms>     -> set pulse duration (ms)");
  Serial.println("  settle <ms>    -> set settle time between pulses (ms)");
  Serial.println("  reset          -> reset pulse counts");
  Serial.println("  status         -> show settings + counts");
  Serial.println("  help           -> show commands\n");
}

bool isIntegerString(const String &s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (c < '0' || c > '9') return false;
  }
  return true;
}

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  stopMotors();
  printHelp();
  printStatus();
}

void loop() {
  if (!Serial.available()) return;

  String s = Serial.readStringUntil('\n');
  s.trim();

  if (s == "help") {
    printHelp();
    return;
  }
  if (s == "status") {
    printStatus();
    return;
  }
  if (s == "reset") {
    pulseCountLeft = 0;
    pulseCountRight = 0;
    Serial.println("Counts reset.");
    return;
  }
  if (s == "l") {
    dir = LEFT;
    Serial.println("Direction = LEFT");
    return;
  }
  if (s == "r") {
    dir = RIGHT;
    Serial.println("Direction = RIGHT");
    return;
  }

  if (s.startsWith("speed ")) {
    int v = s.substring(6).toInt();
    turnSpeed = constrain(v, 0, 255);
    Serial.print("turnSpeed set to "); Serial.println(turnSpeed);
    return;
  }

  if (s.startsWith("pulse ")) {
    int v = s.substring(6).toInt();
    pulseMs = constrain(v, 1, 2000);
    Serial.print("pulseMs set to "); Serial.println(pulseMs);
    return;
  }

  if (s.startsWith("settle ")) {
    int v = s.substring(7).toInt();
    settleMs = constrain(v, 0, 2000);
    Serial.print("settleMs set to "); Serial.println(settleMs);
    return;
  }

  // If they typed just a number like "5"
  if (isIntegerString(s)) {
    int n = s.toInt();
    doNPulses(n);
    return;
  }

  Serial.println("Unknown input. Type 'help' or enter a number like 5.");
}