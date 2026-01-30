import processing.serial.*;

Serial arduino;

// ===== SET THESE =====
int BAUD = 9600;
int PORT_INDEX = 3;
// =====================

String lastCmd = "(none)";
String telemetry = "", telemetry2 = "", telemetry3 = "";

// D-pad geometry
float dCx, dCy;
float btnR = 42;     // button radius
float gap = 58;      // distance from center to each direction button

// Separate stop button
CircleButton stopBtn;

// D-pad buttons
CircleButton upBtn, downBtn, leftBtn, rightBtn, centerBtn;

// Latching drive state (keeps going after release)
char driveLatched = 's';  // 'f','b','s'

void setup() {
  size(720, 520);
  smooth();
  textFont(createFont("Arial", 16));
  textAlign(CENTER, CENTER);

  println("Available serial ports:");
  println(Serial.list());

  arduino = new Serial(this, Serial.list()[PORT_INDEX], BAUD);
  arduino.bufferUntil('\n');

  dCx = width * 0.62;
  dCy = height * 0.45;

  // Build D-pad like the image
  upBtn     = new CircleButton(dCx, dCy - gap, btnR, "▲", 'f'); // forward
  downBtn   = new CircleButton(dCx, dCy + gap, btnR, "▼", 'b'); // backward
  leftBtn   = new CircleButton(dCx - gap, dCy, btnR, "◀", 'l'); // turn left
  rightBtn  = new CircleButton(dCx + gap, dCy, btnR, "▶", 'r'); // turn right
  centerBtn = new CircleButton(dCx, dCy, btnR * 0.55, "•", 'n'); // neutral turn

  stopBtn = new CircleButton(width * 0.20, height * 0.70, 55, "STOP", 's');
}

void draw() {
  background(245);

  drawStatusPanel();
  drawRemoteBody();

  // Draw D-pad
  upBtn.draw();
  downBtn.draw();
  leftBtn.draw();
  rightBtn.draw();
  centerBtn.draw();

  // Draw stop
  stopBtn.draw();

  drawHints();
}

void drawRemoteBody() {
  // A simple “remote” background behind the D-pad
  noStroke();
  fill(235);
  rect(dCx - 150, dCy - 160, 300, 320, 28);

  // little shadow
  fill(230);
  ellipse(dCx, dCy, 240, 240);
}

void drawStatusPanel() {
  int x = 25, y = 25, w = 260, h = 200;

  fill(255);
  stroke(180);
  rect(x, y, w, h, 16);

  fill(20);
  noStroke();
  textSize(18);
  text("STATUS", x + w/2, y + 22);

  textAlign(LEFT, TOP);
  textSize(13);

  float yy = y + 50;
  text("Last command: " + lastCmd, x + 12, yy); yy += 24;
  text("Drive latched: " + driveLatched, x + 12, yy); yy += 24;

  text("Telemetry:", x + 12, yy); yy += 20;
  text("• " + telemetry,  x + 12, yy); yy += 18;
  text("• " + telemetry2, x + 12, yy); yy += 18;
  text("• " + telemetry3, x + 12, yy);

  textAlign(CENTER, CENTER);
}

void drawHints() {
  fill(40);
  textSize(14);
  text("D-pad: ▲ forward (latches), ▼ backward (latches), ◀/▶ turn, • neutral turn", width/2, height - 40);
  text("STOP button sends 's' and cancels everything", width/2, height - 18);
}

// ---------- INPUT LOGIC ----------
// NOTE: We do NOT stop on mouse release.
// Forward/back are “latched” (keep spinning) until STOP or opposite direction.
void mousePressed() {
  if (stopBtn.isInside(mouseX, mouseY)) {
    driveLatched = 's';
    sendCmd('s');
    return;
  }

  // D-pad clicks
  if (upBtn.isInside(mouseX, mouseY)) {
    driveLatched = 'f';
    sendCmd('f'); // keep going after release
    return;
  }

  if (downBtn.isInside(mouseX, mouseY)) {
    driveLatched = 'b';
    sendCmd('b'); // keep going after release
    return;
  }

  if (leftBtn.isInside(mouseX, mouseY)) {
    sendCmd('l'); // turning while driving
    return;
  }

  if (rightBtn.isInside(mouseX, mouseY)) {
    sendCmd('r');
    return;
  }

  if (centerBtn.isInside(mouseX, mouseY)) {
    sendCmd('n'); // neutral turn
    return;
  }
}

// Optional keyboard controls that match your Arduino commands
void keyPressed() {
  if (key == 'w' || key == 'W') { driveLatched = 'f'; sendCmd('f'); }
  if (key == 's' || key == 'S') { driveLatched = 'b'; sendCmd('b'); }
  if (key == 'a' || key == 'A') sendCmd('l');
  if (key == 'd' || key == 'D') sendCmd('r');
  if (key == 'n' || key == 'N') sendCmd('n');
  if (key == ' ') { driveLatched = 's'; sendCmd('s'); }
}

// ---------- SERIAL ----------
void sendCmd(char c) {
  if (arduino == null) return;
  arduino.write(c);
  lastCmd = "'" + c + "'";
  println("Sent: " + c);
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);
  if (line.length() == 0) return;

  telemetry3 = telemetry2;
  telemetry2 = telemetry;
  telemetry = line;
}

// ---------- UI CLASS ----------
class CircleButton {
  float x, y, r;
  String label;
  char cmd;

  CircleButton(float x, float y, float r, String label, char cmd) {
    this.x = x;
    this.y = y;
    this.r = r;
    this.label = label;
    this.cmd = cmd;
  }

  void draw() {
    boolean hover = isInside(mouseX, mouseY);

    // Button styling
    stroke(120);
    fill(hover ? 210 : 225);
    ellipse(x, y, r*2, r*2);

    // inner ring
    noFill();
    stroke(160);
    ellipse(x, y, r*1.55, r*1.55);

    // Label
    fill(30);
    noStroke();
    textSize(24);
    text(label, x, y - 2);
  }

  boolean isInside(float mx, float my) {
    return dist(mx, my, x, y) <= r;
  }
}
