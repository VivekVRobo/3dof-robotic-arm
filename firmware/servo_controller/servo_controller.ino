#include <Servo.h>

Servo baseServo, shoulderServo, elbowServo;
const uint8_t BASE_PIN = 5;
const uint8_t SHOULDER_PIN = 6;
const uint8_t ELBOW_PIN = 9;

String line;

bool inRange(float value) { return value >= 0.0f && value <= 180.0f; }

void setup() {
  Serial.begin(115200);
  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  baseServo.write(90); shoulderServo.write(90); elbowServo.write(90);
}

void handleLine(const String& input) {
  if (!input.startsWith("J,")) return;
  int c1 = input.indexOf(',', 2);
  int c2 = input.indexOf(',', c1 + 1);
  if (c1 < 0 || c2 < 0) return;
  float a = input.substring(2, c1).toFloat();
  float b = input.substring(c1 + 1, c2).toFloat();
  float c = input.substring(c2 + 1).toFloat();
  if (!inRange(a) || !inRange(b) || !inRange(c)) return;
  baseServo.write((int)a);
  shoulderServo.write((int)b);
  elbowServo.write((int)c);
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      handleLine(line);
      line = "";
    } else if (ch != '\r' && line.length() < 64) {
      line += ch;
    }
  }
}
