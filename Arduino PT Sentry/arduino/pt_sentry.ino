#include <Servo.h>

// Pin assignments
constexpr uint8_t PAN_PIN = 9;
constexpr uint8_t TILT_PIN = 10;
constexpr uint8_t LASER_PIN = 7;

// Servo angle limits (adjust for your mount to avoid binding)
constexpr int PAN_MIN = 0;
constexpr int PAN_MAX = 180;
constexpr int TILT_MIN = 0;
constexpr int TILT_MAX = 180;

Servo panServo;
Servo tiltServo;

int panAngle = 90;
int tiltAngle = 90;

void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  moveTo(panServo, panAngle);
  moveTo(tiltServo, tiltAngle);
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) {
    return;
  }

  handleCommand(line);
}

void handleCommand(const String &line) {
  // Commands are uppercase tokens: PAN <deg>, TILT <deg>, LASER <0|1>, HOME
  if (line.startsWith("PAN")) {
    int value = line.substring(3).toInt();
    panAngle = constrain(value, PAN_MIN, PAN_MAX);
    moveTo(panServo, panAngle);
    Serial.println("OK PAN " + String(panAngle));
  } else if (line.startsWith("TILT")) {
    int value = line.substring(4).toInt();
    tiltAngle = constrain(value, TILT_MIN, TILT_MAX);
    moveTo(tiltServo, tiltAngle);
    Serial.println("OK TILT " + String(tiltAngle));
  } else if (line.startsWith("LASER")) {
    int value = line.substring(5).toInt();
    digitalWrite(LASER_PIN, value > 0 ? HIGH : LOW);
    Serial.println("OK LASER " + String(value > 0 ? 1 : 0));
  } else if (line == "HOME") {
    panAngle = 90;
    tiltAngle = 90;
    moveTo(panServo, panAngle);
    moveTo(tiltServo, tiltAngle);
    digitalWrite(LASER_PIN, LOW);
    Serial.println("OK HOME");
  } else {
    Serial.println("ERR");
  }
}

void moveTo(Servo &servo, int angle) {
  servo.write(angle);
}
