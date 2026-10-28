/* Two STEP/DIR drivers. Install AccelStepper in Arduino Library Manager.
   This REPLACES original repository firmware; protocol is APRILTAG_V1.
   Center mount BEFORE resetting board. No physical homing or encoder.
   Configure driver hardware for 1/16 microstepping. Check README first. */
#include <AccelStepper.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const byte PAN_STEP = 8, PAN_DIR = 7, TILT_STEP = 6, TILT_DIR = 5, ENABLE = 12;
const float MICROSTEPS = 16.0;
const float PAN_STEPS_DEG = 200.0 * MICROSTEPS * (144.0 / 17.0) / 360.0;
const float TILT_STEPS_DEG = 200.0 * MICROSTEPS * (64.0 / 21.0) / 360.0;
// Conservative STARTING limits, not verified machine collision limits.
const float PAN_LIMIT_DEG = 45, TILT_LIMIT_DEG = 25;
const float MAX_RATE = 3.0, ACCEL_DEG_S2 = 10.0;
const unsigned long WATCHDOG_MS = 350;
AccelStepper pan(AccelStepper::DRIVER, PAN_STEP, PAN_DIR);
AccelStepper tilt(AccelStepper::DRIVER, TILT_STEP, TILT_DIR);
float desiredPan = 0, desiredTilt = 0, actualPan = 0, actualTilt = 0;
bool armed = false;
unsigned long lastCommand = 0, lastRamp = 0;
char line[64];
byte length = 0;
bool overflow = false;

void haltMotion() {
  desiredPan = desiredTilt = actualPan = actualTilt = 0;
  pan.setSpeed(0); tilt.setSpeed(0);
  armed = false;  // Keep enable LOW if previously enabled, to hold tilt.
}

bool parseNumber(char *s, float &value) {
  if (!s) return false;
  char *end;
  value = strtod(s, &end);
  return end != s && *end == '\0' && isfinite(value);
}

void handleCommand(char *s) {
  if (!strcmp(s, "ARM")) {
    haltMotion();
    digitalWrite(ENABLE, LOW);
    armed = true;
    lastCommand = millis();
    Serial.println(F("ARMED"));
    return;
  }
  if (!strcmp(s, "STOP")) { haltMotion(); return; }
  char *command = strtok(s, " ");
  if (!command || strcmp(command, "V")) { haltMotion(); return; }
  float p, t;
  char *pText = strtok(NULL, " ");
  char *tText = strtok(NULL, " ");
  char *extra = strtok(NULL, " ");
  if (extra || !parseNumber(pText, p) || !parseNumber(tText, t) ||
      fabs(p) > MAX_RATE || fabs(t) > MAX_RATE) { haltMotion(); return; }
  if (!armed) return;
  lastCommand = millis();
  desiredPan = p; desiredTilt = t;
}

float ramp(float current, float target, float delta) {
  if (target > current) return min(target, current + delta);
  return max(target, current - delta);
}

void setup() {
  pinMode(ENABLE, OUTPUT);
  digitalWrite(ENABLE, HIGH); // Outputs disabled until explicit ARM.
  pan.setMaxSpeed(1500); tilt.setMaxSpeed(1500);
  pan.setMinPulseWidth(3); tilt.setMinPulseWidth(3);
  pan.setCurrentPosition(0); tilt.setCurrentPosition(0);
  Serial.begin(115200);
  Serial.println(F("READY APRILTAG_V1"));
  lastRamp = micros();
}

void loop() {
  // Bounded parsing so serial traffic cannot indefinitely starve stepping.
  byte budget = 32;
  while (Serial.available() && budget--) {
    char c = Serial.read();
    if (c == '\n') {
      line[length] = 0;
      if (!overflow) handleCommand(line); else haltMotion();
      length = 0; overflow = false;
    } else if (c != '\r') {
      if (length < sizeof(line) - 1) line[length++] = c;
      else overflow = true;
    }
  }
  if (armed && millis() - lastCommand > WATCHDOG_MS) haltMotion();
  unsigned long now = micros();
  if (now - lastRamp >= 2000) {
    float dt = min((now - lastRamp) / 1000000.0f, .02f);
    lastRamp = now;
    actualPan = ramp(actualPan, desiredPan, ACCEL_DEG_S2 * dt);
    actualTilt = ramp(actualTilt, desiredTilt, ACCEL_DEG_S2 * dt);
    pan.setSpeed(actualPan * PAN_STEPS_DEG);
    tilt.setSpeed(actualTilt * TILT_STEPS_DEG);
  }
  long p = pan.currentPosition(), t = tilt.currentPosition();
  if ((p >= (long)(PAN_LIMIT_DEG * PAN_STEPS_DEG) && actualPan > 0) ||
      (p <= -(long)(PAN_LIMIT_DEG * PAN_STEPS_DEG) && actualPan < 0)) {
    actualPan = 0; pan.setSpeed(0);
  }
  if ((t >= (long)(TILT_LIMIT_DEG * TILT_STEPS_DEG) && actualTilt > 0) ||
      (t <= -(long)(TILT_LIMIT_DEG * TILT_STEPS_DEG) && actualTilt < 0)) {
    actualTilt = 0; tilt.setSpeed(0);
  }
  pan.runSpeed(); tilt.runSpeed();
}
