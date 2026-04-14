// ─────────────────────────────────────────────────────────────
//  QR Tracker Robot — Two-Phase: Align → Follow
//  Serial protocol: "pan,size\n"
//    pan  : 0–180   servo angle (90 = centered)
//    size : 0–1000  QR bounding box width in pixels (0 = not detected)
//
//  Phase logic (decided here on Arduino):
//    ALIGN  → pan is off-center → rotate in place to face QR
//    FOLLOW → pan is centered   → drive forward, speed ∝ distance
//    STOP   → size >= STOP_SIZE → QR is close enough, stop
// ─────────────────────────────────────────────────────────────

#include <Servo.h>

// ── Servo ─────────────────────────────────────────────────────
Servo panServo;
const int SERVO_PIN = 9;

// ── Motor A — Left wheel ──────────────────────────────────────
const int MOTOR_A_RPWM = 5;
const int MOTOR_A_LPWM = 6;
const int MOTOR_A_EN   = 7;

// ── Motor B — Right wheel (RPWM/LPWM physically swapped) ─────
// RPWM on pin 3 (Timer2) — avoids Timer1 conflict with Servo on pin 9
const int MOTOR_B_RPWM = 3;
const int MOTOR_B_LPWM = 11;
const int MOTOR_B_EN   = 12;

// ── Tuning ────────────────────────────────────────────────────

// Alignment
const int   ALIGN_DEAD_ZONE  = 10;   // ±deg from 90 — no rotation needed
const int   ALIGN_ZONE       = 25;   // ±deg — aligned enough to start driving
const int   MIN_ROTATE_PWM   = 70;   // min PWM to overcome stiction while rotating
const int   MAX_ROTATE_PWM   = 160;  // max in-place rotation speed

// Left turn compensation — left wheel forward uses pin 5 (Timer0).
// Boost it slightly so left pivot matches the right pivot in sharpness.
// Increase this (e.g. 1.2, 1.3) if left turn still feels gradual.
const float LEFT_FWD_BOOST   = 1.4;

// Follow
const int   MIN_DRIVE_PWM    = 65;   // min PWM for forward motion
const int   MAX_DRIVE_PWM    = 180;  // max forward speed

// QR size thresholds (bounding box width in pixels, frame = 640px wide)
const int   STOP_SIZE        = 280;  // QR this wide → robot is close enough, stop
const int   FAR_SIZE         = 60;   // QR this small → full speed

// ─────────────────────────────────────────────────────────────

int pan  = 90;
int qrSz = 0;

// Track last direction to coast only when actually switching
bool lastLeftFwd  = true;
bool lastRightFwd = true;
bool motorsWereOn = false;

void setup() {
  Serial.begin(9600);

  panServo.attach(SERVO_PIN);
  panServo.write(90);

  pinMode(MOTOR_A_RPWM, OUTPUT); pinMode(MOTOR_A_LPWM, OUTPUT); pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_RPWM, OUTPUT); pinMode(MOTOR_B_LPWM, OUTPUT); pinMode(MOTOR_B_EN, OUTPUT);

  digitalWrite(MOTOR_A_EN, HIGH);
  digitalWrite(MOTOR_B_EN, HIGH);

  stopMotors();
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();

    int commaIdx = data.indexOf(',');
    if (commaIdx >= 0) {
      pan  = constrain(data.substring(0, commaIdx).toInt(), 0, 180);
      qrSz = constrain(data.substring(commaIdx + 1).toInt(), 0, 1000);

      panServo.write(pan);

      if (qrSz == 0) {
        stopMotors();
        return;
      }

      int error = pan - 90;   // negative = QR left, positive = QR right

      if (abs(error) > ALIGN_ZONE) {
        // ── PHASE 1: ALIGN ──────────────────────────────────
        float ratio   = (float)(abs(error) - ALIGN_ZONE) / (90.0 - ALIGN_ZONE);
        ratio         = constrain(ratio, 0.0, 1.0);
        int rotatePWM = (int)(MIN_ROTATE_PWM + ratio * (MAX_ROTATE_PWM - MIN_ROTATE_PWM));

        if (error > 0) {
          // QR is RIGHT → rotate body clockwise
          // Left wheel REVERSE, Right wheel FORWARD
          bool leftFwd  = false;
          bool rightFwd = true;
          coastIfSwitching(leftFwd, rightFwd);
          motorLeft(rotatePWM, leftFwd);
          motorRight(rotatePWM, rightFwd);
          lastLeftFwd  = leftFwd;
          lastRightFwd = rightFwd;

        } else {
          // QR is LEFT → rotate body counter-clockwise
          // Left wheel FORWARD (boosted), Right wheel REVERSE
          bool leftFwd  = true;
          bool rightFwd = false;
          coastIfSwitching(leftFwd, rightFwd);
          int boostedPWM = constrain((int)(rotatePWM * LEFT_FWD_BOOST), 0, MAX_ROTATE_PWM);
          motorLeft(boostedPWM, leftFwd);
          motorRight(rotatePWM, rightFwd);
          lastLeftFwd  = leftFwd;
          lastRightFwd = rightFwd;
        }

        motorsWereOn = true;

      } else {
        // ── PHASE 2: FOLLOW ─────────────────────────────────
        if (qrSz >= STOP_SIZE) {
          stopMotors();
          return;
        }

        float t      = (float)(qrSz - FAR_SIZE) / (float)(STOP_SIZE - FAR_SIZE);
        t            = constrain(t, 0.0, 1.0);
        int drivePWM = (int)(MAX_DRIVE_PWM - t * (MAX_DRIVE_PWM - MIN_DRIVE_PWM));

        // Both forward — coast only if previously turning
        if (motorsWereOn && (!lastLeftFwd || !lastRightFwd)) {
          coastMotors();
        }
        motorLeft(drivePWM,  true);
        motorRight(drivePWM, true);
        lastLeftFwd  = true;
        lastRightFwd = true;
        motorsWereOn = true;
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  Coast only when a wheel is actually reversing direction
//  Uses delayMicroseconds — does NOT block Timer0 like delay()
// ─────────────────────────────────────────────────────────────
void coastIfSwitching(bool newLeftFwd, bool newRightFwd) {
  if ((motorsWereOn) &&
      (newLeftFwd != lastLeftFwd || newRightFwd != lastRightFwd)) {
    analogWrite(MOTOR_A_RPWM, 0); analogWrite(MOTOR_A_LPWM, 0);
    analogWrite(MOTOR_B_RPWM, 0); analogWrite(MOTOR_B_LPWM, 0);
    delayMicroseconds(10000);   // 10ms — no Timer0 interference
  }
}

// ─────────────────────────────────────────────────────────────
//  Motor helpers
// ─────────────────────────────────────────────────────────────
void motorLeft(int pwm, bool forward) {
  if (forward) {
    analogWrite(MOTOR_A_RPWM, pwm);
    analogWrite(MOTOR_A_LPWM, 0);
  } else {
    analogWrite(MOTOR_A_RPWM, 0);
    analogWrite(MOTOR_A_LPWM, pwm);
  }
}

void motorRight(int pwm, bool forward) {
  // Wires physically swapped on driver — invert logic to compensate
  if (forward) {
    analogWrite(MOTOR_B_RPWM, 0);
    analogWrite(MOTOR_B_LPWM, pwm);
  } else {
    analogWrite(MOTOR_B_RPWM, pwm);
    analogWrite(MOTOR_B_LPWM, 0);
  }
}

void stopMotors() {
  analogWrite(MOTOR_A_RPWM, 0); analogWrite(MOTOR_A_LPWM, 0);
  analogWrite(MOTOR_B_RPWM, 0); analogWrite(MOTOR_B_LPWM, 0);
  motorsWereOn = false;
}

void coastMotors() {
  analogWrite(MOTOR_A_RPWM, 0); analogWrite(MOTOR_A_LPWM, 0);
  analogWrite(MOTOR_B_RPWM, 0); analogWrite(MOTOR_B_LPWM, 0);
  delayMicroseconds(10000);
}
