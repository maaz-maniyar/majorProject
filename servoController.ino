#include <Servo.h>

Servo panServo;
const int SERVO_PIN = 9;

const int MOTOR_A_RPWM = 5;
const int MOTOR_A_LPWM = 6;
const int MOTOR_A_EN   = 7;

const int MOTOR_B_RPWM = 3;
const int MOTOR_B_LPWM = 11;
const int MOTOR_B_EN   = 12;

const int   ALIGN_DEAD_ZONE  = 10;
const int   ALIGN_ZONE       = 25;
const int   MIN_ROTATE_PWM   = 70;
const int   MAX_ROTATE_PWM   = 160;

const float LEFT_FWD_BOOST   = 1.4;

const int   MIN_DRIVE_PWM    = 65;
const int   MAX_DRIVE_PWM    = 180;

const int   STOP_SIZE        = 280;
const int   FAR_SIZE         = 60;

int pan  = 90;
int qrSz = 0;

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

      int error = pan - 90;

      if (abs(error) > ALIGN_ZONE) {
        float ratio   = (float)(abs(error) - ALIGN_ZONE) / (90.0 - ALIGN_ZONE);
        ratio         = constrain(ratio, 0.0, 1.0);
        int rotatePWM = (int)(MIN_ROTATE_PWM + ratio * (MAX_ROTATE_PWM - MIN_ROTATE_PWM));

        if (error > 0) {
          bool leftFwd  = false;
          bool rightFwd = true;
          coastIfSwitching(leftFwd, rightFwd);
          motorLeft(rotatePWM, leftFwd);
          motorRight(rotatePWM, rightFwd);
          lastLeftFwd  = leftFwd;
          lastRightFwd = rightFwd;

        } else {
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
        if (qrSz >= STOP_SIZE) {
          stopMotors();
          return;
        }

        float t      = (float)(qrSz - FAR_SIZE) / (float)(STOP_SIZE - FAR_SIZE);
        t            = constrain(t, 0.0, 1.0);
        int drivePWM = (int)(MAX_DRIVE_PWM - t * (MAX_DRIVE_PWM - MIN_DRIVE_PWM));

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

void coastIfSwitching(bool newLeftFwd, bool newRightFwd) {
  if ((motorsWereOn) &&
      (newLeftFwd != lastLeftFwd || newRightFwd != lastRightFwd)) {
    analogWrite(MOTOR_A_RPWM, 0); analogWrite(MOTOR_A_LPWM, 0);
    analogWrite(MOTOR_B_RPWM, 0); analogWrite(MOTOR_B_LPWM, 0);
    delayMicroseconds(10000);
  }
}

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
