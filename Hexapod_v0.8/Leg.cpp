#include "Leg.h"

Leg::Leg(int servo1, int servo2, int servo3, Adafruit_PWMServoDriver &pwm, float baseOffset, int side, int middle, int q1Min, int q1Max)
  : servo1(servo1), servo2(servo2), servo3(servo3), pwm(pwm), baseOffset(baseOffset), side(side), q1Min(q1Min), q1Max(q1Max) {
}

void Leg::moveLeg(float x, float y, float z) {

  float rotatedX;
  float rotatedY;
  // rotate the input coordinates by the leg angle
  if (baseOffset != 0) {
    float angleRad = (baseOffset) * PI / 180;
    rotatedX = x * cos(angleRad) - y * sin(angleRad);
    rotatedY = x * sin(angleRad) + y * cos(angleRad);
  } else {
    rotatedX = x;
    rotatedY = y;
  }
  q1 = atan2(rotatedY, rotatedX);
  if (isnan(q1)) {q1 = prevQ1;} else {q1 = q1 * 180 / PI; q1 = constrain(q1, q1Min, q1Max);}
  float L1 = sqrt(rotatedX * rotatedX + rotatedY * rotatedY) - l1;
  float L = sqrt(L1 * L1 + z * z);
  float alpha = acos((l2 * l2 + L * L - l3 * l3) / (2 * l2 * L));
  float beta = atan2(z, L1);
  q2 = alpha + beta;
  if (isnan(q2)) {q2 = prevQ2;} else {q2 = q2 * 180 / PI; q2 = map(q2, -90, 90, 180, 0);}
  
  q3 = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3));
  if (isnan(q3)) {q3 = prevQ3;} else {q3 = q3 * 180 / PI; q3 = map(q3, -90, 90, 0, 180); q3 = q3 - 120;}

  float q1r = map(q1, 0, 180, 180, 0);
  float q2r = map(q2, 0, 180, 180, 0);
  float q3r = map(q3, 0, 180, 180, 0);

  if (!side && !middle) {
    setServoAngle(servo1, q1r);
    setServoAngle(servo2, q2);
    setServoAngle(servo3, q3);
  } else if (side && !middle) {
    setServoAngle(servo1, q1);
    setServoAngle(servo2, q2r);
    setServoAngle(servo3, q3r);
  } else if (!side && middle) {
    setServoAngle(servo1, q1);
    setServoAngle(servo2, q2);
    setServoAngle(servo3, q3);
  } else if (side && middle) {
    setServoAngle(servo1, q1r);
    setServoAngle(servo2, q2r);
    setServoAngle(servo3, q3r);
  } else {
    Serial.println("Failed to initialise Leg object");
    return;
  }
  prevQ1 = q1;
  prevQ2 = q2;
  prevQ3 = q3;
}

void Leg::homeLeg() {
  if (!side) {
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 150);
    delay(50);
  } else {
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 30);
    delay(50);
  }
}

  void Leg::setServoAngle(int servoNum, int angle) {
    int minPWM = 500;
    int maxPWM = 2500;
    int minAngle = 0;
    int maxAngle = 180;
    int pwmValue = angleToPWM(minPWM, maxPWM, minAngle, maxAngle, angle);
    pwm.writeMicroseconds(servoNum, pwmValue);
  }

  int Leg::angleToPWM(int minPWM, int maxPWM, int minAngle, int maxAngle, int angle) {
    int pwmValue = map(angle, minAngle, maxAngle, minPWM, maxPWM);
    return constrain(pwmValue, minPWM, maxPWM);
  }
