#include "Leg.h"

/**
    Leg constructor:
    servo1: Servo pin for Joint 1
    servo2: Servo pin for Joint 2
    servo3: Servo pin for Joint 3
    pwm: PWM driver object
    baseOffset: Joint 1 offset from middle
    side: Leg side (0 = Left, 1 = Right)
    middle: Leg is middle leg or not (0 = False, 1 = True)
    q1Min: Minimum limit for Joint 1 (degrees)
    q1Max: Maximum limit for Joint 1 (degrees)
    l1: Length of Link 1 (same unit as X/Y/Z system)
    l2: Length of Link 2 (same unit as X/Y/Z system)
    l3: Length of Link 3 (same unit as X/Y/Z system)
    usePIDForJoint1: Enable PID for Joint 1 (True/False)
    usePIDForJoint2: Enable PID for Joint 2 (True/False)
    usePIDForJoint3: Enable PID for Joint 3 (True/False)
*/

/*
 * TODO
 * update constructor in main sketch, create global coord version of gaitEngine
 * test single leg
 * Test all legs
 */

Leg::Leg(String name, int servo1, int servo2, int servo3, Adafruit_PWMServoDriver &pwm, int q1Min, int q1Max, float l1, float l2, float l3, bool usePIDForJoint1, bool usePIDForJoint2, bool usePIDForJoint3, float joint1X, float joint1Y)
  : name(name), servo1(servo1), servo2(servo2), servo3(servo3), pwm(pwm), q1Min(q1Min), q1Max(q1Max), l1(l1), l2(l2), l3(l3), usePIDForJoint1(usePIDForJoint1), usePIDForJoint2(usePIDForJoint2), usePIDForJoint3(usePIDForJoint3), joint1X(joint1X), joint1Y(joint1Y) {
}

void Leg::moveLegGlobal(float x, float y, float z, float currQ1, float currQ2, float currQ3) { //With PID
  xRel = x - joint1X;
  yRel = y - joint1Y;
  offsetAngle = atan2(joint1Y, joint1X) * 180 / PI;
  //offsetAngle += 90;
  //Serial.print();
  moveLegLocal(xRel, yRel, z, offsetAngle, prevQ1, prevQ2, prevQ3);
}

void Leg::moveLegGlobal(float x, float y, float z) { //No PID
  xRel = x - joint1X;
  yRel = y - joint1Y;
  offsetAngle = atan2(joint1Y, joint1X) * 180 / PI;
  //offsetAngle += 90;
  //Serial.print();
  moveLegLocal(xRel, yRel, z, offsetAngle);
}

void Leg::moveLegLocal(float x, float y, float z, float offsetAngle, float currQ1, float currQ2, float currQ3) { //With PID
  float q1 = atan2(y, x);
  if (isnan(q1)) {
    q1 = 90;
    Serial.print(" Q1 NaN  ");
  } else {
    q1 = q1 * 180 / PI;
    q1 = floatMap(q1, offsetAngle - 90, offsetAngle + 90, 0, 180);
    q1 = constrain(q1, q1Min, q1Max);
    //Serial.print(name + "| Q1: " + String(q1));
  }

  float L1 = sqrt(x * x + y * y) - l1;
  float L = sqrt(L1 * L1 + z * z);
  float alpha = acos((l2 * l2 + L * L - l3 * l3) / (2 * l2 * L));
  float beta = atan2(z, L1);
  float q2 = alpha + beta;
  if (isnan(q2)) {
    q2 = 90;
    //Serial.print(" Q2 NaN  ");
  } else {
    q2 = q2 * 180 / PI;
    q2 = floatMap(q2, -90, 90, 180, 0);
    //Serial.print(" Q2: " + String(q2));
  }

  float q3 = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3));
  if (isnan(q3)) {
    q3 = 90;
    //Serial.print(" Q3 NaN  ");
  } else {
    q3 = q3 * 180 / PI;
    q3 = floatMap(q3, -90, 90, 0, 180);
    q3 = q3 - 120;
    //Serial.print(" Q3: " + String(q3));
  }
  //Serial.println(" XYZ: " + String(x) + ", " + String(y) + ", " + String(z));

  q1NoPID = q1;
  q2NoPID = q2;
  q3NoPID = q3;

  if (usePIDForJoint1) {
    e1 = q1 - currQ1;
    if (e1 >= maxErr) {
      pid1 = Kp * e1 + Ki * ie1 + Kd * (e1 - pe1);
      ie1 += e1;
      pe1 = e1;
      q1 += pid1;
    }
  }

  if (usePIDForJoint2) {
    e2 = q2 - currQ2;
    if (e2 >= maxErr) {
      pid2 = Kp * e2 + Ki * ie2 + Kd * (e2 - pe2);
      ie2 += e2;
      pe2 = e2;
      q2 += pid2;
    }
  }

  if (usePIDForJoint3) {
    e3 = q3 - currQ3;
    if (e3 >= maxErr) {
      pid3 = Kp * e3 + Ki * ie3 + Kd * (e3 - pe3);
      ie3 += e3;
      pe3 = e3;
      q3 += pid3;
    }
  }


  float q1r = floatMap(q1, 0, 180, 180, 0);
  float q2r = floatMap(q2, 0, 180, 180, 0);
  float q3r = floatMap(q3, 0, 180, 180, 0);
  prevQ1 = q1;
  prevQ2 = q2;
  prevQ3 = q3;
  if (joint1Y < 0) {
    setServoAngle(servo1, q1);
    setServoAngle(servo2, q2);
    setServoAngle(servo3, q3);
  } else {
    setServoAngle(servo1, q1);
    setServoAngle(servo2, q2r);
    setServoAngle(servo3, q3r);
  }
}

void Leg::moveLegLocal(float x, float y, float z, float offsetAngle) { //No PID
  float q1 = atan2(y, x);
  if (isnan(q1)) {
    q1 = prevQ1;
    Serial.print(" Q1 NaN  ");
  } else {
    q1 = q1 * 180 / PI;
    q1 = floatMap(q1, offsetAngle - 90, offsetAngle + 90, 0, 180);
    q1 = constrain(q1, q1Min, q1Max);
    //Serial.print(name + "| Q1: " + String(q1));
  }

  float L1 = sqrt(x * x + y * y) - l1;
  float L = sqrt(L1 * L1 + z * z);
  float alpha = acos((l2 * l2 + L * L - l3 * l3) / (2 * l2 * L));
  float beta = atan2(z, L1);
  float q2 = alpha + beta;
  if (isnan(q2)) {
    q2 = prevQ2;
    Serial.print(" Q2 NaN  ");
  } else {
    q2 = q2 * 180 / PI;
    q2 = floatMap(q2, -90, 90, 180, 0);
    //Serial.print(" Q2: " + String(q2));
  }

  float q3 = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3));
  if (isnan(q3)) {
    q3 = prevQ3;
    Serial.print(" Q3 NaN  ");
  } else {
    q3 = q3 * 180 / PI;
    q3 = floatMap(q3, -90, 90, 0, 180);
    q3 = q3 - 120;
    //Serial.print(" Q3: " + String(q3));
  }
  //Serial.println(" XYZ: " + String(x) + ", " + String(y) + ", " + String(z));
  float q1r = floatMap(q1, 0, 180, 180, 0);
  float q2r = floatMap(q2, 0, 180, 180, 0);
  float q3r = floatMap(q3, 0, 180, 180, 0);
  prevQ1 = q1;
  prevQ2 = q2;
  prevQ3 = q3;
  if (name == "FL") {
    setServoAngle(servo1, q1r);
    setServoAngle(servo2, q2);
    setServoAngle(servo3, q3);
  } else if (name == "ML") {
    setServoAngle(servo1, q1r);
    setServoAngle(servo2, q2);
    setServoAngle(servo3, q3);
  }  else if (name == "RL") {
    setServoAngle(servo1, q1);
    setServoAngle(servo2, q2);
    setServoAngle(servo3, q3);
  }  else if (name == "FR") {
    setServoAngle(servo1, q1);
    setServoAngle(servo2, q2r);
    setServoAngle(servo3, q3r);
  } else if (name == "MR") {
    setServoAngle(servo1, q1r);
    setServoAngle(servo2, q2r);
    setServoAngle(servo3, q3r);
  } else if (name == "RR") {
    setServoAngle(servo1, q1r);
    setServoAngle(servo2, q2r);
    setServoAngle(servo3, q3r);
  }
}

void Leg::homeLeg() {
  //delays used to avoid the legs flipping the bot, stand up routine to be implemented once feedback is available
  /*if (joint1Y < 0 && joint1X > 0) { //front left?
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 150);
    delay(50);
  } else if (joint1Y < 0 && joint1X == 0) { //middle left
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 150);
    delay(50);
  } else if (joint1Y < 0 && joint1X < 0) { //rear left
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 30);
    delay(50);
  } else if (joint1Y > 0 && joint1X > 0) { //front right
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 30);
    delay(50);
  } else if (joint1Y > 0 && joint1X == 0) { //middle right
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 30);
    delay(50);
  } else {
    setServoAngle(servo1, 90);
    delay(50);
    setServoAngle(servo2, 90);
    delay(50);
    setServoAngle(servo3, 30);
    delay(50);
  }*/
  if (name == "FL" || name == "ML" || name == "RL") {
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

float Leg::joint1NoPID() {
  return q1NoPID;
}

float Leg::joint2NoPID() {
  return q2NoPID;
}

float Leg::joint3NoPID() {
  return q3NoPID;
}

float Leg::floatMap(float value, float currMin, float currMax, float desiredMin, float desiredMax) {
  return (value - currMin) * (desiredMax - desiredMin) / (currMax - currMin) + desiredMin;
}
