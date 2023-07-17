#ifndef LEG_H
#define LEG_H

#include <Adafruit_PWMServoDriver.h>

class Leg {
  public:
    Leg(String name, int servo1, int servo2, int servo3, Adafruit_PWMServoDriver &pwm, int q1Min, int q1Max, float l1, float l2, float l3, bool usePIDForJoint1, bool usePIDForJoint2, bool usePIDForJoint3, float joint1X, float joint1Y);

    void moveLegGlobal(float x, float y, float z);
    void moveLegGlobal(float x, float y, float z, float currQ1, float currQ2, float currQ3);
    void moveLegLocal(float x, float y, float z, float offsetAngle);
    void moveLegLocal(float x, float y, float z, float offsetAngle, float currQ1, float currQ2, float currQ3);
    void moveLeg(float x, float y, float z, float currQ1, float currQ2, float currQ3);
    void homeLeg();
    float joint1NoPID();
    float joint2NoPID();
    float joint3NoPID();
    float q1NoPID, q2NoPID, q3NoPID;
    float floatMap(float value, float currMin, float currMax, float desiredMin, float desiredMax);
    String getName() const {
      return name;
    }


  private:
    String name;
    int servo1, servo2, servo3;
    Adafruit_PWMServoDriver &pwm;
    int q1Min;
    int q1Max;
    float l1;// = 32.5; // Coxa length
    float l2;// = 80.0; // Femur length
    float l3;// = 190.0; // Tibia V1 length
    bool usePIDForJoint1;
    bool usePIDForJoint2;
    bool usePIDForJoint3;
    float joint1X;
    float joint1Y;
    float offsetAngle;

    float x, y, z;
    float q1, q2, q3;
    float prevQ1, prevQ2, prevQ3;
    float xRel, yRel;

    float maxErr = 1.50f;
    float Kp = 0.1, Ki = 0.05, Kd = 0.01;
    // Error terms
    float e1, e2, e3;
    // Integral of error
    float ie1 = 0, ie2 = 0, ie3 = 0;
    // Previous error
    float pe1 = 0, pe2 = 0, pe3 = 0;
    // PID output
    float pid1, pid2, pid3;

    int angleToPWM(int minPWM, int maxPWM, int minAngle, int maxAngle, int angle);

    void setServoAngle(int servoNum, int angle);
};

#endif
