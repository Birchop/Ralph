#ifndef LEG_H
#define LEG_H

#include <Adafruit_PWMServoDriver.h>

class Leg {
  public:
    Leg(int servo1, int servo2, int servo3, Adafruit_PWMServoDriver &pwm, float baseOffset, int side, int q1Min, int q1Max); // Constructor

    // Methods
    void moveLeg(float x, float y, float z);
    void homeLeg();

  private:
    // Private variables
    int servo1, servo2, servo3;
    Adafruit_PWMServoDriver &pwm;
    int baseOffset;
    int side;
    int q1Min;
    int q1Max;
    const float l1 = 32.5; // Coxa length
    const float l2 = 80.0; // Femur length
    const float l3 = 190.0; // Tibia length

    float x, y, z;
    float q1, q2, q3;

    int angleToPWM(int minPWM, int maxPWM, int minAngle, int maxAngle, int angle);

    void setServoAngle(int servoNum, int angle);
};

#endif
