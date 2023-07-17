#ifndef GAITENGINE_H
#define GAITENGINE_H

#include "Leg.h"
#include <math.h>

enum Gait {
  tri,
  trot,
  ripple,
  wave
};

class gaitEngine {
public:
  gaitEngine(Leg* allLegs[]);
  void move(float stride, float strafe, float yaw);
  void move(float stride, float strafe, float yaw, int increment);
  void move(float stride, float strafe, float yaw, float ground, int increment);
  void move(Gait gait, float stride, float strafe, float yaw);
  void move(Gait gait, float stride, float strafe, float yaw, float stanceWidth, float adduction, float ground, float clearance, int increment);
  void move(Gait gait, float stride, float strafe, float yaw, float ground);
  void setAdduction(float adduction);
  void setIncrement(int increment);
  void setStanceWidth(float stanceWidth);
  void setGround(float ground);
  void setClearance(float clearance);
  void setGait(Gait newGait);
  void shiftRightLeg(Leg** legs, int size, int shiftAmount);
  void shiftRight(float* coords, int size, int shiftAmount);
  void resetLegArray();
  bool rangeCheck(float value, float range);
  void shiftLeftLeg(Leg** legs, int size, int shiftAmount);
  void shiftLeft(float* arr, int size, int shiftAmount);

private:
  float stride = 0.0f, strafe = 0.0f, yaw = 0.0f, stanceWidth = 0.0f, ground = -200.0f, clearance = 45.0f, adduction = 25.0f;
  int increment = 50;
  int j = 0;

  Leg** allLegs;
  Leg* originalLegs[6];
  Leg* waveLegs[6];
  

  Gait gait;
  Gait previousGait;

  /*
     Input variables:
     stride      |
     strafe      |
     yaw         |
     stanceWidth |  additional horizontal distance from body from default, affects y axis
     adduction   |  Front and rear default swing from centerline in standing position, start pointing out horizontally and rotate to face forward (front legs) or back (rear legs)
     ground      |  desired distance from bottom of body to ground, how high to lift the bot
     clearance   |  distance to lift end effector off ground
     increment   |  points to calculate per step
  */

  //* FL, FR, ML, MR, RL, RR
};

#endif // GAITENGINE_H
