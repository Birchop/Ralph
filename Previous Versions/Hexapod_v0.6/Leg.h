#ifndef LEG_H
#define LEG_H

class Leg {
  public:
    Leg(int servo1, int servo2, int servo3);
    
    void move(float x, float y, float z);
    void applyYawToLegPositions(float angle, float &x1, float &y1, float &x2, float &y2, float &x3, float &y3);
    void GaitEngine2(float increment, float stride, float ground, float stance, float strafe, float yaw);
  
  private:
    int servo1, servo2, servo3;
    float q1, q2, q3;
    float l1, l2, l3;
};

#endif
