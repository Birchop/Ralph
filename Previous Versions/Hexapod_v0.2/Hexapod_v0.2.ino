 #include <math.h>
#include <Arduino.h>
#define ENABLE_EASE_CUBIC
#define ENABLE_EASE_QUADRATIC
#include "ServoEasing.hpp"


//# of Legs
int numLegs = 6;

//# of joints per leg
int numJoints = 3;

int servoSpeed = 30;

ServoEasing legFR[3];
ServoEasing legFL[3];
ServoEasing legML[3];
ServoEasing legMR[3];
ServoEasing legRR[3];
ServoEasing legRL[3];


float coxa = 54.282;
float femur = 140.00;
float tibia = 242.58;
float L1_fk;
float L2_fk;
float theta1;
float theta2;
float x;
float y;
float z;
float Lc;

float l1 = 54.282;  // length of link 1
float l2 = 140;  // length of link 2
float l3 = 242.58;  // length of link 3

float i = -1.57;
float j = -2.44;
float increment = 0.01f;
const int resolution = 100; // Number of points to generate

enum Leg {frontLeft, frontRight, middleLeft, middleRight, rearLeft, rearRight, all, pair1, pair2, pair3, tri1, tri2};

bool onTheRun = false;
bool interrupt = false;
float strideLength = 40;
float strideWidth = 0;
float strideLift = 0;


void setup() {
  Serial.begin(115200);
  //Assign pins to servos

  int i1 = 3;
  int i2 = 6;
  int i3 = 9;
  int i4 = 12;
  int i5 = 15;
  for (int i = 0; i < numJoints; i++) {
    legFL[i].attach(i, 90,500, 2500);
    legFL[i].setSpeed(servoSpeed);
    legFR[i].attach(i1, 90, 500, 2500);
    legFR[i].setSpeed(servoSpeed);
    legML[i].attach(i2, 90,500, 2500);
    legML[i].setSpeed(servoSpeed);
    legMR[i].attach(i3, 90, 500, 2500);
    legMR[i].setSpeed(servoSpeed);
    legRL[i].attach(i4, 90, 500, 2500);
    legRL[i].setSpeed(servoSpeed);
    legRR[i].attach(i5, 90, 500, 2500);
    legRR[i].setSpeed(servoSpeed);
    i1++;
    i2++;
    i3++;
    i4++;
    i5++;
    delay(250);
    Serial.println("Attaching joints");
    }
  
    setSpeedForAllServos(20);
    
      legFL[0].setEaseTo(90);
      legFL[1].setEaseTo(90);
      legFL[2].setEaseTo(40);

      legFR[0].setEaseTo(90);
      legFR[1].setEaseTo(90);
      legFR[2].setEaseTo(140);

      legML[0].setEaseTo(90);
      legML[1].setEaseTo(90);
      legML[2].setEaseTo(40);

      legMR[0].setEaseTo(90);
      legMR[1].setEaseTo(90);
      legMR[2].setEaseTo(140);

      legRL[0].setEaseTo(90);
      legRL[1].setEaseTo(90);
      legRL[2].setEaseTo(40);

      legRR[0].setEaseTo(90);
      legRR[1].setEaseTo(90);
      legRR[2].setEaseTo(140);

      synchronizeAllServosStartAndWaitForAllServosToStop();
      delay(2500);

      startPositions();
}


void loop() {
  /*for (int i = 0; i<5; i++){
    triWalk(40,1,0,100);
  }*/
  for (int i = 0; i<5; i++){
    biRipple(40,1,0,100);
  }
}



void inverseKinematics(Leg legNumber, float x, float y, float z, float velocity, bool interrupt) {
  float q1 = atan2(y, x);
  float coxa = l1 * cos(q1);  // adjusted length of coxa
  float d = sqrt((x * x) + (y * y)) - coxa;
  float lc = sqrt((d * d) + ((z - l1) * (z - l1)));
  float alpha = acos(((l2 * l2) + (lc * lc) - (l3 * l3)) / (2 * l2 * lc));
  float beta = atan2(z - l1, d);

  float q2 = PI - alpha - beta;
  float q3 = acos(((l2 * l2) + (l3 * l3) - (lc * lc)) / (2 * l2 * l3));

  q1 = q1 * 180 / PI;  // converting to degrees
  q2 = q2 * 180 / PI;
  q3 = q3 * 180 / PI;

if (isnan(q1)) {
  Serial.println("q1 nan - skipping");
  return;
}
if (isnan(q2)) {
  Serial.println("q2 nan - skipping");
  return;
}
if (isnan(q3)) {
  Serial.println("q3 nan - skipping");
  return;
}

  Serial.println("-------Inverse-------");
  Serial.print("q1: ");
  Serial.print(q1);
  Serial.print(" q2: ");
  Serial.print(q2);
  Serial.print(" q3: ");
  Serial.println(q3);
/*
if (q1 >= 90) {
  q1 = map(q1, 125, 180, 145,90);
} 
if (q1 <= 0) {
  q1 = map(q1, -180, -125, 35,90);
}
*/
if (q2 >= 90) {
  q2 = map(q2, 90, 180, 0,90);
} 
if (q2 <= 0) {
  q2 = map(q2, -180, -90, 90,180);
}

if (q3 >= 40 && q3 <= 130) {
  q3 = map(q3, 40, 130, 180, 90);
}
 
if (q3 >= 130 && q3 <= 180) {
  q3 = map(q3, 130, 180, 90,40);
}
if (q3 <= 0)
{
  q3 = map(q3, -180, -140, 40,0);
}

  moveLeg(legNumber, q1, q2, q3, velocity, interrupt);
/*if (interrupt == true) {
moveLeg2(legNumber, q1, q2, q3, velocity);
}
else {
  moveLeg(legNumber, q1, q2, q3, velocity);
}*/
}
/*
void moveLeg2(Leg legNumber, float q1, float q2, float q3, float velocity) { //tri only, testing interrupts
  float q1l = map(q1, 0, 180, 180, 0);
  float q2l = map(q2, 0, 180, 180, 0);
  float q3l = map(q3, 0, 180, 180, 0);
  switch (legNumber) {
    case 0: 
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

    case 1: 
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 2: 
      setSpeedForAllServos(velocity);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 3:  
      setSpeedForAllServos(velocity);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 4:  
      setSpeedForAllServos(velocity);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 5:  
      setSpeedForAllServos(velocity);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 6: //all  
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 7:  //pair1
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 8:  //pair2
      setSpeedForAllServos(velocity);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 9:  //pair3
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 10:  //tri1
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     case 11:  //tri2
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      setEaseToForAllServosSynchronizeAndStartInterrupt(30);
     break;

     default:
     return;
  }
}*/
void moveLeg(Leg legNumber, float q1, float q2, float q3, float velocity, bool interrupt) {
  //q1 = map(q1, 0, 180, 0, 180);
  //q2 = map(q2, 0, 180, 0, 180);
  //q3 = map(q3, 0, 180, 0, 180);
  float q1l = map(q1, 0, 180, 180, 0);
  float q2l = map(q2, 0, 180, 180, 0);
  float q3l = map(q3, 0, 180, 180, 0);
  
  switch (legNumber) {
    case 0: 
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

    case 1: 
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 2: 
      setSpeedForAllServos(velocity);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 3:  
      setSpeedForAllServos(velocity);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 4:  
      setSpeedForAllServos(velocity);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 5:  
      setSpeedForAllServos(velocity);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 6: //all  
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 7:  //pair1
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 8:  //pair2
      setSpeedForAllServos(velocity);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 9:  //pair3
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 10:  //tri1
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      if (interrupt == true) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     case 11:  //tri2
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      if (interrupt == true) setEaseToForAllServosSynchronizeAndStartInterrupt(30);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
     break;

     default:
     return;
  }
}

void startPositions() {
  //Start position -- needs start conditions, make void
  inverseKinematics(tri1,-20,75,-95,60, true);
  inverseKinematics(tri2,20,75,-95,60, false);
  delay(500);
}

void triWalk(float strideLength, float strideWidth, float strideLift, float velocity) {
  
  strideLift = -95 - strideLift;
  strideWidth = 75 + strideWidth;

  float back = (strideLength/2) - strideLength;
  float mid = (strideLength/2) - (strideLength/2) + 5;
  float ward = strideLength/2;
  
  //Step 1;
  inverseKinematics(tri1, mid, strideWidth, strideLift + 45, velocity, true);   //lift tri 1
  inverseKinematics(tri2, back, strideWidth, strideLift, velocity, false); //move tri 2 - body moving
  inverseKinematics(tri1, ward, strideWidth, strideLift, velocity*1.5, false);   //drop tri 1
  

  //Step 2;
  inverseKinematics(tri2, mid, strideWidth, strideLift + 45, velocity, true); //lift tri 2
  inverseKinematics(tri1, back, strideWidth, strideLift, velocity, false); //move tri 1 - body moving
  inverseKinematics(tri2, ward, strideWidth, strideLift, velocity*1.5, false); //drop tri 2
}

void biRipple(float strideLength, float strideWidth, float strideLift, float velocity) {
  strideLift = -95 - strideLift;
  strideWidth = 75 + strideWidth;
  float returnVelocity = velocity/3;
  float back = (strideLength/2) - strideLength;
  float mid = (strideLength/2) - (strideLength/2) + 5;
  float ward = strideLength/2;

  inverseKinematics(pair1, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 1
  inverseKinematics(pair2, back, strideWidth, strideLift, returnVelocity, true); //move pair 2 - body moving
  inverseKinematics(pair3, mid, strideWidth, strideLift, returnVelocity, false); //move pair 3 - body moving
  inverseKinematics(pair1, ward, strideWidth, strideLift, velocity, false);   //drop pair 1
  
  inverseKinematics(pair2, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 2
  inverseKinematics(pair3, back, strideWidth, strideLift, returnVelocity, true); //move pair 3 - body moving
  inverseKinematics(pair1, mid, strideWidth, strideLift, returnVelocity, false); //move pair 1 - body moving
  inverseKinematics(pair2, ward, strideWidth, strideLift, velocity, false);   //drop pair 2
  
  inverseKinematics(pair3, mid, strideWidth, strideLift + 45, velocity, true);   //lift pair 3
  inverseKinematics(pair1, back, strideWidth, strideLift, returnVelocity, true); //move pair 1 - body moving
  inverseKinematics(pair2, mid, strideWidth, strideLift, returnVelocity, false); //move pair 2 - body moving
  inverseKinematics(pair3, ward, strideWidth, strideLift, velocity, false);   //drop pair 3
}


void forwardKinematics(float q1, float q2, float q3) {
  q1 = q1 * PI / 180;  // converting to radians
  q2 = q2 * PI / 180;
  q3 = q3 * PI / 180;
  
  float x = l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3);
  float y = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);
  float z = l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3);

  Serial.println("-------Forward-------");
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" z: ");
  Serial.println(z);
  
}
