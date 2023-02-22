#include <Arduino.h>
#include <math.h>
#define ENABLE_EASE_CUBIC
#define ENABLE_EASE_QUADRATIC
#include "ServoEasing.hpp"

#define CH1_PIN 17
#define CH2_PIN 18
#define CH3_PIN 19
#define CH4_PIN 20
#define CH5_PIN 21
#define CH6_PIN 22

int numLegs = 6;

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
float theta3;
float x;
float y;
float z;
float Lc;

float l1 = 54.282;
float l2 = 140;
float l3 = 242.58;

float i = -1.57;
float j = -2.44;
float increment = 0.01f;
const int resolution = 100;

enum Leg {frontLeft, frontRight, middleLeft, middleRight, rearLeft, rearRight, all, pair1, pair2, pair3, tri1, tri2};
enum Gait {triGait, biRippleGait};

bool onTheRun = false;
bool interrupt = false;

float strideLength = 35;
float strideWidth = 0;
float strideLift = 0;


void setup() {
  Serial.begin(115200);

  //Assign pins to servos -- to be switched to run over I2C (pins 2, 3) when integrated with Ralph V2 mainboard
  int i1 = 3;
  int i2 = 6;
  int i3 = 9;
  int i4 = 12;
  int i5 = 15;
  for (int i = 0; i < numJoints; i++) {
    legFL[i].attach(i, 90, 500, 2500);
    legFL[i].setSpeed(servoSpeed);
    legFR[i].attach(i1, 90, 500, 2500);
    legFR[i].setSpeed(servoSpeed);
    legML[i].attach(i2, 90, 500, 2500);
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
  /* //Set to home positions
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
    delay(500000);
  */
  startPositions();
}


void loop() {

  int legNumber;
  String input;

  Serial.println("Enter the leg number (1-6):");
  while (!Serial.available());
  input = Serial.readStringUntil('\n');
  legNumber = input.toInt();
  legNumber--;

  Serial.println("Enter the x coordinate:");
  while (!Serial.available());
  input = Serial.readStringUntil('\n');
  x = input.toFloat();

  Serial.println("Enter the y coordinate:");
  while (!Serial.available());
  input = Serial.readStringUntil('\n');
  y = input.toFloat();

  Serial.println("Enter the z coordinate:");
  while (!Serial.available());
  input = Serial.readStringUntil('\n');
  z = input.toFloat();

  Serial.println(String(x) + ", " + String(y) + ", " + String(z));

  inverseKinematics((Leg)legNumber, x, y, z, 100, false);

  /*for (int i = 0; i<5; i++){
    triWalk(25,100,-105,100);
    }
    for (int i = 0; i < 5; i++) {
    triWalk2(25, 0, 60, 100);

    }*/
}



void inverseKinematics(Leg legNumber, float x, float y, float z, float velocity, bool interrupt) {
  float q1 = atan2(y, x);

  float coxa = l1 * cos(q1);  // adjusted length of coxa based on base angle, gets shorter as it turns
  float d = sqrt((x * x) + (y * y)) - coxa;
  float lc = sqrt((d * d) + ((z - l1) * (z - l1)));
  float alpha = acos(((l2 * l2) + (lc * lc) - (l3 * l3)) / (2 * l2 * lc));
  float beta = atan2(z - l1, d);

  float q2 = PI - alpha - beta;
  float q3 = acos(((l2 * l2) + (l3 * l3) - (lc * lc)) / (2 * l2 * l3));

  q1 = q1 * 180 / PI;
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
  //Inverted, but it works..
  if (q2 >= 90) {
    q2 = map(q2, 90, 180, 0, 90);
  }
  if (q2 <= 0) {
    q2 = map(q2, -180, -90, 90, 180);
  }

  if (q3 >= 40 && q3 <= 130) {
    q3 = map(q3, 40, 130, 180, 90);
  }

  if (q3 >= 130 && q3 <= 180) {
    q3 = map(q3, 130, 180, 90, 40);
  }
  if (q3 <= 0)
  {
    q3 = map(q3, -180, -140, 40, 0);
  }

  moveLeg(legNumber, q1, q2, q3, velocity, interrupt);
}

void moveLeg(Leg legNumber, float q1, float q2, float q3, float velocity, bool interrupt) {
  //Invert left side
  float q1l = map(q1, 0, 180, 180, 0);
  float q2l = map(q2, 0, 180, 180, 0);
  float q3l = map(q3, 0, 180, 180, 0);

  switch (legNumber) {
    case 0:
      setSpeedForAllServos(velocity);
      legFL[0].setEaseTo(q1l);
      legFL[1].setEaseTo(q2l);
      legFL[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
      break;

    case 1:
      setSpeedForAllServos(velocity);
      legFR[0].setEaseTo(q1);
      legFR[1].setEaseTo(q2);
      legFR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
      break;

    case 2:
      setSpeedForAllServos(velocity);
      legML[0].setEaseTo(q1l);
      legML[1].setEaseTo(q2l);
      legML[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
      break;

    case 3:
      setSpeedForAllServos(velocity);
      legMR[0].setEaseTo(q1);
      legMR[1].setEaseTo(q2);
      legMR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
      break;

    case 4:
      setSpeedForAllServos(velocity);
      legRL[0].setEaseTo(q1l);
      legRL[1].setEaseTo(q2l);
      legRL[2].setEaseTo(q3l);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
      break;

    case 5:
      setSpeedForAllServos(velocity);
      legRR[0].setEaseTo(q1);
      legRR[1].setEaseTo(q2);
      legRR[2].setEaseTo(q3);
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
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
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
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
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
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
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
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
      if (interrupt) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
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
      if (interrupt == true) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
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
      if (interrupt == true) setEaseToForAllServosSynchronizeAndStartInterrupt(velocity);
      else synchronizeAllServosStartAndWaitForAllServosToStop();
      break;

    default:
      return;
  }
}

void startPositions() {
  //Start position -- needs start conditions, make void
  inverseKinematics(tri1, -20, 75, -95, 60, true);
  inverseKinematics(tri2, 20, 75, -95, 60, false);
  delay(500);
}

void triWalk(float strideLength, float strideWidth, float strideLift, float velocity) {

  strideLift = -95 - strideLift;
  strideWidth = 75 + strideWidth;

  float back = (strideLength / 2) - strideLength;
  float mid = (strideLength / 2) - (strideLength / 2) + 5;
  float ward = strideLength / 2;

  //Step 1;
  inverseKinematics(tri1, mid, strideWidth, strideLift + 75, velocity, true);   //lift tri 1
  inverseKinematics(tri2, back, strideWidth, strideLift, velocity, false); //move tri 2 - body moving
  inverseKinematics(tri1, ward, strideWidth, strideLift, velocity * 1.5, false); //drop tri 1


  //Step 2;
  inverseKinematics(tri2, mid, strideWidth, strideLift + 75, velocity, true); //lift tri 2
  inverseKinematics(tri1, back, strideWidth, strideLift, velocity, false); //move tri 1 - body moving
  inverseKinematics(tri2, ward, strideWidth, strideLift, velocity * 1.5, false); //drop tri 2
}

void triWalk2(float strideLength, float stance, float groundTarget, float velocity) {

  float liftClearance = 75;

  //Front leg positions
  //Lift leg, drop away from body along y axis and toward center body along x, drag back toward body and out to the side
  float frontLiftX = strideLength / 2;
  float frontLiftY = stance;

  float frontDropX = strideLength / 2;
  float frontDropY = stance + (strideLength / 2);

  float frontMoveX = (strideLength / 2) - strideLength;
  float frontMoveY = stance - (strideLength / 2);


  //Mid leg positions
  //Lift leg forward, drop and drag backwards all along x axis
  float midLiftX = (strideLength / 2) - (strideLength / 2) + 5;
  float midLiftY = stance;

  float midDropX = strideLength / 2;
  float midDropY = stance;

  float midMoveX = (strideLength / 2) - strideLength;
  float midMoveY = stance;

  float Back = (strideLength / 2) - strideLength;
  float Mid = (strideLength / 2) - (strideLength / 2) + 5;
  float Forw = strideLength / 2;

  //Rear leg positions
  //Lift leg, drop toward body along y axis and away/forward from center body along x axis, drag back along x axis and push with y axis
  float rearLiftX = -frontLiftX;
  float rearLiftY = stance;

  float rearDropX = -frontDropX;
  float rearDropY = stance - (strideLength / 2);

  float rearMoveX = -frontMoveX;
  float rearMoveY = stance + (strideLength / 2);

  //Move the legs in loop
  Leg ld1, ld2, ld3, bm1, bm2, bm3;

  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      ld1 = frontLeft;
      ld2 = middleRight;
      ld3 = rearLeft;
      bm1 = frontRight;
      bm2 = middleLeft;
      bm3 = rearRight;
    } else {
      ld1 = frontRight;
      ld2 = middleLeft;
      ld3 = rearRight;
      bm1 = frontLeft;
      bm2 = middleRight;
      bm3 = rearLeft;
    }
    inverseKinematics(ld1, frontLiftX, frontLiftY, groundTarget + liftClearance, velocity, true);//front - lift
    inverseKinematics(ld2, midLiftX, midLiftY, groundTarget + liftClearance, velocity, true); //middle - lift
    inverseKinematics(ld3, rearLiftX, rearLiftY, groundTarget + liftClearance, velocity, true); //rear - lift

    inverseKinematics(bm1, frontMoveX, frontMoveY, groundTarget, velocity, true);//front - move
    inverseKinematics(bm2, midMoveX, midMoveY, groundTarget, velocity, true); //middle - move
    inverseKinematics(bm3, rearMoveX, rearMoveY, groundTarget, velocity, false); //rear - move

    inverseKinematics(ld1, frontDropX, frontDropY, groundTarget, velocity, true);//front - drop
    inverseKinematics(ld2, midDropX, midDropY, groundTarget, velocity, true); //middle - drop
    inverseKinematics(ld3, rearDropX, rearDropY, groundTarget, velocity, false); //rear - drop

  }
}

void biRipple(float strideLength, float strideWidth, float strideLift, float velocity) {
  strideLift = -95 - strideLift;
  strideWidth = 75 + strideWidth;
  float returnVelocity = velocity / 3;
  float back = (strideLength / 2) - strideLength;
  float mid = (strideLength / 2) - (strideLength / 2) + 5;
  float ward = strideLength / 2;

  inverseKinematics(pair1, mid, strideWidth, strideLift + 85, velocity, true);   //lift pair 1
  inverseKinematics(pair2, back, strideWidth, strideLift, returnVelocity, true); //move pair 2 - body moving
  inverseKinematics(pair3, mid, strideWidth, strideLift, returnVelocity, false); //move pair 3 - body moving
  inverseKinematics(pair1, ward, strideWidth, strideLift, velocity, false);   //drop pair 1

  inverseKinematics(pair2, mid, strideWidth, strideLift + 85, velocity, true);   //lift pair 2
  inverseKinematics(pair3, back, strideWidth, strideLift, returnVelocity, true); //move pair 3 - body moving
  inverseKinematics(pair1, mid, strideWidth, strideLift, returnVelocity, false); //move pair 1 - body moving
  inverseKinematics(pair2, ward, strideWidth, strideLift, velocity, false);   //drop pair 2

  inverseKinematics(pair3, mid, strideWidth, strideLift + 85, velocity, true);   //lift pair 3
  inverseKinematics(pair1, back, strideWidth, strideLift, returnVelocity, true); //move pair 1 - body moving
  inverseKinematics(pair2, mid, strideWidth, strideLift, returnVelocity, false); //move pair 2 - body moving
  inverseKinematics(pair3, ward, strideWidth, strideLift, velocity, false);   //drop pair 3
}


void forwardKinematics(float q1, float q2, float q3) {
  q1 = q1 * PI / 180;
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

void setBodyPosition(float x, float y, float z) {
  for (int i = 0; i < numLegs; i++) {
    Lc = sqrt(pow(x, 2) + pow(y, 2));
    theta1 = atan2(y, x) - acos((pow(Lc, 2) + pow(l1, 2) - pow(l2, 2)) / (2 * Lc * l1));
    L1_fk = Lc - (l1 * cos(theta1));
    theta2 = acos((pow(l1, 2) + pow(l2, 2) - pow(Lc, 2)) / (2 * l1 * l2));
    L2_fk = sqrt(pow(Lc, 2) + pow(l1, 2) - 2 * Lc * l1 * cos(theta2));
    theta3 = acos((pow(l2, 2) + pow(l3, 2) - pow(L2_fk, 2)) / (2 * l2 * l3));
    legFL[0].setEaseTo(radToServo(theta1));
    legFL[1].setEaseTo(180 - radToServo(theta2));
    legFL[2].setEaseTo(radToServo(theta3));
  }
}

int radToServo(float angle) {
  return (int)((angle * 180) / M_PI + 90);
}


//This needs to run over uart1 (pins 8, 9) for Ralph V2 mainboard
void readRadioControlValues(float &ch1, float &ch2, float &ch3, float &ch4, float &ch5, float &ch6) {
  int pulseWidth1 = pulseIn(CH1_PIN, HIGH);
  int pulseWidth2 = pulseIn(CH2_PIN, HIGH);
  int pulseWidth3 = pulseIn(CH3_PIN, HIGH);
  int pulseWidth4 = pulseIn(CH4_PIN, HIGH);
  int pulseWidth5 = pulseIn(CH5_PIN, HIGH);
  int pulseWidth6 = pulseIn(CH6_PIN, HIGH);

  ch1 = map(pulseWidth1, 1000, 2000, -100, 100);
  ch2 = map(pulseWidth2, 1000, 2000, -100, 100);
  ch3 = map(pulseWidth3, 1000, 2000, -100, 100);
  ch4 = map(pulseWidth4, 1000, 2000, -100, 100);
  ch5 = map(pulseWidth5, 1000, 2000, -100, 100);
  ch6 = map(pulseWidth6, 1000, 2000, -100, 100);
}
