#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "Leg.h"
#include "X6B.h"
#include "Gyro.h"

#define IBUS_FRAME_SIZE 32
#define CHANNELS_TO_READ 10
#define UART_ID uart1
#define RX_PIN 9
#define BAUD_RATE 115200

#define IMU_ADDRESS 0x68
#define MPU6050_INT_PIN 9


TwoWire  W0 = TwoWire(i2c1, 18, 19);
TwoWire  W1 = TwoWire(i2c0, 20, 21);

Gyro gyro(0x68, W0);
unsigned long lastUpdate;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40, W1); // Expander 1
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41, W1); // Expander 2


X6B x6b(RX_PIN, UART_ID, BAUD_RATE, CHANNELS_TO_READ);

Leg FL(0, 1, 2, pwm1, 145.7, 0, 0, 45, 160);
Leg ML(3, 4, 5, pwm1, 0, 0, 1, 45, 135);
Leg RL(6, 7, 8, pwm1, -145.7, 0, 0, 45, 160);
Leg FR(0, 1, 2, pwm2, 145.7, 1, 0, 20, 135);
Leg MR(3, 4, 5, pwm2, 0, 1, 1, 45, 135);
Leg RR(6, 7, 8, pwm2, -145.7, 1, 0, 20, 135);


float stepTracker = 0;

float pitch;
float roll;
float freezePitch;
float freezeRoll;

int mtx1;
int mtx2;

float prevZ[6] = {1, 1, 1, 1, 1, 1};

enum Gait {
  TRI
};

float channel[10];
float channels[10];
int gyroDataAvailable = 0;

void setup() {//core 0
  Serial.begin(115200);
  delay(5000);
  Serial.println("Serial started. pwm1/2 begin - Attaching legs");
  attachLegs();
  Serial.println("Legs attached");
  //for (;;){}; //freeze for calibration
  Serial.println("Loop start");

}

void setup1() {//core 1
  W0.begin();
  W0.setClock(400000);
  gyro.begin();
  gyro.calibrate();
  Serial.println(" Gyro started  ");
  x6b.begin();
  Serial.println("x6b started  ");
}

void loop() {//core 0
  Serial.println("Safety Branch");
  //delay(50);
  if (!mtx1) {
    transferPitchRoll();
  }
  if (!mtx2) {
    transferChannels();
  }
  if (channels[7] > 1900) {
    Serial.println("Control Enabled");
    GaitEngine(TRI, 51, channels[2], -90 + channels[3], 80 + channels[6], -channels[4], -channels[1], channels[5]);
    Serial.println("Test3 - Gait Cycle Complete");
    Serial.print("Pitch: ");
    Serial.print(freezePitch * RAD_TO_DEG);
    Serial.print("  Roll: ");
    Serial.println(freezeRoll * RAD_TO_DEG);
  } else {
    Serial.println("Control Disabled");
    GaitEngine(TRI, 51, 0, -150, 130, 0, 0, -100);
    Serial.print("Pitch: ");
    Serial.print(freezePitch * RAD_TO_DEG);
    Serial.print("  Roll: ");
    Serial.println(freezeRoll * RAD_TO_DEG);
  }
}

void loop1() {//core 1
  
  bool gdaCheck = gyro.dataAvailable();
  if (gdaCheck) {
    gyro.update();
    if (!mtx1) {
      getPitchRoll();
    }
  }
  x6b.update();
  if (!mtx2) {
    mapChannels();
  }
}

void transferChannels() {
  mtx2 = 1;
  memcpy(channels, channel, sizeof(channel));
  mtx2 = 0;

}

void mapChannels() {
  mtx2 = 1;
  channel[1] = x6b.getChannelValue(0);
  channel[2] = x6b.getChannelValue(1);
  channel[3] = map(x6b.getChannelValue(2), -85, 85, -100, -60);
  channel[4] = x6b.getChannelValue(3);
  channel[5] = map(x6b.getChannelValue(4), 0, 100, -120, 0);
  channel[6] = map(x6b.getChannelValue(5), 0, 100, -20, 20);
  channel[7] = x6b.getRxChannel(6);
  channel[8] = x6b.getRxChannel(7);
  channel[9] = x6b.getRxChannel(8);
  mtx2 = 0;
}

void getPitchRoll() {
  mtx1 = 1;
  pitch = -gyro.getFusedRoll(); // mounted 270deg off..
  roll = gyro.getFusedPitch();
  mtx1 = 0;
}

void transferPitchRoll() {
  mtx1 = 1;
  freezePitch = pitch;
  freezeRoll = roll;
  mtx1 = 0;
}

void GaitEngine(Gait gait, float increment, float stride, float ground, float stance, float yaw, float strafe, float FRY) {

  float hStride = stride / 2;

  // front values
  float x_min_f = (stance + hStride) - (yaw / 2) - (strafe / 2);
  float x_max_f = (stance - hStride) - (yaw / 2) - (strafe / 2);
  float y_min_f = FRY - (yaw / 2) - (strafe / 2);
  float y_max_f = y_min_f + yaw + strafe;

  // mid values
  float x_min_m = ((stride / 2) - stride) + (yaw / 3.14) + (strafe / 3.14);
  float x_max_m = (stride / 2) - (yaw / 3.14) - (strafe / 3.14);
  float y_min_m = stance - (yaw / 2) - (strafe / 2);
  float y_max_m = stance + (yaw / 2) + (strafe / 2);

  // rear values
  float x_min_r = -(stance + hStride) - (yaw / 2) + (strafe / 2);
  float x_max_r = -(stance - hStride) - (yaw / 2) + (strafe / 2);
  float y_min_r = FRY + (yaw / 2) + (strafe / 2);
  float y_max_r = y_min_r - yaw + strafe;

  float halfInc = increment / 2;
  float zArc[100];

  Leg tri1[] = {FL, MR, RL};
  Leg tri2[] = {FR, ML, RR};
  float xDest[6];
  float yDest[6];

  float baseX[6] = {-80, 0, -80, 80, 0, 80};
  float baseY[6] = { -54.5, 84.5, -54.5, 54.5, -84.5, 54.5};
  float zAdjustment[6];


  int flip = 0;

  switch (gait) {
    case TRI:
      for (int j = 0; j < 2; j++) {

        for (int i = 0; i <= increment; i++) {

          float xDiff[3] = {
            (x_min_f > x_max_f ? -1 : 1) * (abs(x_min_f - x_max_f) / increment) * i,
            (x_min_m > x_max_m ? -1 : 1) * (abs(x_min_m - x_max_m) / increment) * i,
            (x_min_r > x_max_r ? -1 : 1) * (abs(x_min_r - x_max_r) / increment) * i
          };
          float yDiff[3] = {
            (y_min_f > y_max_f ? -1 : 1) * (abs(y_min_f - y_max_f) / increment) * i,
            (y_min_m > y_max_m ? -1 : 1) * (abs(y_min_m - y_max_m) / increment) * i,
            (y_min_r > y_max_r ? -1 : 1) * (abs(y_min_r - y_max_r) / increment) * i
          };
          if (i >= halfInc) {
            int t  = increment - i;
            zArc[i] = zArc[t];
          } else if (i == 0) {
            zArc[i] = ground;
          }
          else {
            zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
          }
          //                                          J == 0
          Leg& leg1 = (j == 0) ? tri1[0] : tri2[0]; //  FL
          Leg& leg2 = (j == 0) ? tri2[0] : tri1[0]; //  FR
          Leg& leg3 = (j == 0) ? tri1[1] : tri2[1]; //  MR
          Leg& leg4 = (j == 0) ? tri2[1] : tri1[1]; //  ML
          Leg& leg5 = (j == 0) ? tri1[2] : tri2[2]; //  RL
          Leg& leg6 = (j == 0) ? tri2[2] : tri1[2]; //  RR
          
          xDest[0] = x_max_f - xDiff[0]; xDest[1] = x_min_m + xDiff[1]; xDest[2] = x_min_r + xDiff[2];
          xDest[3] = x_min_f + xDiff[0]; xDest[4] = x_max_m - xDiff[1]; xDest[5] = x_max_r - xDiff[2];
          
          yDest[0] = (j == 0) ? y_max_f - yDiff[0] : y_min_f + yDiff[0];
          yDest[1] = (j == 0) ? y_max_m - yDiff[1] : y_min_m + yDiff[1];
          yDest[2] = (j == 0) ? y_max_r - yDiff[2] : y_min_r + yDiff[2];
          yDest[3] = (j == 0) ? y_max_f - yDiff[0] : y_min_f + yDiff[0];
          yDest[4] = (j == 0) ? y_max_m - yDiff[1] : y_min_m + yDiff[1];
          yDest[5] = (j == 0) ? y_max_r - yDiff[2] : y_min_r + yDiff[2];

          if (!mtx1) {
            transferPitchRoll();
          }
            
              for (int k = 0; k < 6; k++) {
              float effectorX = (baseX[k] + xDest[k]) / 2;
              float effectorY = (baseY[k] + yDest[k]) / 2;
              if (!flip) {
                effectorY = -effectorY; // invert the y coordinate for right legs
              }
              if (k == 2 or k == 5) {
                effectorX = -effectorX; // invert the x coordinate for rear legs
              }

              float radialDist = sqrt(effectorX * effectorX + effectorY * effectorY);
              zAdjustment[k] =  radialDist * (sin(freezePitch) * effectorX + sin(freezeRoll) * effectorY) / radialDist;
              zAdjustment[k] = constrain(zAdjustment[k], -35, 35);
              if (abs(zAdjustment[k] - prevZ[k]) < 0.05f) {
                zAdjustment[k] = prevZ[k];
              } else {
                prevZ[k] = zAdjustment[k];
              }
            flip = 1 - flip;
          }
          if (j == 0) {
            leg1.moveLeg(xDest[0], yDest[0], zArc[i] + zAdjustment[0]);
            leg3.moveLeg(-xDest[1], yDest[1], zArc[i] + zAdjustment[1]);
            leg5.moveLeg(xDest[2], yDest[2], zArc[i] + zAdjustment[2]);

            leg2.moveLeg(xDest[3], yDest[3], ground + zAdjustment[3]);
            leg4.moveLeg(-xDest[4], yDest[4], ground + zAdjustment[4]);
            leg6.moveLeg(xDest[5], yDest[5], ground + zAdjustment[5]);

            
          } else {
            leg1.moveLeg(xDest[0], yDest[0], zArc[i] + zAdjustment[3]);
            leg3.moveLeg(-xDest[1], yDest[1], zArc[i] + zAdjustment[4]);
            leg5.moveLeg(xDest[2], yDest[2], zArc[i] + zAdjustment[5]);

            leg2.moveLeg(xDest[3], yDest[3], ground + zAdjustment[0]);
            leg4.moveLeg(-xDest[4], yDest[4], ground + zAdjustment[1]);
            leg6.moveLeg(xDest[5], yDest[5], ground + zAdjustment[2]);
          }


          /* //No Gyro
                    leg1.moveLeg(xDest[0], yDest[0], zArc[i]);
                    leg3.moveLeg(-xDest[1], yDest[1], zArc[i]);
                    leg5.moveLeg(xDest[2], yDest[2], zArc[i]);

                    leg2.moveLeg(xDest[3], yDest[3], ground);
                    leg4.moveLeg(-xDest[4], yDest[4], ground);
                    leg6.moveLeg(xDest[5], yDest[5], ground);
          */
        }
      }
      break;
    default:
      Serial.println("Invalid gait selected.");
      break;
  }
}

void attachLegs() {
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);
  delay(1500);
  FL.homeLeg();
  FR.homeLeg();
  ML.homeLeg();
  MR.homeLeg();
  RL.homeLeg();
  RR.homeLeg();
  delay(3000);
}
