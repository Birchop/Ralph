#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CD74HC4067.h>
#include <math.h>
#include "Leg.h"
#include "X6B.h"
#include "Gyro.h"
#include "gaitEngine.h"

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

CD74HC4067 muxControl(2, 3, 4, 5);

X6B x6b(RX_PIN, UART_ID, BAUD_RATE, CHANNELS_TO_READ);

float j1OriginX[6] = {80, 80, 0, 0, -80, -80};
float j1OriginY[6] = { -54.5, 54.5, -84.5, 84.5, -54.5, 54.5};

Leg FL("FL", 0, 1, 2, pwm1, 45, 160, 32.5, 80.0, 214.0, false, false, false, j1OriginX[0], j1OriginY[0]);
Leg FR("FR", 0, 1, 2, pwm2, 20, 135, 32.5, 80.0, 214.0, false, false, false, j1OriginX[1], j1OriginY[1]);
Leg ML("ML", 3, 4, 5, pwm1, 65, 115, 32.5, 80.0, 214.0, false, false, false, j1OriginX[2], j1OriginY[2]);
Leg MR("MR", 3, 4, 5, pwm2, 65, 115, 32.5, 80.0, 214.0, false, false, false, j1OriginX[3], j1OriginY[3]);
Leg RL("RL", 6, 7, 8, pwm1, 45, 160, 32.5, 80.0, 214.0, false, false, false, j1OriginX[4], j1OriginY[4]);
Leg RR("RR", 6, 7, 8, pwm2, 20, 135, 32.5, 80.0, 214.0, false, false, false, j1OriginX[5], j1OriginY[5]);

Leg* Legs[6] = {&FL, &FR, &ML, &MR, &RL, &RR};

gaitEngine GE(Legs);

float pitch;
float roll;
float freezePitch;
float freezeRoll;

const int mux1 = A0;
const int mux2 = A1;
const int mux3 = A2;

int mtx1;
int mtx2;
float joint1[6];
float joint2[6];
float joint3[6];
float liveJointAngles[6][3];
float j1Cal[6] = {500, 500, 500, 500, 500, 500};
float j2Cal[6] = {500, 500, 500, 500, 500, 500};
float j3Cal[6] = {666, 666, 666, 666, 666, 666};

float channel[10];
float channels[10];
int gyroDataAvailable = 0;



/*enum Gait {
  tri,
  trot,
  ripple,
  wave
};

Leg p1[2] = {FL, RR};
Leg p2[2] = {ML, MR};
Leg p3[2] = {FR, RL};
*/
int j = 0;
int m = 0;

void setup() {
  //core 0
  Serial.begin(115200);
  delay(5000);
  Serial.println("Serial started. pwm1/2 begin - Attaching legs");
  pinMode(mux1, INPUT);
  pinMode(mux2, INPUT);
  pinMode(mux3, INPUT);
  attachLegs();
  Serial.println("Legs attached");
  //for (;;){}; //freeze for calibration
  Serial.println("Loop start");
  GE.setAdduction(25.0f);
  GE.setIncrement(50);
  GE.setStanceWidth(30);
  GE.setClearance(50.0f);
}

void setup1() {
  //core 1
  W0.begin();
  W0.setClock(400000);
  gyro.begin();
  gyro.calibrate();
  Serial.println(" Gyro started  ");
  x6b.begin();
  Serial.println("x6b started  ");
}

void loop() {
  /*
  for (int a = 0; a < 12; a++) {
    if (!mtx1) {
    transferPitchRoll();
  }
  if (!mtx2) {
    transferChannels();
  }
  if (channels[7] > 1900) {
    Serial.println("Control Enabled - TRI");
    //gaitEngine(tri, 50, 0, 0, 0, 25, -175, 50, 50);
    GE.move(tri, channels[1], channels[2], 0, -200.0f + channels[3]); 
  } else {
    Serial.println("Control Disabled");
  }
  }*//*
  for (int a = 0; a < 12; a++) {
    Serial.println("Safety Branch");
  //delay(50);
  if (!mtx1) {
    transferPitchRoll();
  }
  if (!mtx2) {
    transferChannels();
  }
  if (channels[7] > 1900) {
    Serial.println("Control Enabled - WAVE");
    //gaitEngine(tri, 50, 0, 0, 0, 25, -175, 50, 50);
    GE.move(wave, channels[1], channels[2], 0, -200.0f + channels[3]); 
  } else {
    Serial.println("Control Disabled");
  }
  } */
/*
  for (int i = 0; i <6; i++) {
    for (int j = 0; j < 50; j++) {
      if (i == 2 || i == 3) {
      float x = j1OriginX[i] + 100;
      float y = j1OriginY[i] + j;
      float z = -210;
      Legs[i]->moveLegGlobal(x,y,z);  
      } else {
      float x = j1OriginX[i] + j;
      float y = j1OriginY[i] + 100;
      float z = -210;
      Legs[i]->moveLegGlobal(x,y,z);
      }
      }
      Legs[i]->homeLeg();
    }
   */
   GE.move(wave, 50.0f, 0.0f, 0.0f, -200.0f + channels[3]);
}

void loop1() {
  //core 1
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
/*
void gaitEngine(Gait gait, float stride, float strafe, float yaw, float stanceWidth, float adduction, float ground, float clearance, int increment) {
  


}
*/
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
  delay(1000);

  for (int i = 0; i < 6; i++) {
    if (i < 3) {
      muxControl.channel(i);
      delayMicroseconds(10);

      j1Cal[i] = analogRead(mux1);
      j1Cal[i] *= 2;

      j2Cal[i] = analogRead(mux2);
      j2Cal[i] *= 2;

      j3Cal[i] = analogRead(mux1);
      j3Cal[i] = (j3Cal[i] / 150) * 180;
    } else {
      muxControl.channel(i);
      delayMicroseconds(10);

      j1Cal[i] = analogRead(mux1);
      j1Cal[i] *= 2;

      j2Cal[i] = analogRead(mux2);
      j2Cal[i] *= 2;

      j3Cal[i] = analogRead(mux1);
      j3Cal[i] = (j3Cal[i] / 30) * 180;
    }
  }
  delay(1000);
}

float floatMap(float value, float currMin, float currMax, float desiredMin, float desiredMax) {
  return (value - currMin) * (desiredMax - desiredMin) / (currMax - currMin) + desiredMin;
}

void updateJointAngles() {
  for (int i = 0; i < 6; i++) {
    if (i < 3) {
      muxControl.channel(i);
      delayMicroseconds(10);
      liveJointAngles[i][0] = map(analogRead(mux1), 0, j1Cal[i], 0, 180); // Joint 1
      liveJointAngles[i][1] = map(analogRead(mux2), 0, j2Cal[i], 0, 180); // Joint 2
      liveJointAngles[i][2] = map(analogRead(mux3), 0, j3Cal[i], 0, 180); // Joint 3
    } else {
      liveJointAngles[i][0] = map(analogRead(mux1), 0, j1Cal[i], 180, 0); // Joint 1
      liveJointAngles[i][1] = map(analogRead(mux2), 0, j2Cal[i], 180, 0); // Joint 2
      liveJointAngles[i][2] = map(analogRead(mux3), 0, j3Cal[i], 180, 0); // Joint 3
    }
  }
  Serial.println("FR Angles | A1: " + String(liveJointAngles[3][0]) + "  A2: " + String(liveJointAngles[3][1]) + "  A3: " + String(liveJointAngles[3][2]));
  //Serial.println("FL Angles | A1: " + String(liveJointAngles[0][0]) + "  A2: " + String(liveJointAngles[0][1]) + "  A3: " + String(liveJointAngles[0][2]));
  /*
     0 = FL
     1 = ML
     2 = RL
     3 = FR
     4 = MR
     5 = RR
  */
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
  channel[3] = map(x6b.getChannelValue(2), -85, 85, -50, 50);
  channel[4] = x6b.getChannelValue(3);
  channel[5] = map(x6b.getChannelValue(4), 0, 100, -170, 0);
  channel[6] = map(x6b.getChannelValue(5), 0, 100, -20, 20);
  channel[7] = x6b.getRxChannel(6);
  channel[8] = x6b.getRxChannel(7);
  channel[9] = x6b.getRxChannel(8);
  mtx2 = 0;
}

void getPitchRoll() {
  mtx1 = 1;
  pitch = gyro.getFusedRoll(); // mounted 270deg off..
  roll = gyro.getFusedPitch();
  mtx1 = 0;
}

void transferPitchRoll() {
  mtx1 = 1;
  freezePitch = pitch;
  freezeRoll = roll - 0.133;
  mtx1 = 0;
}
