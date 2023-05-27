#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <hardware/uart.h>

#define IBUS_FRAME_SIZE 32
#define CHANNELS_TO_READ 10
#define UART_ID uart1
#define RX_PIN 9
#define BAUD_RATE 115200

TwoWire  W1 = TwoWire(i2c1, 18, 19);

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40, W1); // Expander 1
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41, W1); // Expander 2

const int servosPerArray = 3;

const int legFR[servosPerArray] = {0, 1, 2};
const int legMR[servosPerArray] = {3, 4, 5};
const int legRR[servosPerArray] = {6, 7, 8};
const int legFL[servosPerArray] = {0, 1, 2};
const int legML[servosPerArray] = {3, 4, 5};
const int legRL[servosPerArray] = {6, 7, 8};

const float l1 = 32.5; // Coxa length
const float l2 = 80.0; // Femur length
const float l3 = 190.0; // Tibia length

float x, y, z;
float q1, q2, q3;

float stepTracker = 0;

enum Leg {
  LEG_FL,
  LEG_FR,
  LEG_ML,
  LEG_MR,
  LEG_RL,
  LEG_RR
};

enum ServoMat {
  metal,
  plastic
};

enum Gait {
  TRI
};

uint8_t ibusBuff[IBUS_FRAME_SIZE];
uint8_t ibusIndex = 0;
uint32_t last_frame_received = 0;
uint16_t channelValues[CHANNELS_TO_READ];
int ch[CHANNELS_TO_READ];

void setup() {//core 0
  Serial.begin(115200);
  delay(4000);
  Serial.println("Serial started. pwm1/2 begin - Attaching legs");
  attachLegs();
  //for (;;){}; //freeze for calibration
  Serial.println("Loop start");

}

void setup1() {//core 1
  uart_init(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_ID, true);
  gpio_set_function(RX_PIN, GPIO_FUNC_UART);
}

void loop() {//core 0
  /*Serial.print("CH1: ");
    Serial.println(ch[0]);
    Serial.print("CH2: ");
    Serial.println(ch[1]);
    Serial.print("CH3: ");
    Serial.println(ch[2]);
    Serial.print("CH4: ");
    Serial.println(ch[3]);
    Serial.print("CH5: ");
    Serial.println(ch[4]);
    Serial.print("CH6: ");
    Serial.println(ch[5]);
    Serial.print("CH7: ");
    Serial.println(ch[6]);
    Serial.print("CH8: ");
    Serial.println(ch[7]);
    Serial.print("CH9: ");
    Serial.println(ch[8]);
    Serial.print("CH10: ");
    Serial.println(ch[9]);*/

  //  this block works but adding any more to it messes with the iBus capture somehow, mainly channel 6..
  float channel1 = map(ch[1], 0, 100, -100, 100);
  float channel2 = map(ch[2], 0, 100, -100, 100);
  float channel3 = map(ch[3], 0, 100, -100, 100);
  float channel4 = map(ch[4], 0, 100, -100, 100);
  float channel5 = map(ch[5], 0, 100, -150, 50);
  float channel6 = map(ch[6], 0, 100, 100, 0); // Safety, pos val is switch up/off, find way to revert as this is sketchy af when not connected
  Serial.println("|Channel1: " + String(channel1) + " |Channel2: " + String(channel2) + " |Channel3:" + String(channel3) + " |Channel4: " + String(channel4) + " |Channel5: " + String(channel5) + " |Channel6: " + String(channel6));
  /*plain math should be quicker vv*/
  /*
    float channel1 = (ch[1] - 50) * 2;
    float channel2 = (ch[2] - 50) * 2;
    float channel3 = (ch[3] - 50) * 2;
    float channel4 = (ch[4] - 50) / 5.0;
    float channel5 = ch[5];
    float channel6 = 100 - ch[6];
  */
  if (ch[6] < 55) {
    GaitEngine4(TRI, 25, channel2, -140 + channel3, 100, channel1, channel4, channel5);
  } else {
    GaitEngine4(TRI, 25, 0, -150, 130, 0, 0, -60);
  }
}

void loop1() {//core 1
  while (uart_is_readable(UART_ID)) {
    uint8_t incomingByte = uart_getc(UART_ID);
    uint32_t now = millis();
    //if you change the number below from 500, good luck to you
    if (now - last_frame_received >= 500) {
      ibusIndex = 0;
    }
    last_frame_received = now;

    ibusBuff[ibusIndex++] = incomingByte;

    if (ibusIndex == IBUS_FRAME_SIZE) {
      ibusIndex = 0;


      for (uint8_t i = 0; i < CHANNELS_TO_READ; i++) {
        channelValues[i] = map(getRxChannel(i), 1000, 2000, 0, 100);
        channelValues[i] = constrain(channelValues[i], 0, 100);
        ch[i] = channelValues[i];
        //Serial.print(channelValues[i]);
        //if (i < CHANNELS_TO_READ - 1) {
        //Serial.print(",");
        //}
      }
      //Serial.println();
    }
  }
}

float rotateX(float x, float y, float angle) {
  return x * cos(angle) - y * sin(angle);
}

float rotateY(float x, float y, float angle) {
  return x * sin(angle) + y * cos(angle);
}

void applyYawToLegPositions(float angle, float &x1, float &y1, float &x2, float &y2, float &x3, float &y3) {
  float tempX1 = x1 * cos(angle) - y1 * sin(angle);
  float tempY1 = x1 * sin(angle) + y1 * cos(angle);
  float tempX2 = x2 * cos(angle) - y2 * sin(angle);
  float tempY2 = x2 * sin(angle) + y2 * cos(angle);
  float tempX3 = x3 * cos(angle) - y3 * sin(angle);
  float tempY3 = x3 * sin(angle) + y3 * cos(angle);

  x1 = tempX1;
  y1 = tempY1;
  x2 = tempX2;
  y2 = tempY2;
  x3 = tempX3;
  y3 = tempY3;
}

void GaitEngine5(Gait gait, float increment, float stride, float ground, float stance, float yaw, float strafe, float FRY) {

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
  float zArc[50];

  Leg tri1[3];
  Leg tri2[3];

  switch (gait) {
    case 0:
      for (int j = 0; j < 2; j++) {
        if (j == 0) {
          tri1[0] = LEG_FL; tri1[1] = LEG_MR; tri1[2] = LEG_RL;
          tri2[0] = LEG_FR; tri2[1] = LEG_ML; tri2[2] = LEG_RR;
        } else {
          tri1[0] = LEG_FR; tri1[1] = LEG_ML; tri1[2] = LEG_RR;
          tri2[0] = LEG_FL; tri2[1] = LEG_MR; tri2[2] = LEG_RL;
        }
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
          } else {
            zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
          }

          float xDest[6] = {
            x_max_f - xDiff[0], x_min_m + xDiff[1], x_min_r + xDiff[2],
            x_min_f + xDiff[0], x_max_m - xDiff[1], x_max_r - xDiff[2]
          };

          float yDest[6] = {
            y_max_f - yDiff[0], y_max_m - yDiff[1], y_max_r - yDiff[2],
            y_max_f - yDiff[0], y_max_m - yDiff[1], y_max_r - yDiff[2]
          };

          moveLeg(tri1[0], 0, xDest[0], yDest[0], zArc[i]);
          moveLeg(tri1[1], 1, xDest[1], yDest[1], zArc[i]);
          moveLeg(tri1[2], 2, xDest[2], yDest[2], zArc[i]);
          moveLeg(tri2[0], 0, xDest[3], yDest[3], ground);
          moveLeg(tri2[1], 1, xDest[4], yDest[4], ground);
          moveLeg(tri2[2], 2, xDest[5], yDest[5], ground);
        }
      }
      break;
    default:
      Serial.println("Invalid gait selected.");
      break;
  }
}

void GaitEngine4(Gait gait, float increment, float stride, float ground, float stance, float yaw, float strafe, float FRY) {

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
  float zArc[50];

  switch (gait) {
    case 0:
      for (int j = 0; j < 2; j++) {
        if (j == 0) {
          for (int i = 0; i <= increment; i++) {
            Leg tri1[3] = {LEG_FL, LEG_MR, LEG_RL};
            Leg tri2[3] = {LEG_FR, LEG_ML, LEG_RR};
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
            } else {
              zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
            }

            float xDest[6] = {
              x_max_f - xDiff[0], x_min_m + xDiff[1], x_min_r + xDiff[2],
              x_min_f + xDiff[0], x_max_m - xDiff[1], x_max_r - xDiff[2]
            };

            float yDest[6] = {
              y_max_f - yDiff[0], y_max_m - yDiff[1], y_max_r - yDiff[2],
              y_max_f - yDiff[0], y_max_m - yDiff[1], y_max_r - yDiff[2]
            };

            moveLeg(tri1[0], 0, xDest[0], yDest[0], zArc[i]);
            moveLeg(tri1[1], 1, xDest[1], yDest[1], zArc[i]);
            moveLeg(tri1[2], 2, xDest[2], yDest[2], zArc[i]);
            moveLeg(tri2[0], 0, xDest[3], yDest[3], ground);
            moveLeg(tri2[1], 1, xDest[4], yDest[4], ground);
            moveLeg(tri2[2], 2, xDest[5], yDest[5], ground);
          }
        } else {
          for (int i = 0; i <= increment; i++) {
            Leg tri1[3] = {LEG_FR, LEG_ML, LEG_RR};
            Leg tri2[3] = {LEG_FL, LEG_MR, LEG_RL};
            float xDiff[3] = {
              (x_min_f > x_max_f ? -1 : 1) * (abs(x_max_f - x_min_f) / increment) * i,
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
            } else {
              zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
            }

            float xDest[6] = {
              x_max_f - xDiff[0], x_min_m + xDiff[1], x_min_r + xDiff[2],
              x_min_f + xDiff[0], x_max_m - xDiff[1], x_max_r - xDiff[2]
            };

            float yDest[6] = {
              y_min_f + yDiff[0], y_min_m + yDiff[1], y_min_r + yDiff[2],
              y_min_f + yDiff[0], y_min_m + yDiff[1], y_min_r + yDiff[2]
            };


            moveLeg(tri1[0], 0, xDest[0], yDest[0], zArc[i]);
            moveLeg(tri1[1], 1, xDest[1], yDest[1], zArc[i]);
            moveLeg(tri1[2], 2, xDest[2], yDest[2], zArc[i]);
            moveLeg(tri2[0], 0, xDest[3], yDest[3], ground);
            moveLeg(tri2[1], 1, xDest[4], yDest[4], ground);
            moveLeg(tri2[2], 2, xDest[5], yDest[5], ground);
          }
        }
      }
      break;
    default:
      Serial.println("Invalid gait selected.");
      break;
  }
}

void GaitEngine2(Gait gait, float increment, float stride, float ground, float stance, float strafe, float yaw, float FRY) {
  yaw = yaw * (PI / 180);
  float hStride = stride / 2;

  // front values
  float x_min_f = (stance + hStride) - strafe / 2;
  float x_max_f = (stance - hStride) - strafe / 2;
  float y_min_f = FRY;
  float y_max_f = y_min_f + strafe;

  // mid values
  float x_min_m = ((stride / 2) - stride) + (strafe / 3.14);
  float x_max_m = (stride / 2) - (strafe / 3.14);
  float y_min_m = stance - (strafe / 2);
  float y_max_m = stance + (strafe / 2);

  // rear values
  float x_min_r = -(stance + hStride) - strafe / 2;
  float x_max_r = -(stance - hStride) - strafe / 2;
  float y_min_r = y_max_f;
  float y_max_r = y_min_f;

  // Apply yaw rotation to initial leg positions
  applyYawToLegPositions(yaw, x_min_f, y_min_f, x_min_m, y_min_m, x_min_r, y_min_r);
  applyYawToLegPositions(yaw, x_max_f, y_max_f, x_max_m, y_max_m, x_max_r, y_max_r);

  float halfInc = increment / 2;
  float zArc[50];

  switch (gait) {
    case 0:
      for (int j = 0; j < 2; j++) {
        if (j == 0) {
          for (int i = 0; i <= increment; i++) {
            Leg tri1[3] = {LEG_FL, LEG_MR, LEG_RL};
            Leg tri2[3] = {LEG_FR, LEG_ML, LEG_RR};
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
            } else {
              zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
            }

            float xDest[6] = {
              x_max_f - xDiff[0], x_min_m + xDiff[1], x_min_r + xDiff[2],
              x_min_f + xDiff[0], x_max_m - xDiff[1], x_max_r - xDiff[2]
            };

            float yDest[6] = {
              y_max_f - yDiff[0], y_max_m - yDiff[1], y_max_r - yDiff[2],
              y_max_f - yDiff[0], y_max_m - yDiff[1], y_max_r - yDiff[2]
            };

            // Apply yaw rotation to the destination points
            for (int k = 0; k < 6; k++) {
              float x_rotated = xDest[k] * cos(yaw) - yDest[k] * sin(yaw);
              float y_rotated = xDest[k] * sin(yaw) + yDest[k] * cos(yaw);
              xDest[k] = x_rotated;
              yDest[k] = y_rotated;
            }

            moveLeg(tri1[0], 0, xDest[0], yDest[0], zArc[i]);
            moveLeg(tri1[1], 1, xDest[1], yDest[1], zArc[i]);
            moveLeg(tri1[2], 2, xDest[2], yDest[2], zArc[i]);
            moveLeg(tri2[0], 0, xDest[3], yDest[3], ground);
            moveLeg(tri2[1], 1, xDest[4], yDest[4], ground);
            moveLeg(tri2[2], 2, xDest[5], yDest[5], ground);
          }
        } else {
          for (int i = 0; i <= increment; i++) {
            Leg tri1[3] = {LEG_FR, LEG_ML, LEG_RR};
            Leg tri2[3] = {LEG_FL, LEG_MR, LEG_RL};
            float xDiff[3] = {
              (x_min_f > x_max_f ? -1 : 1) * (abs(x_max_f - x_min_f) / increment) * i,
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
            } else {
              zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
            }

            float xDest[6] = {
              x_max_f - xDiff[0], x_min_m + xDiff[1], x_min_r + xDiff[2],
              x_min_f + xDiff[0], x_max_m - xDiff[1], x_max_r - xDiff[2]
            };

            float yDest[6] = {
              y_min_f + yDiff[0], y_min_m + yDiff[1], y_min_r + yDiff[2],
              y_min_f + yDiff[0], y_min_m + yDiff[1], y_min_r + yDiff[2]
            };

            // Apply yaw rotation to the destination points
            for (int k = 0; k < 6; k++) {
              float x_rotated = xDest[k] * cos(yaw) - yDest[k] * sin(yaw);
              float y_rotated = xDest[k] * sin(yaw) + yDest[k] * cos(yaw);
              xDest[k] = x_rotated;
              yDest[k] = y_rotated;
            }

            moveLeg(tri1[0], 0, xDest[0], yDest[0], zArc[i]);
            moveLeg(tri1[1], 1, xDest[1], yDest[1], zArc[i]);
            moveLeg(tri1[2], 2, xDest[2], yDest[2], zArc[i]);
            moveLeg(tri2[0], 0, xDest[3], yDest[3], ground);
            moveLeg(tri2[1], 1, xDest[4], yDest[4], ground);
            moveLeg(tri2[2], 2, xDest[5], yDest[5], ground);
          }
        }
      }
      break;
    default:
      Serial.println("Invalid gait selected.");
      break;
  }
}

void GaitEngine3(Gait gait, float increment, float stride, float ground, float stance, float strafe, float yaw) {
  yaw = yaw * (PI / 180);
  float hStride = stride / 2;

  // front values
  float x_min_f = (stance + hStride) - strafe / 2;
  float x_max_f = (stance - hStride) - strafe / 2;
  float y_min_f = -25;
  float y_max_f = y_min_f + strafe;

  // mid values
  float x_min_m = ((stride / 2) - stride) + (strafe / 3.14);
  float x_max_m = (stride / 2) - (strafe / 3.14);
  float y_min_m = stance - (strafe / 2);
  float y_max_m = stance + (strafe / 2);

  // rear values
  float x_min_r = -(stance + hStride) - strafe / 2;
  float x_max_r = -(stance - hStride) - strafe / 2;
  float y_min_r = -25;
  float y_max_r = y_max_f;

  // apply yaw rotation to starting leg positions, this needs work
  applyYawToLegPositions(yaw, x_min_f, y_min_f, x_min_m, y_min_m, x_min_r, y_min_r);
  applyYawToLegPositions(yaw, x_max_f, y_max_f, x_max_m, y_max_m, x_max_r, y_max_r);

  float halfInc = increment / 2;
  float zArc[50];

  Leg triLegs[2][3] = {
    {LEG_FL, LEG_MR, LEG_RL},
    {LEG_FR, LEG_ML, LEG_RR}
  };

  switch (gait) {
    case 0:
      for (int j = 0; j < 2; j++) {
        for (int i = 0; i <= increment; i++) {
          float xDiff[3] = {
            (j == 0 ? x_min_f - x_max_f : x_max_f - x_min_f) / increment * i,
            (j == 0 ? x_min_m - x_max_m : x_max_m - x_min_m) / increment * i,
            (j == 0 ? x_min_r - x_max_r : x_max_r - x_min_r) / increment * i
          };
          float yDiff[3] = {
            (j == 0 ? y_min_f - y_max_f : y_max_f - y_min_f) / increment * i,
            (j == 0 ? y_min_m - y_max_m : y_max_m - y_min_m) / increment * i,
            (j == 0 ? y_min_r - y_max_r : y_max_r - y_min_r) / increment * i
          };
          if (i >= halfInc) {
            int t  = increment - i;
            zArc[i] = zArc[t];
          } else {
            zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f))) / increment) * i);
          }

          float xDest[6] = {
            j == 0 ? x_max_f - xDiff[0] : x_min_f + xDiff[0],
            j == 0 ? x_min_m + xDiff[1] : x_max_m - xDiff[1],
            j == 0 ? x_min_r + xDiff[2] : x_max_r - xDiff[2],
            j == 0 ? x_min_f + xDiff[0] : x_max_f - xDiff[0],
            j == 0 ? x_max_m - xDiff[1] : x_min_m + xDiff[1],
            j == 0 ? x_max_r - xDiff[2] : x_min_r + xDiff[2]
          };

          float yDest[6] = {
            j == 0 ? y_max_f - yDiff[0] : y_max_f - yDiff[0],
            j == 0 ? y_max_m - yDiff[1] : y_max_m - yDiff[1],
            j == 0 ? y_max_r - yDiff[2] : y_max_r - yDiff[2],
            j == 0 ? y_max_f - yDiff[0] : y_min_f + yDiff[0],
            j == 0 ? y_max_m - yDiff[1] : y_min_m + yDiff[1],
            j == 0 ? y_max_r - yDiff[2] : y_min_r + yDiff[2]
          };

          // Apply yaw rotation to the destination points
          for (int k = 0; k < 6; k++) {
            float x_rotated = xDest[k] * cos(yaw) - yDest[k] * sin(yaw);
            float y_rotated = xDest[k] * sin(yaw) + yDest[k] * cos(yaw);
            xDest[k] = x_rotated;
            yDest[k] = y_rotated;
          }

          for (int legIndex = 0; legIndex < 3; legIndex++) {
            moveLeg(triLegs[j][legIndex], legIndex, xDest[legIndex], yDest[legIndex], zArc[i]);
            moveLeg(triLegs[1 - j][legIndex], legIndex, xDest[3 + legIndex], yDest[3 + legIndex], ground);
          }
        }
      }
      break;
    default:
      Serial.println("Invalid gait selected.");
      break;
  }
}




void moveLeg(Leg legNumber, float side, float x, float y, float z) {
  /*q1 = atan2(y, x);
    float L1 = sqrt(x * x + y * y) - l1;
    float L = sqrt(L1 * L1 + z * z);
    float alpha = acos((l2 * l2 + L * L - l3 * l3) / (2 * l2 * L));
    float beta = atan2(z, L1);
    q2 = alpha + beta;
    q3 = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3));
    q1 = q1 * 180 / PI;
    q2 = q2 * 180 / PI;
    //q2 = q2 + 90;
    q3 = q3 * 180 / PI;
    q2 = map(q2, -90, 90, 180, 0);
    q3 = map(q3, -90, 90, 0, 180);
    q3 = q3 - 120;

    float q1r = map(q1, 0, 180, 180, 0);
    float q2r = map(q2, 0, 180, 180, 0);
    float q3r = map(q3, 0, 180, 180, 0);*/
  float baseOffset;
  if (side == 0) {
    baseOffset = 145.7;
  } else if (side == 1) {
    baseOffset = 0;
  } else if (side == 2) {
    baseOffset = -145.7;
  }
  // Rotate the input coordinates by the leg angle
  //q1 = atan2(y,x);
  float angleRad = (baseOffset) * PI / 180;
  float rotatedX = x * cos(angleRad) - y * sin(angleRad);
  float rotatedY = x * sin(angleRad) + y * cos(angleRad);


  q1 = atan2(rotatedY, rotatedX);
  float L1 = sqrt(rotatedX * rotatedX + rotatedY * rotatedY) - l1;
  float L = sqrt(L1 * L1 + z * z);
  float alpha = acos((l2 * l2 + L * L - l3 * l3) / (2 * l2 * L));
  float beta = atan2(z, L1);
  q2 = alpha + beta;
  q3 = acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3));
  q1 = q1 * 180 / PI;
  q2 = q2 * 180 / PI;
  q3 = q3 * 180 / PI;
  q2 = map(q2, -90, 90, 180, 0);
  q3 = map(q3, -90, 90, 0, 180);
  q3 = q3 - 120;

  float q1r = map(q1, 0, 180, 180, 0);
  float q2r = map(q2, 0, 180, 180, 0);
  float q3r = map(q3, 0, 180, 180, 0);


  switch (legNumber) {
    case 0:
      //q1 = constrain(q1, 45, 160);
      q2 = constrain(q2, 0, 180);
      q3 = constrain(q3, 0, 180);
      setServoAngle(pwm1, legFL[0], q1r, plastic);
      setServoAngle(pwm1, legFL[1], q2, metal);
      setServoAngle(pwm1, legFL[2], q3, metal);

      break;

    case 1:
      //q1r = constrain(q1r, 20, 135);
      q2r = constrain(q2r, 0, 180);
      q3r = constrain(q3r, 0, 180);
      setServoAngle(pwm2, legFR[0], q1, metal);
      setServoAngle(pwm2, legFR[1], q2r, metal);
      setServoAngle(pwm2, legFR[2], q3r, metal);

      break;

    case 2:
      q1 = constrain(q1, 45, 135);
      q2 = constrain(q2, 0, 180);
      q3 = constrain(q3, 0, 180);
      setServoAngle(pwm1, legML[0], q1, plastic);
      setServoAngle(pwm1, legML[1], q2, metal);
      setServoAngle(pwm1, legML[2], q3, metal);


      break;

    case 3:
      q1r = constrain(q1r, 45, 135);
      q2r = constrain(q2r, 0, 180);
      q3r = constrain(q3r, 0, 180);
      setServoAngle(pwm2, legMR[0], q1r, metal);
      setServoAngle(pwm2, legMR[1], q2r, metal);
      setServoAngle(pwm2, legMR[2], q3r, metal);

      break;

    case 4:
      q1r = constrain(q1r, 45, 160);
      q2 = constrain(q2, 0, 180);
      q3 = constrain(q3, 0, 180);

      setServoAngle(pwm1, legRL[0], q1r, metal);
      setServoAngle(pwm1, legRL[1], q2 - 8, metal);
      setServoAngle(pwm1, legRL[2], q3, plastic);

      break;

    case 5:
      q1 = constrain(q1, 20, 135);
      q2r = constrain(q2r, 0, 180);
      q3r = constrain(q3r, 0, 180);
      setServoAngle(pwm2, legRR[0], q1, metal);
      setServoAngle(pwm2, legRR[1], q2r + 8, metal);
      setServoAngle(pwm2, legRR[2], q3r, metal);
      //Serial.print("| RotatedX: " + String(rotatedX) + " RotatedY: " + String(rotatedY) + " | ");
      //Serial.println("front right  |  q1: " + String(q1r) + " q2: " + String(q2) + " q3: " + String(q3) + " X " + String(x) + " Y " + String(y) + " z " + String(z));
      //delay(75);
      break;

    default:
      return;
  }
}


void attachLegs() {
  pwm1.begin();
  pwm2.begin();
  pwm1.setPWMFreq(60);
  pwm2.setPWMFreq(60);
  delay(4000);
  Serial.println("setting home positions");
  setServoAngle(pwm1, legFL[0], 90, plastic);
  delay(50);
  setServoAngle(pwm1, legFL[1], 90, metal);
  delay(50);
  setServoAngle(pwm1, legFL[2], 150, metal);
  delay(50);
  setServoAngle(pwm1, legML[0], 90, plastic);
  delay(50);
  setServoAngle(pwm1, legML[1], 90, metal);
  delay(50);
  setServoAngle(pwm1, legML[2], 150, metal);
  delay(50);
  setServoAngle(pwm1, legRL[0], 90, metal);
  delay(50);
  setServoAngle(pwm1, legRL[1], 82, metal);
  delay(50);
  setServoAngle(pwm1, legRL[2], 150, plastic);
  delay(50);

  setServoAngle(pwm2, legFR[0], 90, metal);
  delay(50);
  setServoAngle(pwm2, legFR[1], 90, metal);
  delay(50);
  setServoAngle(pwm2, legFR[2], 30, metal);
  delay(50);
  setServoAngle(pwm2, legMR[0], 90, metal);
  delay(50);
  setServoAngle(pwm2, legMR[1], 90, metal);
  delay(50);
  setServoAngle(pwm2, legMR[2], 30, metal);
  delay(50);
  setServoAngle(pwm2, legRR[0], 90, metal);
  delay(50);
  setServoAngle(pwm2, legRR[1], 98, metal);
  delay(50);
  setServoAngle(pwm2, legRR[2], 30, metal);
  Serial.println("Angles set to home positions.");
  delay(3000);
}

void setServoAngle(Adafruit_PWMServoDriver &driver, int channel, int angle, ServoMat material) {
  if (material == 1) { //Plastic
    int minPWM = 500;
    int maxPWM = 2500;
    int minAngle = 0;
    int maxAngle = 180;
    int pwmValue = angleToPWM(minPWM, maxPWM, minAngle, maxAngle, angle);
    //driver.setPWM(channel, 0, pwmValue);
    driver.writeMicroseconds(channel, pwmValue);
  } else {
    int minPWM = 500;
    int maxPWM = 2500;
    int minAngle = 0;
    int maxAngle = 180;
    int pwmValue = angleToPWM(minPWM, maxPWM, minAngle, maxAngle, angle);
    //driver.setPWM(channel, 0, pwmValue);
    driver.writeMicroseconds(channel, pwmValue);
  }

}

int angleToPWM(int minPWM, int maxPWM, int minAngle, int maxAngle, int angle) {
  int pwmValue = map(angle, minAngle, maxAngle, minPWM, maxPWM);
  return constrain(pwmValue, minPWM, maxPWM);
}

uint16_t getRxChannel(uint8_t channel) {
  if (channel >= CHANNELS_TO_READ) {
    return 0;
  }
  uint16_t channelValue = ibusBuff[2 * channel + 1] | (ibusBuff[2 * channel + 2] << 8);
  return channelValue;
}

/*
  void GaitEngine(Gait gait, float increment, float stride, float ground, float stance, float strafe) { //Increment must be odd, zArc has a dead coord in middle of array otherwise, fix later
  float hStride = stride / 2;

  //front values


  float x_min_f = (stance + hStride) - strafe / 2;
  float x_max_f = (stance - hStride) - strafe / 2;
  float y_min_f = -25;
  float y_max_f = y_min_f + strafe;


  //mid values
  float x_min_m = ((stride / 2) - stride) + (strafe / 3.14);
  float x_max_m = (stride / 2) - (strafe / 3.14);
  float y_min_m = stance - (strafe / 2);
  float y_max_m = stance + (strafe / 2);
  float y_m = stance + hStride;

  //rear values
  float x_min_r = -(stance + hStride) - strafe / 2;
  float x_max_r = -(stance - hStride) - strafe / 2;
  float y_min_r = -25;
  float y_max_r = y_max_f;

  float halfInc = increment / 2;

  float zArc[50];

  switch (gait) {
    case 0:
      for (int j = 0; j < 2; j++) {
        if (j == 0) {
          for (int i = 0; i <= increment; i++) {
            Leg tri1[3] = {LEG_FL, LEG_MR, LEG_RL};
            Leg tri2[3] = {LEG_FR, LEG_ML, LEG_RR};
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
            } else {
              zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) / 2) / increment) * i);
            }



            float rotatedX[6] = {
              rotateX(x_max_f - xDiff[0], y_max_f - yDiff[0], yaw),
              rotateX(x_min_m + xDiff[1], y_max_m - yDiff[1], yaw),
              rotateX(x_min_r + xDiff[2], y_max_r - yDiff[2], yaw),
              rotateX(x_min_f + xDiff[0], y_max_f - yDiff[0], yaw),
              rotateX(x_max_m - xDiff[1], y_max_m - yDiff[1], yaw),
              rotateX(x_max_r - xDiff[2], y_max_r - yDiff[2], yaw)
            };

            float rotatedY[6] = {
              rotateY(x_max_f - xDiff[0], y_max_f - yDiff[0], yaw),
              rotateY(x_min_m + xDiff[1], y_max_m - yDiff[1], yaw),
              rotateY(x_min_r + xDiff[2], y_max_r - yDiff[2], yaw),
              rotateY(x_min_f + xDiff[0], y_max_f - yDiff[0], yaw),
              rotateY(x_max_m - xDiff[1], y_max_m - yDiff[1], yaw),
              rotateY(x_max_r - xDiff[2], y_max_r - yDiff[2], yaw)
            };

            /*moveLeg(tri1[0], x_min_f + xDiff[0], y_min_f + yDiff[0], zArc[i]);
              moveLeg(tri1[1], x_min_m + xDiff[1], y_m, zArc[i]);
              moveLeg(tri1[2], x_max_r - xDiff[2], y_max_r - yDiff[1], zArc[i]);
              moveLeg(tri2[0], x_max_f - xDiff[0], y_max_f - yDiff[0], ground);
              moveLeg(tri2[1], x_max_m - xDiff[1], y_m, ground);
              moveLeg(tri2[2], x_min_r + xDiff[2], y_min_r + yDiff[1], ground);

            moveLeg(tri1[0], 0, x_max_f - xDiff[0], y_max_f - yDiff[0], zArc[i]);
            moveLeg(tri1[1], 1, x_min_m + xDiff[1], y_max_m - yDiff[1], zArc[i]);
            moveLeg(tri1[2], 2, x_min_r + xDiff[2], y_max_r - yDiff[2], zArc[i]);
            moveLeg(tri2[0], 0, x_min_f + xDiff[0], y_max_f - yDiff[0], ground);
            moveLeg(tri2[1], 1, x_max_m - xDiff[1], y_max_m - yDiff[1], ground);
            moveLeg(tri2[2], 2, x_max_r - xDiff[2], y_max_r - yDiff[2], ground);

          }

        } else {

          for (int i = 0; i <= increment; i++) {
            Leg tri1[3] = {LEG_FR, LEG_ML, LEG_RR};
            Leg tri2[3] = {LEG_FL, LEG_MR, LEG_RL};
            float xDiff[3] = {
              (x_min_f > x_max_f ? -1 : 1) * (abs(x_max_f - x_min_f) / increment) * i,
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
            } else {
              zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) / 2) / increment) * i);
            }


            /*moveLeg(tri1[0], x_min_f + xDiff[0], y_min_f + yDiff[0], zArc[i]);
              moveLeg(tri1[1], x_min_m + xDiff[1], y_m, zArc[i]);
              moveLeg(tri1[2], x_max_r - xDiff[2], y_max_r - yDiff[1], zArc[i]);
              moveLeg(tri2[0], x_max_f - xDiff[0], y_max_f - yDiff[0], ground);
              moveLeg(tri2[1], x_max_m - xDiff[1], y_m, ground);
              moveLeg(tri2[2], x_min_r + xDiff[2], y_min_r + yDiff[1], ground);
            moveLeg(tri1[0], 0, x_max_f - xDiff[0], y_min_f + yDiff[0], zArc[i]);
            moveLeg(tri1[1], 1, x_min_m + xDiff[1], y_min_m + yDiff[1], zArc[i]);
            moveLeg(tri1[2], 2, x_min_r + xDiff[2], y_min_r + yDiff[2], zArc[i]);
            moveLeg(tri2[0], 0, x_min_f + xDiff[0], y_min_f + yDiff[0], ground);
            moveLeg(tri2[1], 1, x_max_m - xDiff[1], y_min_m + yDiff[1], ground);
            moveLeg(tri2[2], 2, x_max_r - xDiff[2], y_min_r + yDiff[2], ground);
          }
        }

      }
      break;
    default:
      return;
  }
  }

  void easeLeg(Leg legNumber, float x, float y, float z, float x1, float y1, float z1, int increment) {
  float xArray[increment + 1];
  float yArray[increment + 1];
  float zArray[increment + 1];
  float xDiff, yDiff, zDiff;

  for (int i = 0; i <= increment; i++) {
    xDiff = (x > x1 ? -1 : 1) * (abs(x - x1) / increment) * i;
    yDiff = (y > y1 ? -1 : 1) * (abs(y - y1) / increment) * i;
    zDiff = (z > z1 ? -1 : 1) * (abs(z - z1) / increment) * i;

    xArray[i] = x + xDiff;
    yArray[i] = y + yDiff;
    zArray[i] = z + zDiff;

    moveLeg(legNumber, xArray[i], yArray[i], zArray[i]);
  }

  }

  void easeLegArc(Leg legNumber, float x, float y, float z, float x1, float y1, int increment) {
  float xArray[increment + 1];
  float yArray[increment + 1];
  float zArc[increment + 1];
  float xDiff, yDiff;
  int halfInc = increment / 2;

  for (int i = 0; i <= halfInc; i++) {
    xDiff = (x > x1 ? -1 : 1) * (abs(x - x1) / increment) * i;
    yDiff = (y > y1 ? -1 : 1) * (abs(y - y1) / increment) * i;

    xArray[i] = x + xDiff;
    yArray[i] = y + yDiff;
    zArc[i] = z + ((((abs(x - x1) + abs(y - y1)) / 2) / increment) * i);
    moveLeg(legNumber, xArray[i], yArray[i], zArc[i]);
  }

  for (int i = halfInc + 1; i <= increment; i++) {
    int t = increment - i;

    xDiff = (x > x1 ? -1 : 1) * (abs(x - x1) / increment) * i;
    yDiff = (y > y1 ? -1 : 1) * (abs(y - y1) / increment) * i;

    xArray[i] = x + xDiff;
    yArray[i] = y + yDiff;
    zArc[i] = zArc[t];
    moveLeg(legNumber, xArray[i], yArray[i], zArc[i]);
  }
  }*/
