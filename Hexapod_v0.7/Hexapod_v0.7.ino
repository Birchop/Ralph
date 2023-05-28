#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <hardware/uart.h>
#include "Leg.h"

#define IBUS_FRAME_SIZE 32
#define CHANNELS_TO_READ 10
#define UART_ID uart1
#define RX_PIN 9
#define BAUD_RATE 115200

TwoWire  W1 = TwoWire(i2c1, 18, 19);

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40, W1); // Expander 1
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41, W1); // Expander 2

Leg FL(0, 1, 2, pwm1, 145.7, 0, 0, 45, 160);
Leg ML(3, 4, 5, pwm1, 0, 0, 1, 45, 135);
Leg RL(6, 7, 8, pwm1, -145.7, 0, 0, 45, 160);
Leg FR(0, 1, 2, pwm2, 145.7, 1, 0, 20, 135);
Leg MR(3, 4, 5, pwm2, 0, 1, 1, 45, 135);
Leg RR(6, 7, 8, pwm2, -145.7, 1, 0, 20, 135);

float stepTracker = 0;

enum Gait {
  TRI
};

uint8_t ibusBuff[IBUS_FRAME_SIZE];
uint8_t ibusIndex = 0;
uint32_t last_frame_received = 0;
uint16_t channelValues[CHANNELS_TO_READ];
int ch[CHANNELS_TO_READ];
float channel[6];



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
  uart_init(UART_ID, BAUD_RATE);
  uart_set_hw_flow(UART_ID, false, false);
  uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_ID, true);
  gpio_set_function(RX_PIN, GPIO_FUNC_UART);
}

void loop() {//core 0
  //Serial.println("Test1 - Safety Branch");
  //delay(50);
  if (ch[6] < 55) {
 //   Serial.println("Test2 - Control Enabled");
    GaitEngine(TRI, 25, channel[1], -140 + channel[2], 100, channel[0], channel[3], channel[4]);
   // Serial.println("Test3 - Gait Cycle Complete");
  } else {
    //Serial.println("Test2 - Control Disabled");
    GaitEngine(TRI, 25, 0, -150, 130, 0, 0, -100);
  }
  //Serial.println("Loop End");
  //delay(500);
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
      }
    }
    channel[0] = map(ch[1], 0, 100, -100, 100);
    channel[1] = map(ch[2], 0, 100, -100, 100);
    channel[2] = map(ch[3], 0, 100, -100, 100);
    channel[3] = map(ch[4], 0, 100, -100, 100);
    channel[4] = map(ch[5], 0, 100, -150, 50);
    channel[5] = map(ch[6], 0, 100, 100, 0);
  }
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
  float zArc[50];

  Leg tri1[] = {FL, MR, RL};
  Leg tri2[] = {FR, ML, RR};
  float xDest[6];
  float yDest[6];

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
          } else {
            zArc[i] = ground + ((((abs(x_min_f - x_max_f) + abs(y_min_f - y_max_f)) * 2) / increment) * i);
          }
                                                      //  J == 0
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

            leg1.moveLeg(xDest[0], yDest[0], zArc[i]);  
            leg3.moveLeg(-xDest[1], yDest[1], zArc[i]);
            leg5.moveLeg(xDest[2], yDest[2], zArc[i]);
            
            leg2.moveLeg(xDest[3], yDest[3], ground);
            leg4.moveLeg(-xDest[4], yDest[4], ground);
            leg6.moveLeg(xDest[5], yDest[5], ground);
            
            stepTracker++;
          
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

uint16_t getRxChannel(uint8_t channel) {
  if (channel >= CHANNELS_TO_READ) {
    return 0;
  }
  uint16_t channelValue = ibusBuff[2 * channel + 1] | (ibusBuff[2 * channel + 2] << 8);
  return channelValue;
}
