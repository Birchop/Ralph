#ifndef X6B_H
#define X6B_H
#include <Arduino.h>

class X6B {
  public:
    X6B(int RX_PIN, uart_inst_t* UART_ID, int BAUD_RATE, int CHANNELS_TO_READ);
    void begin();
    void update();
    uint16_t getRxChannel(uint8_t Channel);
    float getChannelValue(uint8_t Channel);

  private:
    int _RX_PIN;
    uart_inst_t* _UART_ID;
    int _BAUD_RATE;
    int _CHANNELS_TO_READ;
    uint8_t _ibusBuff[32]; //IBUS_FRAME_SIZE
    uint8_t _ibusIndex;
    uint16_t _channelValues[14]; //CHANNELS_TO_READ
    float _channel[14]; //CHANNELS_TO_READ
    int success = 0;
};

#endif
