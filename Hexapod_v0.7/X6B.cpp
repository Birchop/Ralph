#include <Arduino.h>
//#include <hardware/uart.h>
#include "X6B.h"

X6B::X6B(int RX_PIN, uart_inst_t* UART_ID, int BAUD_RATE, int CHANNELS_TO_READ) {
  _RX_PIN = RX_PIN;
  _UART_ID = UART_ID;
  _BAUD_RATE = BAUD_RATE;
  _CHANNELS_TO_READ = CHANNELS_TO_READ;
  _ibusIndex = 0;
}

void X6B::begin() {
  uart_init(_UART_ID, _BAUD_RATE);
  uart_set_hw_flow(_UART_ID, false, false);
  uart_set_format(_UART_ID, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(_UART_ID, true);
  gpio_set_function(_RX_PIN, GPIO_FUNC_UART);
}

void X6B::update() {
  while (uart_is_readable(_UART_ID)) {
    uint8_t incomingByte = uart_getc(_UART_ID);
    if (incomingByte == 0x20) {
      incomingByte = uart_getc(_UART_ID);
      if (incomingByte == 0x40)
        for (int i = 0; i <= 30; i++) { //IBUS_FRAME_SIZE - 2
          incomingByte = uart_getc(_UART_ID);
          _ibusBuff[_ibusIndex++] = incomingByte;

          if (_ibusIndex == 30) { //IBUS_FRAME_SIZE - 2
            _ibusIndex = 0;
            for (uint8_t i = 0; i < 4; i++) {
              _channelValues[i] = map(getRxChannel(i), 1000, 2000, 0, 100);
              _channel[i] = constrain(_channelValues[i], 0, 100);
              _channel[i] = map(_channelValues[i], 2, 98, -85, 85);
              _channel[i] = constrain(_channel[i], -85, 85);
            }
            for (uint8_t i = 4; i < 6; i++) {
              _channelValues[i] = map(getRxChannel(i), 1000, 2000, 0, 100);
              _channel[i] = constrain(_channelValues[i], 0, 100);
            }
            for (uint8_t i = 6; i < 10; i++) {
              _channelValues[i] = map(getRxChannel(i), 1000, 2000, 0, 2);
              _channel[i] = constrain(_channelValues[i], 0, 2);
            }
            

            return;
          }
        }
    }
  }
}

uint16_t X6B::getRxChannel(uint8_t channel) {
  if (channel >= _CHANNELS_TO_READ) {
    return 0;
  }
  uint16_t channelValue = _ibusBuff[2 * channel] | (_ibusBuff[2 * channel + 1] << 8);
  return channelValue;
}

float X6B::getChannelValue(uint8_t channel) {
  if (channel >= _CHANNELS_TO_READ) {
    return 0;
  }
  return _channel[channel];
}
