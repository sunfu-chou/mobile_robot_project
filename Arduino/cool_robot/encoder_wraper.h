#ifndef __ENCODER_WRAPPER_H__
#define __ENCODER_WRAPPER_H__

#include <Arduino.h>
#include <Encoder.h>

class EncoderWrapper
{
public:
  EncoderWrapper(uint8_t pin1, uint8_t pin2) : encoder(pin1, pin2)
  {
    now = 0;
    last = 0;
    diff = 0;
  }
  int32_t now;
  int32_t last;
  int32_t diff;
  Encoder encoder;

  void read()
  {
    now = -encoder.read();
    diff = now - last;
    last = now;
  }
};
#endif
