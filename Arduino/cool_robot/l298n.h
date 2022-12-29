#ifndef __L298N_H__
#define __L298N_H__

#include <Arduino.h>

struct WheelPinout
{
  uint8_t pinEn;
  uint8_t pinIN1;
  uint8_t pinIN2;
};

class L298N
{
public:
  L298N(uint8_t pinEnableA, uint8_t pinIN1A, uint8_t pinIN2A,
        uint8_t pinEnableB, uint8_t pinIN1B, uint8_t pinIN2B)
        : A{ pinEnableA, pinIN1A, pinIN2A },
          B{ pinEnableB, pinIN1B, pinIN2B }
  {
    pinMode(A.pinIN1, OUTPUT);
    pinMode(A.pinIN2, OUTPUT);
    pinMode(A.pinEn, OUTPUT);
    pinMode(B.pinIN1, OUTPUT);
    pinMode(B.pinIN2, OUTPUT);
    pinMode(B.pinEn, OUTPUT);
  }

  void driveALL(int16_t val_l, int16_t val_r)
  {
    val_l = constrain(val_l, -255, 255);
    val_r = constrain(val_r, -255, 255);

    driveWheel(A, val_l);
    driveWheel(B, val_r);
  }

  void driveWheel(WheelPinout wheel, int16_t val){
    if (val > 0)
    {
      digitalWrite(wheel.pinIN1, HIGH);
      digitalWrite(wheel.pinIN2, LOW);
    }
    else if (val < 0)
    {
      digitalWrite(wheel.pinIN1, LOW);
      digitalWrite(wheel.pinIN2, HIGH);
    }
    else
    {
      digitalWrite(wheel.pinIN1, LOW);
      digitalWrite(wheel.pinIN2, LOW);
    }
    analogWrite(wheel.pinEn, abs(val));
  }
private:
  WheelPinout A, B;

  uint8_t pwm;
};
#endif
