#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <Arduino.h>
#include <Encoder.h>
#include <L298NX2.h>
#include <PID_v1.h>

#define BUF_LEN 100

class Robot
{
public:
  Robot()
    : enc{ { 0, 0, 0, Encoder(3, 12) }, { 0, 0, 0, Encoder(2, 4) } }
    , motors{ L298NX2(5, 8, 9, 6, 10, 11) }
    , pid{ { 0.0, 0.0, 0.0, PID(&pid.left.feedback, &pid.left.output, &pid.left.setpoint, 300, 870, 0.00, DIRECT) },
           { 0.0, 0.0, 0.0, PID(&pid.right.feedback, &pid.right.output, &pid.right.setpoint, 300, 870, 0.00, DIRECT) } }
    , ms{ { 7, 0 }, { 13, 0 }, { A1, 0 } }
    , pr{ A2, A3, 0, 0 }
    , ind_ir_buf(0)
    , ir_buf{ 0 }
    , duty_cycle_ir(0)
  {
    pid.left.handle.SetOutputLimits(-255, 255);
    pid.right.handle.SetOutputLimits(-255, 255);
    pid.left.handle.SetSampleTime(1);
    pid.right.handle.SetSampleTime(1);
    pid.left.handle.SetMode(AUTOMATIC);
    pid.right.handle.SetMode(AUTOMATIC);
  }

  // Encoders
  struct
  {
    struct
    {
      int32_t now;
      int32_t last;
      int32_t diff;
      Encoder handle;

      void read()
      {
        now = handle.read();
        diff = now - last;
        last = now;
      }
    } left, right;

    void read()
    {
      left.read();
      right.read();
    }
  } enc;

  // L298NX2
  struct
  {
    L298NX2 l298n;

    void drive(int16_t val_l, int16_t val_r)
    {
      val_l = constrain(val_l, -255, 255);
      val_r = constrain(val_r, -255, 255);

      l298n.setSpeedA(abs(val_l));
      l298n.setSpeedB(abs(val_r));

      if (val_l > 0)
      {
        l298n.forwardA();
      }
      else if (val_l < 0)
      {
        l298n.backwardA();
      }
      else
      {
        l298n.stopA();
      }

      if (val_r > 0)
      {
        l298n.forwardB();
      }
      else if (val_r < 0)
      {
        l298n.backwardB();
      }
      else
      {
        l298n.stopB();
      }
    }
  } motors;

  // PID
  struct
  {
    struct
    {
      double setpoint;
      double feedback;
      double output;
      PID handle;
    } left, right;

    void set_feedback(double val_l, double val_r)
    {
      left.feedback = val_l;
      right.feedback = val_r;
    }

    void set_setpoint(double val_l, double val_r)
    {
      left.setpoint = val_l;
      right.setpoint = val_r;
    }

    void compute()
    {
      left.handle.Compute();
      right.handle.Compute();
    }
  } pid;

  // MS
  struct
  {
    struct
    {
      int pin;
      int data;

      void init()
      {
        pinMode(pin, INPUT);
      }
      void read()
      {
        data = digitalRead(pin);
      }
    } left, right, mid;

    void init()
    {
      left.init();
      right.init();
      mid.init();
    }

    void read()
    {
      left.read();
      right.read();
      mid.read();
    }
  } ms;

  // pr

  struct
  {
    int pin_a;
    int pin_d;
    int volt;
    int on;

    void reada()
    {
      volt = analogRead(pin_a);
    }
    void readd()
    {
      on = digitalRead(pin_d);
    }
  } pr;

  void init()
  {
    ms.init();
  }

  void run()
  {
    ms.read();
    pr.reada();
    pr.readd();
    enc.read();
    pid.set_feedback(enc.left.diff, enc.right.diff);
    pid.compute();
    motors.drive(pid.left.output, pid.right.output);
  }

  byte ir_buf[BUF_LEN];
  int ind_ir_buf;
  double duty_cycle_ir;
  void readIR()
  {
    ir_buf[ind_ir_buf++] = digitalRead(A5);
    if (ind_ir_buf == BUF_LEN)
      ind_ir_buf = 0;

    duty_cycle_ir = 0;
    for (int i = 0; i < BUF_LEN; ++i)
    {
      duty_cycle_ir += (double)ir_buf[i] / BUF_LEN;
    }


  }
};

#endif
