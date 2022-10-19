#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <Encoder.h>
#include <L298NX2.h>
#include <PID_v1.h>

class Robot
{
public:
  Robot()
    : enc{ { 0, 0, 0, Encoder(3, 12) }, { 0, 0, 0, Encoder(2, 4) } }
    , motors{ L298NX2(5, 8, 9, 6, 10, 11) }
    , pid{ { 0.0, 0.0, 0.0, PID(&pid.left.feedback, &pid.left.output, &pid.left.setpoint, 120, 40, 0.05, DIRECT) },
           { 0.0, 0.0, 0.0, PID(&pid.right.feedback, &pid.right.output, &pid.right.setpoint, 120, 40, 0.05, DIRECT) } }
  {
    pid.left.handle.SetOutputLimits(-255, 255);
    pid.right.handle.SetOutputLimits(-255, 255);
    pid.left.handle.SetSampleTime(1);
    pid.right.handle.SetSampleTime(1);
    // pid.set_setpoint(-3, 3);
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

  void run()
  {
    enc.read();
    pid.set_feedback(enc.left.diff, enc.right.diff);
    pid.compute();
    motors.drive(pid.left.output, pid.right.output);
  }
};

#endif
