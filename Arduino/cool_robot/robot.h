#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <Arduino.h>
#include <PID_v1.h>
// #include <QuickPID.h>
#define ENCODER_USE_INTERRUPTS
#include "encoder_wraper.h"
#include "l298n.h"

class Robot
{
public:
  Robot()
    : encoder{ EncoderWrapper(2, 4), EncoderWrapper(3, 12) }
    , motors(6, 11, 10, 5, 9, 8)
    , pid{ { 0.0, 0.0, 0.0, PID(&pid.left.feedback, &pid.left.output, &pid.left.setpoint, 900, 900, 0.00, DIRECT) },
           { 0.0, 0.0, 0.0, PID(&pid.right.feedback, &pid.right.output, &pid.right.setpoint, 900, 900, 0.00, DIRECT) } }
  {
    pid.left.pid.SetOutputLimits(-255, 255);
    pid.right.pid.SetOutputLimits(-255, 255);
    pid.left.pid.SetSampleTime(1);
    pid.right.pid.SetSampleTime(1);
    pid.left.pid.SetMode(AUTOMATIC);
    pid.right.pid.SetMode(AUTOMATIC);
  }

  // Encoders
  struct
  {
    EncoderWrapper left, right;
  } encoder;

  // L298N
  L298N motors;

  // PID
  struct
  {
    struct
    {
      double setpoint;
      double feedback;
      double output;
      PID pid;
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
      left.pid.Compute();
      right.pid.Compute();
    }
  } pid;

  void run()
  {
    encoder.left.read();
    encoder.right.read();
    pid.set_feedback(encoder.left.diff, encoder.right.diff);
    pid.compute();
    motors.driveALL(pid.left.output, pid.right.output);
  }
};
#endif
