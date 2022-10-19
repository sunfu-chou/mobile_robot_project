#include <ros.h>
#include "robot.h"
#include <Encoder.h>
#include <L298NX2.h>
#include <PID_v1.h>

ros::NodeHandle nh;

int32_t enc_left;
int32_t enc_last_left;
Encoder encHLeft(3, 12);
int32_t enc_speed_left;

int32_t enc_right;
int32_t enc_last_right;
Encoder encHRight(2, 4);
int32_t enc_speed_right;

L298NX2 motors(5, 8, 9, 6, 10, 11);

double lfb, rfb;
double lsp, rsp;
double lct, rct;

PID lPID(&lfb, &lct, &lsp, 40, 15, 0.02, DIRECT);
PID rPID(&rfb, &rct, &rsp, 40, 15, 0.02, DIRECT);

void read_encoders()
{
  enc_left = encHLeft.read();
  enc_right = encHRight.read();

  enc_speed_left = enc_left - enc_last_left;
  enc_speed_right = enc_right - enc_last_right;

  enc_last_left = enc_left;
  enc_last_right = enc_right;
}

void setup()
{
  nh.initNode();
  Serial.begin(9600);

  lsp = 25.04;
  rsp = 25;
  lPID.SetSampleTime(1);
  rPID.SetSampleTime(1);
  lPID.SetMode(AUTOMATIC);
  rPID.SetMode(AUTOMATIC);
}

void loop()
{
  nh.spinOnce();

  read_encoders();
  lfb = enc_speed_left;
  rfb = enc_speed_right;

  lPID.Compute();
  rPID.Compute();

  motors.setSpeedA(lct);
  motors.setSpeedB(rct);
  motors.forwardA();
  motors.forwardB();

  Serial.print(enc_speed_left);
  Serial.print(", ");
  Serial.println(enc_speed_right);
}
