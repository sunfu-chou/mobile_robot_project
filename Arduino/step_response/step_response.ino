#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Encoder.h>
#include <L298NX2.h>

Encoder left(3, 12);
Encoder right(2, 4);

int32_t l_last, l_diff, r_last, r_diff;
L298NX2 motors(5, 8, 9, 6, 10, 11);

ros::NodeHandle nh;

std_msgs::Float32MultiArray speeds;

ros::Publisher speed_pub("speeds", &speeds);

void setup()
{
  nh.initNode();
  nh.advertise(speed_pub);
  motors.setSpeedA(100);
  motors.setSpeedB(100);
  motors.forwardA();
  motors.forwardB();
}

void loop()
{
  nh.spinOnce();

  int32_t l = 0, r = 0;
  l = left.read();
  r = right.read();

  l_diff = l - l_last;
  r_diff = r - r_last;
  l_last = l;
  r_last = r;
  delay(1);
  Serial.print(l_diff);
  Serial.print(",");
  Serial.println(r_diff);
}
