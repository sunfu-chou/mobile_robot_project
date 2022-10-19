#include <ros.h>
#include <std_msgs/Int64.h>

void num_cb(const std_msgs::Int64 &msg);

ros::NodeHandle nh;
std_msgs::Int64 num;

ros::Publisher num_pub("num/arduino", &num);
ros::Subscriber<std_msgs::Int64> num_sub("num/raw", &num_cb);

void num_cb(const std_msgs::Int64& msg){
  num = msg;
  num.data *= 2;
  num_pub.publish(&num);
}

void setup()
{
  nh.initNode();
  nh.advertise(num_pub);
  nh.subscribe(num_sub);
}

void loop()
{
  nh.spinOnce();
}
