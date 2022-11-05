#include <ros.h>
#include <std_msgs/Header.h>>

ros::NodeHandle nh;
std_msgs::Header header;

ros::Publisher header_pub("time", &header);

void setup()
{
  nh.initNode();
  nh.advertise(header_pub);

}

void loop()
{
  nh.spinOnce();
  header.seq += 1;
  header.stamp = nh.now();
  header_pub.publish(&header);
}
