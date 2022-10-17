#include <ros/ros.h>
#include <std_msgs/Int64.h>

ros::Publisher num_pub;
bool arduino_return_state = true;

void num_cb(const std_msgs::Int64::ConstPtr& ptr){
  ROS_INFO_STREAM("Num received from Arduino is: " << ptr->data);
  arduino_return_state = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cp1");
  ros::NodeHandle nh("");
  std_msgs::Int64 num;

  num_pub = nh.advertise<std_msgs::Int64>("num/raw", 10);
  ros::Subscriber num_sub = nh.subscribe("num/arduino", 10, &num_cb);

  try
  {
    ROS_INFO("[Check Point 1]: Initializing node");

    while(ros::ok){
      if (arduino_return_state){
        ROS_INFO_STREAM("Waiting user input...");
        std::cin >> num.data;
        ROS_INFO_STREAM("Num sent to Arduino is: " << num.data);
        num_pub.publish(num);
        arduino_return_state = false;
      }
      ros::spinOnce();
    }
      
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Check Point 1]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Check Point 1]: Unexpected error");
  }

  return 0;
}
