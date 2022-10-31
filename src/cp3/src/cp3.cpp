#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/String.h>

ros::Publisher action_pub;
bool arduino_return_state = true;

class FSM {
private:
  enum Action {forward, right, left, backward};
  
public:
  FSM();
  ~FSM();
  std_msgs::String  process_event();
};

FSM::FSM()
{
  Action action;
  action = forward;
}



void state_cb(const std_msgs::Int64::ConstPtr& ptr){
  ROS_INFO_STREAM("Num received from Arduino is: " << ptr->data);
  arduino_return_state = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cp3");
  ros::NodeHandle nh("");
  std_msgs::String action;
  // action.data.resize(2);
  action_pub = nh.advertise<std_msgs::String>("action", 10);
  ros::Subscriber state_sub = nh.subscribe("state", 10, &state_cb);

  try
  {
    ROS_INFO("[Check Point 3]: Initializing node");
    Action state;
    state = foward;
    while(ros::ok){
      if (arduino_return_state || true){
        // test
        if(right == 1 || left == 1){
          if(right == 1 && left == 1){
            action_pub.publish('b');
            ros.sleep();
          }
          if(right == 1 && left ==0){
            action_pub.publish('l');
            ros.sleep();
          }
          if(right == 0 && left ==1){
            action_pub.publish('r');
            ros.sleep();
          }
        }
        else{
          if(mid == 1){
            //s = stop
            action_pub.publish('s');
          }
          if (mid == 0){
            // ....
          }

        }

        action_pub.publish(action);
        arduino_return_state = false;
      }
      ros::spinOnce();
    }
      
  }


  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Check Point 3]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Check Point 3]: Unexpected error");
  }

  return 0;
}
