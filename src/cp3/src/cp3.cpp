#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/String.h>

ros::Publisher action_pub;
// bool arduino_return_state = true;

// class FSM {
// private:
//   enum Action {forward, right, left, backward};
  
// public:
//   FSM();
//   ~FSM();
//   std_msgs::String  process_event();
// };

// FSM::FSM()
// {
//   Action action;
//   action = forward;
// }

// global statics
int photo = 0;
int left = 0;
int mid = 0;
int right = 0;

void state_cb(const std_msgs::ByteMultiArray::ConstPtr& ptr){
  // ROS_INFO_STREAM("Num received from Arduino is: " << ptr->data);
  photo = ptr->data[0];
  left = ptr->data[1];
  mid = ptr->data[2];
  right = ptr->data[3];
  // arduino_return_state = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "cp3");
  ros::NodeHandle nh("");
  std_msgs::String action;
  // action.data.resize(2);
  action_pub = nh.advertise<std_msgs::String>("action", 10);
  ros::Subscriber state_sub = nh.subscribe("state", 10, &state_cb);
  double last_time = 0.8;
  enum Action {forward, scanRight, scanLeft, done };
  Action state;    
  state = forward;

  try
  {
    ROS_INFO("[Check Point 3]: Initializing node");
    while(ros::ok){
        // test

      if(state == forward){
        if(right == 1 || left == 1){

          double begin = ros::Time::now().toSec();

          if(right == 1 && left == 1){
            double now = ros::Time::now().toSec();
            while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('b');
              ros::spinOnce();
            }

            begin = ros::Time::now().toSec();
            while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('r');
              ros::spinOnce();
            }

            state = scan;
          }
          if(right == 1 && left ==0){
            double now = ros::Time::now().toSec();
            while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('l');
              ros::spinOnce();
            }

            begin = ros::Time::now().toSec();
            while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('f');
              ros::spinOnce();
            }

            state = scanLeft;
          }
          if(right == 0 && left ==1){
            double now = ros::Time::now().toSec();
            while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('r');
              ros::spinOnce();
            }

            begin = ros::Time::now().toSec();
            while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('f');
              ros::spinOnce();
            }
            
            state = scanRight;
          }
        }
        else{
          if(mid == 1){
            //s = stop
            action_pub.publish('s');
            state = done;
          }
        }

      if(state == scanRight){
        int count = 0;
        double begin = ros::Time::now().toSec();
        while(photo != 1 or count < 3){
          double now = ros::Time::now().toSec();
          while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('r');
              ros::spinOnce();
            }
          count += 1;
          ros::spinOnce();
        }
        state = forward;
      }

      if(state == scanLeft){
        int count = 0;
        double begin = ros::Time::now().toSec();
        while(photo != 1 or count < 3){

          double now = ros::Time::now().toSec();
          while((now - begin) < last_time ){
              now = ros::Time::now().toSec();
              action_pub.publish('r');
              ros::spinOnce();
            }
          count += 1;

          ros::spinOnce();
        }
        state = forward;
      }
        
        // arduino_return_state = false;
      }
      ros::spinOnce();
      
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
