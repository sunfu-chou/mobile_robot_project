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

  enum Action {forward, scan, done };
  Action state;    
  state = forward;

  try
  {
    ROS_INFO("[Check Point 3]: Initializing node");
    while(ros::ok){
        // test



      if(state == forward){
        if(right == 1 || left == 1){

          ros::Time begin = ros::Time::now();

          if(right == 1 && left == 1){
            ros::Time now = ros::Time::now();
            while((now - begin) < 1.0 ){
              action_pub.publish('b');
            }
            
            ros::Duration(0.5).sleep();
            action_pub.publish('r');
            ros::Duration(0.5).sleep();
            state = scan;
          }
          if(right == 1 && left ==0){
            action_pub.publish('l');
            ros::Duration(0.5).sleep();
            action_pub.publish('f');
            ros::Duration(0.5).sleep();
            action_pub.publish('s');
            state = scan;
          }
          if(right == 0 && left ==1){
            action_pub.publish('r');
            ros::Duration(0.5).sleep();
            action_pub.publish('f');
            ros::Duration(0.5).sleep();
            action_pub.publish('s');
            state = scan;
          }
        }
        else{
          if(mid == 1){
            //s = stop
            action_pub.publish('s');
            state = done;
          }
        }

      if(state == scan){
        while(photo != 1){
          action_pub.publish('');
        }


      
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
