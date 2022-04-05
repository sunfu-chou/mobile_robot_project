#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Twist.h"

const float PI = 3.14159265358979323;

 class echoListener
 {
 public:
 
   tf::TransformListener tf;
 
   //constructor with name
   echoListener()
   {
 
   }
 
   ~echoListener()
   {
 
   }
 
 private:
 
 };

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle n("~");
	
   

  tf::TransformListener listener;
	geometry_msgs::Twist transform;

	echoListener echoListener;
  
   // Wait for up to one second for the first transforms to become avaiable. 
   //echoListener.tf.waitForTransform("map", "base_footprint", ros::Time(), ros::Duration(1.0));


  std::string odom_s;
  std::string base_footprint_s;
  std::string robot_pose_s("/");
  std::string slash;
  std::string s2("/robot_pose");
  std::string s3("/base_footprint");
  n.getParam("Name", slash);
  n.getParam("Name", base_footprint_s);
  robot_pose_s += slash.c_str();
  robot_pose_s += s2.c_str();
  base_footprint_s += s3.c_str();

  echoListener.tf.waitForTransform("map", base_footprint_s.c_str(), ros::Time(), ros::Duration(1.0));
  ros::Publisher state_pub = n.advertise<geometry_msgs::Twist>(robot_pose_s.c_str(), 50);
  ros::Rate rate(100.0);
	while(n.ok())
	 {
	   try
	   {
	     tf::StampedTransform echo_transform;
             //echoListener.tf.lookupTransform("map", "base_footprint", ros::Time(), echo_transform);
             echoListener.tf.lookupTransform("map", base_footprint_s.c_str(), ros::Time(), echo_transform);
             double yaw, pitch, roll, th;
	     echo_transform.getBasis().getRPY(roll, pitch, yaw);
	     tf::Quaternion q = echo_transform.getRotation();
	     tf::Vector3 v = echo_transform.getOrigin();

			 transform.linear.x = v.getX();
			 transform.linear.y = v.getY();
			 transform.linear.z = v.getZ();
                         transform.angular.x = roll;
                         transform.angular.y = pitch;

                         th = yaw;
                         if(th>=(2*PI)){
                                 th = th - 2*PI;
                         }
                         else if(th<(0)){
                                 th = th + 2*PI;
                         }
                         transform.angular.z = th;

                         state_pub.publish(transform);
                       //  printf("pose published\n");

				/*
	     std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
	     std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
	               << q.getZ() << ", " << q.getW() << "]" << std::endl
	               << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
	               << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
		     //print transform
				*/
	   }
	   catch(tf::TransformException& ex)
	   {
	     std::cout << "Failure at "<< ros::Time::now() << std::endl;
	     std::cout << "Exception thrown:" << ex.what()<< std::endl;
	     std::cout << "The current list of frames is:" <<std::endl;
	     std::cout << echoListener.tf.allFramesAsString()<<std::endl;
	     
	   }
	   rate.sleep();
	 }
  return 0;
};
