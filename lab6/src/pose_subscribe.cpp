#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"

double x_now = 0.0;
double y_now = 0.0;
double th_now = 0.0;

const float PI = 3.14159265358979323;

ros::Subscriber sub;

void GoalCallback(const geometry_msgs::Twist &msg)
{	
	geometry_msgs::Quaternion odom_quat;

	x_now = msg.linear.x;					//unit: m
	y_now = msg.linear.y;					//unit: m
	th_now = msg.angular.z;			//quaternoin

	if(th_now>PI){th_now-=2*PI;}
	else if(th_now<-PI){th_now+=2*PI;}
	
	ROS_INFO("x: %.3f /t y: %.3f /t th: %.3f",x_now,y_now,th_now);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pose_subscribe");
 
  ros::NodeHandle n;
  
  ros::Rate r(100.0);
  while(n.ok()){
 
    ros::spinOnce();               // check for incoming messages
 
		sub = n.subscribe("/robot_pose", 10, GoalCallback);

    r.sleep();
  }
}
