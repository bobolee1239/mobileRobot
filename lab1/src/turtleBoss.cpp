/* turtleBoss.cpp */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#ifndef _VEL
#define VEL 3.0
#define ANG 1.570796327
#endif

int main(int argc, char* argv[])
{
	/* intialize ros */
	ros::init(argc, argv, "turtle_boss");

	ros::NodeHandle n;
	ros::Publisher turtleBoss = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 
																	100);
	ros::Rate loop_rate(1.0);

	unsigned short count = 0; 
	
	while(ros::ok()){
		geometry_msgs::Twist msg;

		if(++count % 2){
			msg.linear.x  = VEL;
			msg.linear.y  = 0;
			msg.linear.z  = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
			msg.angular.z = 0;
		} else {
			msg.angular.z = ANG;
			msg.linear.x  = 0;
			msg.linear.y  = 0;
			msg.linear.z  = 0;
			msg.angular.x = 0;
			msg.angular.y = 0;
		}
			
		ROS_INFO("Publish new mesg ...\n");
		turtleBoss.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
