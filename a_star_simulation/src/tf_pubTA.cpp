#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

const float PI = 3.14159265358979323;

ros::Subscriber sub;

void VelCallback(const geometry_msgs::Twist &msg)
{
	vx = msg.linear.x;
	vy = msg.linear.y;
	vth = msg.angular.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");
 
  ros::NodeHandle n;
  tf::TransformBroadcaster odom_broadcaster;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("robot_pose", 50);  

  double x = -3;
  double y = 3;
  double th = 0.0;
 
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
 
  ros::Rate r(100.0);
  while(n.ok()){
 
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
 
		sub = n.subscribe("cmd_vel", 100, VelCallback);

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
 
    x += delta_x;
    y += delta_y;
    th += delta_th;

		if(th>=(2*PI)){
			th = th - 2*PI;
		}
		else if(th<(0)){
			th = th + 2*PI;
		}
 
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
 
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";
 
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
 
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

	//publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = th;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
