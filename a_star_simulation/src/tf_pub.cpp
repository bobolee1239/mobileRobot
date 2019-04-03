//  Copyright 2019 MobileRobot
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

const double PI = 3.14159265358979323;

/* var to be updated in callback fcn */
double vx  = 0.0;
double vy  = 0.0;
double vth = 0.0;

void velHandler(const geometry_msgs::Twist& msg);

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("robot_pose", 50);

    tf::TransformBroadcaster odom_broadcaster;

    double x  = -3.0;
    double y  = 3.0;
    double th = 0.0;

    ros::Time curTime  = ros::Time::now();
    ros::Time lastTime = ros::Time::now();

    ros::Rate rate(100.0);

    while(nh.ok()) {
        /* to catch callback */
        ros::spinOnce();

        curTime = ros::Time::now();
        sub = nh.subscribe("cmd_vel", 100, velHandler);

        /******************************************************
         ** Compute odometry in a typical way given the
         ** velocities of the robot
         *******************************************************/

        double dt      = (curTime - lastTime).toSec();
        x  += (vx*cos(th) - vy*sin(th)) * dt;
        y  += (vx*sin(th) - vy*cos(th)) * dt;
        th += vth * dt;

        if (th >= 2*PI) {
            th -= 2*PI;
        } else if (th < 0.0) {
            th += 2*PI;
        }

        /* create quaternion for 6D odemetry msg */
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        /* FIRST, publish the transform over tf*/
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp     = curTime;
        odom_trans.header.frame_id  = "map";
        odom_trans.child_frame_id   = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        /* send the transform */
        odom_broadcaster.sendTransform(odom_trans);

        /* SECOND, publish the odometry over ROS */
        nav_msgs::Odometry odom;
        odom.header.stamp    = curTime;
        odom.header.frame_id = "map";

        //  set the position
        odom.pose.pose.position.x   = x;
        odom.pose.pose.position.y   = y;
        odom.pose.pose.position.z   = 0.0;

        odom.pose.pose.orientation.z   = th;

        // set the velocity
        odom.child_frame_id         = "base_link";
        odom.twist.twist.linear.x   = vx;
        odom.twist.twist.linear.y   = vy;
        odom.twist.twist.linear.z   = vth;

        odom_pub.publish(odom);
        lastTime = curTime;

        rate.sleep();
    }

    return 0;
}

void velHandler(const geometry_msgs::Twist& msg) {
    vx  = msg.linear.x;
    vy  = msg.linear.y;
    vth = msg.angular.z;
}
