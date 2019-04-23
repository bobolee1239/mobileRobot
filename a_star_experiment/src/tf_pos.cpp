//  Copyright (c) 2019 Tsung-Han Brian Lee
/*********************************************************
 ** ---------------- tf_pos.cpp -------------------
 **   AUTHOR    : Tsung-Han Brian Lee
 **   REFERENCE : Revised from Dynamic Systems n
 **               Control Lab
 ** -----------------------------------------------
 *********************************************************/
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define PI              3.14159265358979323

/******************************************
 **     Help Fcn & Structure
 ******************************************/
typedef geometry_msgs::Twist Twist;
typedef nav_msgs::Odometry Odometry;
typedef geometry_msgs::Quaternion Quaternion;

typedef struct {
    double x;       // x direction
    double y;       // y direction
    double th;      // angular
} PlanarInfo;

void handleSetVelocity(const Twist& msg);

/******************************************
 ** Global Variable :
 **   => To be caught in callback fcn
 ******************************************/
volatile PlanarInfo vehicle_vel;

int main(int argc, char* argv[]) {
    /* Init ROS node : odemetry_pub */
    ros::init(argc, argv, "odometry_pub");

    ros::NodeHandle nh;
    tf::TransformBroadcaster odom_broadcaster;

    /* Declaration of ROS Publisher & Subscriber */
    ros::Publisher  odom_pub = nh.advertise<Odometry>("/robot_pose", 50);
    ros::Subscriber sub = nh.subscribe("/feedback_Vel", 100, handleSetVelocity);

    PlanarInfo robot_pos;                   //  record robot position
    ros::Rate loopRate(100.0);              //  looping rate
    ros::Time curTime;                      //  current Time step
    ros::Time prevTime = ros::Time::now();  //  last Time step

    while (ros::ok()) {
        ros::spinOnce();            //  be able to handle callback fcn
        curTime = ros::Time::now();

        double timeFly = (curTime - prevTime).toSec();

        robot_pos.x  += (vehicle_vel.x * cos(robot_pos.th)
                         - vehicle_vel.y * sin(robot_pos.th)) * timeFly;
        robot_pos.y  += (vehicle_vel.x * sin(robot_pos.th)
                         + vehicle_vel.y * cos(robot_pos.th)) * timeFly;
        robot_pos.th += vehicle_vel.th * timeFly;

        if (robot_pos.th >= 2*PI) {
            robot_pos.th -= 2*PI;
        } else if (robot_pos.th < 0) {
            robot_pos.th += 2*PI;
        }

        /**
         ** --- Create quaternion from yaw from 6DOF odometry ---
         **     1. fill in transform information
         **     2. publish the transform over tf
         ** ----------------------------------------------------
         **/
        Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robot_pos.th);

        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp     = curTime;
        odom_trans.header.frame_id  = "map";
        odom_trans.child_frame_id   = "base_link";

        odom_trans.transform.translation.x = robot_pos.x;
        odom_trans.transform.translation.y = robot_pos.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation      = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);

        /**
         ** --- Publish odometry info ove ROS ---
         **  1. Fill in position & velocity info
         **  2. Publish to topic
         **/
        Odometry odom;

        odom.header.stamp     = curTime;
        odom.header.frame_id  = "map";
        odom.child_frame_id   = "base_link";

        odom.pose.pose.position.x    = robot_pos.x;
        odom.pose.pose.position.y    = robot_pos.y;
        odom.pose.pose.position.z    = 0.0;
        odom.pose.pose.orientation.z = robot_pos.th;

        odom.twist.twist.linear.x  = vehicle_vel.x;
        odom.twist.twist.linear.y  = vehicle_vel.y;
        odom.twist.twist.angular.z = vehicle_vel.th;

        odom_pub.publish(odom);

        prevTime = curTime;
        loopRate.sleep();
    }

    return 0;
}

void handleSetVelocity(const Twist& msg) {
    vehicle_vel.x  = msg.linear.x;
    vehicle_vel.y  = msg.linear.y;
    vehicle_vel.th = msg.angular.z;
}
