//  Copyright (c) 2019 Tommy Huang, Tsung-Han Brian Lee
/***************************************************************
 ** ---------------- Mobile Vehicle Control -------------------
 **   AUTHOR      : Tommy Huang, Tsung-Han Brian Lee
 **   DESCRIPTION : Revised from Dynamic Systems n Control Lab
 ** -----------------------------------------------------------
 ***************************************************************/
#include <math.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define PI 3.14159265358979323      // circumference ratio.

// [TODO] ... adjusting the control gain !! ...
const double k_rho      = 1.0;
const double k_alpha    = 2.60;
const double k_beta     = 1.0;

/************************************************
 **  Global variables to allow communication
 **  between functions
 ************************************************/
geometry_msgs::Twist command;

float x_now;
float y_now;
float th_now;
/************************************************
 **  Callback Functions
 ************************************************/
void subgoalCallback(const geometry_msgs::Point &msg);
void poseCallback(const nav_msgs::Odometry &location);

int main(int argc, char* argv[]) {
    /* Setup ROS Verbosity Level */
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    /**
     ** Initializing the ROS nh : "nav"
     **/
    ros::init(argc, argv, "nav");
    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber sub2;

    /**
     ** Publish robot movement
     **     => advertise topic
     **/
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    /**
     ** Subscribe robot pose and the reference signal
     **     => install callback function
     **/
    sub = nh.subscribe("/subgoal_position", 10, subgoalCallback);
    sub2 = nh.subscribe("/robot_pose", 10, poseCallback);

    /* Looping in Fs : 100 Hz */
    ros::Rate rate(100);

    while (ros::ok()) {
        /* Attempting to fire callback */
        ros::spinOnce();

        /* Publish control command message */
        pub.publish(command);

        /* constant looping in 100 Hz */
        rate.sleep();
    }

    return 0;
}

void subgoalCallback(const geometry_msgs::Point &msg) {
    /* initializing command ? */
    command.linear.x  = 0.0;
    command.linear.y  = 0.0;
    command.angular.z = 0.0;

    /*********************************************
	 ** Apply control law following
     *********************************************/
    double xDiff = static_cast<double>(msg.x - x_now);
    double yDiff = static_cast<double>(msg.y - y_now);
    double pDiff = static_cast<double>(msg.z);          //  pose angle diff

    /**
     **  Euclidean distance between robot and target
     **/
    double distance = sqrt(pow(xDiff, 2) + pow(yDiff, 2));
    /**
     **  Angle difference between heading direction and target
     **/
    double headingAngleDiff = atan2(yDiff, xDiff) - th_now;
    /* wrapping the angle between -PI ~ PI */
    if (headingAngleDiff > PI) {
        headingAngleDiff -= 2*PI;
    } else if (headingAngleDiff < -PI) {
        headingAngleDiff += 2*PI;
    }

    double linear_vel  = k_rho    * distance;
    double angular_vel = k_alpha * headingAngleDiff;

    /* if the pose angle is setup */
    if (pDiff - 87 > 0.001) {
        angular_vel += k_beta * pDiff;
    }

    /**
     ** 1. Saturating linear velocity
     ** 2. Setting control command
     **/
    if (linear_vel > 0.25) {
        command.linear.x = 0.25;
    } else if (linear_vel < -0.25) {
        command.linear.x = -0.25;
    } else {
        command.linear.x = linear_vel;
    }
    /**
     ** 1. Saturating angular velocity
     ** 2. Setting control command
     **/
    if (angular_vel > 1.0) {
        command.angular.z = 1.0;
    } else if (angular_vel < -1.0) {
        command.angular.z = -1.0;
    } else {
        command.angular.z = angular_vel;
    }
    /**
     **  Handle when close to goal
     **     => in 1 mm
     **/
    if (distance < 0.05) {
        ROS_INFO_STREAM("\n[NAV] WE ARE CLOSE ENOUGH!\n");
        command.angular.z = 0.0;
        command.linear.x  = 0.0;
    }

    ROS_INFO_STREAM("-----------------------------------");
    ROS_INFO_STREAM("[NAV] x_now: " << x_now);
    ROS_INFO_STREAM("[NAV] y_now: " << y_now);
    ROS_INFO_STREAM("[NAV] th_now: " << th_now/PI*180.0);
    ROS_INFO_STREAM("[NAV] xDiff: " << xDiff);
    ROS_INFO_STREAM("[NAV] yDiff: " << yDiff);
    ROS_INFO_STREAM("[NAV] distance: " << distance);
    ROS_INFO_STREAM("[NAV] heading angle diff: " << headingAngleDiff/PI*180.0);
    ROS_INFO_STREAM("[NAV] linear_vel: " << linear_vel);
    ROS_INFO_STREAM("[NAV] angular_vel: " << angular_vel);
}

void poseCallback(const nav_msgs::Odometry &loc) {
    //  Update robot position
    x_now  = loc.pose.pose.position.x;
    y_now  = loc.pose.pose.position.y;
    th_now = loc.pose.pose.orientation.z;
}
