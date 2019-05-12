#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

double x   = 0.0;
double y   = 0.0;
double th  = 0.0;
double vx  = 0.0;
double vy  = 0.0;
double vth = 0.0;

const float pi = 3.1415926;

void callback(const geometry_msgs::Twist& feedbackSpeed) {
  ROS_INFO("x=%f, y=%f, th=%f\n", x, y, th*180/pi);
  vx  = feedbackSpeed.linear.x;
  vy  = feedbackSpeed.linear.y;
  vth = feedbackSpeed.angular.z;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle np, ns;

  //  feedback_wheel_speed
  ros::Subscriber vel_sub = ns.subscribe("feedback_Vel", 50, callback);
  ros::Publisher odom_pub = np.advertise<nav_msgs::Odometry>("odom", 50);

  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(100);

  while (np.ok()) {
    ros::spinOnce();
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    double dx = (vx*cos(th) - vy*sin(th)) * dt;
    double dy = (vx*sin(th) + vy*cos(th)) * dt;
    double dth = vth * dt;

    x  += dx;
    y  += dy;
    th += dth;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);
    //
    nav_msgs::Odometry odom;

    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);
    last_time = current_time;
    r.sleep();
  }
  return 0;
}
