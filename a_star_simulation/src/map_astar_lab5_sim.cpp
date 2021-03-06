//  Copyright 2019 Tsung-Han Lee
/**********************************************************
 ** FILE        : a_star.cpp
 ** DESCRIPTION :
 **     - Algorithms to find the shortest path
 **********************************************************/
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"  //  PoseStamped msg
#include "geometry_msgs/Point.h"
#include "tf/tf.h"                      //  Coord Transform
#include "a-star.h"

#define POSE_TOPIC     "/robot_pose"
#define MAP_TOPIC      "/map"
#define SUBGOAL_TOPIC  "/subgoal_position"
#define VEHICLE_WIDTH  0.2                     //  Unit: meter
#define PI             3.14159265358979323

void buildMap(const nav_msgs::OccupancyGrid& map);
void updatePose(const nav_msgs::Odometry& pose);
void handleRecvGoal(const geometry_msgs::PoseStamped& msg);

typedef geometry_msgs::Point Point;

typedef struct {
    double x;
    double y;
    double th;
} PlanarInfo;
/**********************************************************************
 ** GLOBAL SCOPE VARIABLE : to be accessed in callback fcn
 ***********************************************************************/
/* Pointer to our Map */
Map* myMap = NULL;          //  to be initialized in callback function
volatile PlanarInfo goal;   //  destination and desired pose
/***********************************************************************/

int main(int argc, char* argv[]) {
    /* random random seed */
    unsigned int seed = time(NULL);
    /* Setup ROS Verbosity Level */
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    /**
     ** Initializing the ROS nh : "a_star"
     **/
    ros::init(argc, argv, "a_star");
    ros::NodeHandle nh;

    ros::Publisher  pubSubgoal = nh.advertise<Point>(SUBGOAL_TOPIC, 1);
    ros::Subscriber subPose    = nh.subscribe(POSE_TOPIC, 10, &updatePose);
    ros::Subscriber subMap     = nh.subscribe(MAP_TOPIC, 10, &buildMap);
    ros::Subscriber subGoal    = nh.subscribe("/move_base_simple/goal",
                                              10, &handleRecvGoal);

    /* to be published to topic */
    geometry_msgs::Point subgoal;

    /* looping in 10 Hz */
    ros::Rate rate(10);

    while (ros::ok()) {
        /* receive callback fcn */
        ros::spinOnce();

        if (myMap == NULL) continue;

        Node dest(goal.x, goal.y);
        ROS_DEBUG_STREAM("[Navigator]: Finding path from "
                          << myMap->dac(*static_cast<Node*>(
                              myMap->at(myMap->getRobot().getPosition())))
                          << " -> " << dest);
        auto path = myMap->aStar(dest);

        std::cout << "Found path: ";
        for (auto&& node : path) {
            std::cout << static_cast<Node>(node) << " -> ";
        }
        std::cout << std::endl;

        if (path.empty()) {
            ROS_INFO_STREAM("[ASTAR]: RANDOM WALK FOR NO PATH FOUND!");
            subgoal.x = myMap->getRobot().getPosition().getX()
                        + static_cast<double>(rand_r(&seed)) / RAND_MAX * myMap->getResolution();
            subgoal.y = myMap->getRobot().getPosition().getY()
                        + static_cast<double>(rand_r(&seed)) / RAND_MAX * myMap->getResolution();
            subgoal.z = 87;
        } else if (path.size() > 6) {
            subgoal.x = path[6].getX();
            subgoal.y = path[6].getY();
            subgoal.z = 87;
        } else if (path.size() > 1) {
            subgoal.x = path.back().getX();
            subgoal.y = path.back().getY();
            subgoal.z = goal.th;
        } else {
            /* we're now in the goal */
            subgoal.x = myMap->getRobot().getPosition().getX();
            subgoal.y = myMap->getRobot().getPosition().getY();
            subgoal.z = 87;
        }

        pubSubgoal.publish(subgoal);
        ROS_INFO_STREAM("[ASTAR] subgoal:("
                        << subgoal.x << ","<< subgoal.y << ")");
        rate.sleep();
    }

    return 0;
}

/**
 **  Handle user setting goal event
 **/
void handleRecvGoal(const geometry_msgs::PoseStamped& msg) {
    goal.x  = msg.pose.position.x;
    goal.y  = msg.pose.position.y;
    /*****************************************************
     ** ------------------ NOTE --------------------
     **  1. Theta is transformed from quaternion to
     **     raw pitch yaw (rpy).
     **  2. Range: -PI ~ PI
     **  3. Unit : rad
     ** ---------------------------------------------
     *****************************************************/
    goal.th = tf::getYaw(msg.pose.orientation);

    ROS_DEBUG_STREAM("[Navigator]: Goal (" << goal.x << ", " << goal.y << ") @"
                     << goal.th);
}

/**
 **  Build & Update our map
 **/
void buildMap(const nav_msgs::OccupancyGrid& map) {
    if (myMap) {
        /* update our map for the following circumstance */
        myMap->updateMap(map);
    } else {
        /* build our map for the very first time */
        myMap = new Map(map, VEHICLE_WIDTH);
    }
}

/**
 **  Update the vehicle pose
 **/
void updatePose(const nav_msgs::Odometry& loc) {
    if (myMap == NULL) return;
    myMap->moveRobotTo(loc.pose.pose.position.x, loc.pose.pose.position.y);
    /* update pose */
    myMap->getRobot().setPose(loc.pose.pose.orientation.z);

    ROS_DEBUG_STREAM("[ASTAR] pos:" << myMap->dac(*static_cast<Node*>(
                     myMap->at(myMap->getRobot().getPosition()))));
}
