//  Copyright 2019 Tsung-Han Lee
/**********************************************************
 ** FILE        : a_star.cpp
 ** DESCRIPTION :
 **     - Algorithms to find the shortest path
 **********************************************************/
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "a_star.h"

#define POSE_TOPIC     "/robot_pose"
#define MAP_TOPIC      "/map"
#define SUBGOAL_TOPIC  "/subgoal_position"

void buildMap(const nav_msgs::OccupancyGrid& map);
void updatePose(const nav_msgs::Odometry& pose);
Node& coordTranslation(Node& curPos);


int main(int argc, char* argv[]) {
    ros::NodeHandle nh;

    ros::Publisher  pubSubgoal = nh.advertis<geometry_msgs::Point>(SUBGOAL_TOPIC, 1);
    ros::Subscriber subPose    = nh.subscribe(POSE_TOPIC, 10, &updatePose);
    ros::Subscriber subMap     = nh.subscribe(MAP_TOPIC, 10, &buildMap);

    geometry_msgs::Point subgoal(0.0, 0.0, 0.0);

    /* Map and something */
    Map* myMap = NULL;         //  to be initialized in callback function

    /* looping in 10 Hz */
    ros::Rate rate(10);

    while (ros::ok()) {
        /* receive callback fcn */
        ros::spinOnce();

        /* publish message to topic */


        rate.sleep();
    }

    return 0;
}

/**
 **  Build & Update our map
 **/
void buildMap(const nav_msgs::OccupancyGrid& map) {
    if (myMap) {
        /* update our map for the following circumstance */
        myMap.updateMap(map);
    } else {
        /* build our map for the very first time */
        myMap = new Map(map);
    }
}

/**
 **  Update the vehicle pose 
 **/
void updatePose(const nav_msgs::Odometry& pose) {
}

/**
 **  Transform the objective target to our map coordinate
 **/
Node& coordTranslation(Node& curPos) {

}
