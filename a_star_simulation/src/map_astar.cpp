//  Copyright 2019 Tsung-Han Lee
/**********************************************************
 ** FILE        : a_star.cpp
 ** DESCRIPTION :
 **     - Algorithms to find the shortest path
 **********************************************************/
#include <string.h>
#include <iostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "a_star.h"

#define POSE_TOPIC     "/robot_pose"
#define MAP_TOPIC      "/map"
#define SUBGOAL_TOPIC  "/subgoal_position"

void buildMap(const nav_msgs::OccupancyGrid& map);
void updatePose(const nav_msgs::Odometry& pose);

typedef geometry_msgs::Point Point;

/**********************************************************************
 ** GLOBAL SCOPE VARIABLE : to be accessed in callback fcn
 ***********************************************************************/
double th_now = 0.0;
std::vector<Node> goals({Node(3.0, 4.0),
                         Node(3.0, -2.0),
                         Node(-4.0, -3.0),
                         Node(-3.0, -3.0)});
/* Pointer to our Map */
Map* myMap = NULL;         //  to be initialized in callback function
/***********************************************************************/

int main(int argc, char* argv[]) {
    /**
     ** Initializing the ROS nh : "a_star"
     **/
    ros::init(argc, argv, "a_star");
    ros::NodeHandle nh;

    ros::Publisher  pubSubgoal = nh.advertise<Point>(SUBGOAL_TOPIC, 1);
    ros::Subscriber subPose    = nh.subscribe(POSE_TOPIC, 10, &updatePose);
    ros::Subscriber subMap     = nh.subscribe(MAP_TOPIC, 10, &buildMap);

    /* to be published to topic */
    geometry_msgs::Point subgoal;

    /* looping in 10 Hz */
    ros::Rate rate(10);

    while (ros::ok()) {
        /* receive callback fcn */
        ros::spinOnce();

        if (myMap == NULL) continue;
        if (goals.empty()) {
            ROS_INFO_STREAM("[ASTAR] Jobs Done !!");
            continue;
        }
        ROS_DEBUG_STREAM("Find path from "
                        << myMap->dac(*static_cast<Node*>(
                            myMap->at(myMap->getRobot().getPosition())))
                        << " -> " << goals.front());
        auto path = myMap->aStar(goals.front());

        std::cout << "Found path: ";
        for (auto&& node : path) {
            std::cout << static_cast<Node>(node) << " -> ";
        }
        std::cout << std::endl;

        if (path.empty()) {
            continue;
        } else if (path.size() > 3) {
            subgoal.x = path[2].getX();
            subgoal.y = path[2].getY();
        } else {
            subgoal.x = path.front().getX();
            subgoal.y = path.front().getY();
        }
        pubSubgoal.publish(subgoal);
        ROS_INFO_STREAM("[ASTAR] subgoal:("
                        << subgoal.x << ","<< subgoal.y << ")");
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
        myMap->updateMap(map);
    } else {
        /* build our map for the very first time */
        myMap = new Map(map);
    }
}

/**
 **  Update the vehicle pose
 **/
void updatePose(const nav_msgs::Odometry& loc) {
    if (myMap == NULL) return;
    myMap->moveRobotTo(loc.pose.pose.position.x, loc.pose.pose.position.y);
    ROS_INFO_STREAM("[ASTAR] pos:" << myMap->dac(*static_cast<Node*>(
        myMap->at(myMap->getRobot().getPosition()))));
    if ((!goals.empty()) &&
        (goals[0].distanceTo(loc.pose.pose.position.x, loc.pose.pose.position.y) < 0.5)) {
            ROS_INFO_STREAM("[ASTAR]Next Goal");
            goals.erase(goals.begin());
    }
}
