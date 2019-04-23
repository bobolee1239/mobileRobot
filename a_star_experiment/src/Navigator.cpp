//  Copyright (c) 2019 Tsung-Han Brian Lee
/*********************************************************
 ** -------------- Navigator.cpp -----------------
 **   AUTHOR      : Tsung-Han Brian Lee
 **   DESCRIPTION : Navigator with A* algorithm
 ** -----------------------------------------------
 *********************************************************/
#include <time.h>                       //  random seed
#include <stdlib.h>                     //  random walk
#include "ros/ros.h"                    //  ROS
#include "ros/console.h"                //  To setup verbosity
#include "nav_msgs/OccupancyGrid.h"     //  OccupancyGrid msg
#include "nav_msgs/Odometry.h"          //  Odometry msg
#include "geometry_msgs/PoseStamped.h"  //  PoseStamped msg
#include "geometry_msgs/Point.h"        //  Point msg
#include "tf/tf.h"                      //  Coord Transform
#include "a-star.h"                     //  My A* algo Library
/*--------------------------------------------------------------------*/
#define PI              3.14159265358979323
#define VEHICLE_WIDTH   0.46            // Unit: meter

/* define topic name as following */
#define POSE_TOPIC      "/robot_pose"
#define MAP_TOPIC       "/map"
#define SUBGOAL_TOPIC   "/subgoal_position"
/******************************************
 ** ----- Help Fcn & Structure -----
 ******************************************/
typedef geometry_msgs::Point Point;

typedef struct {
    double x;
    double y;
    double th;
} PlanarInfo;

void handleRecvGoal(const geometry_msgs::PoseStamped& msg);
void buildMap(const nav_msgs::OccupancyGrid& map);
void updatePose(const nav_msgs::Odometry& pose);
/******************************************
 ** ----- Global Variable -----
 **   => To be caught in callback fcn
 ******************************************/
/* Pointer to our Map */
Map* myMap = NULL;          //  to be initialized in callback fcn
volatile PlanarInfo goal;   //  destination and desired pose
/*--------------------------------------------------------------------*/
int main(int argc, char* argv[]) {
    /* random random seed */
    unsigned int seed = time(NULL);

    /* Init ROS node : navigator */
    ros::init(argc, argv, "Navigator");

    /* Setup ROS Verbosity Level */
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle nh;

    ros::Publisher  pubSubgoal = nh.advertise<Point>(SUBGOAL_TOPIC, 1);
    ros::Subscriber subPose    = nh.subscribe(POSE_TOPIC, 10, &updatePose);
    ros::Subscriber subMap     = nh.subscribe(MAP_TOPIC, 10, &buildMap);
    ros::Subscriber subGoal    = nh.subscribe("/move_base_simple/goal",
                                              10, &handleRecvGoal);


    /**
     **  subgoal :
     **     x : x coordinate of destination
     **     y : y coordinate of destination
     **     z : vehicle pose, that is the heading direction
     **         - shall in the range -PI ~ PI
     **         - 87 for not specified
     **/
    geometry_msgs::Point subgoal;       // to be published to SUBGOAL_TOPIC
    ros::Rate loopRate(100.0);          //  Unit: Hz
    while (ros::ok()) {
        ros::spinOnce();                // to be able to fire callback fcn

        /* do nothing if myMap is not yet created */
        if (myMap == NULL)  continue;

        Node dest(goal.x, goal.y);
        ROS_DEBUG_STREAM("[Navigator]: Finding path from "
                          << myMap->dac(*static_cast<Node*>(
                              myMap->at(myMap->getRobot().getPosition())))
                          << " -> " << dest);
        auto path = myMap->aStar(dest);
        /*
         std::cout << "Found path: ";
         for (auto&& node : path) {
             std::cout << static_cast<Node>(node) << " -> ";
         }
         std::cout << std::endl;
         */
        if (path.empty()) {
            ROS_INFO_STREAM("[Navigator]: RANDOM WALK FOR NO PATH FOUND!");
            subgoal.x = myMap->getRobot().getPosition().getX()
                        + static_cast<double>(rand_r(&seed)) / RAND_MAX * myMap->getResolution();
            subgoal.y = myMap->getRobot().getPosition().getY()
                        + static_cast<double>(rand_r(&seed)) / RAND_MAX * myMap->getResolution();
            subgoal.z = 87;
        } else if (path.size() > 5) {
            subgoal.x = path[2].getX();
            subgoal.y = path[2].getY();
            subgoal.z = 87;
        } else if (path.size() > 2) {
            subgoal.x = path[2].getX();
            subgoal.y = path[2].getY();
            subgoal.z = goal.th - myMap->getRobot().getPose();
            /* limit z in the range -PI ~ PI */
            if (subgoal.z > PI) {
                subgoal.z -= 2*PI;
            } else if (subgoal.z < -PI) {
                subgoal.z += 2*PI;
            }
        } else {
            /* we're now in the goal */
            subgoal.x = myMap->getRobot().getPosition().getX();
            subgoal.y = myMap->getRobot().getPosition().getY();
            subgoal.z = 87;
        }

        pubSubgoal.publish(subgoal);

        ROS_INFO_STREAM("[Navigator]: subgoal ("
                        << subgoal.x << "," << subgoal.y << ")");

        loopRate.sleep();
    }

    return 0;
}
/*--------------------------------------------------------------------*/
/******************************************
 ** ----- Help Fcn Implementation -----
 ******************************************/
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
    /**
     ** ----------------------------------------------
     **   1. Create our map for the very first time
     **   2. Update our map if myMap is existed
     ** ----------------------------------------------
     **/
    if (myMap) {
        myMap->updateMap(map);
    } else {
        myMap = new Map(map, VEHICLE_WIDTH);
    }
}

/**
 **  Update the vehicle pose
 **/
void updatePose(const nav_msgs::Odometry& loc) {
    /* do nothing if map is not created yet */
    if (myMap == NULL)  return;

    /* update postion */
    myMap->moveRobotTo(loc.pose.pose.position.x, loc.pose.pose.position.y);
    /* update pose */
    myMap->getRobot().setPose(loc.pose.pose.orientation.z);

    ROS_DEBUG_STREAM("[Navigator]: bot pos:" << myMap->dac(*static_cast<Node*>(
                      myMap->at(myMap->getRobot().getPosition())))
                      << " @ " << myMap->getRobot().getPose() / PI * 180.0);
}
