//  Copyright (c) 2019 Tsung-Han Lee
#include <string.h>
#include <math.h>
#include <fstream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

/***********************************************
 ** Map shall be declared in global scope,
 ** so that callback function can see it!
 ***********************************************/
nav_msgs::OccupancyGrid map;

void mapRawCallback(const nav_msgs::OccupancyGrid& msg);

int main(int argc, char* argv[]) {
    //  init ros node and naming the node
    ros::init(argc, argv, "map_cal");

    ros::NodeHandle nh;

    //  publisher to publish to topic: map
    ros::Publisher  pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
    ros::Subscriber sub = nh.subscribe("map_raw", 10, &mapRawCallback);

    //  looping in 10 Hz
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        pub.publish(map);
        rate.sleep();
    }

    return 0;
}

void mapRawCallback(const nav_msgs::OccupancyGrid& msg) {
    map.header.frame_id           = "map";
    map.header.stamp              = ros::Time::now();
    map.info.width                = msg.info.width;
    map.info.height               = msg.info.height;
    map.info.resolution           = msg.info.resolution;
    map.info.origin.position.x    = msg.info.origin.position.x;
    map.info.origin.position.y    = msg.info.origin.position.y;
    map.info.origin.position.z    = msg.info.origin.position.z;
    map.info.origin.orientation.x = msg.info.origin.orientation.x;
    map.info.origin.orientation.y = msg.info.origin.orientation.y;
    map.info.origin.orientation.z = msg.info.origin.orientation.z;

    map.data.resize(map.info.width*map.info.height);

    int idx = 0;
    for (auto&& d : msg.data) {
        map.data[idx++] = d;
    }
}
