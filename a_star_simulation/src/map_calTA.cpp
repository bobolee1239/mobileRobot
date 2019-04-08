#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "math.h"

ros::Publisher pub;
ros::Subscriber sub;

nav_msgs::OccupancyGrid map;

void MapRawCallback(const nav_msgs::OccupancyGrid &msg){ 
//Subscribe map_rae information	
	map.header.frame_id="map";		//set ground coordinate
  map.header.stamp = ros::Time::now(); 
  map.info.resolution = msg.info.resolution;         
  map.info.width      = msg.info.width;           
  map.info.height     = msg.info.height;           
  map.info.origin.position.x = msg.info.origin.position.x;
  map.info.origin.position.y = msg.info.origin.position.y;
  map.info.origin.position.z = msg.info.origin.position.z;
  map.info.origin.orientation.x = msg.info.origin.orientation.x;
  map.info.origin.orientation.y = msg.info.origin.orientation.y;
  map.info.origin.orientation.z = msg.info.origin.orientation.z;
  map.info.origin.orientation.w = msg.info.origin.orientation.w;	

	map.data.resize(map.info.width * map.info.height);

	for(int i = 0; i < map.info.width * map.info.height; i++){
			map.data[i] = msg.data[i];
	}

}

int main(int argc, char * argv[]) {

  ros::init(argc, argv, "map_cal");

  ros::NodeHandle nh;

  pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
	sub = nh.subscribe("map_raw", 10, MapRawCallback);

	ros::Rate rate(10);
	
  while (ros::ok())
  {
			ros::spinOnce();
      pub.publish(map);
			rate.sleep();
  }

  return 0;
}

