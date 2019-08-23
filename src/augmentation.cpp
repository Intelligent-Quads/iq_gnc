#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "augmentation");
	ros::NodeHandle aug_node;

	state_sub = aug_node.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	state_sub = aug_node.subscribe<sensor_msgs::Range>("/mavros/rangefinder/rangefinder", 10, range_cb);





	//request takeoff
	takeoff(3);
