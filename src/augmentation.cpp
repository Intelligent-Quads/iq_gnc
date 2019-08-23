#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/RCIn.h>

mavros_msgs::State current_state_g;
sensor_msgs::Range altitude;

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}
void range_cb(const mavros_msgs::State::ConstPtr& msg)
{
 	altitude = *msg;
}

altitude_loop()

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "augmentation");
	ros::NodeHandle aug_node;

	state_sub = aug_node.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	range_sub = aug_node.subscribe<sensor_msgs::Range>("/mavros/rangefinder/rangefinder", 10, range_cb);
	rc_pub = aug_node.advertise<mavros_msgs::RCIn>("/mavros/rc/in", 10);
	while(ros::ok())
	{

	}


}
