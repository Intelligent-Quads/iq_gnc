#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
	}	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_sub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);

	ros::spin();

	return 0;
}

