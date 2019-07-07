#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <gnc_functions.hpp>

// mode_g denotes the flight opperations
//		0 - search
//		1 - rescue 
int mode_g = 0;

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
		if(msg->bounding_boxes[i].Class == "person")
		{
			mode_g = 1; 
			ROS_INFO("Person found. Starting Rescue Operation");
		}
	}	

}


int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(10);


	//specify some waypoints 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	float range = 50;
	float spacing = 10;
	int rows = 5; 
	int row;
	for(int i=0; i<5; i++)
	{
		row = i*2; 
		nextWayPoint.x = row*spacing;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = row*spacing;
		nextWayPoint.y = range;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = (row+1)*spacing;
		nextWayPoint.y = range;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);

		nextWayPoint.x = (row+1)*spacing;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
	}
	
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		if(mode_g == 0)
		{
			ros::spinOnce();
			rate.sleep();
			if(check_waypoint_reached(.3) == 1)
			{
				if (counter < waypointList.size())
				{
					set_destination(waypointList[counter].x,waypointList[counter].y,waypointList[counter].z, waypointList[counter].psi);
					counter++;	
				}else{
					//land after all waypoints are reached
					land();
				}	
			}
		}
		if(mode_g == 1)
		{
			land();
			ROS_INFO("Landing Started");
			break;
		}	
		
	}

	return 0;
}

