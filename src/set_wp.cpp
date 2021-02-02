#include <gnc_functions.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
    init_publisher_subscriber(gnc_node);
	ros::Rate rate(2.0);
	while(1)
	{
		set_destination_lla(34.10777843135508, -99.10721725576575, 300, 45);
		rate.sleep();
	}
    
	// auto_set_current_waypoint(3);
}