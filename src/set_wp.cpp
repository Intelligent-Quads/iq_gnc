#include <gnc_functions.hpp>
//include API 
int main(int argc, char** argv)
{
    //initialize ros 
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");
    init_publisher_subscriber(gnc_node);
  
    ros::Rate rate(2.0);
    wait4connect();
    wait4start();
    arm();
    takeoff_global(30.2, 80.5, 5);

    while(ros::ok())
    {
        set_yaw(10, 20, -1, 1);
        set_destination_lla_raw(-35.364261,149.165230, 5, 45);
        rate.sleep();
    }    
} 