#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>


/**
\defgroup control_functions
This module is designed to make high level control programming more simple. 
*/


mavros_msgs::State current_state_g;
nav_msgs::Odometry current_pose_g;
geometry_msgs::Pose correction_vector_g;
geometry_msgs::Point local_offset_pose_g;
geometry_msgs::PoseStamped waypoint_g;

float current_heading_g;
float local_offset_g;
float correction_heading_g = 0;
float local_desired_heading_g; 



ros::Publisher local_pos_pub;
ros::Subscriber currentPos;
ros::Subscriber state_sub;
ros::ServiceClient arming_client;
ros::ServiceClient land_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient takeoff_client;
ros::ServiceClient command_client;
/**
\ingroup control_functions
This structure is a convenient way to format waypoints
*/
struct gnc_api_waypoint{
	float x; ///< distance in x with respect to your reference frame
	float y; ///< distance in y with respect to your reference frame
	float z; ///< distance in z with respect to your reference frame
	float psi; ///< rotation about the third axis of your reference frame
};

//get armed state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_g = *msg;
}
geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu)
{
  float x = current_pose_enu.pose.pose.position.x;
  float y = current_pose_enu.pose.pose.position.y;
  float z = current_pose_enu.pose.pose.position.z;
  float deg2rad = (M_PI/180);
  geometry_msgs::Point current_pos_local;
  current_pos_local.x = x*cos((local_offset_g - 90)*deg2rad) - y*sin((local_offset_g - 90)*deg2rad);
  current_pos_local.y = x*sin((local_offset_g - 90)*deg2rad) + y*cos((local_offset_g - 90)*deg2rad);
  current_pos_local.z = z;

  return current_pos_local;

  //ROS_INFO("Local position %f %f %f",X, Y, Z);
}
//get current position of drone
void pose_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pose_g = *msg;
  enu_2_local(current_pose_g);
  float q0 = current_pose_g.pose.pose.orientation.w;
  float q1 = current_pose_g.pose.pose.orientation.x;
  float q2 = current_pose_g.pose.pose.orientation.y;
  float q3 = current_pose_g.pose.pose.orientation.z;
  float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) );
  //ROS_INFO("Current Heading %f ENU", psi*(180/M_PI));
  //Heading is in ENU
  //IS YAWING COUNTERCLOCKWISE POSITIVE?
  current_heading_g = psi*(180/M_PI) - local_offset_g;
  //ROS_INFO("Current Heading %f origin", current_heading_g);
  //ROS_INFO("x: %f y: %f z: %f", current_pose_g.pose.pose.position.x, current_pose_g.pose.pose.position.y, current_pose_g.pose.pose.position.z);
}
geometry_msgs::Point get_current_location()
{
	geometry_msgs::Point current_pos_local;
	current_pos_local = enu_2_local(current_pose_g);
	return current_pos_local;

}
float get_current_heading()
{
	return current_heading_g;
}


//set orientation of the drone (drone should always be level) 
// Heading input should match the ENU coordinate system
/**
\ingroup control_functions
This function is used to specify the drone’s heading in the local reference frame. Psi is a counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_heading(float heading)
{
  local_desired_heading_g = heading; 
  heading = heading + correction_heading_g + local_offset_g;
  
  ROS_INFO("Desired Heading %f ", local_desired_heading_g);
  float yaw = heading*(M_PI/180);
  float pitch = 0;
  float roll = 0;

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  waypoint_g.pose.orientation.w = qw;
  waypoint_g.pose.orientation.x = qx;
  waypoint_g.pose.orientation.y = qy;
  waypoint_g.pose.orientation.z = qz;
}
// set position to fly to in the local frame
/**
\ingroup control_functions
This function is used to command the drone to fly to a waypoint. These waypoints should be specified in the local reference frame. This is typically defined from the location the drone is launched. Psi is counter clockwise rotation following the drone’s reference frame defined by the x axis through the right side of the drone with the y axis through the front of the drone. 
@returns n/a
*/
void set_destination(float x, float y, float z, float psi)
{
	set_heading(psi);
	//transform map to local
	float deg2rad = (M_PI/180);
	float Xlocal = x*cos((correction_heading_g + local_offset_g - 90)*deg2rad) - y*sin((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Ylocal = x*sin((correction_heading_g + local_offset_g - 90)*deg2rad) + y*cos((correction_heading_g + local_offset_g - 90)*deg2rad);
	float Zlocal = z;

	x = Xlocal + correction_vector_g.position.x + local_offset_pose_g.x;
	y = Ylocal + correction_vector_g.position.y + local_offset_pose_g.y;
	z = Zlocal + correction_vector_g.position.z + local_offset_pose_g.z;
	ROS_INFO("Destination set to x: %f y: %f z: %f origin frame", x, y, z);

	waypoint_g.pose.position.x = x;
	waypoint_g.pose.position.y = y;
	waypoint_g.pose.position.z = z;

	local_pos_pub.publish(waypoint_g);
	
}
/**
\ingroup control_functions
Wait for connect is a function that will hold the program until communication with the FCU is established.
@returns 0 - connected to fcu 
@returns -1 - failed to connect to drone
*/
int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	// wait for FCU connection
	while (ros::ok() && !current_state_g.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state_g.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
	
}
/**
\ingroup control_functions
Wait for strat will hold the program until the user signals the FCU to enther mode guided. This is typically done from a switch on the safety pilot’s remote or from the ground control station.
@returns 0 - mission started
@returns -1 - failed to start mission
*/
int wait4start()
{
	ROS_INFO("Waiting for user to set mode to GUIDED");
	while(ros::ok() && current_state_g.mode != "GUIDED")
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
  	}
  	if(current_state_g.mode == "GUIDED")
	{
		ROS_INFO("Mode set to GUIDED. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}
/**
\ingroup control_functions
This function will create a local reference frame based on the starting location of the drone. This is typically done right before takeoff. This reference frame is what all of the the set destination commands will be in reference to.
@returns 0 - frame initialized
*/
int initialize_local_frame()
{
	//set the orientation of the local reference frame
	ROS_INFO("Initializing local coordinate system");
	local_offset_g = 0;
	for (int i = 1; i <= 30; i++) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		

		float q0 = current_pose_g.pose.pose.orientation.w;
		float q1 = current_pose_g.pose.pose.orientation.x;
		float q2 = current_pose_g.pose.pose.orientation.y;
		float q3 = current_pose_g.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2) + pow(q3,2))) ); // yaw

		local_offset_g += psi*(180/M_PI);

		local_offset_pose_g.x = local_offset_pose_g.x + current_pose_g.pose.pose.position.x;
		local_offset_pose_g.y = local_offset_pose_g.y + current_pose_g.pose.pose.position.y;
		local_offset_pose_g.z = local_offset_pose_g.z + current_pose_g.pose.pose.position.z;
		// ROS_INFO("current heading%d: %f", i, local_offset_g/i);
	}
	local_offset_pose_g.x = local_offset_pose_g.x/30;
	local_offset_pose_g.y = local_offset_pose_g.y/30;
	local_offset_pose_g.z = local_offset_pose_g.z/30;
	local_offset_g /= 30;
	ROS_INFO("Coordinate offset set");
	ROS_INFO("the X' axis is facing: %f", local_offset_g);
	return 0;
}

int arm()
{
	//intitialize first waypoint of mission
	set_destination(0,0,0,0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
		return 0;
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}
}

/**
\ingroup control_functions
The takeoff function will arm the drone and put the drone in a hover above the initial position. 
@returns 0 - nominal takeoff 
@returns -1 - failed to arm 
@returns -2 - failed to takeoff
*/
int takeoff(float takeoff_alt)
{
	//intitialize first waypoint of mission
	set_destination(0,0,takeoff_alt,0);
	for(int i=0; i<100; i++)
	{
		local_pos_pub.publish(waypoint_g);
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	// arming
	ROS_INFO("Arming drone");
	mavros_msgs::CommandBool arm_request;
	arm_request.request.value = true;
	while (!current_state_g.armed && !arm_request.response.success && ros::ok())
	{
		ros::Duration(.1).sleep();
		arming_client.call(arm_request);
		local_pos_pub.publish(waypoint_g);
	}
	if(arm_request.response.success)
	{
		ROS_INFO("Arming Successful");	
	}else{
		ROS_INFO("Arming failed with %d", arm_request.response.success);
		return -1;	
	}

	//request takeoff
	
	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = takeoff_alt;
	if(takeoff_client.call(srv_takeoff)){
		sleep(3);
		ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
	}else{
		ROS_ERROR("Failed Takeoff");
		return -2;
	}
	sleep(2);
	return 0; 
}
/**
\ingroup control_functions
This function returns an int of 1 or 0. THis function can be used to check when to request the next waypoint in the mission. 
@return 1 - waypoint reached 
@return 0 - waypoint not reached
*/
int check_waypoint_reached(float pos_tolerance=0.3, float heading_tolerance=0.01)
{
	local_pos_pub.publish(waypoint_g);
	
	//check for correct position 
	float deltaX = abs(waypoint_g.pose.position.x - current_pose_g.pose.pose.position.x);
    float deltaY = abs(waypoint_g.pose.position.y - current_pose_g.pose.pose.position.y);
    float deltaZ = 0; //abs(waypoint_g.pose.position.z - current_pose_g.pose.pose.position.z);
    float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
    // ROS_INFO("dMag %f", dMag);
    // ROS_INFO("current pose x %F y %f z %f", (current_pose_g.pose.pose.position.x), (current_pose_g.pose.pose.position.y), (current_pose_g.pose.pose.position.z));
    // ROS_INFO("waypoint pose x %F y %f z %f", waypoint_g.pose.position.x, waypoint_g.pose.position.y,waypoint_g.pose.position.z);
    //check orientation
    float cosErr = cos(current_heading_g*(M_PI/180)) - cos(local_desired_heading_g*(M_PI/180));
    float sinErr = sin(current_heading_g*(M_PI/180)) - sin(local_desired_heading_g*(M_PI/180));
    
    float headingErr = sqrt( pow(cosErr, 2) + pow(sinErr, 2) );

    // ROS_INFO("current heading %f", current_heading_g);
    // ROS_INFO("local_desired_heading_g %f", local_desired_heading_g);
    // ROS_INFO("current heading error %f", headingErr);

    if( dMag < pos_tolerance && headingErr < heading_tolerance)
	{
		return 1;
	}else{
		return 0;
	}
}
/**
\ingroup control_functions
this function changes the mode of the drone to a user specified mode. This takes the mode as a string. ex. set_mode("GUIDED")
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int set_mode(std::string mode)
{
	mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = mode.c_str();
    if(set_mode_client.call(srv_setMode)){
      ROS_INFO("setmode send ok");
	  return 0;
    }else{
      ROS_ERROR("Failed SetMode");
      return -1;
    }
}

/**
\ingroup control_functions
this function changes the mode of the drone to land
@returns 1 - mode change successful
@returns 0 - mode change not successful
*/
int land()
{
  mavros_msgs::CommandTOL srv_land;
  if(land_client.call(srv_land) && srv_land.response.success)
  {
    ROS_INFO("land sent %d", srv_land.response.success);
    return 0;
  }else{
    ROS_ERROR("Landing failed");
    return -1;
  }
}
/**
\ingroup control_functions
This function is used to change the speed of the vehicle in guided mode. it takes the speed in meters per second as a float as the input
@returns 0 for success
*/
int set_speed(float speed__mps)
{
	mavros_msgs::CommandLong speed_cmd;
	speed_cmd.request.command = 178;
	speed_cmd.request.param1 = 1; // ground speed type 
	speed_cmd.request.param2 = speed__mps;
	speed_cmd.request.param3 = -1; // no throttle change
	speed_cmd.request.param4 = 0; // absolute speed
	ROS_INFO("setting speed to %f", speed__mps);
	if(command_client.call(speed_cmd))
	{
		ROS_INFO("change speed command succeeded %d", speed_cmd.response.success);
		return 0;
	}else{
		ROS_ERROR("change speed command failed %d", speed_cmd.response.success);
		ROS_ERROR("change speed result was %d ", speed_cmd.response.result);
		return -1;
	}
	ROS_INFO("change speed result was %d ", speed_cmd.response.result);
	return 0;
}
/**
\ingroup control_functions
This function is called at the beginning of a program and will start of the communication links to the FCU. The function requires the program's ros nodehandle as an input 
@returns n/a
*/
int init_publisher_subscriber(ros::NodeHandle controlnode)
{
	std::string ros_namespace;
	if (!controlnode.hasParam("namespace"))
	{

		ROS_INFO("using default namespace");
	}else{
		controlnode.getParam("namespace", ros_namespace);
		ROS_INFO("using namespace %s", ros_namespace.c_str());
	}
	local_pos_pub = controlnode.advertise<geometry_msgs::PoseStamped>((ros_namespace + "/mavros/setpoint_position/local").c_str(), 10);
	currentPos = controlnode.subscribe<nav_msgs::Odometry>((ros_namespace + "/mavros/global_position/local").c_str(), 10, pose_cb);
	state_sub = controlnode.subscribe<mavros_msgs::State>((ros_namespace + "/mavros/state").c_str(), 10, state_cb);
	arming_client = controlnode.serviceClient<mavros_msgs::CommandBool>((ros_namespace + "/mavros/cmd/arming").c_str());
	land_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/land").c_str());
	set_mode_client = controlnode.serviceClient<mavros_msgs::SetMode>((ros_namespace + "/mavros/set_mode").c_str());
	takeoff_client = controlnode.serviceClient<mavros_msgs::CommandTOL>((ros_namespace + "/mavros/cmd/takeoff").c_str());
	command_client = controlnode.serviceClient<mavros_msgs::CommandLong>((ros_namespace + "/mavros/cmd/command").c_str());
	return 0;
}