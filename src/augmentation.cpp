#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <deque>
#include <bell_functions.hpp>
#include <std_msgs/Float64.h>

float altitude;
float velocityErrSumX = 0;
float velocityErrSumY = 0;
float TARGET_VELOCITY_X = 0;
float TARGET_VELOCITY_Y = 0.1;
std::deque<sensor_msgs::Range> range_hist_g;


struct throttle_stamp{
	int throttle;
	double time;
};
std::deque<throttle_stamp> throttle_hist_g;

struct velocity_stamp{
	float velocity;
	double time;
};
std::deque<velocity_stamp> vel_x_hist_g;
std::deque<velocity_stamp> vel_y_hist_g;

void vely_cmd_cb(const std_msgs::Float64::ConstPtr& msg)
{
	TARGET_VELOCITY_Y = msg->data;
	ROS_INFO("desired velocity Y %f", TARGET_VELOCITY_Y);
}
void range_cb(const sensor_msgs::Range::ConstPtr& msg)
{
	sensor_msgs::Range range_msg = *msg;
 	altitude = msg->range;
 	altitude=altitude*1000; //meters to millimeters
 	//ROS_INFO("alt %f", altitude);
 	range_hist_g.push_front(range_msg);
 	if(range_hist_g.size() > 4)
 	{
 		range_hist_g.pop_back();
 	}
}
float constrain(float value, float min, float max)
{
	if(value > max)
	{
		value = max;
	}
	if(value < min)
	{
		value = min;
	}
	return value;
}
//not used rn
float calc_zvel()
{
	if(range_hist_g.size() == 4)
	{
		float sumdzdt = 0;
		for(int j=0; j<range_hist_g.size(); j++)
    	{
        	sumdzdt = (range_hist_g[j].range - range_hist_g[j+1].range)/(range_hist_g[j].header.stamp.toSec() - range_hist_g[j+1].header.stamp.toSec()) + sumdzdt;
    	}
    	sumdzdt = sumdzdt/4;
    	//ROS_INFO("Z velocity %f", sumdzdt);
    	return sumdzdt;
	}
	return -1;
} 
float throttle_d()
{
	if(throttle_hist_g.size() == 4)
	{
		float sumdThrottle = 0;
		for(int j=0; j<throttle_hist_g.size(); j++)
    	{
        	sumdThrottle = (throttle_hist_g[j].throttle - throttle_hist_g[j+1].throttle)/(throttle_hist_g[j].time - throttle_hist_g[j+1].time) + sumdThrottle;
    	}
    	sumdThrottle = sumdThrottle/4;
    	//ROS_INFO("d throttle %f", sumdThrottle);
    	return sumdThrottle;
	}else{
		return 0;
	}

}
float vel_d(std::deque<velocity_stamp> vel)
{
	
	if(vel.size() == 4)
	{
		float sumdVel = 0;
		for(int j=0; j<vel.size(); j++)
    	{
        	sumdVel = (vel[j].velocity - vel[j+1].velocity)/(vel[j].time - vel[j+1].time) + sumdVel;
    	}
    	sumdVel = sumdVel/4;
    	//ROS_INFO("d velocity %f", sumdVel);
    	return sumdVel;
	}else{
		return 0;
	}

}

mavros_msgs::OverrideRCIn altitude_loop()
{

	///////////////////////////////////
	// CONTROL PARAMETERS
	//parameters ALT_HOLD_MODE
	float TARGET_ALT = 4000; //mm
	float THROTTLE_HOVER = 1500;
	float ALTITUDE_P = 0.8;
	float ALTITUDE_D = 0.2;
	int THROTTLE_AUTHORITY = 50; // total amount of throttle auto is allowed away from hover throttle   

	//parameters POS_HOLD_MODE
	float VELOCITY_XY_P = 75;
	float VELOCITY_XY_I = 5;
	float VELOCITY_XY_D = 5;
	float VELOCITY_ERR_MAX = 10;
	int ROLL_AUTHORITY = 150;
	int PITCH_AUTHORITY = 150;
	int ROLL_THROTTLE_LEVEL = 1500;
	int PITCH_THROTTLE_LEVEL = 1500;
	///////////////////////////////////

	//Control logic
	int autoThrottle;
	float altErr = TARGET_ALT - altitude;
	//ROS_INFO("Alt err %f", altErr);
	autoThrottle = (int)((altErr*ALTITUDE_P) + THROTTLE_HOVER);

    
    // generate time stamp
    ros::Time currentTime = ros::Time::now();
  	double currentTime_d = currentTime.toSec();
  	
  	// package throttle value and stamp
  	throttle_stamp current_throttle;
  	current_throttle.throttle = autoThrottle;
  	current_throttle.time = currentTime_d;

    throttle_hist_g.push_front(current_throttle);
    if(throttle_hist_g.size() > 4 )
    {
    	throttle_hist_g.pop_back();
    }

    float dThrottle = throttle_d();

    autoThrottle = (int)(dThrottle*ALTITUDE_D) + autoThrottle;

    //Limit Throttle
    autoThrottle = constrain(autoThrottle, THROTTLE_HOVER - THROTTLE_AUTHORITY, THROTTLE_HOVER + THROTTLE_AUTHORITY);



    //position hold logic begin 
	geometry_msgs::Point current_velocity;
    current_velocity = get_current_velocity();
    //format like optical flow
    float xVel = current_velocity.x;
    float yVel = current_velocity.y;

	float velocityErrX = TARGET_VELOCITY_X - xVel;
	float velocityErrY = TARGET_VELOCITY_Y - yVel;

	int autoThrottlePitch = (int)((-velocityErrY*VELOCITY_XY_P) + PITCH_THROTTLE_LEVEL);
	autoThrottlePitch = constrain(autoThrottlePitch, PITCH_THROTTLE_LEVEL - PITCH_AUTHORITY, PITCH_THROTTLE_LEVEL + THROTTLE_AUTHORITY);
	int autoThrottleRoll = (int)((-velocityErrX*VELOCITY_XY_P) + ROLL_THROTTLE_LEVEL);
	autoThrottlePitch = constrain(autoThrottlePitch, PITCH_THROTTLE_LEVEL - PITCH_AUTHORITY, PITCH_THROTTLE_LEVEL + THROTTLE_AUTHORITY);


	//velocity D controller 
	velocity_stamp current_xVel;
  	current_xVel.velocity = xVel;
  	current_xVel.time = currentTime_d;

  	velocity_stamp current_yVel;
  	current_yVel.velocity = yVel;
  	current_yVel.time = currentTime_d;

    vel_y_hist_g.push_front(current_yVel);
    vel_x_hist_g.push_front(current_xVel);
    if(vel_y_hist_g.size() > 4 )
    {
    	vel_y_hist_g.pop_back();
    }

    float accelx = vel_d(vel_x_hist_g);
    float accely = vel_d(vel_y_hist_g);

    autoThrottleRoll = (int)(accelx*VELOCITY_XY_D) + autoThrottleRoll;
    autoThrottlePitch = (int)(accely*VELOCITY_XY_D) + autoThrottlePitch;
 

 	//velocity I controller

    velocityErrSumX = velocityErrX*VELOCITY_XY_I + velocityErrSumX;
    velocityErrSumX = constrain(velocityErrSumX, -VELOCITY_ERR_MAX, VELOCITY_ERR_MAX);
    velocityErrSumY = velocityErrY*VELOCITY_XY_I + velocityErrSumY;
    velocityErrSumY = constrain(velocityErrSumY, -VELOCITY_ERR_MAX, VELOCITY_ERR_MAX);

    autoThrottleRoll = (int)(velocityErrSumX) + autoThrottleRoll;
    autoThrottlePitch = (int)(velocityErrSumY) + autoThrottlePitch;

    ROS_INFO("X: P %f I %f D %f", velocityErrX*VELOCITY_XY_P, velocityErrSumX, accelx*VELOCITY_XY_D);
    ROS_INFO("Y: P %f I %f D %f", velocityErrY*VELOCITY_XY_P, velocityErrSumY, accely*VELOCITY_XY_D);

    ROS_INFO("velocity x: %f y: %f", current_velocity.x, current_velocity.y);
    //ROS_INFO("velocity autoThrottle roll: %d pitch: %d", autoThrottleRoll, autoThrottlePitch);



	//rc output
	mavros_msgs::OverrideRCIn rc_msg;
	rc_msg.channels[0] = (int)autoThrottleRoll;
	rc_msg.channels[1] = (int)autoThrottlePitch;
 	rc_msg.channels[2] = (int)autoThrottle;
 	rc_msg.channels[3] = 1500;
 	rc_msg.channels[4] = 1800;
 	rc_msg.channels[5] = 1100;
 	rc_msg.channels[6] = 1100;
 	rc_msg.channels[7] = 1100;
 	//ROS_INFO("Auto Throttle %d", rc_msg.channels[2]);
 	return rc_msg;
}

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "augmentation");
	ros::NodeHandle aug_node;

	ros::Subscriber range_sub = aug_node.subscribe<sensor_msgs::Range>("/mavros/rangefinder/rangefinder", 1, range_cb);
	ros::Subscriber velY_cmd_sub = aug_node.subscribe<std_msgs::Float64>("/drone_vel_cmd", 1, vely_cmd_cb);
	ros::Publisher rc_pub = aug_node.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);


	init_publisher_subscriber(aug_node);
	
	//create local reference frame 
	initialize_local_frame();

	ros::Rate rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		ros::spinOnce();
		rate.sleep();
		mavros_msgs::OverrideRCIn rc_msg2;
		rc_msg2 = altitude_loop();
		rc_pub.publish(rc_msg2);
	

	}


}
