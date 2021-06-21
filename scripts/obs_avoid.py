#! /usr/bin/env python
# Import ROS.
import rospy
# Import LaserScan message from package sensor_msgs.
from sensor_msgs.msg import LaserScan
# Import the API.
from iq_gnc.py_gnc_functions import *
# Import the needed math functions.
from math import cos, sin, pow, radians, sqrt
# Importing Point message from package geometry_msgs.
from geometry_msgs.msg import Point

# Create an object for the API and making it a global variable.
drone = gnc_api()


def laser_cb(msg):
    # Callback function of the subscriber.
    cr_scan = LaserScan()
    cr_scan = msg
    avoid_x = 0.0
    avoid_y = 0.0
    avoid = False

    for i in range(1, len(cr_scan.ranges)):
        d0 = 3.0
        k = 0.5
        if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 0.35:
            avoid = True
            x = cos(cr_scan.angle_increment * i)
            y = sin(cr_scan.angle_increment * i)
            u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))

            avoid_x += (x*u)
            avoid_y += (y*u)

    # Getting the current_heading of the drone and converting it to radians.
    cr_heading = radians(drone.get_current_heading())
    avoid_x = (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading))
    avoid_y = (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading))

    if avoid:
        dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))

        if dist > 3:
            avoid_x = (3 * (avoid_x/dist))
            avoid_y = (3 * (avoid_y/dist))

        cur_pose = Point()
        # Getting the current location from the drone.
        cur_pose = drone.get_current_location()
        # Sending the goal.
        drone.set_destination(avoid_x + cur_pose.x,
                              avoid_y + cur_pose.y,
                              2, 0)


def main():
    # Initializing the ROS node.
    rospy.init_node("obs_avoider", anonymous=True)
    # Creating a subscriber for the topic '/spur/laser/scan'.
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)

    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 2m.
    drone.takeoff(2)

    # Used to keep the node running.
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
