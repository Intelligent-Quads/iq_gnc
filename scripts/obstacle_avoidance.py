#! /usr/bin/env python
import rospy
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *
from sensor_msgs.msg import LaserScan
from math import cos, sin, pow, radians, sqrt

drone = gnc_api()


def scan_cb(msg):
    current_scan = msg
    avoidance_x = avoidance_y = 0.0
    avoid = False

    for i in range(1, len(current_scan.ranges)):
        d0 = 3
        k = 0.5
        if .35 < current_scan.ranges[i] < d0:
            avoid = True
            x = cos(current_scan.angle_increment * i)
            y = sin(current_scan.angle_increment * i)
            u = -0.5 * k * pow(((1/current_scan.ranges[i]) - (1/d0)), 2)

            avoidance_x += x*u
            avoidance_y += y*u

    current_heading = drone.current_heading_g
    avoidance_x = avoidance_x * \
        cos(radians(current_heading)) - \
        avoidance_y * sin(radians(current_heading))
    avoidance_y = avoidance_x * \
        sin(radians(current_heading)) + \
        avoidance_y * cos(radians(current_heading))

    if avoid:
        dist = sqrt(pow(avoidance_x, 2) + pow(avoidance_y, 2))
        if dist > 3:
            avoidance_x = 3 * (avoidance_x/dist)
            avoidance_y = 3 * (avoidance_y/dist)

        current_pose = drone.get_current_location()
        drone.set_destination(x=avoidance_x + current_pose.x,
                              y=avoidance_y + current_pose.y,
                              z=2, psi=0)


def main():
    rospy.init_node("drone_controller", anonymous=True)
    rospy.Subscriber(name="/laser/scan", data_class=LaserScan,
                     queue_size=1, callback=scan_cb)

    drone.wait4connect()
    drone.wait4start()
    drone.takeoff(2)
    drone.set_destination(0, 0, 2, 0)

    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
