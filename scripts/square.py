#! /usr/bin/env python
import rospy
from iq_gnc.py_gnc_functions import *

rospy.init_node('robot', anonymous=True)

drone = gnc_api()
drone.wait4connect()
drone.wait4start()

drone.initialize_local_frame()
drone.takeoff(3)
hz = rospy.Rate(3)

wps = [[0, 0, 3, 0], [5, 0, 3, -90], [5, 5, 3, 0],
       [0, 5, 3, 90], [0, 0, 3, 180], [0, 0, 3, 0]]
goals = iter(wps)
goal = next(goals, False)
try:
    while True:
        drone.set_destination(x=goal[0], y=goal[1], z=goal[2], psi=goal[3])
        hz.sleep()
        if drone.check_waypoint_reached():
            goal = next(goals, False)
            if goal is False:
                drone.land()
                rospy.loginfo("All waypoints reached landing now.")
                break

except KeyboardInterrupt:
    exit()
