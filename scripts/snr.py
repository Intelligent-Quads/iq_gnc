#! /usr/bin/env python
# Import ROS.
import rospy
# Importing BoundingBoxes message from package darknet_ros_msgs.
from darknet_ros_msgs.msg import BoundingBoxes
# Import the API.
from iq_gnc.py_gnc_functions import *
# To print colours (optional).
from iq_gnc.PrintColours import *


mode_g = False
# True - Rescue
# False - Search


def detection_cb(msg):
    # Callback function of the subscriber.
    # Assigning bounding_boxes of the message to bbox variable.
    bbox = msg.bounding_boxes
    for i in range(len(bbox)):
        # Printing the detected object with its probability in percentage.
        rospy.loginfo("{}% certain {} detected.".format(
            float(bbox[i].probability * 100), bbox[i].Class))
        if bbox[i].Class == "person":
            global mode_g
            # Setting mode_g to True to mark presence of person.
            mode_g = True
            rospy.loginfo(
                CBLUE + "Person found. Starting Rescue Operation" + CEND)


def main():
    # Initialise ROS node
    rospy.init_node("search_and_rescue_node")
    # Creating a subscriber for the topic "/darknet_ros/bounding_boxes".
    rospy.Subscriber(name="/darknet_ros/bounding_boxes",
                     data_class=BoundingBoxes,
                     callback=detection_cb,
                     queue_size=1)
    # Create an object for the API.
    drone = gnc_api()
    # Wait for FCU connection.
    drone.wait4connect()
    # Wait for the mode to be switched.
    drone.wait4start()
    # Create local reference frame.
    drone.initialize_local_frame()
    # Request takeoff with an altitude of 10m.
    drone.takeoff(10)

    # Specify some waypoints
    wps = []
    # Amount to move in the y-direction (local frame) in the snake like pattern.
    spacing = 10.0
    # No of times the snake like pattern needs to repeat.
    rows = 5
    # Amount to move in the x-direction (local frame) in the snake like pattern.
    drange = 50.0

    for i in range(rows):
        # Creating the snake like pattern and pushing it to the waypoints list.
        row = i * 2
        x = row * spacing
        y = 0
        z = 10
        psi = 0
        wps.append([x, y, z, psi])

        x = row * spacing
        y = drange
        z = 10
        psi = 0
        wps.append([x, y, z, psi])

        x = (row+1) * spacing
        y = drange
        z = 10
        psi = 0
        wps.append([x, y, z, psi])

        x = (row+1) * spacing
        y = 0
        z = 10
        psi = 0
        wps.append([x, y, z, psi])

    # Specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish.
    rate = rospy.Rate(2)
    i = 0
    global mode_g
    while i < len(wps):
        if not mode_g:
            drone.set_destination(
                x=wps[i][0], y=wps[i][1], z=wps[i][2], psi=wps[i][3])
            rate.sleep()
            if drone.check_waypoint_reached():
                i += 1
        elif mode_g:
            break

    drone.land()
    # Land after all waypoints is reached or if human is detected.
    print((CGREEN2 + "Human detected Landing" + CEND)
          if mode_g else (CRED2 + "Landing no humans detected" + CEND))
    # Shutdown node.
    rospy.signal_shutdown()


# Driver code.
if __name__ == '__main__':
    try:
        main()
        # Used to keep the node running.
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        exit()
