#! /usr/bin/env python
# Import ROS.
import rospy
# Importing BoundingBoxes message from package darknet_ros_msgs.
from darknet_ros_msgs.msg import BoundingBoxes


def detection_cb(msg):
    # Callback function of the subscriber.
    # Assigning bounding_boxes of the message to bbox variable.
    bbox = msg.bounding_boxes
    for i in range(len(bbox)):
        # Printing the detected object with its probability in percentage.
        rospy.loginfo("{}% certain {} detected.".format(
            float(bbox[i].probability * 100), bbox[i].Class))


def main():
    # Initializing ROS node.
    rospy.init_node("Detection_sub")
    # Creating a subscriber for the topic "/darknet_ros/bounding_boxes".
    rospy.Subscriber(name="/darknet_ros/bounding_boxes",
                     data_class=BoundingBoxes,
                     callback=detection_cb,
                     queue_size=1)


# Driver code.
if __name__ == '__main__':
    try:
        main()
        # Used to keep the node running.
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Closing")
        exit()
