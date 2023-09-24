#!/usr/bin/env python

# Import rospy to enable using ROS with Python
import rospy

# Import Image message type
from sensor_msgs.msg import Image

# Import library for converting OpenCV images (of type cv::Mat)
# into a ROS Image message, and viceversa
from cv_bridge import CvBridge

# Import OpenCV
import cv2


# Create a name for the subscriber node
subscriber_node_name = 'camera_sensor_subscriber'

# Create a name for the topic to subscribe to
topic_name = 'raw_frame'


# Callback
def callback(message):
    bridge = CvBridge()
    frame_to_cv = bridge.imgmsg_to_cv2(message)
    cv2.imshow("Frame", frame_to_cv)
    cv2.waitKey(5)
    rospy.loginfo("Frame received successfully")


# Initialize the node
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
rospy.init_node(subscriber_node_name, anonymous=True)

# Create a subscriber object
# http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html"""
subscriber = rospy.Subscriber(topic_name, Image, callback=callback)

# Execute code indefinitely, until ctrl + C is pressed
rospy.spin()

cv2.destroyAllWindows()