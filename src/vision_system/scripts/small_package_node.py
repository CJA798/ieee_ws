#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from vision_system.srv import get_small_package_coordinates


class SmallPackageHandler:

    def __init__(self):
        sub_topic_name = "raw_frame"
        pub_topic_name = "small_package_coordinates"

        self.subscriber = rospy.Subscriber(sub_topic_name, Image, self.frame_callback)
        self.publisher = rospy.Publisher(pub_topic_name, Point, queue_size=10)        

    def frame_callback(self, message):
        bridge = CvBridge()
        frame_to_cv = bridge.imgmsg_to_cv2(message)
        cv2.imshow("Frame", frame_to_cv)
        cv2.waitKey(5)
        rospy.loginfo("Frame received successfully")

        rospy.wait_for_service("get_small_package_coordinates")
        get_coords = rospy.ServiceProxy("get_small_package_coordinates", get_small_package_coordinates)
        #coords = get_coords(message)
        #print(coords.coordinates)
 


def main():
    node_name = "small_packages"
    rospy.init_node(node_name)
    SmallPackageHandler()
    rospy.spin()


if __name__ == "__main__":
    main()