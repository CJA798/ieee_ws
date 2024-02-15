#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge


class RawImageSubscriber():
    def __init__(self):
        # Initialize the node
        rospy.init_node('raw_image__subscriber', anonymous=True)

        # Create a subscriber for the arm_raw_image topic
        self.arm_image_sub = rospy.Subscriber('arm_raw_image', Image, self.image_cb)

        # Create a subscriber for the front_raw_image topic
        self.front_image_sub = rospy.Subscriber('front_raw_image', Image, self.image_cb)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Create variables to store the images
        self.arm_image = None
        self.front_image = None

    def image_cb(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        # Check which topic the message came from
        if msg._connection_header['topic'] == '/arm_raw_image':
            self.arm_image = cv_image
        elif msg._connection_header['topic'] == '/front_raw_image':
            self.front_image = cv_image

    def display_images(self):
        # Check that both images are available
        if self.arm_image is None and self.front_image is None:
            return
        
        r_arm_image = cv2.rotate(self.arm_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        r_front_image = cv2.rotate(self.front_image, cv2.ROTATE_90_CLOCKWISE)


        # Add text to the images
        cv2.putText(r_arm_image, 'Arm Camera', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(r_front_image, 'Front Camera', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Stack the images
        stacked_img = cv2.hconcat([r_arm_image, r_front_image])

        # Display the stacked image
        cv2.imshow('Arm and Front Images', stacked_img)
        cv2.waitKey(1)

    def run(self):
        # Execute code indefinitely, until ctrl + C is pressed
        while not rospy.is_shutdown():
            self.display_images()
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    raw_image_subscriber = RawImageSubscriber()
    raw_image_subscriber.run()
