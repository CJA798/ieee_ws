#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


class RawImagePublisher():
    def __init__(self):
        # Initialize the node
        rospy.init_node('raw_image_publisher', anonymous=True)

        # Create the image publishers
        self.arm_publisher = rospy.Publisher('arm_raw_image', Image, queue_size=10)
        self.front_publisher = rospy.Publisher('front_raw_image', Image, queue_size=10)
        
        # Setup the arm camera
        self.arm_cam = self._initialize_camera(0, img_resize_factor=4)
        # Setup the front camera
        self.front_cam = self._initialize_camera(2, img_resize_factor=4)
        
        # Create CvBridge object to convert images to messages
        self.bridge = CvBridge()

    def _initialize_camera(self, camera_index, img_resize_factor=4):
        cam = cv2.VideoCapture(camera_index, cv2.CAP_V4L2)
        if not cam.isOpened():
            rospy.logerr('Unable to open camera at index {}'.format(camera_index))
            exit(0)
        img_width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        img_height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, round(img_width/img_resize_factor))
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, round(img_height/img_resize_factor))
        return cam

    def publish_images(self):
        try:
            while not rospy.is_shutdown():
                # Get frame from the arm camera
                ret1, arm_frame = self.arm_cam.read()
                if not ret1:
                    rospy.logerr("Can't receive frame from arm camera")
                    break

                # Get frame from the front camera
                ret2, front_frame = self.front_cam.read()
                if not ret2:
                    rospy.logerr("Can't receive frame from front camera")
                    break

                # Convert frames to ROS Image messages
                arm_img_to_publish = self.bridge.cv2_to_imgmsg(arm_frame)
                front_img_to_publish = self.bridge.cv2_to_imgmsg(front_frame)

                # Publish frames to topics
                self.arm_publisher.publish(arm_img_to_publish)
                self.front_publisher.publish(front_img_to_publish)
        finally:
            self.cleanup()

    def cleanup(self):
        # Close the cameras
        self.arm_cam.release()
        self.front_cam.release()
        cv2.destroyAllWindows()

        
if __name__ == "__main__":
    raw_image_publisher = RawImagePublisher()
    raw_image_publisher.publish_images()
