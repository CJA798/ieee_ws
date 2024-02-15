#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class RawImagePublisher():
    def __init__(self):
        # Initialize the node
        rospy.init_node('raw_image_publisher', anonymous=True)

        # Create the image publishers
        self.publisher = rospy.Publisher('front_raw_image', Image, queue_size=10)
        self.publisher = rospy.Publisher('arm_raw_image', Image, queue_size=10)
        
        # TODO: Create a function or some other method to obtain the same camera ports always
        
        # Setup the arm camera
        self.arm_cam = cv2.VideoCapture(0)
        if not self.arm_cam.isOpened():
            print('Unable to open arm camera')
            exit(0)
        img_resize_factor = 4
        img_width = self.arm_cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        img_height = self.arm_cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.arm_cam.set(cv2.CAP_PROP_FRAME_WIDTH, round(img_width/img_resize_factor))
        self.arm_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, round(img_height/img_resize_factor))
        
        # Setup the front camera
        self.front_cam = cv2.VideoCapture(2)
        if not self.front_cam.isOpened():
            print('Unable to open front camera')
            exit(0)
        img_resize_factor = 4
        img_width = self.front_cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        img_height = self.front_cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.front_cam.set(cv2.CAP_PROP_FRAME_WIDTH, round(img_width/img_resize_factor))
        self.front_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, round(img_height/img_resize_factor))
        
        # Create CvBridge object to convert images to messages
        self.bridge = CvBridge()

    def publish_images(self):
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

    def cleanup(self):
        # Close the cameras
        self.arm_cam.release()
        self.front_cam.release()
        cv2.destroyAllWindows()

        
if __name__ == "__main__":
    raw_image_publisher = RawImagePublisher()
    raw_image_publisher.publish_images()
    raw_image_publisher.cleanup()