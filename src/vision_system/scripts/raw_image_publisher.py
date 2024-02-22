#!/usr/bin/env python

# Import rospy to enable using ROS with Python
import rospy

# Import Image message type
from sensor_msgs.msg import Image

# Import OpenCV
import cv2

# Import library for converting OpenCV images (of type cv::Mat)
# into a ROS Image message, and viceversa
from cv_bridge import CvBridge

# Import the globals dictionary
from utils.globals import globals

# Create a name for the publisher node
publisher_node_name = 'raw_image_publisher'

# Create a name for the topic where the messages are going to be published
topic_name = 'raw_image'

# Initialize the node
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
rospy.init_node(publisher_node_name, anonymous=True)

# Create a publisher object
# http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers#Choosing_a_good_queue_size
publisher = rospy.Publisher(topic_name, Image, queue_size=60)

# Set message transmission rate (Hz)
rate = rospy.Rate(30)

# Create video capture object
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print('Unable to open camera')
    exit(0)

# TODO: Make the image resize factor a global variable
img_resize_factor = globals['img_resize_factor']
rospy.loginfo(f"Image resize factor: {img_resize_factor}")
#img_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#img_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
img_width = 1920
img_height = 1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width//img_resize_factor)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height//img_resize_factor)

# Create CvBridge object to convert images to messages
bridge = CvBridge()

while not rospy.is_shutdown():
    ret, raw_image = cap.read()

    if ret:
        cv2.imshow("Raw Image", raw_image)
        cv2.waitKey(5)
        #rospy.loginfo("Frame2 received successfully")

        # Convert frame to msg format
        img_to_publish = bridge.cv2_to_imgmsg(raw_image)
        
        # Publish frame to topic
        publisher.publish(img_to_publish)

        # Assert publication
        #rospy.loginfo("Video frame published successfully")
    
    rate.sleep()

cv2.destroyAllWindows()
cap.release()