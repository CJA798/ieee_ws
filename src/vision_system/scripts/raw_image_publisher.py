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

# Import Bool message type
from std_msgs.msg import Bool

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

# Create a callback function to handle the camera enable topic
def camera_enable_callback(data):
    globals['publish_raw_image'] = data.data
    rospy.logwarn(f"Camera enable state updated: {globals['publish_raw_image']}")

# Create a subscriber to the camera enable topic
camera_enable_topic = 'Camera_En'
rospy.Subscriber(camera_enable_topic, Bool, camera_enable_callback)

# Set message transmission rate (Hz)
rate = rospy.Rate(30)

# Create video capture object
device_path = "/dev/v4l/by-id/usb-Arducam_Arducam_5MP_Camera_Module_YL20230518V0-video-index0"
cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

if not cap.isOpened():
    print('Unable to open camera')
    exit(0)

#img_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#img_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
img_width = globals['current_cam_res'][0]
img_height = globals['current_cam_res'][1]
cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)

# Set Contrast
cap.set(cv2.CAP_PROP_CONTRAST, 3)   # Makes things gray
# Set Saturation
cap.set(cv2.CAP_PROP_SATURATION, 150)   # Makes purple magenta

# Create CvBridge object to convert images to messages
bridge = CvBridge()

while not rospy.is_shutdown():
    if globals['publish_raw_image']:
        # Get frame from video capture object
        ret, raw_image = cap.read()

        # Check if frame was received successfully
        if ret:
            #cv2.imshow("Raw Image", raw_image)
            #cv2.waitKey(5)
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