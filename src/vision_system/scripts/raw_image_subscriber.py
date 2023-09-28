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

from vision_system.srv import getCoords, getCoordsRequest
from vision_system.msg import CoordinatesList

# Create a name for the subscriber node
subscriber_node_name = 'camera_sensor_subscriber'

# Create a name for the topic to subscribe to
topic_name = 'raw_frame'

def request_coords(object_type, image_msg):
    rospy.wait_for_service('get_coords_service')

    try:
        # Create a service proxy for the get_coords_service
        get_coords_proxy = rospy.ServiceProxy('get_coords_service', getCoords)

        # Prepare the request
        request = getCoordsRequest()
        request.object_type = object_type  # Specify the object type you want coordinates for
        request.frame = image_msg

        # Call the service to request coordinates
        response = get_coords_proxy(request)

        # Process the response
        if response:
            coordinates_list = response.coordinates
            for coords in coordinates_list.coordinates:
                rospy.loginfo("Coordinates: x=%.2f, y=%.2f, z=%.2f", coords.x, coords.y, coords.z)
        else:
            rospy.logwarn("No coordinates received from the service")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

# Callback
def callback(message):
    bridge = CvBridge()
    frame_to_cv = bridge.imgmsg_to_cv2(message)
    request_coords(object_type="small_boxes", image_msg=message)

    cv2.imshow("Frame", frame_to_cv)
    cv2.waitKey(5)
    rospy.loginfo("Frame received successfully")

# Callback 2
def callback2(message):
    bridge = CvBridge()
    frame_to_cv = bridge.imgmsg_to_cv2(message)
    cv2.imshow("Frame2", frame_to_cv)
    cv2.waitKey(5)
    rospy.loginfo("Frame2 received successfully")




# Initialize the node
# http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
rospy.init_node(subscriber_node_name, anonymous=True)

# Create a subscriber object
# http://docs.ros.org/en/kinetic/api/rospy/html/rospy.topics.Subscriber-class.html"""
subscriber = rospy.Subscriber(topic_name, Image, callback=callback)
subscriber2 = rospy.Subscriber("coords_image", Image, callback=callback2)

# Execute code indefinitely, until ctrl + C is pressed
rospy.spin()

cv2.destroyAllWindows()