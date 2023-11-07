#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from vision_system.srv import getCoords, getCoordsRequest
from vision_system.msg import CoordinatesList
from std_msgs.msg import Float64MultiArray


class CameraNode():
    def __init__(self):
        #self.delta_xyz_publisher = rospy.Publisher("delta_xyz", Float64MultiArray, queue_size=10)
        #self.raw_image_subscriber = rospy.Subscriber("raw_image", Image, self.raw_image_cb)
        self.coords_image_subscriber = rospy.Subscriber("coords_image", Image, self.coords_image_cb)

    def raw_image_cb(self, message):
        bridge = CvBridge()
        frame_to_cv = bridge.imgmsg_to_cv2(message)
        cv2.imshow("Raw Image", frame_to_cv)
        cv2.waitKey(5)
        #rospy.loginfo("Frame received successfully")

    def coords_image_cb(self, message):
        bridge = CvBridge()
        frame_to_cv = bridge.imgmsg_to_cv2(message)
        cv2.imshow("Coords Image", frame_to_cv)
        cv2.waitKey(5)
        rospy.loginfo("Coords frame received successfully")

    def request_coords(self, object_type, image_msg):
        rospy.wait_for_service('get_coords_service')

        try:
            # Create a service proxy for the get_coords_service
            get_coords_proxy = rospy.ServiceProxy('get_coords_service', getCoords)

            # Prepare the request
            request = getCoordsRequest()
            request.object_type = String(data=object_type)  # Specify the object type you want coordinates for
            request.frame = image_msg

            # Call the service to request coordinates
            response = get_coords_proxy(request)

            # Process the response
            if response:
                coordinates_list = response.coordinates

                # Check if coordinates_list is not empty
                if len(coordinates_list.coordinates) > 0:
                    print(coordinates_list)
                    for index, coords in enumerate(coordinates_list.coordinates):
                        rospy.loginfo("%s#%d: x=%.2f, y=%.2f, z=%.2f", object_type, index, coords.x, coords.y, coords.z)
                else:
                    rospy.logwarn("No coordinates received from the service")
            else:
                rospy.logwarn("No response received from the service")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)


if __name__ == "__main__":
	node_name = "CAM"
	rospy.init_node(node_name)
	CameraNode()
	rospy.spin()

