#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg._MultiArrayLayout import MultiArrayLayout
from std_msgs.msg._MultiArrayDimension import MultiArrayDimension
from robot_states import RobotStates

import cv2
from board_objects import BoardObjects
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from vision_system.srv import getCoords, getCoordsRequest
from vision_system.msg import CoordinatesList


class MainNode():
    def __init__(self):
        self.manual_subscriber = rospy.Subscriber("manual", Float32MultiArray, callback=self.manual_cb)
        self.current_pose_subscriber = rospy.Subscriber("current_pose", Float32MultiArray, callback=self.current_pose_cb)
        self.state_subscriber = rospy.Subscriber("state", String, callback=self.state_cb)
        self.raw_image_subscriber = rospy.Subscriber("raw_image", Image, callback=self.raw_frame_cb)
        
        self.next_pose_publisher = rospy.Publisher("next_pose", Float32MultiArray, queue_size=10)
        
        self.current_state = RobotStates.IDLE
        self.arm_current_pose = []
        self.current_image = None

    def manual_cb(self, data):
        new_pose = data.data
        self.arm_current_pose = new_pose
        rospy.loginfo("Arm pose updated manually to: ", new_pose)

    def current_pose_cb(self, data):
        new_pose = data.data
        self.arm_current_pose = new_pose
        rospy.loginfo("Arm pose updated to: ", new_pose)

    def raw_frame_cb(self, message):
        bridge = CvBridge()
        raw_image = bridge.imgmsg_to_cv2(message)
        self.current_image = raw_image
        cv2.imshow("Raw Image", raw_image)
        cv2.waitKey(5)

    def state_cb(self, data):
        new_state = data.data
        rospy.loginfo("Received new state: %s", new_state)

        if new_state == self.current_state.value:
            rospy.loginfo("Already in state: %s, no action taken.", new_state)
            return

        if new_state not in RobotStates.__members__:
            rospy.logwarn("Received an invalid state: %s. Ignoring.", new_state)
            return

        rospy.loginfo("Changing state from %s to %s", self.current_state.value, new_state)
        self.current_state = RobotStates(new_state)

        # Perform state-specific actions based on the new state
        if self.current_state == RobotStates.IDLE:
            self.handle_idle()
        elif self.current_state == RobotStates.SCANNING_POSE:
            self.handle_scanning_pose()
        elif self.current_state == RobotStates.SCANNING_FOR_PURPLE_BOXES:
            self.handle_scanning_for_purple_boxes()
        elif self.current_state == RobotStates.VERIFYING_POSE:
            self.handle_verifying_pose()
        elif self.current_state == RobotStates.VERIFYING_PURPLE_BOX:
            self.handle_verifying_purple_box()

    def handle_idle(self):
        rospy.loginfo("Handling IDLE")
    
    def handle_scanning_pose(self):
        rospy.loginfo("Handling SCANNING POSE")
    
    def handle_scanning_for_purple_boxes(self):
        rospy.loginfo("Handling SCANNING FOR PURPLE BOXES")
        # Convert current frame to Image message
        object_type = BoardObjects.SMALL_PACKAGE.value

        bridge = CvBridge()
        image = self.current_image
        image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        
        coords_list = self.request_coords(object_type, image_msg)
        if not coords_list:
            rospy.logwarn("No coordinates obtained for small package. Trying again...")
            

    def handle_verifying_pose(self):
        rospy.loginfo("Handling VERIFYING POSE")
    
    def handle_verifying_purple_box(self):
        rospy.loginfo("Handling VERIFYING PURPLE BOX")

    def set_Float32MultiArray_msg(self, label, data, size, stride=0, data_offset=0):
        msg = Float32MultiArray()
        msg.layout.data_offset = data_offset
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = label
        msg.layout.dim[0].size = size
        msg.layout.dim[0].stride = stride
        msg.data = data

        return msg

    def request_coords(self, object_type, image_msg):
        rospy.wait_for_service('get_coords_service')
        coordinates_list = []

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

                    return coordinates_list
                
                else:
                    rospy.logwarn("No coordinates received from the service")

            else:
                rospy.logwarn("No response received from the service")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

        return []

        

if __name__ == "__main__":
    node_name = "MAIN"
    rospy.init_node(node_name)
    main_node = MainNode()
    rospy.spin()
