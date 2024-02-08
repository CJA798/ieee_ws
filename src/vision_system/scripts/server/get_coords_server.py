#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from vision_system.msg import CoordinatesList, GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback
from time import time
from typing import List
import cv2
import numpy as np
from image_utils.image_processor import ImageProcessor
from cv_bridge import CvBridge



class GetCoordsServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('get_coords', GetCoordsAction, self.execute, False)
        self.server.start()
        self.current_frame = np.ndarray(1)
        self.raw_img_subscriber = rospy.Subscriber('raw_image', Image, callback=self._sub_callback)
        self.coords_img_publisher = rospy.Publisher('coords_image', Image, queue_size=60)

    def _sub_callback(self, message):
        bridge = CvBridge()
        self.current_frame = bridge.imgmsg_to_cv2(message)


    def execute(self, goal):
        # Start the timer for the action
        start_time = time()

        # Extract the timeout and number of expected pairs from the goal
        timeout = goal.timeout
        expected_pairs = goal.expected_pairs.data
        object_type = goal.object_type.data
        arm_pose = goal.arm_pose.data

        # Initialize the action result and feedback
        result = GetCoordsResult()
        feedback = GetCoordsFeedback()

        # Initialize coordinates array
        coordinates = CoordinatesList()
        
        # Create CvBridge object to convert images to messages
        bridge = CvBridge()

        # Create image processor to deal with frames
        ip = ImageProcessor()

        # Keep scanning until three coordinate pairs are found or the action time outs
        while not (self._timeout_reached(start_time, timeout) or self._enough_coordinate_pairs_found(expected_pairs, coordinates.coordinates)):
            # Preempt the action if necessary
            if self.server.is_preempt_requested():
                rospy.loginfo('get_coords Preempted')
                self.server.set_preempted()
                return
            # Create a copy of the current frame to find the coordinates
            ip.image = self.current_frame.copy()

            #print(f'Object type: {object_type}  |   Arm pose: {arm_pose}')
            coordinates.coordinates, coords_image = ip.get_coords(object_type = object_type, pose = arm_pose)
            
            # Convert frame to msg format
            img_to_publish = bridge.cv2_to_imgmsg(coords_image)
            
            # Publish frame to topic
            self.coords_img_publisher.publish(img_to_publish)

            #box1 = Point()
            #box2 = Point()
            #box3 = Point()
            #coordinates.coordinates = [box1, box2]
            
            # Provide feedback during the process
            feedback.current_coordinates = coordinates
            self.server.publish_feedback(feedback)

        elapsed_time = time() - start_time
        result.coordinates = coordinates
        result.elapsed_time.data = elapsed_time

        if self._timeout_reached(start_time, timeout):
            rospy.loginfo('get_coords Timed Out')
            #self.server.set_aborted()
            #return

        self.server.set_succeeded(result)
        
    def _timeout_reached(self, start_time, timeout, tolerance=1e-3) -> bool:
        current_time = time()
        elapsed_time = current_time - start_time
        return elapsed_time >= (timeout.data - tolerance)

    
    def _enough_coordinate_pairs_found(self, expected_pairs: int, coordinates: List[List]) -> bool:
        return expected_pairs == len(coordinates)

        
if __name__ == '__main__':
    rospy.init_node('get_coords_server')
    server = GetCoordsServer()
    rospy.spin()