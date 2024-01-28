#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from vision_system.msg import CoordinatesList, GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback
from time import time
from typing import List

class GetCoordsServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('get_coords', GetCoordsAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        # Start the timer for the action
        start_time = time()

        # Extract the timeout and number of expected pairs from the goal
        timeout = goal.timeout
        expected_pairs = goal.expected_pairs.data

        # Initialize the action result and feedback
        result = GetCoordsResult()
        feedback = GetCoordsFeedback()

        # Initialize coordinates array
        coordinates = CoordinatesList()

        # Keep scanning until three coordinate pairs are found or the action time outs
        while not (self._timeout_reached(start_time, timeout) or self._enough_coordinate_pairs_found(expected_pairs, coordinates.coordinates)):
            # Preempt the action if necessary
            if self.server.is_preempt_requested():
                rospy.loginfo('get_coords Preempted')
                self.server.set_preempted()
                return
            
            box1 = Point()
            box2 = Point()
            box3 = Point()

            coordinates.coordinates = [box1, box2, box3]
            
            # Provide feedback during the process
            feedback.current_coordinates = coordinates
            self.server.publish_feedback(feedback)
        elapsed_time = time() - start_time
        result.coordinates = coordinates
        result.elapsed_time.data = elapsed_time

        if self._timeout_reached(start_time, timeout):
            rospy.loginfo('get_coords Timed Out')
            self.server.set_aborted()
            return

        self.server.set_succeeded(result)
        
    def _timeout_reached(self, start_time, timeout, tolerance=1e-3) -> bool:
        current_time = time()
        elapsed_time = current_time - start_time
        return elapsed_time >= (timeout.data - tolerance)

    
    def _enough_coordinate_pairs_found(self, expected_pairs: int, coordinates: List[List]) -> bool:
        print(f'LEN COORDINATES: {expected_pairs == len(coordinates)}')
        return expected_pairs == len(coordinates)


if __name__ == '__main__':
    rospy.init_node('get_coords_server')
    server = GetCoordsServer()
    rospy.spin()
