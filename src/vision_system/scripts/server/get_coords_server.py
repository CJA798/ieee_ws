#!/usr/bin/env python

import rospy
import actionlib
from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback
import time

class GetCoordsServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('get_coords', GetCoordsAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        result = GetCoordsResult()
        feedback = GetCoordsFeedback()

        start_time = time.time()
        
        for letter in alphabet:
            if self.server.is_preempt_requested():
                rospy.loginfo('Preempted')
                self.server.set_preempted()
                return

            feedback.current_letter = letter
            feedback.elapsed_time = rospy.Duration.from_sec(time.time() - start_time)
            self.server.publish_feedback(feedback)

            time.sleep(.1)  # Simulate some time-consuming task

        result.elapsed_time = rospy.Duration.from_sec(time.time() - start_time)
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('get_coords_server')
    server = GetCoordsServer()
    rospy.spin()
