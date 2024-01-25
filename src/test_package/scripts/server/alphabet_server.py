#!/usr/bin/env python

import rospy
import actionlib
from test_package.msg import AlphabetTraversalAction, AlphabetTraversalGoal, AlphabetTraversalResult, AlphabetTraversalFeedback
import time

class AlphabetServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('alphabet_traversal', AlphabetTraversalAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        result = AlphabetTraversalResult()
        feedback = AlphabetTraversalFeedback()

        alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'

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
    rospy.init_node('alphabet_server')
    server = AlphabetServer()
    rospy.spin()
