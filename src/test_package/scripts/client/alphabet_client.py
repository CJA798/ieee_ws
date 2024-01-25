#!/usr/bin/env python

import rospy
import actionlib
from test_package.msg import AlphabetTraversalAction, AlphabetTraversalGoal, AlphabetTraversalResult, AlphabetTraversalFeedback

def feedback_callback(feedback):
    rospy.loginfo(f'Current Letter: {feedback.current_letter}, Elapsed Time: {feedback.elapsed_time}')

if __name__ == '__main__':
    rospy.init_node('alphabet_client')
    client = actionlib.SimpleActionClient('alphabet_traversal', AlphabetTraversalAction)
    client.wait_for_server()

    goal = AlphabetTraversalGoal()

    client.send_goal(goal, feedback_cb=feedback_callback)

    client.wait_for_result()

    result = client.get_result()

    rospy.loginfo(f'Final Elapsed Time: {result.elapsed_time}')
