#!/usr/bin/env python
import rospy
import numpy as np
from robot_states import RobotStates
from std_msgs.msg import String


class StateNode():
    def __init__(self):
        self.state_publisher = rospy.Publisher("state", String, queue_size=10)

    def set_state(self, state):
        msg = String()
        msg.data = state
        self.state_publisher.publish(msg)

if __name__ == "__main__":
    node_name = "STATE"
    rospy.init_node(node_name)
    state_node = StateNode()

    delay = 5
    
    state_node.set_state(RobotStates.IDLE.value)
    rospy.sleep(delay)  # Add a delay to allow the ROS node to process the message

    state_node.set_state(RobotStates.SCANNING_POSE.value)
    rospy.sleep(delay)

    state_node.set_state(RobotStates.SCANNING_FOR_PURPLE_BOXES.value)
    rospy.sleep(delay*2)

    state_node.set_state(RobotStates.VERIFYING_POSE.value)
    rospy.sleep(delay)

    state_node.set_state(RobotStates.VERIFYING_PURPLE_BOX.value)

    # Continue running ROS node
    rospy.spin()
