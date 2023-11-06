#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg._MultiArrayLayout import MultiArrayLayout
from std_msgs.msg._MultiArrayDimension import MultiArrayDimension
from robot_states import RobotStates

class MainNode():
    def __init__(self):
        self.manual_subscriber = rospy.Subscriber("manual", Float32MultiArray, callback=self.manual_cb)
        self.current_pose_subscriber = rospy.Subscriber("current_pose", Float32MultiArray, callback=self.current_pose_cb)
        self.state_subscriber = rospy.Subscriber("state", String, callback=self.state_cb)
        self.next_pose_publisher = rospy.Publisher("next_pose", Float32MultiArray, queue_size=10)
        self.current_state = RobotStates.IDLE
    def manual_cb(self):
        pass

    def current_pose_cb(self):
        pass

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


if __name__ == "__main__":
    node_name = "MAIN"
    rospy.init_node(node_name)
    main_node = MainNode()
    rospy.spin()
