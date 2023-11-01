#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class AdderNode():
    def __init__(self):
        self.next_xyz_publisher = rospy.Publisher("next_xyz", Float64MultiArray, queue_size=10)
        self.current_xyz_subscriber = rospy.Subscriber("current_xyz", Float64MultiArray, self.current_xyz_cb)
        self.delta_xyz = rospy.Subscriber("delta_xyz", Float64MultiArray, self.delta_xyz_cb)

    def current_xyz_cb(self):
        rospy.loginfo("Current xyz callback")

    def delta_xyz_cb(self):
        rospy.loginfo("Delta xyz callback")

if __name__ == "__main__":
	node_name = "ADDER"
	rospy.init_node(node_name)
	AdderNode()
	rospy.spin()
