#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class IKNode():
    def __init__(self):
        self.pose_publisher = rospy.Publisher("pose", Float64MultiArray, queue_size=10)
        self.manual_q_subscriber = rospy.Subscriber("manual_q", Float64MultiArray, self.manual_q_cb)
        self.next_xyz = rospy.Subscriber("next_xyz", Float64MultiArray, self.next_xyz_cb)

    def manual_q_cb(self):
        rospy.loginfo("Manual q callback")

    def next_xyz_cb(self):
        rospy.loginfo("Next xyz callback")

if __name__ == "__main__":
	node_name = "IK"
	rospy.init_node(node_name)
	IKNode()
	rospy.spin()
