#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class FKNode():
    def __init__(self):
        self.task_space_publisher = rospy.Publisher("current_xyz", Float64MultiArray, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("pose", Float64MultiArray, self.pose_cb)
 
    def pose_cb(self):
        rospy.loginfo("Pose callback")

if __name__ == "__main__":
	node_name = "FK"
	rospy.init_node(node_name)
	FKNode()
	rospy.spin()
