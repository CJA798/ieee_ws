#!/usr/bin/env python
import rospy
import numpy as np
import math

from std_msgs.msg import Float64MultiArray
from arm_system.srv import inverseKinematics, inverseKinematicsResponse

class IKServer():
    def __init__(self):
		self.ik_service = rospy.Service("ik_service", inverseKinematics, self.ik_cb)

    def _calculate_ik(self, x, y, z, theta):
        q = [0,0,0,0,0]

        return

    def ik_cb(self, request):
        if not request:
            rospy.logwarn("No input array received")
            return inverseKinematicsResponse()
        
        response = inverseKinematicsResponse()
		response.pose = calculate_ik(*request.task_space)

		rospy.loginfo("Inverse kinematics returned")

		return response


if __name__ == "__main__":
	node_name = "ik_node"
	rospy.init_node(node_name)
	IKServer()
	rospy.spin()