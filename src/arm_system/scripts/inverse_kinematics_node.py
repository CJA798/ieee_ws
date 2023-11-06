#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class IKNode():
    def __init__(self):
        self.next_pose_subscriber = rospy.Subscriber("next_pose", Float32MultiArray, callback=self.next_pose_cb)
        self.arm_angles_publisher = rospy.Publisher("arm_angles", Float32MultiArray, queue_size=10)
    
    def next_pose_cb(self, data):
        # Calculate IK
        next_pose_data = data.data
        arm_angles_array = self.calculate_inverse_kinematics(*next_pose_data)

        # Publish arm angles
        arm_angles_msg = self.set_Float32MultiArray(label='arm_angles', data=arm_angles_array, size=len(arm_angles_array))
        self.arm_angles_publisher.publish(arm_angles_msg)

    def calculate_inverse_kinematics(self, x, y, z, theta, phi):
        # q array for final joint angles
        q = [0, 0, 0, 0, 0, 0]
        # l array for predefined arm lengths
        l = [0.0325, 0.162, 0.024, 0.024, 0.1485, 0.07534, 0.017, 0, 0]

        # Calculation of substeps
        x1 = np.sqrt(x**2 + z**2) + l[6] - l[3]
        y1 = y - l[0] + l[5]
        l[7] = np.sqrt(l[2]**2 + l[4]**2)
        l[8] = np.sqrt(x1**2 + y1**2)
        beta = np.arccos((l[1]**2 + l[8]**2 - l[7]**2) / (2 * l[1] * l[8]))
        alpha = np.arccos((l[7]**2 + l[1]**2 - l[8]**2) / (2 * l[7] * l[1]))
        omega = np.arccos(l[2] / l[7])
        phi1 = np.pi / 2 - np.arctan(y1 / x1)

        # Calculations of final angles
        q[0] = np.arctan2(-z, x)
        q[1] = beta - phi1
        q[2] = alpha + omega - np.pi
        q[3] = -q[1] - q[2] - np.pi/2
        q[4] = theta
        q[5] = phi

        return q
    
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
	node_name = "IK"
	rospy.init_node(node_name)
	ik_node = IKNode()
	rospy.spin()
