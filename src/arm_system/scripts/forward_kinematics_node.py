#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class FKNode():
    def __init__(self):
        self.arm_angles_subscriber = rospy.Subscriber("arm_angles", Float32MultiArray, callback=self.arm_angles_cb)
        self.current_pose_publisher = rospy.Publisher("current_pose", Float32MultiArray, queue_size=10)
    
    def arm_angles_cb(self, data):
        # Calculate IK
        arm_angles_data = data.data
        current_pose_array = self.calculate_forward_kinematics(arm_angles_data)

        # Publish arm angles
        current_pose_msg = self.set_Float32MultiArray(label='current_pose', data=current_pose_array, size=len(current_pose_array))
        self.current_pose_publisher.publish(current_pose_msg)

    def calculate_forward_kinematics(self, q):
        # l array for predefined arm lengths
        l = [0.0325, 0.162, 0.024, 0.024, 0.1485, 0.07534, 0.017, 0, 0]

        # Calculation/verification of forward kinematics
        r2 = l[3] - l[1] * np.sin(q[1]) - l[2] * np.sin(q[2] + q[1]) + l[4] * np.cos(q[2] + q[1]) - l[6]
        x2 = r2 * np.cos(q[0])
        y2 = l[0] + l[1] * np.cos(q[1]) + l[2] * np.cos(q[1] + q[2]) + l[4] * np.sin(q[1] + q[2]) - l[5]
        z2 = -r2 * np.sin(q[0])
        theta2 = q[4]
        phi2 = q[5]
        T = [x2, y2, z2, theta2, phi2]

        return T
    
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
	node_name = "FK"
	rospy.init_node(node_name)
	fk_node = FKNode()
	rospy.spin()
