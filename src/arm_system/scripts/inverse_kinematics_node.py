#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray

class IKNode():
    def __init__(self):
        self.pose_publisher = rospy.Publisher("pose", Float64MultiArray, queue_size=10)
        self.manual_q_subscriber = rospy.Subscriber("manual_q", Float64MultiArray, callback=self.manual_q_cb)
        self.next_xyz = rospy.Subscriber("next_xyz", Float64MultiArray, callback=self.next_xyz_cb)
        self.sum = 0
    def manual_q_cb(self):
        rospy.loginfo("Manual q callback")

    def next_xyz_cb(self, data):
        rospy.loginfo("Next xyz callback")
        self.sum += 1
        print('Received ', self.sum)

if __name__ == "__main__":
	node_name = "IK"
	rospy.init_node(node_name)
	IKNode()
	rospy.spin()


"""
import numpy as np

# Input in meters
x = 0.000
y = 0.150
z = -0.0500
theta = 0
phi = 0

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

# Output of q angles in degrees for 7 servos
Q = [q[0], q[1], q[1], q[2], 0, q[3], q[4], q[5]
# Convert angles to degrees
Q = [np.degrees(angle) for angle in Q]

# Calculation/verification of forward kinematics
r2 = l[3] - l[1] * np.sin(q[1]) - l[2] * np.sin(q[2] + q[1]) + l[4] * np.cos(q[2] + q[1]) - l[6]
x2 = r2 * np.cos(q[0])
y2 = l[0] + l[1] * np.cos(q[1]) + l[2] * np.cos(q[1] + q[2]) + l[4] * np.sin(q[1] + q[2]) - l[5]
z2 = -r2 * np.sin(q[0])
theta2 = q[4]
phi2 = q[5]
T = [x2, y2, z2, theta2, phi2]

print("Joint Angles (in degrees):", Q)
print("Forward Kinematics (x, y, z, theta, phi):", T)

"""