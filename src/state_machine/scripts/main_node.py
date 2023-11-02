#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg._MultiArrayLayout import MultiArrayLayout
from std_msgs.msg._MultiArrayDimension import MultiArrayDimension

class MainNode():
    def __init__(self):
        self.next_xyz_publisher = rospy.Publisher("next_xyz", Float64MultiArray, queue_size=20)
        self.manual_q = rospy.Publisher("manual_q", Float64MultiArray, queue_size=10)
        self.pose_publisher = rospy.Publisher("pose", Float64MultiArray, queue_size=10)
        self.delta_xyz_publisher = rospy.Publisher("delta_xyz", Float64MultiArray, queue_size=10)

    def set_Float64MultiArray_msg(self, label, data, size, stride=0, data_offset=0):
        msg = Float64MultiArray()
        msg.layout.data_offset = data_offset
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim[0].label = label
        msg.layout.dim[0].size = size
        msg.layout.dim[0].stride = stride
        msg.data = data

        return msg


if __name__ == "__main__":
    node_name = "Main"
    rospy.init_node(node_name)
    main_node = MainNode()
    rate = rospy.Rate(100)
    
    for x in range(1500):
        data = np.random.rand(3)
        msg = main_node.set_Float64MultiArray_msg(label='data', data=data, size=len(data))
        main_node.next_xyz_publisher.publish(msg)
        rospy.loginfo("Layout and Data published to next_xyz topic")
        print('Sent ', x)
        rate.sleep()


    rospy.spin()
