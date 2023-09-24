#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def main():
    publisher_name = 'state_publisher'
    topic_name = 'bot_state'

    publisher = rospy.Publisher(topic_name, String, queue_size=10)
    rospy.init_node(publisher_name)
    rate = rospy.Rate(10)

    if not rospy.is_shutdown():
        for _ in range(100):
            state = 'idle'
            rospy.loginfo(state)
            publisher.publish(state)
            rate.sleep()

    while not rospy.is_shutdown():
        state = 'find small packages'
        rospy.loginfo(state)
        publisher.publish(state)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass