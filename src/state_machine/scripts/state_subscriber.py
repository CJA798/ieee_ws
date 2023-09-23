#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "State: %s", data.data)


def main():
    subscrier_name = 'state_subscriber'
    topic_name = 'bot_state'

    rospy.init_node(subscrier_name, anonymous=True)
    rospy.Subscriber(topic_name, String, callback)
    rospy.spin()


if __name__ == '__main__':
    main()