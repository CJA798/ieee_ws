#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "State: %s", data.data)

def led_cb(data):
    rospy.loginfo("LED Reading: %d", data.data)


def main():
    subscrbier_name = 'state_subscriber'
    topic_name = 'bot_state'

    rospy.init_node(subscrbier_name, anonymous=True)
    rospy.Subscriber(topic_name, String, callback)
    rospy.Subscriber('start_led', Int32, led_cb)
    rospy.spin()


if __name__ == '__main__':
    main()