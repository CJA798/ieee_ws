#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import sys
from std_msgs.msg import Int8, Int16, Bool, Float32MultiArray
from image_utils.board_objects import BoardObjects
from image_utils.poses import Poses
from utils.states import *
from utils.globals import globals
from utils.areas import Areas
from utils.callbacks import *

# Create publishers
#task_space_pub = rospy.Publisher('Task_Space', Float32MultiArray, queue_size=10)
#arm_angles_pub = rospy.Publisher('Arm_Angles', Float32MultiArray, queue_size=10)
#state_SM2Nav_pub = rospy.Publisher('State_SM2Nav', Int8, queue_size=10)
move_pub = rospy.Publisher('Move', Float32MultiArray, queue_size=10)

# Create subscribers
start_led_state_sub = rospy.Subscriber("LED_State", Bool, callback=start_led_callback)
#arm_done_sub = rospy.Subscriber("Arm_Done", Int8, callback=state_arm2sm_cb)
#state_Nav2SM_sub = rospy.Subscriber("State_Nav2SM", Int8, callback=state_nav2arm_cb)
#TOF_Front = rospy.Subscriber("TOF_Front", Int16, callback=tof_front_cb)
#move_done_sub = rospy.Subscriber("Move_Done", Int8, callback=move_done_cb)
gravity_vector_sub = rospy.Subscriber("IMU_Grav", Int16, callback=gravity_vector_cb)


def main():
    rospy.init_node('STATE_MACHINE')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container

        # Initialize all devices, variables, windows, etc.
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'succeeded':'READING_START_LED', 'aborted':'INITIALIZE'})
        
        # Read the start green LED and wait for it to be detected
        smach.StateMachine.add('READING_START_LED', ReadingStartLED(), 
                               transitions={'green_led_detected': 'GO_TO_DROP_OFF_AREA',
                                            'green_led_not_detected':'READING_START_LED'})
        
        # TODO: Add pickup states

        # Go to dropoff area
        smach.StateMachine.add('GO_TO_DROP_OFF_AREA', GoTo_(Areas.DROP_OFF, move_publisher=move_pub), 
                                   transitions={'arrived':'END', 'not_arrived':'GO_TO_DROP_OFF_AREA'})
        



    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/START')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Stop the introspection server
    sis.stop()
    rospy.loginfo('State machine execution completed.')

    # Terminate the program
    rospy.signal_shutdown('State machine execution completed.')  # Gracefully shutdown ROS
    sys.exit(0)  # Exit the program with exit code 0 (indicating successful termination)


if __name__ == '__main__':
    main()