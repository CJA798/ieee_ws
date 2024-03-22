#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import sys
from std_msgs.msg import Empty, Int8, Int16, Bool, Int16MultiArray, Float32MultiArray
from image_utils.board_objects import BoardObjects
from image_utils.poses import Poses
from utils.states import *
from utils.areas import Areas
from utils.callbacks import *


# Create publishers
task_space_pub = rospy.Publisher('Task_Space', Float32MultiArray, queue_size=1)
arm_angles_pub = rospy.Publisher('Arm_Angles', Float32MultiArray, queue_size=10)
move_pub = rospy.Publisher('Move', Float32MultiArray, queue_size=1)
misc_angles_pub = rospy.Publisher('Misc_Angles', Float32MultiArray, queue_size=1)
init_state_pub = rospy.Publisher('Init_State', Bool, queue_size=10)
grav_enable_pub = rospy.Publisher('Grav_En', Bool, queue_size=10)
camera_enable_pub = rospy.Publisher('Camera_En', Bool, queue_size=10)

# Create subscribers
start_led_state_sub = rospy.Subscriber("LED_State", Bool, callback=start_led_callback)
arm_done_sub = rospy.Subscriber("Arm_Done", Int8, callback=state_arm2sm_cb)
misc_done = rospy.Subscriber("Misc_Done", Int8, callback=misc_done_cb)
move_done_sub = rospy.Subscriber("Move_Done", Int8, callback=move_done_cb)
gravity_vector_sub = rospy.Subscriber("IMU_Grav", Int16, callback=gravity_vector_cb)
tof_back_sub = rospy.Subscriber("Local_Data", Int16MultiArray, callback=tof_back_cb)
heartbeat_sub = rospy.Subscriber("Heartbeat", Empty, callback=heartbeat_cb)


def main():
    rospy.init_node('STATE_MACHINE')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        smach.StateMachine.add('SET_BULK_GRABBER_ARMS_DOWN', DropOff(BoardObjects.BIG_PACKAGE, misc_angles_publisher=misc_angles_pub),
                                transitions={'packages_dropped_off':'FUEL_TANKS_PLACING', 'packages_not_dropped_off':'SET_BULK_GRABBER_ARMS_DOWN'})

        smach.StateMachine.add('FUEL_TANKS_PLACING', FuelTankPlacer(task_space_publisher=task_space_pub),
                               transitions = {'fuel_tanks_placed':'END', 'fuel_tanks_not_placed':'FUEL_TANKS_PLACING'})
        
        
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/START')
    sis.start()

    # Create a timer to check the Arduino's heartbeat every second
    rospy.Timer(rospy.Duration(1), lambda event: check_heartbeat_cb(arm_angles_pub, move_pub, misc_angles_pub))

    # Execute SMACH plan
    outcome = sm.execute()

    # Stop the introspection server
    sis.stop()
    rospy.loginfo('State machine execution completed.')

    

if __name__ == '__main__':
    try:
        main()
    
    finally:
        # Terminate the program
        rospy.signal_shutdown('State machine execution completed.')  # Gracefully shutdown ROS
        sys.exit(0)  # Exit the program with exit code 0 (indicating successful termination)