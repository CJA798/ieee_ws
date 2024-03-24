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
        # Add states to the container

        # Initialize all devices, variables, windows, etc.
        smach.StateMachine.add('INITIALIZE', Initialize(init_state_pub=init_state_pub), 
                               transitions={'succeeded':'READING_START_LED', 'aborted':'INITIALIZE'})
        
        # Read the start green LED and wait for it to be detected
        smach.StateMachine.add('READING_START_LED', ReadingStartLED(), 
                               transitions={'green_led_detected': 'SET_INITIAL_ARMS',     
                                            'green_led_not_detected':'READING_START_LED'})

        # Set the robot arms to the initial position
        smach.StateMachine.add('SET_INITIAL_ARMS', SetPose(pose=Poses.SET_INITIAL_ARMS, misc_angles_publisher=misc_angles_pub, arm_angles_publisher=arm_angles_pub),
                               transitions={'pose_reached':'SCAN_POSE', 'pose_not_reached':'SET_INITIAL_ARMS'})
        
        
        smach.StateMachine.add('SCAN_POSE', ScanPose(arm_angles_pub=arm_angles_pub),
                                transitions={'pose_reached':'GET_SP_COORDS', 'pose_not_reached':'SCAN_POSE'})
        smach.StateMachine.add('GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose=Poses.SMALL_PACKAGE_SCAN.value, timeout=5, expected_pairs=3, camera_enable_publisher=camera_enable_pub),
                                transitions={'coords_received':'PICK_UP_SP', 'coords_not_received':'GET_SP_COORDS'})
        smach.StateMachine.add('PICK_UP_SP', PickUpSmallPackage(task_space_pub=task_space_pub),
                                transitions={'packages_picked_up':'HOME_POSE',
                                            'sweep_needed': 'SWEEP_SMALL_PACKAGES',
                                            'soft_sweep_needed': 'SOFT_SWEEP',
                                            'no_coordinates_received': 'HOME_POSE'})
        smach.StateMachine.add('HOME_POSE', RestPose(arm_angles_pub=arm_angles_pub),
                                transitions={'pose_reached':'END', 'pose_not_reached':'HOME_POSE'})
        
        # Sweep if necessary
        # After a sweep, a new scan is necessary to update the coordinates list
        smach.StateMachine.add('SWEEP_SMALL_PACKAGES', Sweep(task_space_pub=task_space_pub),
                                transitions={'succeeded':'SCAN_POSE', 'aborted':'SWEEP_SMALL_PACKAGES'})
        smach.StateMachine.add('SOFT_SWEEP', Sweep(task_space_pub=task_space_pub, soft=True),
                                transitions={'succeeded':'SCAN_POSE', 'aborted':'SOFT_SWEEP'})

        smach.StateMachine.add('REST_POSE', RestPose(arm_angles_pub=arm_angles_pub),
                                transitions={'pose_reached':'END', 'pose_not_reached':'REST_POSE'})
    


        
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
