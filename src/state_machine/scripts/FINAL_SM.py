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
from utils.fuel_tank_utils import fuel_tanks

# Create publishers
task_space_pub = rospy.Publisher('Task_Space', Float32MultiArray, queue_size=1)
arm_angles_pub = rospy.Publisher('Arm_Angles', Float32MultiArray, queue_size=10)
#state_SM2Nav_pub = rospy.Publisher('State_SM2Nav', Int8, queue_size=10)
move_pub = rospy.Publisher('Move', Float32MultiArray, queue_size=10)
misc_angles_pub = rospy.Publisher('Misc_Angles', Float32MultiArray, queue_size=10)
init_state_pub = rospy.Publisher('Init_State', Bool, queue_size=10)


# Create subscribers
start_led_state_sub = rospy.Subscriber("LED_State", Bool, callback=start_led_callback)
arm_done_sub = rospy.Subscriber("Arm_Done", Int8, callback=state_arm2sm_cb)
misc_done = rospy.Subscriber("Misc_Done", Int8, callback=misc_done_cb)
#state_Nav2SM_sub = rospy.Subscriber("State_Nav2SM", Int8, callback=state_nav2arm_cb)
#TOF_Front = rospy.Subscriber("TOF_Front", Int16, callback=tof_front_cb)
move_done_sub = rospy.Subscriber("Move_Done", Int8, callback=move_done_cb)
gravity_vector_sub = rospy.Subscriber("IMU_Grav", Int16, callback=gravity_vector_cb)
bearing_sub = rospy.Subscriber("IMU_Bearing", Int16, callback=bearing_cb)
tof_back_sub = rospy.Subscriber("TOF_Back", Int16, callback=tof_back_cb)


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
                               #transitions={'succeeded':'PACKAGE_PICKUP', 'aborted':'INITIALIZE'})
        
        # Read the start green LED and wait for it to be detected
        smach.StateMachine.add('READING_START_LED', ReadingStartLED(), 
                               transitions={'green_led_detected': 'PACKAGE_PICKUP',
                                            'green_led_not_detected':'READING_START_LED'})
        
        # Create a concurrent state machine for package pickup
        package_pickup_sm = smach.Concurrence(outcomes=['packages_picked_up','packages_not_picked_up'],
                                    default_outcome='packages_not_picked_up',
                                    outcome_map={'packages_picked_up':{
                                        'PICK_BIG_PACKAGES':'packages_picked_up',
                                        'PICK_SMALL_PACKAGES':'packages_picked_up'
                                    }})
        with package_pickup_sm:
            small_packages_sm = smach.StateMachine(outcomes=['packages_picked_up', 'packages_not_picked_up'])

            with small_packages_sm:
                smach.StateMachine.add('SCAN_POSE', ScanPose(arm_angles_pub=arm_angles_pub),
                                        transitions={'pose_reached':'GET_SP_COORDS', 'pose_not_reached':'SCAN_POSE'})
                smach.StateMachine.add('GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose=Poses.SMALL_PACKAGE_SCAN.value),
                                        transitions={'coords_received':'VERIFY_POSE', 'coords_not_received':'GET_SP_COORDS'})
                smach.StateMachine.add('VERIFY_POSE', VerifyPose(task_space_pub=task_space_pub),
                                        transitions={'pose_reached':'DONE_POSE', 'pose_not_reached':'VERIFY_POSE'})
                smach.StateMachine.add('DONE_POSE', ScanPose(arm_angles_pub=arm_angles_pub),
                                        transitions={'pose_reached':'packages_picked_up', 'pose_not_reached':'SCAN_POSE'})
                
            big_packages_sm = smach.StateMachine(outcomes=['packages_picked_up', 'packages_not_picked_up'])

            with big_packages_sm:
                smach.StateMachine.add('MOVE_TO_BIG_PACKAGE_WALL', GoTo_(Areas.BIG_PACKAGE_WALL, move_publisher=move_pub),
                                        transitions={'arrived':'PUSH_BIG_PACKAGES', 'not_arrived':'MOVE_TO_BIG_PACKAGE_WALL'})
                smach.StateMachine.add('PUSH_BIG_PACKAGES', GoTo_(Areas.PUSH_BIG_PACKAGES, move_publisher=move_pub),
                                        transitions={'arrived':'packages_picked_up', 'not_arrived':'PUSH_BIG_PACKAGES'})

            smach.Concurrence.add('PICK_BIG_PACKAGES', PickUpBigPackages())
            smach.Concurrence.add('PICK_SMALL_PACKAGES', small_packages_sm)

        smach.StateMachine.add('PACKAGE_PICKUP', package_pickup_sm,
                                transitions={'packages_picked_up':'END',
                                            'packages_not_picked_up':'PACKAGE_PICKUP'})

        # Go to dropoff area
        smach.StateMachine.add('GO_TO_DROP_OFF_AREA', GoTo_(Areas.DROP_OFF, move_publisher=move_pub), 
                                   transitions={'arrived':'GO_TO_FUEL_TANK_AREA', 'not_arrived':'GO_TO_DROP_OFF_AREA'})
        
        # TODO: Add dropoff states

        # Go to fuel tank area
        smach.StateMachine.add('GO_TO_FUEL_TANK_AREA', GoTo_(Areas.FUEL_TANK, move_publisher=move_pub), 
                                   transitions={'arrived':'GO_TO_CRATER_AREA', 'not_arrived':'GO_TO_FUEL_TANK_AREA'})

        # TODO: Add fuel tank pickup states

      #  smach.StateMachine.add('PICK_UP_FUEL_TANKS', PickUpFuelTanks(arm_angles_publisher=arm_angles_pub),
                          #     transitions={'fuel_tanks_picked_up':'END', 'fuel_tanks_not_picked_up':'PICK_UP_FUEL_TANKS'})

        # Go to crater
        smach.StateMachine.add('GO_TO_CRATER_AREA', GoTo_(Areas.CRATER, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub), 
                                   transitions={'arrived':'GO_TO_FINAL', 'not_arrived':'GO_TO_CRATER_AREA'})
        
        smach.StateMachine.add('GO_TO_FINAL', GoTo_(Areas.BUTTON, move_publisher=move_pub), 
                               transitions={'arrived':'BUTTON_PRESS', 'not_arrived':'GO_TO_FINAL'})
        
        
        smach.StateMachine.add('SPIRIT_CELEBRATION', SpiritCelebration(misc_angles_publisher = misc_angles_pub),
                               transitions = {'succeeded':'BUTTON_PRESS', 'aborted':'SPIRIT_CELEBRATION'})

        
        smach.StateMachine.add('BUTTON_PRESS', ButtonPress(move_publisher=move_pub), 
                               transitions={'succeeded':'END', 'aborted':'BUTTON_PRESS'})
 


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