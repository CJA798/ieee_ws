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
move_pub = rospy.Publisher('Move', Float32MultiArray, queue_size=10)
misc_angles_pub = rospy.Publisher('Misc_Angles', Float32MultiArray, queue_size=10)
init_state_pub = rospy.Publisher('Init_State', Bool, queue_size=10)


# Create subscribers
start_led_state_sub = rospy.Subscriber("LED_State", Bool, callback=start_led_callback)
arm_done_sub = rospy.Subscriber("Arm_Done", Int8, callback=state_arm2sm_cb)
misc_done = rospy.Subscriber("Misc_Done", Int8, callback=misc_done_cb)
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
                                        transitions={'pose_reached':'REST_POSE', 'pose_not_reached':'VERIFY_POSE'})
                smach.StateMachine.add('REST_POSE', RestPose(arm_angles_pub=arm_angles_pub),
                                        transitions={'pose_reached':'packages_picked_up', 'pose_not_reached':'REST_POSE'})
                
            big_packages_sm = smach.StateMachine(outcomes=['packages_picked_up', 'packages_not_picked_up'])

            with big_packages_sm:
                # TODO:
                # Lower top grabber arm a bit -> Done, needs fine-tunning
                # Move to big package wall -> Done, needs fine-tunning
                # Move forward a little bit
                # Finish moving top arm -> Done, needs fine-tunning
                # Go back to start while raising bulk grabber arms -> Done, will be modified as needed
                # Set 'big_package_pick_up' flag to True -> Done
                smach.StateMachine.add('SET_BULK_GRABBER_ARMS', SetPose(pose=Poses.SET_BULK_GRABBER_ARMS, misc_angles_publisher=misc_angles_pub),
                                        transitions={'pose_reached':'MOVE_TO_BIG_PACKAGE_WALL', 'pose_not_reached':'SET_BULK_GRABBER_ARMS'})
                smach.StateMachine.add('MOVE_TO_BIG_PACKAGE_WALL', GoTo_(Areas.BIG_PACKAGE_WALL, move_publisher=move_pub),
                                        transitions={'arrived':'CLOSE_TOP_BULK_GRABBER_ARM', 'not_arrived':'MOVE_TO_BIG_PACKAGE_WALL'})
                
                # TODO: figure out why PUSH_BIG_PACKAGES makes the bot rotate a bit instead of just going straight
                # TODO: add this state cack when the above is fixed
                #smach.StateMachine.add('PUSH_BIG_PACKAGES', GoTo_(Areas.PUSH_BIG_PACKAGES, move_publisher=move_pub),
                                        #transitions={'arrived':'packages_picked_up', 'not_arrived':'PUSH_BIG_PACKAGES'})

                smach.StateMachine.add('CLOSE_TOP_BULK_GRABBER_ARM', SetPose(pose=Poses.CLOSE_TOP_BULK_GRABBER_ARM, misc_angles_publisher=misc_angles_pub),
                                        transitions={'pose_reached':'RAISE_BULK_GRABBER', 'pose_not_reached':'CLOSE_TOP_BULK_GRABBER_ARM'})
                smach.StateMachine.add('RAISE_BULK_GRABBER', SetPose(pose=Poses.RAISE_BULK_GRABBER, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub),
                                        transitions={'pose_reached':'packages_picked_up', 'pose_not_reached':'RAISE_BULK_GRABBER'})
            
            smach.Concurrence.add('PICK_BIG_PACKAGES', big_packages_sm)
            #smach.Concurrence.add('PICK_BIG_PACKAGES', PickUpBigPackages())
            smach.Concurrence.add('PICK_SMALL_PACKAGES', small_packages_sm)
            #smach.Concurrence.add('PICK_SMALL_PACKAGES', PickUpBigPackages())


        smach.StateMachine.add('PACKAGE_PICKUP', package_pickup_sm,
                                transitions={'packages_picked_up':'GO_TO_DROP_OFF_AREA',
                                            'packages_not_picked_up':'PACKAGE_PICKUP'})

        # Go to dropoff area
        smach.StateMachine.add('GO_TO_DROP_OFF_AREA', GoTo_(Areas.DROP_OFF, move_publisher=move_pub), 
                                transitions={'arrived':'PACKAGE_DROP_OFF', 'not_arrived':'GO_TO_DROP_OFF_AREA'})
                               #transitions={'arrived':'END', 'not_arrived':'GO_TO_DROP_OFF_AREA'})

        # Create a concurrent state machine for package drop off
        package_dropoff_sm = smach.Concurrence(outcomes=['packages_dropped_off','packages_not_dropped_off'],
                                    default_outcome='packages_not_dropped_off',
                                    outcome_map={'packages_dropped_off':{
                                        'DROP_BIG_PACKAGES':'packages_dropped_off',
                                        'DROP_SMALL_PACKAGES':'packages_dropped_off'
                                    }})
        
        with package_dropoff_sm:
            small_packages_sm = smach.StateMachine(outcomes=['packages_dropped_off', 'packages_not_dropped_off'])
            
            with small_packages_sm:
                smach.StateMachine.add('DROP_OFF_SMALL_PACKAGES_POSE', SetPose(pose=Poses.DROP_OFF_SMALL_PACKAGES, arm_angles_publisher=arm_angles_pub),
                                        transitions={'pose_reached':'RELEASE', 'pose_not_reached':'DROP_OFF_SMALL_PACKAGES_POSE'})
                smach.StateMachine.add('RELEASE', SetPose(pose=Poses.RELEASE_SMALL_PACKAGES, arm_angles_publisher=arm_angles_pub),
                                        transitions={'pose_reached':'FUEL_TANK_SCAN_POSE', 'pose_not_reached':'RELEASE'})
                smach.StateMachine.add('FUEL_TANK_SCAN_POSE', SetPose(pose=Poses.FUEL_TANK_SCAN, arm_angles_publisher=arm_angles_pub),
                                        transitions={'pose_reached':'packages_dropped_off', 'pose_not_reached':'FUEL_TANK_SCAN_POSE'})
            
            big_packages_sm = smach.StateMachine(outcomes=['packages_dropped_off', 'packages_not_dropped_off'])

            with big_packages_sm:
                # TODO: Add dropoff state to fix bulk grabber arm positions, so that it doesn't cover TOF_Right
                smach.StateMachine.add('DROP_OFF_PACKAGES', DropOff(BoardObjects.BIG_PACKAGE, arm_angles_publisher=arm_angles_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'packages_dropped_off':'packages_dropped_off', 'packages_not_dropped_off':'DROP_OFF_PACKAGES'})

            smach.Concurrence.add('DROP_BIG_PACKAGES', big_packages_sm)
            smach.Concurrence.add('DROP_SMALL_PACKAGES', small_packages_sm)

        smach.StateMachine.add('PACKAGE_DROP_OFF', package_dropoff_sm,
                                transitions={'packages_dropped_off':'GO_TO_FUEL_TANK_AREA',
                                            'packages_not_dropped_off':'PACKAGE_DROP_OFF'})

        # Go to fuel tank area
        smach.StateMachine.add('GO_TO_FUEL_TANK_AREA', GoTo_(Areas.FUEL_TANK, move_publisher=move_pub), 
                                   transitions={'arrived':'GO_TO_CRATER_AREA', 'not_arrived':'GO_TO_FUEL_TANK_AREA'})

        # TODO: Add fuel tank pickup states

      # smach.StateMachine.add('PICK_UP_FUEL_TANKS', PickUpFuelTanks(arm_angles_publisher=arm_angles_pub),
                          #        transitions={'fuel_tanks_picked_up':'END', 'fuel_tanks_not_picked_up':'PICK_UP_FUEL_TANKS'})

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