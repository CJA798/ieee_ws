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
#bearing_sub = rospy.Subscriber("IMU_Bearing", Int16, callback=bearing_cb)
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
                               transitions={'green_led_detected': 'PACKAGE_PICK_UP',     
                                            'green_led_not_detected':'READING_START_LED'})
        
        package_pickup_sm = smach.StateMachine(outcomes=['packages_picked_up', 'packages_not_picked_up'])

        with package_pickup_sm:
            # Pickup small packages
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
                                    transitions={'pose_reached':'SET_BULK_GRABBER_ARMS', 'pose_not_reached':'HOME_POSE'})
            
            # Sweep if necessary
            # After a sweep, a new scan is necessary to update the coordinates list
            smach.StateMachine.add('SWEEP_SMALL_PACKAGES', Sweep(task_space_pub=task_space_pub),
                                    transitions={'succeeded':'SCAN_POSE', 'aborted':'SWEEP_SMALL_PACKAGES'})
            smach.StateMachine.add('SOFT_SWEEP', Sweep(task_space_pub=task_space_pub, soft=True),
                                    transitions={'succeeded':'SCAN_POSE', 'aborted':'SOFT_SWEEP'})
            

            #################################################################################################################################
            # Pickup big packages
            smach.StateMachine.add('SET_BULK_GRABBER_ARMS', SetPose(pose=Poses.SET_BULK_GRABBER_ARMS, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'MOVE_TO_BIG_PACKAGE_WALL', 'pose_not_reached':'SET_BULK_GRABBER_ARMS'})
            smach.StateMachine.add('MOVE_TO_BIG_PACKAGE_WALL', GoTo_(Areas.BIG_PACKAGE_WALL, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'arrived':'CLOSE_TOP_BULK_GRABBER_ARM', 'not_arrived':'MOVE_TO_BIG_PACKAGE_WALL'})
            smach.StateMachine.add('CLOSE_TOP_BULK_GRABBER_ARM', SetPose(pose=Poses.CLOSE_TOP_BULK_GRABBER_ARM, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'RAISE_BULK_GRABBER', 'pose_not_reached':'CLOSE_TOP_BULK_GRABBER_ARM'})
            smach.StateMachine.add('RAISE_BULK_GRABBER', SetPose(pose=Poses.RAISE_BULK_GRABBER, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'PACKAGE_STATE_RESOLVER', 'pose_not_reached':'RAISE_BULK_GRABBER'})
            

            #################################################################################################################################
            # Determine the next state based on the userdata.
            # This state determines whether to pick up, move to re-scan, or do both after the big block pickup
            # The userdata is set by PickUpSmallPackage and the respective data keys are 'pick_after_big_packages' and 'move_after_big_packages'
            smach.StateMachine.add('PACKAGE_STATE_RESOLVER', PackageStateResolver(),
                                    transitions={'pick_after_big_packages':'POST_SCAN_POSE', 'move_after_big_packages':'MOVE_FORWARD_TO_RE_SCAN', 'packages_picked_up':'packages_picked_up'})
            

            #################################################################################################################################
            # If a package was out of range and needs to be picked up after picking up the big packages (exclusive), do this:
            smach.StateMachine.add('MOVE_FORWARD_TO_RE_SCAN', GoTo_(Areas.RE_SCAN, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub),
                                   transitions={'arrived':'RE_SCAN_POSE', 'not_arrived':'MOVE_FORWARD_TO_RE_SCAN'})
            smach.StateMachine.add('RE_SCAN_POSE', ScanPose(arm_angles_pub=arm_angles_pub),
                                    transitions={'pose_reached':'RE_GET_SP_COORDS', 'pose_not_reached':'RE_SCAN_POSE'})
            smach.StateMachine.add('RE_GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose=Poses.SMALL_PACKAGE_SCAN.value, timeout=1.5, expected_pairs=4, camera_enable_publisher=camera_enable_pub),
                                    transitions={'coords_received':'RE_PICK_UP_SP', 'coords_not_received':'RE_GET_SP_COORDS'})
            smach.StateMachine.add('RE_PICK_UP_SP', PickUpSmallPackage(task_space_pub=task_space_pub, in_re_scan=True),
                                   transitions={'packages_picked_up':'REST_POSE',
                                                'sweep_needed': 'RE_SWEEP_SMALL_PACKAGES',
                                                'soft_sweep_needed': 'RE_SOFT_SWEEP',
                                                'no_coordinates_received': 'REST_POSE'})
        
            # Sweep if necessary
            # After a sweep, a new scan is necessary to update the coordinates list
            smach.StateMachine.add('RE_SWEEP_SMALL_PACKAGES', Sweep(task_space_pub=task_space_pub),
                                    transitions={'succeeded':'SCAN_POSE', 'aborted':'SWEEP_SMALL_PACKAGES'})
            smach.StateMachine.add('RE_SOFT_SWEEP', Sweep(task_space_pub=task_space_pub, soft=True),
                                    transitions={'succeeded':'SCAN_POSE', 'aborted':'SOFT_SWEEP'})


            #################################################################################################################################
            # If a package was too close to the big packages and needs to be picked up after picking up the big packages (exclusive), do this:
            smach.StateMachine.add('POST_SCAN_POSE', ScanPose(arm_angles_pub=arm_angles_pub),
                                    transitions={'pose_reached':'POST_GET_SP_COORDS', 'pose_not_reached':'POST_SCAN_POSE'})
            smach.StateMachine.add('POST_GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose=Poses.SMALL_PACKAGE_SCAN.value, timeout=1.5, expected_pairs=3, camera_enable_publisher=camera_enable_pub),
                                    transitions={'coords_received':'POST_PICK_UP_SP', 'coords_not_received':'POST_GET_SP_COORDS'})
            smach.StateMachine.add('POST_PICK_UP_SP', PickUpSmallPackage(task_space_pub=task_space_pub, after_big_packages=True),
                                   transitions={'packages_picked_up':'POST_PACKAGE_STATE_RESOLVER',
                                                'sweep_needed': 'POST_SWEEP_SMALL_PACKAGES',
                                                'soft_sweep_needed': 'POST_SOFT_SWEEP',
                                                'no_coordinates_received': 'POST_PACKAGE_STATE_RESOLVER'})
            
            smach.StateMachine.add('POST_SWEEP_SMALL_PACKAGES', Sweep(task_space_pub=task_space_pub),
                                    transitions={'succeeded':'POST_SCAN_POSE', 'aborted':'POST_SWEEP_SMALL_PACKAGES'})
            smach.StateMachine.add('POST_SOFT_SWEEP', Sweep(task_space_pub=task_space_pub, soft=True),
                                    transitions={'succeeded':'POST_SCAN_POSE', 'aborted':'POST_SOFT_SWEEP'})

            # Determine if move to re-scan is necessary after picking up the packages that were too close to the big packages
            smach.StateMachine.add('POST_PACKAGE_STATE_RESOLVER', PackageStateResolver(),
                                    transitions={'pick_after_big_packages':'POST_SCAN_POSE', 'move_after_big_packages':'MOVE_FORWARD_TO_RE_SCAN', 'packages_picked_up':'packages_picked_up'})
        
            #################################################################################################################################
            # Set the arm to a safe (non-blocking) pose after picking up all packages
            smach.StateMachine.add('REST_POSE', RestPose(arm_angles_pub=arm_angles_pub),
                                    transitions={'pose_reached':'packages_picked_up', 'pose_not_reached':'REST_POSE'})
            

        smach.StateMachine.add('PACKAGE_PICK_UP', package_pickup_sm,
                                transitions={'packages_picked_up': 'GO_TO_DROP_OFF_AREA',
                                            'packages_not_picked_up':'PACKAGE_PICK_UP'})
        
        
        # Go to dropoff area
        smach.StateMachine.add('GO_TO_DROP_OFF_AREA', GoTo_(Areas.DROP_OFF,
                                                            move_publisher=move_pub,
                                                            arm_angles_publisher=arm_angles_pub,
                                                            task_space_publisher=task_space_pub,
                                                            misc_angles_publisher=misc_angles_pub,
                                                            grav_enable_publisher=grav_enable_pub), 
                               transitions={'arrived':'PACKAGE_DROP_OFF', 'not_arrived':'GO_TO_DROP_OFF_AREA'})
                               #transitions={'arrived':'GO_TO_FUEL_TANK_AREA', 'not_arrived':'GO_TO_DROP_OFF_AREA'})

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
                #smach.StateMachine.add('DROP_OFF_SMALL_PACKAGES_POSE', SetPose(pose=Poses.DROP_OFF_SMALL_PACKAGES, arm_angles_publisher=arm_angles_pub),
                                        #transitions={'pose_reached':'RELEASE', 'pose_not_reached':'DROP_OFF_SMALL_PACKAGES_POSE'})
                #smach.StateMachine.add('RELEASE', SetPose(pose=Poses.RELEASE_SMALL_PACKAGES, arm_angles_publisher=arm_angles_pub),
                                        #transitions={'pose_reached':'FUEL_TANK_SCAN_POSE', 'pose_not_reached':'RELEASE'})
                #smach.StateMachine.add('FUEL_TANK_SCAN_POSE', SetPose(pose=Poses.FUEL_TANK_SCAN, arm_angles_publisher=arm_angles_pub),
                                        #transitions={'pose_reached':'packages_dropped_off', 'pose_not_reached':'FUEL_TANK_SCAN_POSE'})
                smach.StateMachine.add('DROP_OFF_PACKAGES', DropOff(BoardObjects.SMALL_PACKAGE, arm_angles_publisher=arm_angles_pub, misc_angles_publisher=misc_angles_pub, task_space_publisher=task_space_pub),
                                    transitions={'packages_dropped_off':'packages_dropped_off', 'packages_not_dropped_off':'DROP_OFF_PACKAGES'})

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
        smach.StateMachine.add('GO_TO_FUEL_TANK_AREA', GoTo_(Areas.FUEL_TANK, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub, grav_enable_publisher=grav_enable_pub), 
                                       transitions={'arrived':'PICK_UP_FUEL_TANKS', 'not_arrived':'GO_TO_FUEL_TANK_AREA'})

        # Pick up fuel tanks
        smach.StateMachine.add('PICK_UP_FUEL_TANKS', PickUp(board_object=BoardObjects.FUEL_TANK, arm_angles_publisher=arm_angles_pub, misc_angles_publisher=misc_angles_pub, move_publisher=move_pub),
                                      transitions={'packages_picked_up':'FUEL_TANK_SORT_AND_FINAL_AREA', 'packages_not_picked_up':'PICK_UP_FUEL_TANKS'})
              
        # Create a concurrent state machine for movement to final area and fuel tank sorting
        fuel_tank_sort_and_final_area_sm = smach.Concurrence(outcomes=['succeeded','aborted'],
                                    default_outcome='aborted',
                                    outcome_map={'succeeded':{
                                        'STORE_FUEL_TANKS':'fuel_tanks_stored',
                                        'GO_TO_FINAL':'arrived'
                                    }})
        
        with fuel_tank_sort_and_final_area_sm:
            # State machine for fuel tank sorting       
            fuel_tank_sort_sm = smach.StateMachine(outcomes=['fuel_tanks_stored', 'fuel_tanks_not_stored'])
            with fuel_tank_sort_sm:
                smach.StateMachine.add('SCAN_FUEL_TANK_POSE', ScanFuelTankPose(arm_angles_pub=arm_angles_pub),
                                        transitions={'pose_reached':'GET_FT_COORDS', 'pose_not_reached':'SCAN_FUEL_TANK_POSE'})
                smach.StateMachine.add('GET_FT_COORDS', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value, timeout=1.5, expected_pairs=3, camera_enable_publisher=camera_enable_pub),
                                        transitions={'coords_received':'PICK_UP_FUEL_TANK', 'coords_not_received':'GET_FT_COORDS'})
                smach.StateMachine.add('PICK_UP_FUEL_TANK', PickUpFuelTank(task_space_publisher=task_space_pub),
                                        transitions={'fuel_tank_picked_up':'STORE_FUEL_TANK', 'fuel_tank_not_picked_up':'PICK_UP_FUEL_TANK'})
                smach.StateMachine.add('STORE_FUEL_TANK', StoreFuelTank(task_space_publisher=task_space_pub, arm_angles_publisher=arm_angles_pub, slot_number=3),
                                        transitions={'fuel_tank_stored':'GET_FT_COORDS2', 'fuel_tank_not_stored':'STORE_FUEL_TANK'})
                
                
                smach.StateMachine.add('GET_FT_COORDS2', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value, timeout=1.5, expected_pairs=2, camera_enable_publisher=camera_enable_pub),
                                        transitions={'coords_received':'PICK_UP_FUEL_TANK2', 'coords_not_received':'GET_FT_COORDS2'})
                smach.StateMachine.add('PICK_UP_FUEL_TANK2', PickUpFuelTank(task_space_publisher=task_space_pub),
                                        transitions={'fuel_tank_picked_up':'STORE_FUEL_TANK2', 'fuel_tank_not_picked_up':'PICK_UP_FUEL_TANK2'})
                smach.StateMachine.add('STORE_FUEL_TANK2', StoreFuelTank(task_space_publisher=task_space_pub, arm_angles_publisher=arm_angles_pub, slot_number=2),
                                        transitions={'fuel_tank_stored':'GET_FT_COORDS3', 'fuel_tank_not_stored':'STORE_FUEL_TANK2'})

                
                smach.StateMachine.add('GET_FT_COORDS3', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value, timeout=1.5, expected_pairs=1, camera_enable_publisher=camera_enable_pub),
                                        transitions={'coords_received':'PICK_UP_FUEL_TANK3', 'coords_not_received':'GET_FT_COORDS3'})
                smach.StateMachine.add('PICK_UP_FUEL_TANK3', PickUpFuelTank(task_space_publisher=task_space_pub),
                                        transitions={'fuel_tank_picked_up':'STORE_FUEL_TANK3', 'fuel_tank_not_picked_up':'PICK_UP_FUEL_TANK3'})
                smach.StateMachine.add('STORE_FUEL_TANK3', StoreFuelTank(task_space_publisher=task_space_pub, arm_angles_publisher=arm_angles_pub, slot_number=1),
                                        transitions={'fuel_tank_stored':'fuel_tanks_stored', 'fuel_tank_not_stored':'STORE_FUEL_TANK3'})
                
                smach.StateMachine.add('REST_POSE', RestPose(arm_angles_pub=arm_angles_pub),
                                    transitions={'pose_reached':'fuel_tanks_stored', 'pose_not_reached':'REST_POSE'})
            
    

            # State machine for movement to final area
            go_to_final_sm = smach.StateMachine(outcomes=['arrived', 'not_arrived'])
            with go_to_final_sm:
                # Go to crater and deploy bridge
                smach.StateMachine.add('GO_TO_CRATER_AREA', GoTo_(Areas.CRATER, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub, grav_enable_publisher=grav_enable_pub), 
                                        transitions={'arrived':'GO_TO_FINAL', 'not_arrived':'GO_TO_CRATER_AREA'})
                # Cross bridge and go to final area
                smach.StateMachine.add('GO_TO_FINAL', GoTo_(Areas.BUTTON, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub), 
                                    transitions={'arrived':'arrived', 'not_arrived':'GO_TO_FINAL'})
                
            smach.Concurrence.add('STORE_FUEL_TANKS', fuel_tank_sort_sm)
            smach.Concurrence.add('GO_TO_FINAL', go_to_final_sm)

        
        smach.StateMachine.add('FUEL_TANK_SORT_AND_FINAL_AREA', fuel_tank_sort_and_final_area_sm,
                                transitions={'succeeded':'FUEL_TANKS_PLACING', 'aborted':'FUEL_TANK_SORT_AND_FINAL_AREA'})
        

        smach.StateMachine.add('FUEL_TANKS_PLACING', FuelTankPlacer(task_space_publisher=task_space_pub),
                               transitions = {'fuel_tanks_placed':'SPIRIT_CELEBRATION', 'fuel_tanks_not_placed':'FUEL_TANKS_PLACING'})
        
        
        smach.StateMachine.add('SPIRIT_CELEBRATION', SpiritCelebration(misc_angles_publisher = misc_angles_pub, arm_angles_publisher=arm_angles_pub),
                               transitions = {'succeeded':'BUTTON_PRESS', 'aborted':'SPIRIT_CELEBRATION'})

        
        smach.StateMachine.add('BUTTON_PRESS', ButtonPress(move_publisher=move_pub), 
                               transitions={'succeeded':'END', 'aborted':'BUTTON_PRESS'})
 

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

    # Terminate the program
    rospy.signal_shutdown('State machine execution completed.')  # Gracefully shutdown ROS
    sys.exit(0)  # Exit the program with exit code 0 (indicating successful termination)


if __name__ == '__main__':
    main()