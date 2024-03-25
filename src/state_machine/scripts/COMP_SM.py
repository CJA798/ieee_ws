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
                               transitions={'pose_reached':'PACKAGE_PICK_UP', 'pose_not_reached':'SET_INITIAL_ARMS'})
        
        # Package pickup state machine
        package_pickup_sm = smach.StateMachine(outcomes=['packages_picked_up', 'packages_not_picked_up'])

        with package_pickup_sm:
            # Scan initial area before path planning
            smach.StateMachine.add('MAIN_SCAN', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value,
                                                          pose=Poses.MAIN_SCAN.value,
                                                          timeout=1,
                                                          expected_pairs=3,
                                                          camera_enable_publisher=camera_enable_pub),
                                    transitions={'coords_received':'PATH_PLANNING', 'coords_not_received':'MAIN_SCAN'})
            # Figure out the path planning for the robot
            smach.StateMachine.add('PATH_PLANNING', PathPlanning(),
                                    transitions={'path_found':'SET_BULK_GRABBER_ARMS', 'path_not_found':'PATH_PLANNING'})
            
            # Pickup big packages
            smach.StateMachine.add('SET_BULK_GRABBER_ARMS', SetPose(pose=Poses.SET_BULK_GRABBER_ARMS, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'MOVE_TO_BIG_PACKAGE_WALL', 'pose_not_reached':'SET_BULK_GRABBER_ARMS'})
            smach.StateMachine.add('MOVE_TO_BIG_PACKAGE_WALL', GoTo_(Areas.BIG_PACKAGE_WALL, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'arrived':'CLOSE_TOP_BULK_GRABBER_ARM', 'not_arrived':'MOVE_TO_BIG_PACKAGE_WALL'})
            smach.StateMachine.add('CLOSE_TOP_BULK_GRABBER_ARM', SetPose(pose=Poses.CLOSE_TOP_BULK_GRABBER_ARM, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'MINI_RAISE_BULK_GRABBER', 'pose_not_reached':'CLOSE_TOP_BULK_GRABBER_ARM'})
            smach.StateMachine.add('MINI_RAISE_BULK_GRABBER', SetPose(pose=Poses.MINI_RAISE_BULK_GRABBER, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'GO_TO_NEXT_AREA', 'pose_not_reached':'MINI_RAISE_BULK_GRABBER'})
            
            smach.StateMachine.add('GO_TO_NEXT_AREA', PathResolver(move_publisher=move_pub),
                                   transitions={'area_reached':'SP_SCAN',
                                                'go_to_dropoff':'BACK_TO_INITIAL_AREA'})
            # Pickup small packages
            # Set the arm in small package scan pose
            smach.StateMachine.add('SP_SCAN', PickupScanPose(arm_angles_pub=arm_angles_pub),
                                    transitions={'pose_reached':'SP_GET_COORDS', 'pose_not_reached':'SP_SCAN'})
            smach.StateMachine.add('SP_GET_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose=Poses.SMALL_PACKAGE_SCAN.value, timeout=0.5, expected_pairs=3, camera_enable_publisher=camera_enable_pub),
                                    transitions={'coords_received':'GO_TO_NEXT_AREA', 'coords_not_received':'SP_GET_COORDS'})
            #smach.StateMachine.add('PICK_UP_SMALL_PACKAGES', PickUpSmallPackage(task_space_pub=task_space_pub),
                                    #transitions={'packages_picked_up':'GO_TO_NEXT_AREA', 'no_coordinates_received':'GO_TO_NEXT_AREA'})
            
            
            
            #smach.StateMachine.add('SCAN_SMALL_PACKAGES', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose=Poses.SMALL_PACKAGE_SCAN.value, timeout=0.5, expected_pairs=3, camera_enable_publisher=camera_enable_pub),
            #                        transitions={'coords_received':'PICK_UP_SMALL_PACKAGES', 'coords_not_received':'SCAN_SMALL_PACKAGES'})
            #smach.StateMachine.add('PICK_UP_SMALL_PACKAGES', PickUp(board_object=BoardObjects.SMALL_PACKAGE, arm_angles_publisher=arm_angles_pub, misc_angles_publisher=misc_angles_pub, move_publisher=move_pub),
            #                        transitions={'packages_picked_up':'GO_TO_NEXT_AREA', 'packages_not_picked_up':'PICK_UP_SMALL_PACKAGES'})


            
            
            smach.StateMachine.add('BACK_TO_INITIAL_AREA', GoTo_(Areas.INITIAL_AREA, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'arrived':'RAISE_BULK_GRABBER', 'not_arrived':'MOVE_TO_BIG_PACKAGE_WALL'})
            smach.StateMachine.add('RAISE_BULK_GRABBER', SetPose(pose=Poses.RAISE_BULK_GRABBER, move_publisher=move_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'pose_reached':'packages_picked_up', 'pose_not_reached':'RAISE_BULK_GRABBER'})
            
    
        # Pickup small packages
        smach.StateMachine.add('PACKAGE_PICK_UP', package_pickup_sm,
                                transitions={'packages_picked_up': 'GO_TO_DROP_OFF_AREA',#'GO_TO_DROP_OFF_AREA',
                                            'packages_not_picked_up':'PACKAGE_PICK_UP'})
        

        # Go to drop off area
        smach.StateMachine.add('GO_TO_DROP_OFF_AREA', GoTo_(Areas.DROP_OFF,
                                                            move_publisher=move_pub,
                                                            arm_angles_publisher=arm_angles_pub,
                                                            task_space_publisher=task_space_pub,
                                                            misc_angles_publisher=misc_angles_pub,
                                                            grav_enable_publisher=grav_enable_pub), 
                               transitions={'arrived':'PACKAGE_DROP_OFF', 'not_arrived':'GO_TO_DROP_OFF_AREA'})

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
                #smach.StateMachine.add('ADJUST_TO_RED_CORNER', GoTo_(Areas.RED_CORNER, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub, misc_angles_publisher=misc_angles_pub),
                #                    transitions={'arrived':'DROP_OFF_PACKAGES', 'not_arrived':'ADJUST_TO_RED_CORNER'})
                smach.StateMachine.add('DROP_OFF_PACKAGES', DropOff(BoardObjects.SMALL_PACKAGE, arm_angles_publisher=arm_angles_pub, misc_angles_publisher=misc_angles_pub, task_space_publisher=task_space_pub),
                                    transitions={'packages_dropped_off':'packages_dropped_off', 'packages_not_dropped_off':'DROP_OFF_PACKAGES'})

            big_packages_sm = smach.StateMachine(outcomes=['packages_dropped_off', 'packages_not_dropped_off'])

            with big_packages_sm:
                smach.StateMachine.add('DROP_OFF_PACKAGES', DropOff(BoardObjects.BIG_PACKAGE, arm_angles_publisher=arm_angles_pub, misc_angles_publisher=misc_angles_pub),
                                    transitions={'packages_dropped_off':'packages_dropped_off', 'packages_not_dropped_off':'DROP_OFF_PACKAGES'})

            smach.Concurrence.add('DROP_BIG_PACKAGES', big_packages_sm)
            smach.Concurrence.add('DROP_SMALL_PACKAGES', small_packages_sm)

        # Dropoff packages
        smach.StateMachine.add('PACKAGE_DROP_OFF', package_dropoff_sm,
                                transitions={'packages_dropped_off':'GO_TO_FUEL_TANK_AREA',
                                            'packages_not_dropped_off':'PACKAGE_DROP_OFF'})

        # Go to fuel tank area
        smach.StateMachine.add('GO_TO_FUEL_TANK_AREA', GoTo_(Areas.FUEL_TANK, move_publisher=move_pub, grav_enable_publisher=grav_enable_pub, misc_angles_publisher=misc_angles_pub), 
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
                smach.StateMachine.add('GET_FT_COORDS', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value, timeout=0.5, expected_pairs=3, camera_enable_publisher=camera_enable_pub),
                                        transitions={'coords_received':'PICK_UP_FUEL_TANK', 'coords_not_received':'GET_FT_COORDS'})
                smach.StateMachine.add('PICK_UP_FUEL_TANK', PickUpFuelTank(task_space_publisher=task_space_pub),
                                        transitions={'fuel_tank_picked_up':'STORE_FUEL_TANK', 'fuel_tank_not_picked_up':'PICK_UP_FUEL_TANK'})
                smach.StateMachine.add('STORE_FUEL_TANK', StoreFuelTank(task_space_publisher=task_space_pub, arm_angles_publisher=arm_angles_pub, slot_number=3),
                                        transitions={'fuel_tank_stored':'GET_FT_COORDS2', 'fuel_tank_not_stored':'STORE_FUEL_TANK'})
                
                
                smach.StateMachine.add('GET_FT_COORDS2', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value, timeout=0.5, expected_pairs=2, camera_enable_publisher=camera_enable_pub),
                                        transitions={'coords_received':'PICK_UP_FUEL_TANK2', 'coords_not_received':'GET_FT_COORDS2'})
                smach.StateMachine.add('PICK_UP_FUEL_TANK2', PickUpFuelTank(task_space_publisher=task_space_pub),
                                        transitions={'fuel_tank_picked_up':'STORE_FUEL_TANK2', 'fuel_tank_not_picked_up':'PICK_UP_FUEL_TANK2'})
                smach.StateMachine.add('STORE_FUEL_TANK2', StoreFuelTank(task_space_publisher=task_space_pub, arm_angles_publisher=arm_angles_pub, slot_number=2),
                                        transitions={'fuel_tank_stored':'GET_FT_COORDS3', 'fuel_tank_not_stored':'STORE_FUEL_TANK2'})

                
                smach.StateMachine.add('GET_FT_COORDS3', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value, timeout=0.5, expected_pairs=1, camera_enable_publisher=camera_enable_pub),
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

        # Movement to final area and fuel tank sorting
        smach.StateMachine.add('FUEL_TANK_SORT_AND_FINAL_AREA', fuel_tank_sort_and_final_area_sm,
                                transitions={'succeeded':'SET_BULK_GRABBER_ARMS_DOWN', 'aborted':'FUEL_TANK_SORT_AND_FINAL_AREA'})
        

        smach.StateMachine.add('SET_BULK_GRABBER_ARMS_DOWN', DropOff(BoardObjects.BIG_PACKAGE, misc_angles_publisher=misc_angles_pub),
                                transitions={'packages_dropped_off':'FUEL_TANKS_PLACING', 'packages_not_dropped_off':'SET_BULK_GRABBER_ARMS_DOWN'})

        smach.StateMachine.add('FUEL_TANKS_PLACING', FuelTankPlacer(task_space_publisher=task_space_pub),
                               transitions = {'fuel_tanks_placed':'SPIRIT_CELEBRATION_AND_BUTTON_PRESS', 'fuel_tanks_not_placed':'FUEL_TANKS_PLACING'})
        
        # Spirit celebration and button press setup
        spirit_celebration_and_button_press_sm = smach.Concurrence(outcomes=['succeeded','aborted'],
                                    default_outcome='aborted',
                                    outcome_map={'succeeded':{
                                        'SPIRIT_CELEBRATION':'succeeded',
                                        'BUTTON_PRESS':'succeeded'
                                    }})
        
        with spirit_celebration_and_button_press_sm:

            spirit_celebration_sm = smach.StateMachine(outcomes=['succeeded','aborted'])
            with spirit_celebration_sm:
                smach.StateMachine.add('SPIRIT_CELEBRATION', SpiritCelebration(misc_angles_publisher = misc_angles_pub, arm_angles_publisher=arm_angles_pub),
                                    transitions = {'flag_raised':'succeeded', 'flag_not_raised':'SPIRIT_CELEBRATION'})

            button_press_sm = smach.StateMachine(outcomes=['succeeded','aborted'])
            with button_press_sm:
                smach.StateMachine.add('BUTTON_PRESS', ButtonPress(move_publisher=move_pub), 
                                    transitions={'button_pressed':'succeeded', 'button_not_pressed':'BUTTON_PRESS'})

            smach.Concurrence.add('SPIRIT_CELEBRATION', spirit_celebration_sm)
            smach.Concurrence.add('BUTTON_PRESS', button_press_sm)
        # Spirit celebration and button press
        smach.StateMachine.add('SPIRIT_CELEBRATION_AND_BUTTON_PRESS', spirit_celebration_and_button_press_sm,
                                transitions={'succeeded':'END', 'aborted':'SPIRIT_CELEBRATION_AND_BUTTON_PRESS'})
        
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
