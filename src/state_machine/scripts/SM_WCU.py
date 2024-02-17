#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import crcmod
import sys
from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
from image_utils.board_objects import BoardObjects
from image_utils.poses import Poses
from utils.states import *


# Global variables
green_detected = False
arm_done = False
nav_state = None

class Wait4Nav(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        global nav_state
        #rospy.loginfo('Executing state Wait4Nav')
        if nav_state == Nav2SMStates.DONE.value:
            nav_state = Nav2SMStates.RESET.value
            rospy.sleep(5)
            return 'succeeded'
        rospy.loginfo('nav_state: %s', nav_state)
        return 'aborted'
    
# define state ReadingStartLED
class ReadingStartLED(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['green_led_detected','green_led_not_detected'])
        rospy.loginfo('Executing state ReadingStartLED')
        rospy.sleep(0.2)

    def execute(self, userdata):
        global green_detected

        if green_detected:
            rospy.sleep(0.2)
            return 'green_led_detected'
        return 'green_led_not_detected'


# define state SetPose
class ScanPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state ScanPose')
        global arm_done
        self.arm_angles_pub = arm_angles_pub
        arm_done = False

    def execute(self, userdata):
        global arm_done
        angles_ = Float32MultiArray()
        
        angles_.data = [2164.0, 1776.0, 1776.0, 2787.0, 2048.0, 556.0, 3147782.0, 1446.0]
        self.arm_angles_pub.publish(angles_)
        #rospy.sleep(5)

        while not arm_done:
            rospy.sleep(5)
            arm_done = False
            return 'pose_reached'
        return 'pose_not_reached'

# define state VerifyPose
class VerifyPose(smach.State):
    def __init__(self, task_space_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'],
                             input_keys=['coordinates'])
        rospy.loginfo(f'Executing state VerifyPose')
        rospy.loginfo(f'Executing state VerifyPose')

        global arm_done
        arm_done = False
        self.task_space_pub = task_space_pub

    def execute(self, userdata):
        global arm_done
        # store the coordinates list: CoordinatesList -> coordinates[coordinates]
        coordinates = userdata.coordinates.coordinates
        rospy.loginfo(f'Coordinates: {coordinates}')
        # Go to first coordinate
        task_space = Float32MultiArray()
        target = coordinates[0]
        task_space.data = [target.x, target.y, target.z, 2048, 1700]
        self.task_space_pub.publish(task_space)
        # wait 5 seconds and go to next state
        rospy.sleep(3)

        #while not arm_done:
        #    rospy.sleep(10)
        #    arm_done = False
        #    return 'pose_reached'
        return 'pose_reached'
    

# define state PickUp
class PickUp(smach.State):
    def __init__(self, task_space_pub):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'],
                             input_keys=['coordinates'],)
        global arm_done
        arm_done = False
        self.task_space_pub = task_space_pub

    def execute(self, userdata):
        global arm_done
        # store the coordinates list: CoordinatesList -> coordinates[coordinates]
        coordinates = userdata.coordinates.coordinates
        rospy.loginfo(f'Coordinates: {coordinates}')
        # Go to first coordinate
        task_space = Float32MultiArray()
        target = coordinates[0]
        task_space.data = [target.x, target.y, target.z, 2048, 2100]
        self.task_space_pub.publish(task_space)
        # wait 5 seconds and go to next state
        rospy.sleep(2)

        #while not arm_done:
        #    rospy.sleep(10)
        #    arm_done = False
        #    return 'pose_reached'
        return 'packages_picked_up'


# Create publishers
task_space_pub = rospy.Publisher('Task_Space', Float32MultiArray, queue_size=10)
arm_angles_pub = rospy.Publisher('Arm_Angles', Float32MultiArray, queue_size=10)
state_SM2Nav_pub = rospy.Publisher('State_SM2Nav', Int8, queue_size=10)

# Callback function to handle start green LED state updates
def start_led_callback(data):
    global green_detected
    try:
        green_detected = data.data
    except Exception as e:
        rospy.logerr("Error in start_led_callback: {}".format(e))

# Callback function to handle arm state updates
def state_arm2sm_cb(data):
    global arm_done
    try:
        if not arm_done:
            arm_done = data.data
        #rospy.loginfo("Arm Done: %s", arm_done)
    except Exception as e:
        rospy.logerr("Error in state_arm2sm_cb: {}".format(e))

# Callback function to handle navigation state updates
def state_nav2arm_cb(data):
    global nav_state
    try:
        nav_state = data.data
        #rospy.loginfo("Nav State: %s", nav_state)
    except Exception as e:
        rospy.logerr("Error in state_nav_cb: {}".format(e))

# Create subscribers
start_led_state_sub = rospy.Subscriber("LED_State", Bool, callback=start_led_callback)
arm_done_sub = rospy.Subscriber("Arm_Done", Int8, callback=state_arm2sm_cb)
state_Nav2SM_sub = rospy.Subscriber("State_Nav2SM", Int8, callback=state_nav2arm_cb)

# Debug for navigation. This Enables/disables the package pickup states
skip_pickup = True
test_cv = True

def main():
    rospy.init_node('STATE_MACHINE')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'succeeded':'FUEL_TANK_PICKUP' if test_cv else 'READING_START_LED', 'aborted':'INITIALIZE'})
        smach.StateMachine.add('READING_START_LED', ReadingStartLED(), 
                               transitions={'green_led_detected': 'GO_TO_DROP_OFF_AREA' if skip_pickup else 'PACKAGE_PICKUP',
                                            'green_led_not_detected':'READING_START_LED'})


        # Create the sub SMACH state machine
        package_pickup_sm = smach.Concurrence(outcomes=['packages_picked_up','aborted'],
                                    default_outcome='aborted',
                                    outcome_map={'packages_picked_up':{
                                        'PICK_BIG_PACKAGES':'packages_picked_up',
                                        'PICK_SMALL_PACKAGES':'packages_picked_up'
                                    }})

        with package_pickup_sm:
            small_packages_sm = smach.StateMachine(outcomes=['packages_picked_up', 'packages_not_picked_up'])

            with small_packages_sm:
                smach.StateMachine.add('SCAN_POSE', ScanPose(arm_angles_pub=arm_angles_pub),
                                        transitions={'pose_reached':'GET_SP_COORDS', 'pose_not_reached':'SCAN_POSE'})
                smach.StateMachine.add('GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose='SCAN'),
                                        transitions={'coords_received':'VERIFY_POSE', 'coords_not_received':'GET_SP_COORDS'})
                smach.StateMachine.add('VERIFY_POSE', VerifyPose(task_space_pub=task_space_pub),
                                        transitions={'pose_reached':'PICK_UP', 'pose_not_reached':'VERIFY_POSE'})
                #smach.StateMachine.add('VERIFY_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose='VERIFY'),
                                        #transitions={'coords_received':'PICK_UP', 'coords_not_received':'VERIFY_COORDS'})
                smach.StateMachine.add('PICK_UP', PickUp(task_space_pub=task_space_pub),
                                        transitions={'packages_picked_up':'STORE_SMALL_PACKAGE', 'packages_not_picked_up':'PICK_UP'})
                smach.StateMachine.add('STORE_SMALL_PACKAGE', Store(task_space_pub=task_space_pub),
                                        transitions={'packages_stored':'STORE_SMALL_PACKAGE', 'packages_not_stored':'STORE_SMALL_PACKAGE'})

            smach.Concurrence.add('PICK_BIG_PACKAGES', PickUpBigPackages())
            smach.Concurrence.add('PICK_SMALL_PACKAGES', small_packages_sm)

        smach.StateMachine.add('PACKAGE_PICKUP', package_pickup_sm,
                                transitions={'packages_picked_up':'GO_TO_DROP_OFF_AREA',
                                            'aborted':'INITIALIZE'})
        
        smach.StateMachine.add('GO_TO_DROP_OFF_AREA', GoTo(state_SM2Nav_pub=state_SM2Nav_pub, area=SM2NavStates.DROP_OFF_AREA.value),
                                transitions={'succeeded':'DRIVING_TO_DROP_OFF_AREA', 'aborted':'GO_TO_DROP_OFF_AREA'})
        
        smach.StateMachine.add('DRIVING_TO_DROP_OFF_AREA', Wait4Nav(),
                                transitions={'succeeded':'PACKAGE_DROP_OFF', 'aborted':'DRIVING_TO_DROP_OFF_AREA'})


        # Create the sub SMACH state machine
        package_drop_off_sm = smach.Concurrence(outcomes=['packages_dropped_off','packages_not_dropped_off'],
                                    default_outcome='packages_not_dropped_off',
                                    outcome_map={'packages_dropped_off':{
                                        'DROP_BIG_PACKAGES':'packages_dropped_off',
                                        'DROP_SMALL_PACKAGES':'packages_dropped_off'
                                    }})

        with package_drop_off_sm:
            smach.Concurrence.add('DROP_BIG_PACKAGES', DropOffBigPackages())
            smach.Concurrence.add('DROP_SMALL_PACKAGES', DropOffSmallPackages())

        smach.StateMachine.add('PACKAGE_DROP_OFF', package_drop_off_sm,
                                transitions={'packages_dropped_off':'GO_TO_FUEL_TANK_AREA',
                                            'packages_not_dropped_off':'PACKAGE_DROP_OFF'})
        


        smach.StateMachine.add('GO_TO_FUEL_TANK_AREA', GoTo(state_SM2Nav_pub=state_SM2Nav_pub, area=SM2NavStates.FUEL_TANK_AREA.value),
                                transitions={'succeeded':'DRIVING_TO_FUEL_TANK_AREA', 'aborted':'GO_TO_FUEL_TANK_AREA'})
        
        smach.StateMachine.add('DRIVING_TO_FUEL_TANK_AREA', Wait4Nav(),
                                transitions={'succeeded':'FUEL_TANK_PICKUP', 'aborted':'DRIVING_TO_FUEL_TANK_AREA'})
              

        fuel_tank_pickup_sm = smach.StateMachine(outcomes=['packages_picked_up','packages_not_picked_up'])
        with fuel_tank_pickup_sm:
            smach.StateMachine.add('GET_FT_COORDS', GetCoords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FRONT.value),
                                transitions={'coords_received':'VERIFY_POSE', 'coords_not_received':'GET_FT_COORDS'})
            smach.StateMachine.add('VERIFY_POSE', VerifyPose(task_space_pub=task_space_pub),
                                transitions={'pose_reached':'FUEL_TANK_PICKUP', 'pose_not_reached':'VERIFY_POSE'})
            smach.StateMachine.add('PICK_UP', PickUp(task_space_pub=task_space_pub),
                                transitions={'packages_picked_up':'STORE',
                                             'packages_not_picked_up':'PICK_UP'})
            smach.StateMachine.add('STORE', Store(task_space_pub=task_space_pub),
                                        transitions={'packages_stored':'packages_picked_up',
                                             'packages_not_stored':'packages_not_picked_up'})
            
        smach.StateMachine.add('FUEL_TANK_PICKUP', fuel_tank_pickup_sm,
                                transitions={'packages_picked_up':'GO_TO_THRUSTER_AREA',
                                             'packages_not_picked_up':'FUEL_TANK_PICKUP'})
        
        smach.StateMachine.add('GO_TO_THRUSTER_AREA', GoTo(state_SM2Nav_pub=state_SM2Nav_pub, area=SM2NavStates.THRUSTER_AREA.value),
                                transitions={'succeeded':'DRIVING_TO_THRUSTER_AREA', 'aborted':'GO_TO_THRUSTER_AREA'})
        
        smach.StateMachine.add('DRIVING_TO_THRUSTER_AREA', Wait4Nav(),
                                transitions={'succeeded':'END', 'aborted':'DRIVING_TO_THRUSTER_AREA'})

        
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