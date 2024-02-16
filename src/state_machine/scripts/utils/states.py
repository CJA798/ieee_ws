#!/usr/bin/env python

import rospy
import smach
import smach_ros
import crcmod
from enum import Enum

from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
import actionlib

from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback


class SM2NavStates(Enum):
    DROP_OFF_AREA = 0
    FUEL_TANK_AREA = 1
    THRUSTER_AREA = 2

class Nav2SMStates(Enum):
    RESET = 0
    IN_PROGRESS = 0
    DONE = 1


# define state Initialize
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.bot_initialized = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        # Run initialization logic
        self.bot_initialized = True
        rospy.sleep(0.2)

        if self.bot_initialized:
            return 'succeeded'
        
        return 'aborted'
    

# define state ReadingStartLED
class ReadingStartLED(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['green_led_detected','green_led_not_detected'])
        rospy.loginfo('Executing state ReadingStartLED')

    def execute(self, userdata):
        global green_detected
        #rospy.sleep(0.1)

        #TODO
        #if green_detected:
        if True:
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
    
# define state GetCoords
class GetCoords(smach.State):
    def __init__(self, object_type, pose):
        smach.State.__init__(self, outcomes=['coords_received','coords_not_received'],
                             output_keys=['coordinates'])
        self.object_type = object_type
        self.pose = pose
        #TODO for some reason, removing this sleep brings back the fucking
        # raise ValueError(f"Object type {object_type} not recognized.")
        # ValueError: Object type SMALL_PACKAGE not recognized.

        #rospy.sleep(5)

    def feedback_callback(self, feedback):
        rospy.loginfo(f'Current Coordinates List: {feedback.current_coordinates}')

    def execute(self, userdata):
        rospy.loginfo('Executing state GetCoords(small_packages, scan)')
        
        client = actionlib.SimpleActionClient('get_coords', GetCoordsAction)
        client.wait_for_server()

        goal = GetCoordsGoal()
        goal.timeout.data = 5.0
        goal.expected_pairs.data = 3
        goal.object_type.data = self.object_type
        goal.arm_pose.data = self.pose

        client.send_goal(goal, feedback_cb=self.feedback_callback)

        client.wait_for_result()
        result = client.get_result()
        #rospy.loginfo(f'Final Elapsed Time: {result.elapsed_time}')
        #print(result)
        rospy.loginfo(f'Final Coordinates List: {result.coordinates}    |    Total time: {result.elapsed_time.data}')

        if result.coordinates:
            userdata.coordinates = result.coordinates

        if True:
            #rospy.sleep(5)
            return 'coords_received'
        return 'coords_not_received'
    

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

# define state Store
class Store(smach.State):
    def __init__(self, task_space_pub):
        smach.State.__init__(self, outcomes=['packages_stored','packages_not_stored'])
        self.task_space_pub = task_space_pub

    def execute(self, userdata):
        task_space = Float32MultiArray()
        task_space.data = [100, 100, 0, 2048, 2100]
        self.task_space_pub.publish(task_space)

        if True:
            return 'packages_stored'
        return 'packages_not_stored'
    
# define state PickUpBigPackages
class PickUpBigPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpBigPackages')
        rospy.sleep(5)
        if True:
            return 'packages_picked_up'
        return 'packages_not_picked_up'

# define state PickUpFuelTanks
class PickUpFuelTanks(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['fuel_tanks_picked_up','fuel_tanks_not_picked_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpFuelTanks')
        rospy.sleep(5)
        if True:
            return 'fuel_tanks_picked_up'
        return 'fuel_tanks_not_picked_up'
     
# define state GoToDropOffArea
class GoTo(smach.State):
    def __init__(self, state_SM2Nav_pub, area):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.state_SM2Nav_pub = state_SM2Nav_pub
        self.area = area

    def execute(self, userdata):
        state_SM2Nav = Int8()
        state_SM2Nav.data = self.area

        rospy.sleep(7)
        self.state_SM2Nav_pub.publish(state_SM2Nav)
        rospy.loginfo('Executing state GoToDropOffArea')
        return 'succeeded'
        


# define state DropOffBigPackages
class DropOffBigPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['packages_dropped_off','packages_not_dropped_off'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state DropOffBigPackages')
        rospy.sleep(5)
        if True:
            return 'packages_dropped_off'
        return 'packages_not_dropped_off'
    
# define state DropOffSmallPackages
class DropOffSmallPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['packages_dropped_off','packages_not_dropped_off'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state DropOffSmallPackages')
        rospy.sleep(5)
        if True:
            return 'packages_dropped_off'
        return 'packages_not_dropped_off'