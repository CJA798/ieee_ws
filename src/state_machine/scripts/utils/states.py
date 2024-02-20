#!/usr/bin/env python

import rospy
import smach
import smach_ros
from enum import Enum

from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
import actionlib

from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback

from utils.areas import Areas
from utils.globals import gravity_vector

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
        rospy.sleep(5)

        if self.bot_initialized:
            return 'succeeded'
        
        return 'aborted'
    

# define state Store
class Store(smach.State):
    def __init__(self, task_space_pub):
        smach.State.__init__(self, outcomes=['packages_stored','packages_not_stored'])
        self.task_space_pub = task_space_pub

    def execute(self, userdata):
        task_space = Float32MultiArray()
        task_space.data = [100, 100, 0, 2048, 2200, 10]
        self.task_space_pub.publish(task_space)

        if True:
            rospy.sleep(5)
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

        rospy.sleep(0.2)
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