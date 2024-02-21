#!/usr/bin/env python

import rospy
import smach
import smach_ros
from enum import Enum

from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
import actionlib

from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback

from utils.areas import Areas
from utils.globals import globals

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

class ReadingStartLED(smach.State):
    '''State to read the start green LED status'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['green_led_detected','green_led_not_detected'])
        rospy.loginfo('Executing state ReadingStartLED')
        rospy.sleep(0.2)

    def execute(self, userdata):
        '''Execute the state logic to read the start green LED status
        
        Args:
            userdata: The data passed to the state (Not used)
        
        Returns:
            str: The outcome of the state ('green_led_detected' or 'green_led_not_detected')
                
        Raises:
            Exception: Any exception that occurs during the state execution'''
        
        rate = rospy.Rate(20)  # 10 Hz
        try:
            while not globals['green_detected'] and not rospy.is_shutdown():
                rate.sleep()
            return 'green_led_detected'
        
        except Exception as e:
            rospy.logerr("Error in ReadingStartLED: {}".format(e))
            return 'green_led_not_detected'
        

class GoTo_(smach.State):
    # Dictionary mapping Areas enum values to method names
    AREA_METHODS = {
        Areas.SECOND_SLOPE: "GoToSecondSlope",
        Areas.DROP_OFF: "GoToDropOffArea",
        Areas.FUEL_TANK: "GoToFuelTankArea",
        Areas.THRUSTER: "GoToThrusterArea"
    }

    def __init__(self, area, move_publisher):
        # Initialize the state with outcomes 'arrived' and 'not_arrived'
        smach.State.__init__(self, outcomes=['arrived', 'not_arrived'])
        self.area = area
        self.move_pub = move_publisher
        rospy.loginfo(f"Executing state GoTo{self.area}")

    def execute(self, userdata):
        # Check if the area is valid and has a corresponding action
        if self.area in self.AREA_METHODS:
            # Get the action name corresponding to the area
            method_name = self.AREA_METHODS[self.area]
            # Log the execution of the state
            rospy.loginfo(f"Executing state {method_name}")
            # Call the corresponding method dynamically using getattr
            outcome = getattr(self, method_name)()
            
            return outcome
        else:
            # Log that the area is invalid
            rospy.loginfo('Invalid area')
            return 'not_arrived'
    
    def GoToDropOffArea(self):
        rate = rospy.Rate(20)
        try:
            message = Float32MultiArray()
            message.data = [200, 1, 0, 100]
            self.move_pub.publish(message)

            while globals['gravity_vector'] > -25:
                rate.sleep()

            rospy.loginfo('Second slope reached')

            while globals['gravity_vector'] < -10:
                rate.sleep()

            message.data = [200, 200, 0, 100]
            self.move_pub.publish(message)

            while not globals['move_done']:
                rate.sleep()
            
            # Reset the move_done global variable
            globals['move_done'] = False
            return 'arrived'
        
        except Exception as e:
            rospy.logerr(f"Error in GoToDropOffArea: {e}")
            return 'not_arrived'


    def GoToFuelTankArea(self):
        rate = rospy.Rate(20)
        try:
            globals['move_done'] = False
            message = Float32MultiArray()
            message.data = [0, 0, -90, 100]
            self.move_pub.publish(message)

            while not globals['move_done']:
                rate.sleep()
            
            rospy.loginfo('Turned 90 degrees')

            globals['move_done'] = False
            message.data = [100, 270, -90, 100]
            self.move_pub.publish(message)

            while not globals['move_done']:
                rate.sleep()

            # Reset the move_done global variable
            globals['move_done'] = False
            return 'arrived'

        except Exception as e:
            rospy.logerr(f"Error in GoToFuelTankArea: {e}")
            return 'not_arrived'
        
    def GoToThrusterArea(self):
        # Placeholder method for handling the Thruster area
        pass





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