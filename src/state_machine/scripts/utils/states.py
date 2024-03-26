#!/usr/bin/env python

import rospy
import smach
import smach_ros

from typing import Optional, Iterable, Callable
from rospy import Publisher
from rospy.msg import AnyMsg

from enum import Enum
from math import sqrt, atan2, sin, cos, degrees, radians

from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
import actionlib

from vision_system.msg import CoordinatesList, GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback

from utils.areas import Areas
from utils.globals import globals
from utils.callbacks import get_coords_fb_cb
from utils.fuel_tank_utils import fuel_tanks

from image_utils.poses import Poses
from image_utils.board_objects import BoardObjects


def publish_command(pub: Publisher, message_type: AnyMsg, message_data: Iterable, delay: Optional[int]=0) -> bool:
    """
    Publishes a command message using the provided publisher.

    Args:
    - pub (Publisher): The ROS publisher object.
    - message_type  AnyMsg): The message type of the command.
    - message_data (Any): The data to be included in the command message.
    - delay (int): The delay to wait after publishing the message (in seconds).

    Returns:
    - bool: True if the message was successfully published, False otherwise.
    """
    try:
        # Create the message
        message = message_type()
        message.data = message_data

        # Publish the message
        pub.publish(message)

        # If a delay is provided, wait for the delay
        if 0 < delay < 10:
            rospy.loginfo(f"Waiting for {delay} seconds")
            rospy.sleep(delay)

        return True
    except Exception as e:
        rospy.logerr(f"Error in publish_command: {e}")
        return False


def publish_and_wait(pub: Publisher, wait_for_topic: str, message_type: AnyMsg, message_data: Iterable, delay: int=1, timeout_function: Callable=None) -> bool:
    message = message_type()
    message.data = message_data
    pub.publish(message)

    try:
        rospy.wait_for_message(wait_for_topic, message_type, timeout=delay)

    except:
        if timeout_function:
            rospy.loginfo(f"Executing timeout function {timeout_function.__name__} for {wait_for_topic}")
            try:
                timeout_function()
            except Exception as e:
                rospy.logerr(f"Error in timeout function {timeout_function.__name__}: {e}")
        else:
            rospy.logwarn(f"Ignoring timeout waiting for message on topic {wait_for_topic}...")
    
    return True
    

def stop_move(move_pub: Publisher) -> bool:
    try:
        return publish_command(move_pub, Float32MultiArray, [0, 0, 0, 0])
    except Exception as e:
        rospy.logerr(f"Error in stop_move: {e}")
        return False
    

# define state Initialize
class Initialize(smach.State):
    ''' State to initialize the robot'''
    def __init__(self, init_state_pub):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.init_state_pub = init_state_pub

    def execute(self, userdata):
        '''Delay simulate the initialization of the robot
        
        Args:
            userdata: The data passed to the state (Not used)
        
        Returns:
            str: The outcome of the state ('succeeded' or 'aborted')
        
        Raises:
            Exception: Any exception that occurs during the state execution'''
        try:
            rospy.loginfo('Executing state Initialize')

            # Check if the heartbeat is received, i.e. the Arduino is connected
            if not globals['heartbeat_on']:
                rospy.logerr_once('Heartbeat not received')
                rospy.sleep(2)
                return 'aborted'
            
            # Publish the init message to the Arduino
            rospy.wait_for_message("Heartbeat", Bool, timeout=5)
            init_msg = Bool()
            init_msg.data = True
            self.init_state_pub.publish(init_msg)
            rospy.loginfo('Init message published')
            return 'succeeded'

        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr("Error in Initialize: {}".format(e))
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
        try:
            rospy.wait_for_message("LED_State", Bool, timeout=None)
            rospy.loginfo('Green LED detected')
            return 'green_led_detected'
        except:
            rospy.logerr_once('Green LED not detected')
            return 'green_led_not_detected'
              
class EmergencyStop(smach.State):
    def __init__(self, move_publisher, misc_angles_publisher, task_space_publisher):
        smach.State.__init__(self, outcomes=['stopped','not_stopped'])
        self.move_pub = move_publisher
        self.misc_angles_pub = misc_angles_publisher
        self.task_space_pub = task_space_publisher
        rospy.logfatal_once('Executing state EmergencyStop')

    def execute(self, userdata):
        try:
            # Stop the robot
            rospy.logfatal('Stopping the robot')
            publish_command(self.move_pub, Float32MultiArray, [0, 0, 0, 0])
            rospy.logfatal('Move stopped')

            # Reset the arm
            angles = [100, 50, 0, 2048, 2048, 20, 10]
            publish_command(self.task_space_publisher, Float32MultiArray, angles)
            rospy.logfatal('Arm stopped')

            # Reset misc angles
            publish_command(self.misc_angles_pub, Float32MultiArray, [-1, -1, -1, -1])
            rospy.logfatal('Misc angles stopped')
            return 'stopped'
        
        except Exception as e:
            rospy.logerr(f"Error in EmergencyStop: {e}")
            return 'not_stopped'
        

class EmergencyGoToDropoff(smach.State):
    '''State to move the robot to the drop off area in case of emergency'''
    def __init__(self, move_publisher):
        smach.State.__init__(self, outcomes=['arrived','ERROR'])
        self.move_pub = move_publisher
        rospy.loginfo('Executing state EmergencyGoToDropoff')

    def execute(self, userdata):
        '''Execute the state logic to move the robot to the drop off area in case of emergency
        
        Args:
            userdata: The data passed to the state (Not used)
        
        Returns:
            str: The outcome of the state ('arrived' or 'ERROR')
        
        Raises:
            Exception: Any exception that occurs during the state execution'''
        try:
            # Publish command to go a bit past the dropoff area
            rospy.loginfo('Moving to drop off area')
            if publish_and_wait(pub=self.move_pub,
                                wait_for_topic="Move_Done",
                                message_type=Float32MultiArray,
                                message_data=[200, 130, 0, 100],
                                delay=20,
                                timeout_function=None):
                rospy.loginfo('Arrived to drop off area')
                return 'arrived'
            else:
                rospy.logerror('Move to drop off area failed')
                return 'ERROR'
        except Exception as e:
            rospy.logerr(f"Error in EmergencyGoToDropoff: {e}")
            return 'ERROR'
        

class GoTo_(smach.State):
    # Dictionary mapping areas to method names
    AREA_METHODS = {
        Areas.INITIAL_AREA: "GoToInitialArea",
        Areas.BIG_PACKAGE_WALL: "GoToBigPackageWall",
        Areas.PUSH_BIG_PACKAGES: "GoToPushBigPackages",
        Areas.RE_SCAN: "GoToReScan",
        Areas.DROP_OFF: "GoToDropOffArea",
        Areas.RED_CORNER: "GoToRedCorner",
        Areas.FUEL_TANK: "GoToFuelTankArea",
        Areas.CRATER: "GoToCraterArea",
        Areas.BUTTON: "GoToButtonArea"
    }

    def __init__(self, area, move_publisher, arm_angles_publisher=None, task_space_publisher=None, misc_angles_publisher=None, grav_enable_publisher=None):
        # Initialize the state with outcomes 'arrived' and 'not_arrived'
        smach.State.__init__(self, outcomes=['arrived', 'not_arrived', 'emergency_stop'])
        self.area = area
        self.move_pub = move_publisher
        self.arm_angles_pub = arm_angles_publisher
        self.task_space_pub = task_space_publisher
        self.misc_angles_pub = misc_angles_publisher
        self.grav_enable_pub = grav_enable_publisher

        rospy.loginfo(f"Executing state GoTo{self.area}")

    def execute(self, userdata):
        '''Execute the state logic to move the robot to the specified area
        
        Args:
            userdata: The data passed to the state (Not used)
            
        Returns:
            str: The outcome of the state ('arrived' or 'not_arrived')
            
        Raises:
            Exception: Any exception that occurs during the state execution'''
        # Check if the area is valid and has a corresponding action
        if self.area in self.AREA_METHODS:
            # Get the action name corresponding to the area
            method_name = self.AREA_METHODS[self.area]

            # Enable the gravity vector
            grav_enable_msg = Bool()
            grav_enable_msg.data = True
            self.grav_enable_pub.publish(grav_enable_msg)
            rospy.logwarn('Gravity vector enabled')

            # Log the execution of the state
            rospy.loginfo(f"Executing state {method_name}")
            # Call the corresponding method dynamically using getattr
            try:
                outcome = getattr(self, method_name)()

                # Disable the gravity vector
                #grav_enable_msg.data = False
                #self.grav_enable_pub.publish(grav_enable_msg)
                #rospy.logwarn('Gravity vector disabled')
                
            # Handle any exceptions that occur during the state execution
            except Exception as e:
                rospy.logerr(f"Error in GoTo{self.area}: {e}")
                return 'not_arrived'        
            
            return outcome
        else:
            # Log that the area is invalid
            rospy.loginfo('Invalid area')
            return 'not_arrived'

    def GoToInitialArea(self):
        '''State to move the robot to the initial area'''
        # Publish command to move the robot to the initial area
        rospy.loginfo('Moving to initial area')
        if publish_and_wait(pub=self.move_pub,
                            wait_for_topic="Move_Done",
                            message_type=Float32MultiArray,
                            message_data=[-180, 0, 0, 100],
                            delay=2,
                            timeout_function=lambda: stop_move(self.move_pub)):
            rospy.loginfo('Arrived to initial area')
            return 'arrived'
        else:
            rospy.logerror('Move to initial area failed')
            return 'not_arrived'

    def GoToBigPackageWall(self):
        '''State to move the robot to the big package wall'''
        rospy.loginfo('Moving to big package wall')
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[-78, 0, 0, 100],
                        delay=2,
                        timeout_function=lambda: stop_move(self.move_pub))
        return 'arrived'

        
    def GoToReScan(self):
        '''State to move the robot to the re-scan area'''
        # Publish command to move the robot to the re-scan area
        rospy.loginfo('Moving to re-scan area')
        publish_and_wait(self.move_pub,
                        "Move_Done",
                        Float32MultiArray,
                        [0, 1, 0, 50],
                        delay=0.8,
                        timeout_function=lambda: stop_move(self.move_pub))
        return 'arrived'
 
        
    def GoToDropOffArea(self):
        '''State to move the robot to the drop off area'''
        rate = rospy.Rate(20)
        
        # Publish command to go straight indefinitely
        publish_command(self.move_pub, Float32MultiArray, [200, 1, 0, 100])
        rospy.loginfo('Going towards first ramp')

        # Wait for the move to complete
        while globals['gravity_vector'] > -25  and not rospy.is_shutdown():
            rospy.loginfo('Going up the ramp')
            rate.sleep()
        rospy.loginfo('Reached the top of the ramp')
        # Keep moving until the robot crosses the second slope
        while globals['gravity_vector'] < -10  and not rospy.is_shutdown():
            rospy.loginfo('Going down the ramp')
            rate.sleep()

        rospy.loginfo('Crossed ramp')

        # Lower the bridge a bit
        bridge = globals['mid_bridge']
        publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1])
        rospy.loginfo('Bridge lowered a bit')

        # Set the arm to drop off pose
        jaw = globals['gripper_bulk_hold']
        speed = 35 #speed updated
        angles = [495.0, 1669.0, 1664.0, 2507.0, 2087.0, 946.0, 3915.0, jaw, speed, 10]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)
        rospy.loginfo('Arm set to drop off pose')
        
        # Publish command to go a bit past the dropoff area
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[200, 130, 0, 100],
                        delay=10,
                        timeout_function=None)
        rospy.loginfo('Moved past dropoff area')

        # Publish command to go back a little to avoid possible purple package collision
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[200, 180, 0, 100],
                        delay=10,
                        timeout_function=None)
        rospy.loginfo('Moved back a bit')


        # Publish the command to turn 90 degrees
        if publish_and_wait(pub=self.move_pub,
                            wait_for_topic="Move_Done",
                            message_type=Float32MultiArray,
                            message_data=[0, 0, -90, 100],
                            delay=1,
                            timeout_function=None):
            rospy.loginfo('Rotated 90 degrees')
            return 'arrived'
        else:
            rospy.logerror('Error rotating 90 degrees')
            return 'emergency_stop'
        

    def GoToRedCorner(self):
        '''State to move the robot to the red corner'''
        # Move until the red corner is reached
        if publish_and_wait(pub=self.move_pub,
                            wait_for_topic="Move_Done",
                            message_type=Float32MultiArray,
                            message_data=[0, -1, -90, 20],
                            delay=0.2,
                            timeout_function=lambda: stop_move(self.move_pub)):
            rospy.loginfo('Reached red corner')
            return 'arrived'
        else:
            rospy.logerror('Move to red corner failed')
            return 'not_arrived'

    def GoToFuelTankArea(self):
        '''State to move the robot to the fuel tank area'''
        # Raise the bridge
        publish_and_wait(pub=self.misc_angles_pub,
                        wait_for_topic="Misc_Done",
                        message_type=Float32MultiArray,
                        message_data=[globals['raised_bridge'], -1, -1, -1],
                        delay=1,
                        timeout_function=None)
        # Go forward using the rear right TOF until fuel tank area is reached
        if publish_and_wait(pub=self.move_pub,
                            wait_for_topic="Move_Done",
                            message_type=Float32MultiArray,
                            #message_data=[-140, 65, -90, 100],
                            message_data=[-140, 85, -90, 100],
                            delay=5,
                            timeout_function=None):
            rospy.loginfo('Reached fuel tank area')
            return 'arrived'
        else:
            rospy.logerror('Move to fuel tank area failed')
            return 'not_arrived'

        
    def GoToCraterArea(self):
        rate = rospy.Rate(30)
        record_tof_back = 0
        
        # Back up a bit
        publish_and_wait(pub=self.move_pub,
                         wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[0, 150, -90, 100],
                        delay=2,
                        timeout_function=None)
        rospy.loginfo('Backed up a bit')
        
        # Publish command to rotate
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[0, 0, -180, 100],
                        delay=5,
                        timeout_function=None)

        rospy.loginfo('Rotated 180 degrees')

        record_tof_back = globals['tof_back']

        # Publish command to go forward
        publish_command(self.move_pub, Float32MultiArray, [200, 1, -180, 50])
        while globals['gravity_vector'] < 28  and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo('Half second ramp reached')
        while globals['gravity_vector'] > 5  and not rospy.is_shutdown():
            rate.sleep()
        while ((globals['tof_back'] - record_tof_back) > 5) and not rospy.is_shutdown():
            rospy.loginfo('Top second ramp reached')
            rospy.sleep(0.5)
            break

        # Rotate to drop bridge
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[0, 0, 0, 20],
                        delay=5,
                        timeout_function=None)
        
        rospy.loginfo('Rotated to place bridge')

  
        # Publish command  to back up
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[200, -1, 0, 20],
                        delay=4,
                        timeout_function=None)
        
        rospy.loginfo('Backed up')

        #Maxwells edit here
        '''
        bridge = globals['lowered_bridge']
        publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1])
        try:
            rospy.wait_for_message("Misc_Done", Int8, timeout=10)
        except:
            pass
        rospy.loginfo('Bridge dropped')
        '''
        
        # Publish command to drop the bridge
        bridge = globals['lowered_bridge']
        publish_and_wait(pub=self.misc_angles_pub,
                        wait_for_topic="Misc_Done",
                        message_type=Float32MultiArray,
                        message_data=[bridge, -1, -1, -1],
                        delay=10,
                        timeout_function=None)
        
        rospy.loginfo('Bridge dropped')

        # Publish command to go fwd
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[200, 1, 0, 20],
                        delay=2,
                        timeout_function=lambda: stop_move(self.move_pub))
        
        rospy.loginfo('DONE FORWARD FOR 2 SECONDS AHHH!')

        # Publiish command to raise the bridge
        bridge = globals['raised_bridge']
        if publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1]):
            rospy.loginfo('Bridge raised')
            return 'arrived'

        return 'not_arrived'


    def GoToButtonArea(self):
        rate = rospy.Rate(20)

        #Back until finding the flat area        
        publish_command(self.move_pub, Float32MultiArray, [200, -1, 0, 50])

        #wait until we cross bridge
        while globals['gravity_vector'] < 10  and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo('Crossing bridge')

        while globals['gravity_vector'] > 2  and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo('Crossed bridge')

        while globals['gravity_vector'] < 10  and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo('Going down the ramp')

        while globals['gravity_vector'] > 2  and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo('Flat area reached')

        publish_command(self.move_pub, Float32MultiArray, [0, 0, 90, 65])
        rospy.wait_for_message("Move_Done", Int8, timeout=10)

        #Here we would like the bot to stop until the fuel tanks have been sorted and placed
        #publish_command(self.move_pub, Float32MultiArray, [0, 0, 0, 0])
        #rospy.wait_for_message("Move_Done", Int8, timeout=10)

        publish_command(self.move_pub, Float32MultiArray, [120, 180, 90, 100])
        rospy.wait_for_message("Move_Done", Int8, timeout=10)

        if publish_command(self.move_pub, Float32MultiArray, [0, 0, 0, 0]):
           rospy.loginfo('Button Pressed')
           return 'arrived'
        
        return 'not arrived'

        
       

        #reset move_done
        #globals['move_done'] = False

        # Rotate
        #message.data = [0, 0, 90, 100]
        #self.move_pub.publish(message)

        #while not globals['move_done']  and not rospy.is_shutdown():
        #    rate.sleep()

        # Reset the move_done global variable
       # globals['move_done'] = False

        # Aim camera to thruster assembly
        #message.data = [100, 250, 90, 100]
        #self.move_pub.publish(message)

        # Wait for the move to complete
       # while not globals['move_done']  and not rospy.is_shutdown():
        #    rate.sleep()

        # Reset the move_done global variable
        #globals['move_done'] = False

        #Stop and hit the button
        #message.data = [0, 0, 0, 0]
        #self.move_pub.publish(message)

        # Reset the move_done global variable
        #globals['move_done'] = False

        # next state
        #return 'arrived'


class SetPose(smach.State):
    POSES = {
        Poses.SET_INITIAL_ARMS: "SetInitialArms",
        Poses.SET_BULK_GRABBER_ARMS: "SetBulkGrabberArms",
        Poses.CLOSE_TOP_BULK_GRABBER_ARM: "CloseTopBulkGrabberArm",
        Poses.MINI_RAISE_BULK_GRABBER: "MiniRaiseBulkGrabber",
        Poses.RAISE_BULK_GRABBER: "RaiseBulkGrabberArms",
        Poses.DROP_OFF_SMALL_PACKAGES: "DropOffSmallPackages",
        Poses.RELEASE_SMALL_PACKAGES: "ReleaseSmallPackages",
        Poses.FUEL_TANK_SCAN: "FuelTankScan"
    }

    def __init__(self, pose: str,
                 move_publisher: Optional[Publisher] = None,
                 arm_angles_publisher: Optional[Publisher] = None,
                 task_space_publisher: Optional[Publisher] = None,
                 misc_angles_publisher: Optional[Publisher] = None) -> None:
        # Initialize the state with outcomes 'pose_reached' and 'pose_not_reached'
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached']) 
        
        # Store the pose and publishers
        self.pose = pose
        self.move_pub = move_publisher
        self.arm_angles_pub = arm_angles_publisher
        self.task_space_pub = task_space_publisher
        self.misc_angles_pub = misc_angles_publisher

        rospy.loginfo(f"Executing state SetPose: {self.pose}")

    def execute(self, userdata):
        # Check if the pose is valid and has a corresponding action
        if self.pose in self.POSES:
            # Get the action name corresponding to the pose
            method_name = self.POSES[self.pose]
            # Log the execution of the state
            rospy.loginfo(f"Executing state {method_name}")
            # Call the corresponding method dynamically using getattr
            try:
                outcome = getattr(self, method_name)()
            except Exception as e:
                rospy.logerr(f"Error in SetPose: {e}")
                return 'pose_not_reached'
            
            return outcome
        else:
            # Log that the pose is invalid
            rospy.logwarn('Invalid pose')
            rospy.logwarn(f'Is {self.pose} in POSES, {Poses}, and a method of the class {self.__class__.__name__}?')
            return 'pose_not_reached'
    
    def SetInitialArms(self):
        # Set the bulk grabber arms
        publish_and_wait(pub=self.misc_angles_pub,
                         wait_for_topic="Misc_Done",
                         message_type=Float32MultiArray,
                         message_data=[-1, -1, 1270, -1],
                         delay=0.1,
                         timeout_function=None)
        rospy.loginfo('Bulk grabber arms in set pose')

        # Set the main arm to scan pose

        angles = globals['SP_SCAN_POSE']
        if publish_and_wait(pub=self.arm_angles_pub,
                            wait_for_topic='Arm_Done',
                            message_type=Float32MultiArray,
                            message_data=angles,
                            delay=1,
                            timeout_function=None):
            # Wait 2 seconds for the arm to stabilize
            rospy.sleep(1)
            rospy.loginfo('Arm set to scan pose')
            return 'pose_reached'
        else:
            rospy.logerr('Error setting arm to scan pose')
            return 'pose_not_reached'
    
    def SetBulkGrabberArms(self):
        # Set the misc angles
        bridge = -1
        top_bulk = globals['set_bulk_top']
        bottom_bulk = globals['set_bulk_bottom']
        flag = -1

        publish_and_wait(pub=self.misc_angles_pub,
                         wait_for_topic="Misc_Done",
                        message_type=Float32MultiArray,
                        message_data=[bridge, bottom_bulk, 1270, flag],
                        delay=2,
                        timeout_function=None)
        rospy.loginfo('Bulk grabber arms in set pose')
        if publish_command(self.misc_angles_pub, Float32MultiArray, [-1, -1, -3, -1]):
            rospy.loginfo("Top bulk grabber arm's torque disabled")
            return 'pose_reached'
        else:
            return 'pose_not_reached'
        
    def CloseTopBulkGrabberArm(self):
        # Set the misc angles
        bridge = -1
        top_bulk = globals['close_bulk_top']
        bottom_bulk = globals['set_bulk_bottom']
        flag = -1

        if publish_and_wait(pub=self.misc_angles_pub,
                        wait_for_topic="Misc_Done",
                        message_type=Float32MultiArray,
                        message_data=[bridge, bottom_bulk, 1050, flag],
                        delay=1,
                        timeout_function=None):
            rospy.loginfo('Top arm of bulk grabber closed')
            return 'pose_reached'
        else:
            return 'pose_not_reached'
        
    def MiniRaiseBulkGrabber(self):
        # Set the misc angles
        bridge = -1
        top_bulk = globals['raise_bulk_top']
        bottom_bulk = globals['raise_bulk_bottom']
        flag = -1

        if publish_and_wait(pub=self.misc_angles_pub,
                            wait_for_topic="Misc_Done",
                            message_type=Float32MultiArray,
                            message_data=[bridge, bottom_bulk+50, 1050+100, flag],
                            delay=1,
                            timeout_function=None):
            rospy.loginfo('Bulk grabber mini raise done')
            return 'pose_reached'
        else:
            rospy.logerr('Error MINI raising bulk grabber arms')
            return 'pose_not_reached'

    def RaiseBulkGrabberArms(self):
        # Set the misc angles
        bridge = -1
        top_bulk = globals['raise_bulk_top']
        bottom_bulk = globals['raise_bulk_bottom']
        flag = -1

        if publish_and_wait(pub=self.misc_angles_pub,
                            wait_for_topic="Misc_Done",
                            message_type=Float32MultiArray,
                            message_data=[bridge, bottom_bulk, top_bulk, flag],
                            delay=1,
                            timeout_function=None):
            rospy.loginfo('Bulk grabber arms raised')
            globals['big_packages_picked_up'] = True
            return 'pose_reached'
        else:
            rospy.logerr('Error raising bulk grabber arms')
            globals['big_packages_picked_up'] = True
            return 'pose_not_reached'

    def DropOffSmallPackages(self):
        # TODO: Format this method better
        # Publish command to move arm over drop off area
        jaw = globals['gripper_bulk_hold']
        speed = 25 #updated speed
        angles = [608.0, 1634.0, 1638.0, 2721.0, 2023.0, 841.0, 2194.0, jaw, speed, 10]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)
        # Wait for the arm to reach the pose
        rospy.wait_for_message("Arm_Done", Int8, timeout=10) 

        # Publish command to lower arm
        jaw = globals['gripper_bulk_hold']
        speed = 25 #updated speed
        angles = [497.0, 1242.0, 1245.0, 2730.0, 2022.0, 1250.0, 2077.0, jaw, speed, 10]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)
        # Wait for the arm to reach the pose
        rospy.wait_for_message("Arm_Done", Int8, timeout=10) 

        return 'pose_reached'


    def ReleaseSmallPackages(self):
        # Publish command to release the small packages
        jaw = globals['gripper_bulk_release']
        speed = 50 #updated speed
        angles = [497.0, 1242.0, 1245.0, 2730.0, 2022.0, 1250.0, 2077.0, jaw, speed, 10]
        if publish_command(self.arm_angles_pub, Float32MultiArray, angles):
            # Wait for the arm to reach the pose
            rospy.wait_for_message("Arm_Done", Int8, timeout=10) 
            return 'pose_reached'
        else:
            rospy.logerr('Error releasing small packages')
            return 'pose_not_reached'


# Define state PickUp
class PickUp(smach.State):
    # Dictionary mapping board objects to method names
    BOARD_OBJECT_METHODS = {
        BoardObjects.SMALL_PACKAGE: "PickUpSmallPackages",
        BoardObjects.BIG_PACKAGE: "PickUpBigPackages",
        BoardObjects.FUEL_TANK: "PickUpFuelTanks"
    }
    def __init__(self, board_object, arm_angles_publisher= None, camera_pose=None, task_space_publisher=None, misc_angles_publisher=None, move_publisher=None):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])
        self.board_object = board_object
        self.camera_pose = camera_pose
        self.arm_angles_pub = arm_angles_publisher
        self.task_space_pub = task_space_publisher
        self.misc_angles_pub = misc_angles_publisher
        self.move_pub = move_publisher

    def execute(self, userdata):
        '''Execute the state logic to pick up the specified board object
        
        Args:
            userdata: The data passed to the state (Not used)
            
        Returns:
            str: The outcome of the state ('packages_picked_up' or 'packages_not_picked_up')
            
        Raises:
            Exception: Any exception that occurs during the state execution'''
        # Check if the board object is valid and has a corresponding action
        if self.board_object in self.BOARD_OBJECT_METHODS:
            # Get the action name corresponding to the board object
            method_name = self.BOARD_OBJECT_METHODS[self.board_object]
            # Log the execution of the state
            rospy.loginfo(f"Executing state {method_name}")
            # Call the corresponding method dynamically using getattr
            outcome = getattr(self, method_name)()
            
            return outcome
        else:
            # Log that the board object is invalid
            rospy.loginfo('Invalid board object')
            return 'packages_not_picked_up'
        

    def PickUpFuelTanks(self):
        
        # Set the misc angles
        raised_bridge = globals['raised_bridge']
        raised_bridge = globals['raised_bridge']
        top_bulk = globals['fuel_tank_close_top']
        bottom_bulk = globals['set_bulk_bottom']
        flag = globals['lowered_flag']
        raise_bulk_offset = globals['fuel_tank_raise_bulk_offset']

        # Lower the top arm a bit
        publish_and_wait(pub=self.misc_angles_pub,
                        wait_for_topic="Misc_Done",
                        message_type=Float32MultiArray,
                        message_data=[-1, -1, 1270, -1],
                        delay=1,
                        timeout_function=None)
        rospy.loginfo('Top arm of bulk grabber lowered a bit')

        # Close both arms of the bulk grabber
        publish_and_wait(pub=self.misc_angles_pub,
                        wait_for_topic="Misc_Done",
                        message_type=Float32MultiArray,
                        message_data=[-1, 1200, 1000, -1],
                        delay=1,
                        timeout_function=None)
        
        rospy.loginfo('Closed both arms of bulk grabber')

        # Raise the bulk grabber arms
        if publish_and_wait(pub=self.misc_angles_pub,
                            wait_for_topic="Misc_Done",
                            message_type=Float32MultiArray,
                            message_data=[-1, 3200, 3000, -1],
                            delay=1,
                            timeout_function=None):
            rospy.loginfo('Fuel tanks picked up')
            return 'packages_picked_up'
        else:
            rospy.logerr('Error picking up fuel tanks')
            return 'packages_not_picked_up'
    

class DropOff(smach.State):
    OBJECTS = {
        BoardObjects.SMALL_PACKAGE: "DropOffSmallPackages",
        BoardObjects.BIG_PACKAGE: "DropOffBigPackages",
    }

    def __init__(self, board_object, arm_angles_publisher=None, task_space_publisher=None, misc_angles_publisher=None):
        smach.State.__init__(self, outcomes=['packages_dropped_off','packages_not_dropped_off'])
        self.board_object = board_object
        self.arm_angles_pub = arm_angles_publisher
        self.task_space_pub = task_space_publisher
        self.misc_angles_pub = misc_angles_publisher

    def execute(self, userdata):
        # Check if the board object is valid and has a corresponding action
        if self.board_object in self.OBJECTS:
            # Get the action name corresponding to the board object
            method_name = self.OBJECTS[self.board_object]
            # Log the execution of the state
            rospy.loginfo(f"Executing state {method_name}")
            # Call the corresponding method dynamically using getattr
            outcome = getattr(self, method_name)()
            
            return outcome
        else:
            # Log that the board object is invalid
            rospy.loginfo('Invalid board object')
            rospy.loginfo(f'Is {self.board_object} in OBJECTS, {BoardObjects}, and a method of the class {self.__class__.__name__}?')
            return 'packages_not_dropped_off'
    
    def DropOffSmallPackages(self):
        # Go over red area
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[-210,0,210,3600,2048,20,200],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo('Arm over red area')

        # Lower the arm
        publish_and_wait(pub=self.task_space_pub,
                        wait_for_topic='Arm_Done',
                        message_type=Float32MultiArray,
                        message_data=[-220,0,210,3600,2048,-70,200],
                        delay=3,
                        timeout_function=None)
        rospy.loginfo('Arm lowered into red area')

        # Open the gripper
        publish_and_wait(pub=self.task_space_pub,
                        wait_for_topic='Arm_Done',
                        message_type=Float32MultiArray,
                        message_data=[-220,-70,210,3600,1400,100,100],
                        delay=3,
                        timeout_function=None)
        rospy.loginfo('Jaws released')

        # Raise arm back over red area
        jaw = 1400
        speed = 50
        angles = [495.0, 1669.0, 1664.0, 2507.0, 2087.0, 946.0, 3915.0, jaw, speed, 200]
        publish_and_wait(pub=self.arm_angles_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=angles,
                         delay=1.75,
                         timeout_function=None)
        rospy.loginfo('Arm raised back over red area')

        # Go to scan fuel tank position
        gripper = 2440
        angles = [769.0, 2388.0, 2379.0, 2092.0, 2075.0, 644.0, 1800.0, gripper, speed, 10]
        if publish_command(self.arm_angles_pub, Float32MultiArray, angles):
            rospy.loginfo('Moving arm to fuel tank scan position')
            return 'packages_dropped_off'
        else:
            rospy.logerr('Error dropping off small packages')
            return 'packages_not_dropped_off'
        

    def DropOffBigPackages(self):
        # Set the misc angles
        raised_bridge = globals['raised_bridge']
        mid_bridge = globals['mid_bridge']
        top_bulk = globals['drop_bulk_top']
        bottom_bulk = globals['drop_bulk_bottom']
        flag = -1

        publish_and_wait(pub=self.misc_angles_pub,
                            wait_for_topic="Misc_Done",
                            message_type=Float32MultiArray,
                            message_data=[mid_bridge, 1050, 1270, flag],
                            delay=2,
                            timeout_function=None)
        rospy.loginfo('Big packages dropped off')

        # Raise top bulk grabber's arm
        if publish_and_wait(pub=self.misc_angles_pub,
                            wait_for_topic="Misc_Done",
                            message_type=Float32MultiArray,
                            message_data=[mid_bridge, 1050, 1800, flag],
                            delay=2,
                            timeout_function=None):
            rospy.loginfo('Big packages dropped off')
            return 'packages_dropped_off'
        else:
            rospy.logerr('Error dropping off big packages')
            return 'packages_not_dropped_off'


class SpiritCelebration(smach.State):
    def __init__(self, misc_angles_publisher = None, arm_angles_publisher = None):
        smach.State.__init__(self, outcomes = ['flag_raised','flag_not_raised'])
        self.misc_angles_pub = misc_angles_publisher
        self.arm_angles_pub = arm_angles_publisher

    def execute(self, userdata):
        '''Execute the state logic to celebrate the spirit
        
        Args:
            userdata: The data passed to the state (Not used)
            
        Returns:
            str: The outcome of the state ('succeeded' or 'aborted')

        Raises:
            Exception: Any exception that occurs during the state execution'''
        
        try:
            # Set the misc angles
            bridge = globals['raised_bridge']
            top_bulk = globals['close_bulk_top']
            bottom_bulk = globals['set_bulk_bottom']
            flag = globals['raised_flag']
            jaw = globals['gripper_bulk_hold']
            speed = 40 #updated speed

            angles = [2041.0, 2023.0, 2017.0, 2748.0, 2078.0, 478.0, 2040.0, 2005.0, 10, 10]  #scan pose
            publish_command(self.arm_angles_pub, Float32MultiArray, angles, delay=.5) #gets the arm out of the way for the flag
            #rospy.wait_for_message("Arm_Done", Int8, timeout=10)
            

            # Publish the misc angles to close the top bulk grabber arm
            if publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, bottom_bulk, top_bulk, flag]):
                # Waitfor the bulk grabber arms to reach the pose
                rospy.wait_for_message("Misc_Done", Int8, timeout=10)
                return 'flag_raised'
            else:
                return 'flag_not_raised'
            
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in SpiritCelebration: {e}")
            return 'aborted'


# define state ButtonPress
class ButtonPress(smach.State):
    def __init__(self, move_publisher):
        smach.State.__init__(self, outcomes=['button_pressed','button_not_pressed'])
        self.move_pub = move_publisher

    def execute(self, userdata):
        '''Execute the state logic to press the button
        
        Args:
            userdata: The data passed to the state (Not used)
            
        Returns:
            str: The outcome of the state ('succeeded' or 'aborted')
            
        Raises:
            Exception: Any exception that occurs during the state execution'''
        if publish_and_wait(pub=self.move_pub,
                            wait_for_topic="Move_Done",
                            message_type=Float32MultiArray,
                            message_data=[0, -1, 90, 100],
                            delay=2,
                            timeout_function=None):
            rospy.loginfo('Button Pressed')
            stop_move(self.move_pub)
            return 'button_pressed'
        else:
            return 'button_not_pressed'
        
    
# define state SetPose
class ScanPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state ScanPose')
        self.arm_angles_pub = arm_angles_pub

    def execute(self, userdata):
        angles = globals['SP_SCAN_POSE']
        if publish_and_wait(pub=self.arm_angles_pub,
                            wait_for_topic='Arm_Done',
                            message_type=Float32MultiArray,
                            message_data=angles,
                            delay=1,
                            timeout_function=None):
            # Wait 2 seconds for the arm to stabilize
            rospy.sleep(2)
            rospy.loginfo('Arm set to scan pose')
            return 'pose_reached'
        else:
            rospy.logerr('Error setting arm to scan pose')
            return 'pose_not_reached'
        


class ScanFuelTankPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state ScanFuelTankPose')
        self.arm_angles_pub = arm_angles_pub

    def execute(self, userdata):
        speed = 50 #updated speed
        gripper = 2440
        rospy.loginfo('Moving to scan pose')
        # Publish command to set the arm to the scan pose
        publish_and_wait(pub=self.arm_angles_pub,
                        wait_for_topic='Arm_Done',
                        message_type=Float32MultiArray,
                        message_data=globals['FT_SCAN_POSE'],
                        delay=0.1,
                        timeout_function=None)
        rospy.sleep(1)
        return 'pose_reached'


class PickupScanPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state PickupScanPose')
        self.arm_angles_pub = arm_angles_pub

    def execute(self, userdata):
        angles = globals['PICKUP_SP_SCAN_POSE']
        if publish_and_wait(pub=self.arm_angles_pub,
                            wait_for_topic='Arm_Done',
                            message_type=Float32MultiArray,
                            message_data=angles,
                            delay=1,
                            timeout_function=None):
            # Wait 2 seconds for the arm to stabilize
            rospy.sleep(2)
            rospy.loginfo('Arm set to pickup scan pose')
            return 'pose_reached'
        else:
            rospy.logerr('Error setting arm to pickup scan pose')
            return 'pose_not_reached' 


# define state SetPose
class RestPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state RestPose')
        self.arm_angles_pub = arm_angles_pub

    def execute(self, userdata):
        try:
            # Reset the arm_done global variable
            globals['arm_done'] = False
            speed = 10 #updated speed
            jaw = globals['gripper_bulk_hold']
            angles_ = Float32MultiArray()
            angles_.data = [2009.0, 2721.0, 2706.0, 1635.0, 2166.0, 1661.0, 1886.0, jaw, speed, 10]
            self.arm_angles_pub.publish(angles_)
            rospy.loginfo('Moving to rest pose')
            
            return 'pose_reached'
        except Exception as e:
            rospy.logerr(f"Error in RestPose: {e}")
            return 'pose_not_reached'


# Define state GetCoords
# Define state GetCoords
class GetCoords(smach.State):
    def __init__(self, object_type, pose, timeout=2.0, expected_pairs=3, camera_enable_publisher=None, disable_camera_when_done=False):
        smach.State.__init__(self, outcomes=['coords_received','coords_not_received'],
                             input_keys=['coordinates_list'],
                             output_keys=['coordinates_list'])
        self.object_type = object_type
        self.pose = pose
        self.timeout = timeout
        self.expected_pairs = expected_pairs
        self.camera_enable_pub = camera_enable_publisher
        self.disable_camera_when_done = disable_camera_when_done

    def execute(self, userdata):
        # If userdata coord exist, reset them
        try:
            userdata.coordinates_list.clear()
        except:
            pass
        #if not globals['big_packages_picked_up']:
        #    rospy.logwarn('Big package pick up not done yet')
        #    rospy.sleep(1)
        #    return 'coords_not_received'
        # Enable camera
        if self.camera_enable_pub:
            enable_msg = Bool()
            enable_msg.data = True
            self.camera_enable_pub.publish(enable_msg)
            
        try:
            rospy.loginfo(f'Executing state GetCoords({self.object_type}, {self.pose})')

            client = actionlib.SimpleActionClient('get_coords', GetCoordsAction)
            client.wait_for_server()

            goal = GetCoordsGoal()
            goal.timeout.data = self.timeout
            goal.expected_pairs.data = self.expected_pairs
            goal.object_type.data = self.object_type
            goal.arm_pose.data = self.pose

            client.send_goal(goal, feedback_cb=get_coords_fb_cb)

            client.wait_for_result()
            result = client.get_result()
            #rospy.loginfo(f'Final Elapsed Time: {result.elapsed_time}')
            #print(result)
            rospy.loginfo(f'Final Coordinates List: {result.coordinates}    |    Total time: {result.elapsed_time.data}')

            # Return the coordinates as userdata if they are received
            if result.coordinates:
                #userdata.coordinates_list = []
                userdata.coordinates_list = result.coordinates
                # Disable camera
                if self.camera_enable_pub and self.disable_camera_when_done:
                    enable_msg = Bool()
                    enable_msg.data = False
                    self.camera_enable_pub.publish(enable_msg)
            return 'coords_received'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GetCoords: {e}")
            return 'coords_not_received'
        

class PathPlanning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['path_found','path_not_found'],
                             input_keys=['coordinates_list', 'current_path_state'],
                             output_keys=['current_path_state', 'close_right', 'close_left', 'far_right', 'far_left'])
    
    def execute(self, userdata):
        # Get the coordinates list
        coordinates = userdata.coordinates_list.coordinates
        
        # Reset the userdata values
        try:
            userdata.current_path_state = 'close_right'
        except:
            pass
        userdata.close_right = False
        userdata.close_left = False
        userdata.far_right = False
        userdata.far_left = False
        
        # Check if the coordinates list is empty
        if not coordinates:
            return 'path_not_found'
        
        for coordinate in coordinates:
            # Get the X and Z coordinates
            x = coordinate.x
            z = coordinate.z
            # Check if the X and Z coordinates are within the range
            if self.in_close_right(x, z):
                rospy.loginfo(f'Coordinate {coordinate} is in the close right zone')
                userdata.close_right = True
                userdata.current_path_state = 'close_right'
                continue
            elif self.in_close_left(x, z):
                rospy.loginfo(f'Coordinate {coordinate} is in the close left zone')
                userdata.close_left = True
                if userdata.current_path_state != 'close_right':
                    userdata.current_path_state = 'close_left'
                continue
            elif self.in_far_right(x, z):
                rospy.loginfo(f'Coordinate {coordinate} is in the far right zone')
                userdata.far_right = True
                if userdata.current_path_state != 'close_right' and userdata.current_path_state != 'close_left':
                    userdata.current_path_state = 'far_right'
                continue
            elif self.in_far_left(x, z):
                rospy.loginfo(f'Coordinate {coordinate} is in the far left zone')
                userdata.far_left = True
                if userdata.current_path_state != 'close_right' and userdata.current_path_state != 'close_left' and userdata.current_path_state != 'far_right':
                    userdata.current_path_state = 'far_left'
                continue
            else:
                rospy.logwarn(f'Coordinate {coordinate} is not in any zone')
        
        return 'path_found'
    
    def in_close_right(self, x, z):
        return z > 250 and x < 280
    
    def in_close_left(self, x, z):
        return z < 350 and x < 280
    
    def in_far_right(self, x, z):
        return z > 250 and x >= 280
    
    def in_far_left(self, x, z):
        return z < 350 and x >= 280
    

class PathResolver(smach.State):
    def __init__(self, move_publisher):
        smach.State.__init__(self,
                             input_keys=['current_path_state', 'close_right', 'close_left', 'far_right', 'far_left'],
                             output_keys=['current_path_state', 'close_right', 'close_left', 'far_right', 'far_left'],
                             outcomes=['area_reached', 'go_to_dropoff'])
        self.move_pub = move_publisher
    
    def execute(self, userdata):
        rospy.loginfo(f'Executing state PathResolver({userdata.current_path_state})')
        rospy.sleep(2)
        if userdata.close_right:
            if userdata.current_path_state != 'close_right':    
                rospy.loginfo('Moving to close right zone')
                publish_and_wait(pub=self.move_pub,
                                wait_for_topic='Move_Done',
                                message_type=Float32MultiArray,
                                message_data=[-78, 0, 0, 100],
                                delay=1,
                                timeout_function=lambda: stop_move(self.move_pub))
            userdata.close_right = False
            userdata.current_path_state = 'close_right'
            return 'area_reached'
        elif userdata.close_left:
            rospy.loginfo('Moving to close left zone')
            publish_and_wait(pub=self.move_pub,
                            wait_for_topic='Move_Done',
                            message_type=Float32MultiArray,
                            message_data=[300, 0, 0, 100],
                            delay=1,
                            timeout_function=lambda: stop_move(self.move_pub))
            userdata.close_left = False
            userdata.current_path_state = 'close_left'
            return 'area_reached'
        elif userdata.far_right:
            rospy.loginfo('Moving to far right zone')
            if userdata.current_path_state == 'close_right' or userdata.current_path_state == 'close_left':
                publish_and_wait(pub=self.move_pub,
                                wait_for_topic='Move_Done',
                                message_type=Float32MultiArray,
                                message_data=[0, 1, 0, 50],
                                delay=1,
                                timeout_function=lambda: stop_move(self.move_pub))
                rospy.loginfo('Moved to far zone')
            if userdata.current_path_state == 'far_left' or userdata.current_path_state == 'close_left':
                publish_and_wait(pub=self.move_pub,
                                wait_for_topic='Move_Done',
                                message_type=Float32MultiArray,
                                message_data=[-78, 0, 0, 100],
                                delay=1,
                                timeout_function=lambda: stop_move(self.move_pub))
                rospy.loginfo('Moved to far right zone')
            userdata.far_right = False
            userdata.current_path_state = 'far_right'
            return 'area_reached'
        elif userdata.far_left:
            rospy.loginfo('Moving to far left zone')
            if userdata.current_path_state == 'close_right' or userdata.current_path_state == 'close_left':
                publish_and_wait(pub=self.move_pub,
                                wait_for_topic='Move_Done',
                                message_type=Float32MultiArray,
                                message_data=[0, 1, 0, 50],
                                delay=1,
                                timeout_function=lambda: stop_move(self.move_pub))
                rospy.loginfo('Moved to far zone')
            if userdata.current_path_state == 'far_right' or userdata.current_path_state == 'close_right':
                publish_and_wait(pub=self.move_pub,
                                wait_for_topic='Move_Done',
                                message_type=Float32MultiArray,
                                message_data=[-300, 0, 0, 100],
                                delay=1,
                                timeout_function=lambda: stop_move(self.move_pub))
                rospy.loginfo('Moved to far left zone')
            userdata.far_left = False
            userdata.current_path_state = 'far_left'
            return 'area_reached'
        else:
            rospy.loginfo('Pickup path done. Moving to dropoff zone.')
            userdata.current_path_state = 'dropoff'
            return 'go_to_dropoff'



class PickUpSmallPackage(smach.State):
    def __init__(self, task_space_pub, in_re_scan=False, after_big_packages=False, safe_mode=False):
        smach.State.__init__(self,
                             input_keys=['coordinates_list'],
                             output_keys=['sweep_coordinates_list', 'left_wall_zone', 'right_wall_zone', 'pick_after_big_packages', 'move_after_big_packages'],
                             outcomes=['packages_picked_up', 'no_coordinates_received'])
        self.task_space_pub = task_space_pub
        self.in_re_scan = in_re_scan
        self.after_big_packages = after_big_packages
        self.in_safe_mode = safe_mode

    def execute(self, userdata):
        # Reset userdata values
        userdata.pick_after_big_packages = False
        userdata.move_after_big_packages = False

        # Store the coordinates list: CoordinatesList -> coordinates[coordinates]
        coordinates = userdata.coordinates_list.coordinates

        rospy.loginfo(f'Coordinates: {coordinates}')

        # Check if the coordinates list is empty
        if not coordinates:
            return 'no_coordinates_received'
        
        
        num_coordinates = len(coordinates)
                
        for i in range(num_coordinates):
            # Get first coordinate
            target = coordinates[i]
            userdata.sweep_coordinates_list = coordinates[i]
            x = int(target.x)
            area = int(target.y)
            z = int(target.z)
            # Set the jaw value to hold the small package grabber
            jaw = globals['small_package_jaw_closed']
            wrist = globals['regular_wrist_angle']
            
            # Move over target small package to pick up
            rospy.loginfo(f'Moving over small package {target} to pick it up')
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x, 70, z, wrist, jaw, 50, 50],
                                delay=3,
                                timeout_function=None)
            
            # Lower the arm to pick up the small package
            rospy.loginfo(f'Lowering arm to pick up small package {target}')
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x, 70, z, wrist, jaw, -170, 10],
                                delay=3,
                                timeout_function=None)
            
            # Raise the arm to avoid pushing other blocks around
            rospy.loginfo(f'Raising arm after picking up small package {target}')
            if z >140: 
                z = z-50
            elif z <140: 
                z = z+50
            if x > 220:
                x = x -50
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x, 70, z, wrist, jaw, 100, 50],
                                delay=3,
                                timeout_function=None)
            
            rospy.loginfo(f'Successfully picked up small package {target}')
        return 'packages_picked_up'
    

class Sweep(smach.State):
    def __init__(self, task_space_pub, soft=False):
        smach.State.__init__(self,
                             input_keys=['sweep_coordinates_list'],
                             outcomes=['succeeded','aborted'])
        self.task_space_pub = task_space_pub
        self.soft_sweep = soft

    def execute(self, userdata):
        if self.soft_sweep:
            # Go to soft sweep pose
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[50, 20, 180, 2048, 1980, 100, 10],
                                delay=2,
                                timeout_function=None)
            # Lower the arm
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[50, -50, 180, 2048, 1980, 10, 10],
                                delay=2,
                                timeout_function=None)
            # Sweep slowly
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[200, -70, 180, 2048, 1980, 10, 10],
                                delay=2,
                                timeout_function=None)
            # Raise the arm to avoid pushing other blocks around
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[200, 50, 180, 2048, 1980, 10, 10],
                                delay=2,
                                timeout_function=None)
            
        else:
            target = userdata.sweep_coordinates_list
            x = target.x - 40
            y = 0
            z = target.z
            wrist = 2048
            jaw = globals['small_package_jaw_closed']

            # Move over target small package to pick up
            rospy.loginfo(f'Moving over small package cluster to sweep it')
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x, y, z, wrist, jaw, 50, 10],
                                delay=3,
                                timeout_function=None)
            # Lower the arm
            y = -70
            rospy.loginfo(f'Lowering arm to sweep small package cluster')
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x, y, z, wrist, jaw, 100, 10],
                                delay=2,
                                timeout_function=None)
            # Sweep
            y = 50
            x_sweep = x + 70
            rospy.loginfo(f'Sweeping small package cluster')
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x_sweep, y, z, wrist, jaw, 100, 10],
                                delay=2,
                                timeout_function=None)
            # Raise the arm to avoid pushing other blocks around
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x_sweep, y, z, wrist, jaw, 100, 10],
                                delay=2,
                                timeout_function=None)
        return 'succeeded'


class PickUpFuelTank(smach.State):
    def __init__(self, task_space_publisher):
        smach.State.__init__(self,
                             input_keys=['coordinates_list'],
                             outcomes=['fuel_tank_picked_up','fuel_tank_not_picked_up'])
        self.task_space_pub = task_space_publisher

    def execute(self, userdata):
        # Get the sorted coordinates list from the userdata
        #coordinates = userdata.coordinates_list.coordinates
        sorted_coords = userdata.coordinates_list.coordinates
        rospy.loginfo(f'Sorted Coordinates: {sorted_coords}')
        # Make sure the sorted coordinates list is not empty
        if not sorted_coords:
            rospy.logwarn('Sorted coordinates list is empty')
            return 'fuel_tank_picked_up'
        
        # Get the first target from the sorted coordinates list
        target = sorted_coords[0]
        rospy.loginfo(f'Target: {target}')

        x = target.x
        y = 70
        z = 67  #target.z

        # Map wrist based on x-coordinate using polynomial regression
        wrist = round(-0.0007742 * x**3 - 0.01569 * x**2 + 10.43 * x + 1007, 1)
        gripper = globals['fuel_tank_gripper_open']
        speed = 100 #updated speed
        
        # Go on top of fuel tank
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, speed, 100],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo(f'Going to [{x}, {y}, {z}]')
        
        # Lower the arm
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, -40, 10],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo('Lowering arm')
        
        # Close the gripper
        gripper = 2800
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y-40, z, wrist, gripper, speed, 10],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo("Closing gripper")
        # Go up
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, speed, 100, 10],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo("Raising arm")
        

        return 'fuel_tank_picked_up'
    

class StoreFuelTank(smach.State):
    # Dictionary to map the slot number to the corresponding coordinates
    OVER_SLOT_COORDS = {
        1: [-120, 70, -86.2, 2700, 2640, 100, 100],      
        2: [-35, 70, -84.2, 2400, 2640, 100, 100],   #updated speeds
        3: [30, 70, -82.2, 1700, 2640, 100, 100]
    }

    IN_SLOT_COORDS = {
        1: [-120, 70, -86.2, 2700, 2640, -55, 100],      
        2: [-35, 70, -84.2, 2400, 2640, -55, 100],   #updated speeds
        3: [30, 70, -82.2, 1700, 2640, -55, 100]
    }

    IN_SLOT_OPEN = {
        1: [-120, 15, -86.2, 2700, 2540, 100, 10],      
        2: [-35, 15, -84.2, 2400, 2540, 100, 10],   #updated speeds
        3: [30, 15, -82.2, 1700, 2540, 100, 10]
    }



    def __init__(self, task_space_publisher, arm_angles_publisher, slot_number=1):
        smach.State.__init__(self, outcomes=['fuel_tank_stored','fuel_tank_not_stored'])
        self.task_space_pub = task_space_publisher
        self.arm_angles_pub = arm_angles_publisher
        self.slot_number = slot_number

    def execute(self, userdata):
        # Go over the respective slot
        rospy.loginfo(f"Moving over slot {self.slot_number} located at {self.OVER_SLOT_COORDS[self.slot_number]}")
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=self.OVER_SLOT_COORDS[self.slot_number],
                         delay=1,
                         timeout_function=None)
        # Lower the arm
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=self.IN_SLOT_COORDS[self.slot_number],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo("Lowering the arm into slot {self.slot_number}")
        
        # Open the gripper
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=self.IN_SLOT_OPEN[self.slot_number],
                         delay=1,
                         timeout_function=None)
        rospy.loginfo("Releasing fuel tank into slot {self.slot_number}")
        
        # If done storing fuel tanks, return
        if self.slot_number == 1:
            return 'fuel_tank_stored'
        
        # Go back to scan fuel tank pose
        rospy.loginfo('Moving to scan pose')
        # Publish command to set the arm to the scan pose
        publish_and_wait(pub=self.arm_angles_pub,
                        wait_for_topic='Arm_Done',
                        message_type=Float32MultiArray,
                        message_data=globals['FT_SCAN_POSE'],
                        delay=1,
                        timeout_function=None)
        
        rospy.sleep(0.75)
        return 'fuel_tank_stored'


class PackageStateResolver(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             input_keys=['pick_after_big_packages', 'move_after_big_packages'],
                             output_keys=['pick_after_big_packages', 'move_after_big_packages'],
                             outcomes=['pick_after_big_packages', 'move_after_big_packages', 'packages_picked_up'])

    def execute(self, userdata):
        if userdata.pick_after_big_packages:
            # Reset the pick_after_big_packages flag
            userdata.pick_after_big_packages = False
            return 'pick_after_big_packages'
        elif userdata.move_after_big_packages:
            # Reset the move_after_big_packages flag
            userdata.move_after_big_packages = False
            return 'move_after_big_packages'
        else:
            return 'packages_picked_up'


class FuelTankPlacer(smach.State):
    def __init__(self,task_space_publisher):
        smach.State.__init__(self,
                              outcomes=['fuel_tanks_placed','fuel_tanks_not_placed'])
        self.task_space_pub = task_space_publisher

    def execute(self, userdata):
                
        #hover over the fuel tank holder
        publish_and_wait(pub=self.task_space_pub,
                    wait_for_topic='Arm_Done',
                    message_type=Float32MultiArray,
                    message_data=[-40, 100, -128, 2230, 1980, 100, 100], #-40 -> -35 -> -45
                    delay=3,
                    timeout_function=None)
        

        #get lower to grab the device
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[-40, -10, -128, 2230, 1980, 50, 10],
                delay=6,
                timeout_function=None)    

        #close gripper
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[-40, -10, -128, 2230, 2800, 100, 100],
                delay=1,
                timeout_function=None)    
        
        #rospy.sleep(3)
        
        #position the arm just above the thruster assembly zone
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 76, 2230, 2700, 50, 100],
                delay=3,
                timeout_function=None)

        #twist the wrist
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[130, 100, 76, 3645, 2700, 50, 10], # changed the fourth value from 3700 -> 3650 for less rotation
                # first value was 135 changing to 130
                delay=3,
                timeout_function=None)


        # try 2 movements for placing device

        
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[130, 0, 85, 3625, 2700, 10, 5], #changed tolerance to 5, and positive z by 5
                delay=3,
                timeout_function=None)

        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[130, 0, 89, 3625, 2700, -100, 10], #changed tolerance to 5, and positive z by 5 85 -> 89
                delay=3,
                timeout_function=None)
        
        
        #old code down here
        '''
        #place the device
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 81, 3700, 2700, -200, 5], #changed tolerance to 5, and positive z by 5
                delay=3,
                timeout_function=None)

        '''
        
        #release
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, -100, 76, 3700, 2000, 100, 10],
                delay=2,
                timeout_function=None)
        
        #get to a better position to press the button
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 76, 3700, 2000, 100, 100],
                delay=2,
                timeout_function=None)
        
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[100, 100, 100, 2048, 2000, 100, 100],
                delay=2,
                timeout_function=None)
        
        return 'fuel_tanks_placed'