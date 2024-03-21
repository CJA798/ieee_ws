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
        smach.State.__init__(self, outcomes=['arrived', 'not_arrived'])
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
        '''
        This is the sensor-based version
        
        publish_command(self.move_pub, Float32MultiArray, [-260, 0, 0, 20])
        #publish_command(self.misc_angles_pub, Float32MultiArray, [-1, -1, -3, -1], delay=0.1)
        rospy.wait_for_message('Move_Done',Int8, timeout = 10)

        # Wait for the move_done message
        #rospy.wait_for_message("Move_Done", Int8, timeout=5)
        if stop_move(self.move_pub):
            return 'arrived'
        # TODO: implement timeout routine
        rospy.logerror('Move to big package wall failed')
        return 'not_arrived'
        '''
        
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
        # Move until crossing the ramp (time based)
        '''
        publish_and_wait(pub=self.move_pub,
                        wait_for_topic="Move_Done",
                        message_type=Float32MultiArray,
                        message_data=[170, 1, 0, 100],
                        delay=6,
                        timeout_function=None)
        
        # After crossing the ramp, prepare the bot for dropoff and send a new command to reach the dropoff area

        # Lower the bridge a bit
        bridge = globals['mid_bridge']
        publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1])
        # Set the arm to drop off pose
        jaw = globals['gripper_bulk_hold']
        speed = 35 #speed updated
        angles = [495.0, 1669.0, 1664.0, 2507.0, 2087.0, 946.0, 3915.0, jaw, speed, 10]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)
        
        # Move until drop off area is reached
        publish_command(self.move_pub, Float32MultiArray, [170, 180, 0, 100])
        '''
        rate = rospy.Rate(20)
        
        # Publish command to go straight indefinitely
        publish_command(self.move_pub, Float32MultiArray, [200, 1, 0, 100])

        # Wait for the move to complete
        while globals['gravity_vector'] > -25  and not rospy.is_shutdown():
            rate.sleep()

        # Keep moving until the robot crosses the second slope
        while globals['gravity_vector'] < -10  and not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo('Crossed ramp')

        # Lower the bridge a bit
        bridge = globals['mid_bridge']
        publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1])
        # Set the arm to drop off pose
        jaw = globals['gripper_bulk_hold']
        speed = 35 #speed updated
        angles = [495.0, 1669.0, 1664.0, 2507.0, 2087.0, 946.0, 3915.0, jaw, speed, 10]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)

        # Publish command to reach to the drop off area
        publish_command(self.move_pub, Float32MultiArray, [200, 180, 0, 100])
        
        # Wait for the move to complete
        rospy.wait_for_message("Move_Done", Int8, timeout=10)

        # Publish the command to turn 90 degrees
        if publish_command(self.move_pub, Float32MultiArray, [0, 0, -90, 100]):
            # Wait for the move to complete
            rospy.wait_for_message("Move_Done", Int8, timeout=10)
            #rospy.sleep(2000)
            return 'arrived'
        else:
            return 'not_arrived'

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
                            message_data=[-140, 65, -90, 100],
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
        publish_command(self.move_pub, Float32MultiArray, [200, 1, -180, 100])
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
        publish_command(self.move_pub, Float32MultiArray, [200, -1, 0, 100])

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
        speed = 100 #updated speed
        jaw = globals['gripper_bulk_hold']
        tolerance = 10
        angles = [2041.0, 2023.0, 2015.0, 2660.0, 2083.0, 500.0, 2039.0, jaw, speed, tolerance]
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


    def FuelTankScan(self):
        # Publish command to set the arm to fuel tank scan pose
        jaw = globals['gripper_bulk_release']
        speed = 25 #updated speed
        angles = [1185.0, 2236.0, 2231.0, 2151.0, 2017.0, 692.0, 2109.0, jaw, speed, 10]
        
        if publish_command(self.arm_angles_pub, Float32MultiArray, angles):
            return 'pose_reached'
            # Wait for the arm to reach the pose
            rospy.wait_for_message("Arm_Done", Int8, timeout=10) 
            return 'pose_reached'
        else:
            rospy.logerr('Error setting arm to fuel tank scan pose')
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
                         message_data=[-220,0,210,3600,2048,20,200],
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
                            delay=1,
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
        speed = 100 #updated speed
        jaw = globals['gripper_bulk_hold']
        tolerance = 10
        angles = [2041.0, 2023.0, 2015.0, 2660.0, 2083.0, 500.0, 2039.0, jaw, speed, tolerance]
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
                        message_data=[1081.0, 2488.0, 2474.0, 1959.0, 2045.0, 675.0, 2089.0, gripper, speed],
                        delay=1,
                        timeout_function=rospy.sleep(1))
        return 'pose_reached'
    

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
        
class PickUpSmallPackage(smach.State):
    def __init__(self, task_space_pub, in_re_scan=False, after_big_packages=False, safe_mode=False):
        smach.State.__init__(self,
                             input_keys=['coordinates_list'],
                             output_keys=['sweep_coordinates_list', 'left_wall_zone', 'right_wall_zone', 'pick_after_big_packages', 'move_after_big_packages'],
                             outcomes=['packages_picked_up', 'sweep_needed', 'soft_sweep_needed', 'no_coordinates_received'])
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

            # Check if it's within X-axis range
            if not self.x_coord_within_range(x, z) and not self.in_re_scan:
                globals['scan_after_big_package_pickup'] = True
                userdata.move_after_big_packages = True
                rospy.logwarn(f'Coordinate {target} is not within X-axis range. Scan after big package pickup needed')
                continue

            ##############################################################################################################
            ##                     THE LINE BELOW IGNORES ANY PURPLE BOXES NOT IN FRONT OF THE ROBOT                    ##
            ##############################################################################################################
            # The safe mode is to ignore any purple boxes not in front of the robot path to the dropoff area.
            
            if self.in_safe_mode and not self.within_safe_mode_range(x,z):
                rospy.logwarn(f'Coordinate {target} is outside of safe mode range')
                # Determine the area where the out-of-safe-mode-range package is
                if self.in_left_wall_zone(x, z):
                    rospy.logwarn(f'Left wall pickup needed')
                    userdata.left_wall_zone = True
                    return 'left_wall_zone'
                elif self.in_right_wall_zone(x, z):
                    rospy.logwarn(f'Right wall pickup needed')
                    userdata.right_wall_zone = True
                    return 'right_wall_zone'
                continue
            
            ##############################################################################################################
            ##                     THE LINE ABOVE IGNORES ANY PURPLE BOXES NOT IN FRONT OF THE ROBOT                    ##
            ##############################################################################################################
    
            # Check the area of the contour to verify it's a single box
            if not self.area_within_range(area):
                rospy.logwarn(f'Coordinate {target} has an area of {area}, which is bigger than the max area for a single package')
                # Double-check with the number of coordinates
                if num_coordinates < 3:
                    # Sweep
                    rospy.logwarn(f'Potential cluster of small packages detected. Sweep needed')
                    return 'sweep_needed'
                rospy.logwarn('Picking up abnormally large package anyways')
                # else, try to pick it up. It could just be a greater contour than normal 
            
            # Check distance with respect to other packages to make sure it's not too close
            if self.too_close_to_other_package(x, z, coordinates):
                rospy.logwarn(f'Coordinate {target} is too close to another package. Sweep needed')
                return 'sweep_needed'
            
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
            publish_and_wait(pub=self.task_space_pub,
                                wait_for_topic='Arm_Done',
                                message_type=Float32MultiArray,
                                message_data=[x, 70, z, wrist, jaw, 100, 50],
                                delay=3,
                                timeout_function=None)
            
            rospy.loginfo(f'Successfully picked up small package {target}')
        return 'packages_picked_up'
    
    def within_safe_mode_range(self, x, z):
        return (120 <= x <= 250) and (-170 <= z <= 170)
    
    def needs_pickup_after_big_packages(self, z):
        return z >= 170
    
    def in_left_wall_zone(self, x, z):
        return (70 <= x <= 100) and (-200 <= z <= 200)
    
    def in_right_wall_zone(self, x, z):
        return (250 <= x <= 280) and (-200 <= z <= 200)

    def x_coord_within_range(self, x, z):
        return (70 < x < 250)
    
    def z_coord_within_range(self, z):
        return (-200 < z < 200)
    
    def area_within_range(self, area):
        return area < globals['max_small_package_area']
    
    def too_close_to_other_package(self, x, z, coordinates):
        pass
    
    def in_big_package_area(self, x, z):
        return (100 < x < 160) and (170 < z < 200)
    
    def pick_after_big_packages(self, x, z):
        return (100 < x < 160) and (200 <= z)
    
    def get_wrist_angle(self, z):
        return 400 if z > 0 else 3600

    def adjust_xz_to_wrist(self, x, z, wrist):
        return (x, z) if wrist < 2048 else (x, z) # less than 2048 for the wrist means the arm is going towards the right wall


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
        y = target.y
        z = target.z
        # Map wrist based on x-coordinate using polynomial regression
        wrist = round(5.189e-7 * x**5 + 1.895e-5 * x**4 - 0.003994 * x**3 - 0.1149 * x**2 + 12.87 * x + 2146, 1)
        gripper = globals['fuel_tank_gripper_open']
        speed = 25 #updated speed

        #publish_command(self.task_space_pub, Float32MultiArray, [x, y, z, wrist, gripper, speed])
        #rospy.wait_for_message("Arm_Done", Int8, timeout=10)
        # Go on top of fuel tank
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, speed, 10],
                         delay=1,
                         timeout_function=None)
        
        # Lower the arm
        y = globals['fuel_tank_Y_lower_arm_offset']
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, speed, 10],
                         delay=1,
                         timeout_function=None)
        
        # Close the gripper
        gripper = globals['fuel_tank_gripper_close']
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, speed, 10],
                         delay=1,
                         timeout_function=None)
        # Go up
        y = globals['fuel_tank_Y_higher_arm_offset']
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=[x, y, z, wrist, gripper, speed, 10],
                         delay=1,
                         timeout_function=None)
        

        return 'fuel_tank_picked_up'
    

class StoreFuelTank(smach.State):
    # Dictionary to map the slot number to the corresponding coordinates
    '''
        OVER_SLOT_COORDS = {
        1: [-115, 100, -95, 3500, 2640, 25, 10],      
        2: [-30, 100, -90, 3500, 2640, 25, 10],   #updated speeds
        3: [40, 100, -85, 3500, 2640, 25, 10]
    }

        IN_SLOT_COORDS = {
        1: [-115, 50, -95, 3500, 2640, 25, 10],
        2: [-30, 50, -90, 3500, 2640, 25, 10],
        3: [40, 50, -85, 3500, 2640, 25, 10]
    }

    '''

    OVER_SLOT_COORDS = {
        1: [-127, 100, -80.2, 3500, 2640, 25, 10],      
        2: [-40.2, 100, -85.2, 3500, 2640, 25, 10],   #updated speeds
        3: [40, 100, -85.2, 3500, 2640, 25, 10]
    }

    IN_SLOT_COORDS = {
        1: [-127, 30, -80.2, 3500, 2640, 25, 10],
        2: [-40.2, 30, -85.2, 3500, 2640, 25, 10],
        3: [40, 30, -85.2, 3500, 2640, 25, 10]
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
                         delay=2,
                         timeout_function=None)
        rospy.sleep(2)
        # Lower the arm
        publish_and_wait(pub=self.task_space_pub,
                         wait_for_topic='Arm_Done',
                         message_type=Float32MultiArray,
                         message_data=self.IN_SLOT_COORDS[self.slot_number],
                         delay=2,
                         timeout_function=None)
        
        # Open the gripper
        gripper = globals['fuel_tank_gripper_open']
        open_gripper_msg = self.IN_SLOT_COORDS[self.slot_number]
        open_gripper_msg[4] = gripper
        publish_command(self.task_space_pub, Float32MultiArray, open_gripper_msg, delay=1)

        # Go back to scan fuel tank pose
        speed = 50 #updated speed
        gripper = 2440
        rospy.loginfo('Moving to scan pose')
        # Publish command to set the arm to the scan pose
        publish_and_wait(pub=self.arm_angles_pub,
                        wait_for_topic='Arm_Done',
                        message_type=Float32MultiArray,
                        message_data=[1081.0, 2488.0, 2474.0, 1959.0, 2045.0, 675.0, 2089.0, gripper, speed],
                        delay=1,
                        timeout_function=None)
        
        rospy.sleep(1)
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
                    message_data=[-40, 100, -133, 2230, 1980, 10, 10],
                    delay=5,
                    timeout_function=None)

        #get lower to grab the device
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[-40, -10, -133, 2230, 1980, 10, 10],
                delay=4,
                timeout_function=None)    

        #close gripper
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[-40, -10, -133, 2230, 2700, 10, 10],
                delay=5,
                timeout_function=None)    
        
        #position the arm just above the thruster assembly zone
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 76, 2230, 2700, 10, 100],
                delay=8,
                timeout_function=None)

        #twist the wrist
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 76, 3700, 2700, 10, 10],
                delay=8,
                timeout_function=None)
        
        #place the device
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 81, 3700, 2700, -200, 5], #changed tolerance to 5, and positive z by 5
                delay=6,
                timeout_function=None)
        
        #release
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, -100, 76, 3700, 1400, 100, 10],
                delay=2,
                timeout_function=None)
        
        #get to a better position to press the button
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[135, 100, 76, 3700, 1400, 50, 100],
                delay=2,
                timeout_function=None)
        
        publish_and_wait(pub=self.task_space_pub,
                wait_for_topic='Arm_Done',
                message_type=Float32MultiArray,
                message_data=[100, 100, 100, 2048, 2700, 50, 100],
                delay=2,
                timeout_function=None)
        
        return 'fuel_tanks_placed'