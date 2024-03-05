#!/usr/bin/env python

import rospy
import smach
import smach_ros

from typing import Optional, Iterable
from rospy import Publisher
from rospy.msg import AnyMsg

from enum import Enum
from math import sqrt, atan2, sin, cos, degrees, radians

from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
import actionlib

from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback

from utils.areas import Areas
from utils.globals import globals
from utils.callbacks import get_coords_fb_cb
from utils.fuel_tank_utils import fuel_tanks

from image_utils.poses import Poses
from image_utils.board_objects import BoardObjects


def publish_move(move_pub, message=Float32MultiArray(), data=[0, 0, 0, 0]):
    # Reset the move_done global variable
    globals['move_done'] = False

    # Add the data to the message
    message.data = data

    # Publish the message
    move_pub.publish(message)

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
            rospy.sleep(2)

            # Check if the heartbeat is received, i.e. the Arduino is connected
            if not globals['heartbeat_on']:
                rospy.logerr('Heartbeat not received')
                return 'aborted'
            
            # Publish the init message to the Arduino
            init_msg = Bool()
            init_msg.data = True
            self.init_state_pub.publish(init_msg)
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
        
        rate = rospy.Rate(20)  # 10 Hz
        # Wait for the green LED to be detected
        try:
            while not globals['green_detected'] and not rospy.is_shutdown():
                rate.sleep()
            return 'green_led_detected'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr("Error in ReadingStartLED: {}".format(e))
            return 'green_led_not_detected'
        
        

class GoTo_(smach.State):
    # Dictionary mapping areas to method names
    AREA_METHODS = {
        Areas.BIG_PACKAGE_WALL: "GoToBigPackageWall",
        Areas.PUSH_BIG_PACKAGES: "GoToPushBigPackages",
        Areas.DROP_OFF: "GoToDropOffArea",
        Areas.FUEL_TANK: "GoToFuelTankArea",
        Areas.CRATER: "GoToCraterArea",
        Areas.BUTTON: "GoToButtonArea"
    }

    def __init__(self, area, move_publisher, misc_angles_publisher=None):
        # Initialize the state with outcomes 'arrived' and 'not_arrived'
        smach.State.__init__(self, outcomes=['arrived', 'not_arrived'])
        self.area = area
        self.move_pub = move_publisher
        self.misc_angles_pub = misc_angles_publisher

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
            # Log the execution of the state
            rospy.loginfo(f"Executing state {method_name}")
            # Call the corresponding method dynamically using getattr
            try:
                outcome = getattr(self, method_name)()

            # Handle any exceptions that occur during the state execution
            except Exception as e:
                rospy.logerr(f"Error in GoTo{self.area}: {e}")
                return 'not_arrived'
            
            return outcome
        else:
            # Log that the area is invalid
            rospy.loginfo('Invalid area')
            return 'not_arrived'
    
    def GoToBigPackageWall(self):
        '''State to move the robot to the big package wall'''
        # Publish move to the big package wall
        y_offset = globals['big_package_Y_offset']
        publish_move(self.move_pub, data=[y_offset, 0, 0, 50])

        # Wait for the move_done message
        rospy.wait_for_message("Move_Done", Int8, timeout=10)
        if stop_move(self.move_pub):
            return 'arrived'
        # TODO: implement timeout routine

        return 'arrived'

    def GoToPushBigPackages(self):
        '''State to move the robot to the push big packages area'''
        # Publish command to push the big packages forward
        y_offset = globals['big_package_Y_offset']
        publish_command(self.move_pub, Float32MultiArray, [y_offset, 1, 0, 50])

        # Wait for the move to complete
        # This one is an exception bc we don't have a rear TOF
        rospy.sleep(1)

        # Stop
        if stop_move(self.move_pub):
            return 'arrived'
        else:
            return 'not_arrived'
        

        
    def GoToDropOffArea(self):
        '''State to move the robot to the drop off area'''
        rate = rospy.Rate(20)
        
        # Publish command to go straight indefinitely
        publish_command(self.move_pub, Float32MultiArray, [200, 1, 0, 100])

        # Wait for the move to complete
        while globals['gravity_vector'] > -25  and not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo('Second slope reached')

        # Keep moving until the robot crosses the second slope
        while globals['gravity_vector'] < -10  and not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo('Crossed first ramps')

        # Publish command to reach to the drop off area
        publish_command(self.move_pub, Float32MultiArray, [170, 180, 0, 100])

        # Wait for the move to complete
        rospy.wait_for_message("Move_Done", Int8, timeout=10)

        # Publish the command to turn 90 degrees
        if publish_command(self.move_pub, Float32MultiArray, [0, 0, -90, 100]):
            # Wait for the move to complete
            rospy.wait_for_message("Move_Done", Int8, timeout=10)
            return 'arrived'
        else:
            return 'not_arrived'



    def GoToFuelTankArea(self):
        '''State to move the robot to the fuel tank area'''

        # Publish command to reach to the drop off area
        if publish_command(self.move_pub, Float32MultiArray, [147, 255, -90, 100]):

        # Wait for the move to complete
            rospy.wait_for_message("Move_Done", Int8, timeout=10)
            return 'arrived'
        else:
            return 'not arrived'


        
    def GoToCraterArea(self):
        rate = rospy.Rate(30)
        record_tof_back = 0
        #angle = 0
      
        # Reset the move_done global variable
        #globals['move_done'] = False

        # Rotate
        message = Float32MultiArray() # create an instance of the Float32MultiArray message
        #message.data = [0, 0, angle, 100]
        #self.move_pub.publish(message)

        # Wait for the move to complete
       # while not globals['move_done']  and not rospy.is_shutdown():
       #     rate.sleep()
        
        # Publish command to reach to the drop off area
        publish_command(self.move_pub, Float32MultiArray, [0, 0, -180, 100])

        # Wait for the move to complete
        rospy.wait_for_message("Move_Done", Int8, timeout=10)
        
        rospy.loginfo('Rotated 180 degrees')

        record_tof_back = globals['tof_back']


        # Publish command to reach to the drop off area
        publish_command(self.move_pub, Float32MultiArray, [200, 1, -180, 100])

        # Wait for the move to complete
        #rospy.wait_for_message("Move_Done", Int8, timeout=10)

        # Reset the move_done global variable 
        #globals['move_done'] = False

        # Move to the top second ramp
        #message.data = [200, 1, angle, 75]
        #self.move_pub.publish(message)

        # Wait for the move to complete
        while globals['gravity_vector'] < 28  and not rospy.is_shutdown():
            rate.sleep()
        
        rospy.loginfo('Half second ramp reached')
       

        while globals['gravity_vector'] > 5  and not rospy.is_shutdown():
            rate.sleep()
   
            
        while ((globals['tof_back'] - record_tof_back) > 5) and not rospy.is_shutdown():
            rospy.loginfo('Top second ramp reached')
            rospy.sleep(0.5)
            break

        #stop_move(self.move_pub)
        #rospy.wait_for_message("Move_Done", Int8, timeout=10)
        #rate2 = rospy.Rate(1)
        #rate3 = rospy.Rate(2)
        #rate3.sleep()
        # Publish command to reach to the drop off area
        publish_command(self.move_pub, Float32MultiArray, [0, 0, 0, 100])

        # Wait for the move to complete
        rospy.wait_for_message("Move_Done", Int8, timeout=10)

        
        # Move to the top second ramp
        #message.data = [0, 0, angle, 0]
        #self.move_pub.publish(message)

        

    
        rospy.loginfo('Rotated to place bridge')

  
         # Publish command 
        publish_command(self.move_pub, Float32MultiArray, [0, -1, 0, 20])
        rospy.loginfo('Backed up')

        # Publish command to drop the bridge
        bridge = globals['lowered_bridge']
        publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1])
        rospy.wait_for_message("Misc_Done", Int8, timeout=10)
        rospy.loginfo('Bridge dropped')

        # Publish command to go fwd 
        publish_command(self.move_pub, Float32MultiArray, [0, 1, 0, 20])
        rospy.sleep(2)
        stop_move(self.move_pub)

        # Publiish command to raise the bridge
        bridge = globals['raised_bridge']
        if publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, -1, -1, -1]):
            rospy.loginfo('Bridge raised')
            return 'arrived'

        return 'not_arrived'


    def GoToButtonArea(self):
        rate = rospy.Rate(20)

        #Back until finding the flat area        
        publish_command(self.move_pub, Float32MultiArray, [0, -1, 0, 40])

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

        publish_command(self.move_pub, Float32MultiArray, [0, 0, 90, 100])
        rospy.wait_for_message("Move_Done", Int8, timeout=10)

        publish_command(self.move_pub, Float32MultiArray, [100, 250, 90, 100])
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
    
    def SetBulkGrabberArms(self):
        # Set the misc angles
        bridge = globals['raised_bridge']
        top_bulk = globals['set_bulk_top']
        bottom_bulk = globals['set_bulk_bottom']
        flag = globals['lowered_flag']

        # Publish the misc angles to set the bulk grabber arms to the init pose
        if publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, top_bulk, bottom_bulk, flag]):
            # Waitfor the bulk grabber arms to reach the pose
            rospy.wait_for_message("Misc_Done", Int8, timeout=10)
            # TODO: remove this sleep without breaking anything
            rospy.sleep(2)
            return 'pose_reached'
        else:
            return 'pose_not_reached'
        
    def CloseTopBulkGrabberArm(self):
        # Set the misc angles
        bridge = globals['raised_bridge']
        top_bulk = globals['close_bulk_top']
        bottom_bulk = globals['set_bulk_bottom']
        flag = globals['lowered_flag']

        # Publish the misc angles to close the top bulk grabber arm
        if publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, top_bulk, bottom_bulk, flag]):
            # Waitfor the bulk grabber arms to reach the pose
            rospy.wait_for_message("Misc_Done", Int8, timeout=10)
            return 'pose_reached'
        else:
            return 'pose_not_reached'
        
    def RaiseBulkGrabberArms(self):
        # Set the misc angles
        bridge = globals['raised_bridge']
        top_bulk = globals['raise_bulk_top']
        bottom_bulk = globals['raise_bulk_bottom']
        flag = globals['lowered_flag']

        # Publish the misc angles to raise the bulk grabber arms
        if publish_command(self.move_pub, Float32MultiArray, [-175, 0, 0, 100]) and publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, top_bulk, bottom_bulk, flag]):
            # Waitfor the bulk grabber arms to reach the pose
            rospy.wait_for_message("Move_Done", Int8, timeout=10)
            # Set big package pick up flag to True
            globals['big_packages_picked_up'] = True
            return 'pose_reached'
        else:
            return 'pose_not_reached'

    def DropOffSmallPackages(self):
        # TODO: Format this method better
        # Publish command to move arm over drop off area
        jaw = globals['gripper_bulk_hold']
        speed = 1
        angles = [608.0, 1634.0, 1638.0, 2721.0, 2023.0, 841.0, 2194.0, jaw, speed]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)
        # Wait for the arm to reach the pose
        rospy.wait_for_message("Arm_Done", Int8, timeout=10) 

        # Publish command to lower arm
        jaw = globals['gripper_bulk_hold']
        speed = 1
        angles = [497.0, 1242.0, 1245.0, 2730.0, 2022.0, 1250.0, 2077.0, jaw, speed]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles)
        # Wait for the arm to reach the pose
        rospy.wait_for_message("Arm_Done", Int8, timeout=10) 

        return 'pose_reached'


    def ReleaseSmallPackages(self):
        # Publish command to release the small packages
        jaw = globals['gripper_bulk_release']
        speed = 1
        angles = [497.0, 1242.0, 1245.0, 2730.0, 2022.0, 1250.0, 2077.0, jaw, speed]
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
        speed = 1
        angles = [1185.0, 2236.0, 2231.0, 2151.0, 2017.0, 692.0, 2109.0, jaw, speed]
        
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
        
    def PickUpSmallPackages(self):
        # Create a rate object to control the loop rate
        # This one should have a higher rate than the other states
        rate = rospy.Rate(40)
        rate2 = rospy.Rate(1/2)

        try:

            # Reset the arm_done global variable
            globals['arm_done'] = False

            # Go to scan pose
            pose = Float32MultiArray()
            speed = 1
            jaw = globals['gripper_bulk_hold']
            pose.data = [1940.0, 2125.0, 2120.0, 2443.0, 2179.0, 716.0, 2003.0, jaw, speed]
            self.arm_angles_pub.publish(pose)

            # Wait for the arm to move to the scan pose
            while not globals['arm_done']  and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Arm moved to scan pose')
            rate2.sleep()
            # Reset the arm_done global variable
            globals['arm_done'] = False

            # Get coordinates of detected small packages
            # Create an action client for the GetCoords action
            client = actionlib.SimpleActionClient('get_coords', GetCoordsAction)
            client.wait_for_server()

            # Create a goal for the GetCoords action
            goal = GetCoordsGoal()
            goal.timeout.data = 5.0
            if self.camera_pose == Poses.FRONT.value and self.object_type == BoardObjects.FUEL_TANK.value:
                goal.expected_pairs.data = 1
            else:
                goal.expected_pairs.data = 3
            goal.object_type.data = self.board_object.value
            goal.arm_pose.data = self.camera_pose.value

            # Send the goal to the action server
            client.send_goal(goal, feedback_cb=get_coords_fb_cb)
            
            # Wait for the action to complete
            client.wait_for_result()
            result = client.get_result()

            rospy.loginfo(f'Final Coordinates List: {result.coordinates}    |    Total time: {result.elapsed_time.data}')
            # If no coordinates are detected, return 'packages_not_picked_up'

            # Reset the arm_done global variable
            globals['arm_done'] = False

            # Go to over small package grabber
            rospy.loginfo('Moving to over small package grabber')
            speed = 1
            pose.data = [2903.0, 2186.0, 2179.0, 1652.0, 2058.0, 1504.0, 1825.0, 1935.0, speed]
            self.arm_angles_pub.publish(pose)
            rospy.loginfo('WASD to over small package grabber')
            # Wait for the arm to move over the small package grabber
            #while not globals['arm_done']  and not rospy.is_shutdown():
             #   rate.sleep()

            rospy.loginfo('Arm moved over small package grabber')
            rate2.sleep()

            # Close gripper
            speed = 1
            pose.data = [2903.0, 2186.0, 2179.0, 1652.0, 2058.0, 1504.0, 1825.0, 2041.0, speed]
            self.arm_angles_pub.publish(pose)

            rate2.sleep()

            rospy.loginfo('Arm closed gripper')
            # Reset the arm_done global variable
            globals['arm_done'] = False


            # Grab the detected small packages
                
            rospy.sleep(10)

            return 'packages_picked_up'

        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in PickUpSmallPackages: {e}")
            return 'packages_not_picked_up'
        
    def PickUpBigPackages(self):
        rate = rospy.Rate(20)
        try:
            rate.sleep()
            return 'packages_picked_up'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in PickUpBigPackages: {e}")
            return 'packages_not_picked_up'

    def PickUpFuelTanks(self):
        
        # Set the misc angles
        raised_bridge = globals['raised_bridge']
        raised_bridge = globals['raised_bridge']
        top_bulk = globals['bulk_top_close']
        bottom_bulk = globals['bulk_fuel_bottom']
        flag = globals['lowered_flag']
        raise_bulk_offset = globals['fuel_tank_raise_bulk_offset']

        # TODO: format this method better
        # Publish the misc angles to set the bulk grabber arms to the init pose
        publish_command(self.misc_angles_pub, Float32MultiArray, [raised_bridge, -1, bottom_bulk, flag])
        
        # Wait for the bulk grabber arms to reach the pose
        rospy.wait_for_message("Misc_Done", Int8, timeout=10)
        
        publish_command(self.move_pub, Float32MultiArray, [0, 110, -90, 100])
        rospy.wait_for_message("Move_Done", Int8, timeout=10)
        
        publish_command(self.misc_angles_pub, Float32MultiArray, [raised_bridge, top_bulk, bottom_bulk, flag])
        rospy.wait_for_message("Misc_Done", Int8, timeout=10)

        if publish_command(self.misc_angles_pub, Float32MultiArray, [raised_bridge, (top_bulk+raise_bulk_offset), (bottom_bulk-raise_bulk_offset), flag]):
            rospy.wait_for_message("Misc_Done", Int8, timeout=10)
        
        
            return 'packages_picked_up'
        else:
            return 'packages_not_picked_up'
      # pass
    

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
        jaw = globals['gripper_bulk_hold']
        speed = 1
        angles = [495.0, 1669.0, 1664.0, 2507.0, 2087.0, 946.0, 3915.0, jaw, speed]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles, delay=4)
        rospy.loginfo('Moving arm over red area')
        #rospy.wait_for_message("Arm_Done", Int8, timeout=10) 

        # Lower the arm
        jaw = globals['gripper_bulk_hold']
        speed = 2
        angles = [493.0, 1459.0, 1460.0, 2196.0, 2087.0, 1432.0, 3824.0, jaw, speed]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles, delay=4)
        rospy.loginfo('Lowering arm')
        #rospy.wait_for_message("Arm_Done", Int8, timeout=10) 

        # Open the gripper
        jaw = globals['gripper_bulk_release']
        speed = 5
        angles = [493.0, 1459.0, 1460.0, 2196.0, 2087.0, 1432.0, 3824.0, jaw, speed]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles, delay=2)
        #rospy.wait_for_message("Arm_Done", Int8, timeout=10) 
        rospy.loginfo('Small packages released')
        
        # Raise arm back over red area
        angles = [495.0, 1669.0, 1664.0, 2507.0, 2087.0, 946.0, 3915.0, jaw, speed]
        publish_command(self.arm_angles_pub, Float32MultiArray, angles, delay=2)
        #rospy.wait_for_message("Arm_Done", Int8, timeout=10)

        # Go to scan fuel tank position
        angles = [799.0, 2419.0, 2401.0, 1808.0, 2024.0, 836.0, 1788.0, jaw, speed]
        if publish_command(self.arm_angles_pub, Float32MultiArray, angles):
            rospy.loginfo('Moving arm to fuel tank scan position')
            return 'packages_dropped_off'
        else:
            rospy.logerr('Error releasing small packages')
            return 'packages_not_dropped_off'

    def DropOffBigPackages(self):
        # Set the misc angles
        raised_bridge = globals['raised_bridge']
        mid_bridge = globals['mid_bridge']
        top_bulk = globals['drop_bulk_top']
        bottom_bulk = globals['drop_bulk_bottom']
        flag = globals['lowered_flag']

        # TODO: format this method better
        # Publish the misc angles to set the bulk grabber arms to the init pose
        if publish_command(self.misc_angles_pub, Float32MultiArray, [mid_bridge, top_bulk, bottom_bulk, flag]):
            # Wait for the bulk grabber arms to reach the pose
            rospy.wait_for_message("Misc_Done", Int8, timeout=10)

            # Raise bot arm to avoid interferring with TOF_Right readings
            top_bulk = globals['raise_bulk_top']
            bottom_bulk = globals['raise_bulk_bottom']
            publish_command(self.misc_angles_pub, Float32MultiArray, [raised_bridge, top_bulk, bottom_bulk, flag])
            
            # Wait for the bulk grabber arms to reach the pose
            rospy.wait_for_message("Misc_Done", Int8, timeout=10)

            return 'packages_dropped_off'
        else:
            return 'packages_not_dropped_off'

class SpiritCelebration(smach.State):
    def __init__(self, misc_angles_publisher = None, arm_angles_publisher = None):
        smach.State.__init__(self, outcomes = ['succeeded','aborted'])
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

            # Publish the misc angles to close the top bulk grabber arm
            if publish_command(self.misc_angles_pub, Float32MultiArray, [bridge, top_bulk, bottom_bulk, flag]):
                # Waitfor the bulk grabber arms to reach the pose
                rospy.wait_for_message("Misc_Done", Int8, timeout=10)
                return 'succeeded'
            else:
                return 'aborted'
            
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in SpiritCelebration: {e}")
            return 'aborted'

# define state ButtonPress
class ButtonPress(smach.State):
    def __init__(self, move_publisher):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.move_pub = move_publisher

    def execute(self, userdata):
        '''Execute the state logic to press the button
        
        Args:
            userdata: The data passed to the state (Not used)
            
        Returns:
            str: The outcome of the state ('succeeded' or 'aborted')
            
        Raises:
            Exception: Any exception that occurs during the state execution'''
        # Press the button
        try:
            rate = rospy.Rate(1)

            
            # Reset the move_done global variable
            globals['move_done'] = False

            # Move to the button
            message = Float32MultiArray()
            message.data = [0, -1, 90, 50]
            self.move_pub.publish(message)

            rate.sleep()

            # Wait for the move to complete
            globals['move_done'] = False

            # Stop
            message.data = [0, 0, 0, 0]
            self.move_pub.publish(message)

            # Reset the move_done global variable
            globals['move_done'] = False

            # finished
            return 'succeeded'

        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in ButtonPress: {e}")
            return 'aborted'
        


    
# define state SetPose
class ScanPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state ScanPose')
        self.arm_angles_pub = arm_angles_pub

    def execute(self, userdata):
        speed = 1
        jaw = globals['gripper_bulk_hold']
        rospy.loginfo('Moving to scan pose')
        # Publish command to set the arm to the scan pose
        if publish_command(self.arm_angles_pub, Float32MultiArray, [2041.0, 2023.0, 2015.0, 2660.0, 2083.0, 489.0, 2039.0, jaw, speed]):
            # Wait for the arm to reach the pose
            #rospy.wait_for_message("Arm_Done", Int8, timeout=15)
            rospy.sleep(4)
            return 'pose_reached'
        else:
            return 'pose_not_reached'

    
# define state SetPose
class RestPose(smach.State):
    def __init__(self, arm_angles_pub):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state RestPose')
        self.arm_angles_pub = arm_angles_pub

    def execute(self, userdata):
        try:
            rate = rospy.Rate(100)
            # Reset the arm_done global variable
            globals['arm_done'] = False
            speed = 1
            jaw = globals['gripper_bulk_hold']
            angles_ = Float32MultiArray()
            angles_.data = [1058.0, 2852.0, 2847.0, 1405.0, 2109.0, 1736.0, 1038.0, jaw, speed]
            self.arm_angles_pub.publish(angles_)
            rospy.loginfo('Moving to rest pose')
            
            return 'pose_reached'
        except Exception as e:
            rospy.logerr(f"Error in RestPose: {e}")
            return 'pose_not_reached'

# define state GetCoords
class GetCoords(smach.State):
    def __init__(self, object_type, pose):
        smach.State.__init__(self, outcomes=['coords_received','coords_not_received'],
                             output_keys=['coordinates_list'])
        self.object_type = object_type
        self.pose = pose
        #TODO for some reason, removing this sleep brings back the fucking
        # raise ValueError(f"Object type {object_type} not recognized.")
        # ValueError: Object type SMALL_PACKAGE not recognized.

        #rospy.sleep(5)

    def execute(self, userdata):
        #if not globals['big_packages_picked_up']:
        #    rospy.logwarn('Big package pick up not done yet')
        #    rospy.sleep(1)
        #    return 'coords_not_received'
        try:
            rospy.loginfo('Executing state GetCoords(small_packages, scan)')
            
            client = actionlib.SimpleActionClient('get_coords', GetCoordsAction)
            client.wait_for_server()

            goal = GetCoordsGoal()
            goal.timeout.data = 5.0
            if self.pose == Poses.FRONT.value and self.object_type == BoardObjects.FUEL_TANK.value:
                goal.expected_pairs.data = 1
            else:
                goal.expected_pairs.data = 3
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
                userdata.coordinates_list = result.coordinates

            return 'coords_received'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GetCoords: {e}")
            return 'coords_not_received'
        


# define state VerifyPose
class VerifyPose(smach.State):
    def __init__(self, task_space_pub, offset=0.0):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'],
                             input_keys=['coordinates_list'],
                             output_keys=['coordinates_list_out'])
        rospy.loginfo(f'Executing state VerifyPose')
        self.task_space_pub = task_space_pub

    def execute(self, userdata):
        rate = rospy.Rate(50)
        # Store the coordinates list: CoordinatesList -> coordinates[coordinates]
        coordinates = userdata.coordinates_list.coordinates
        rospy.loginfo(f'Coordinates: {coordinates}')
        
        # Check if the coordinates list is empty
        if not coordinates:
            return 'pose_reached'
        
        task_space = Float32MultiArray()
        jaw = globals['gripper_bulk_hold']

        for i in range(len(coordinates)):
            # Reset the arm_done global variable
            globals['arm_done'] = False

            # Go to first coordinate
            target = coordinates[i]
            vertical_distance = -60
            speed = 1

            x = target.x
            y = target.y
            z = target.z

            grabber_offset = 0

            # Calculate offset from end effector to small package grabber
            magnitude = sqrt(z**2 + x**2) + grabber_offset
            angle = degrees(atan2(x, z))
            Kx = 1.0
            Kz = 1.0
            x_grabber = magnitude * sin(radians(angle)) * Kx
            z_grabber = magnitude * cos(radians(angle)) * Kz

            print(f'x_offset: {x_grabber}, z_offset: {z_grabber}')

            #x_grabber = target.x 
            #z_grabber = target.z 

            # Reset the arm_done global variable
            globals['arm_done'] = False

            # Move to coordinates
            task_space.data = [int(x_grabber), int(y), int(z_grabber), 2048, jaw, vertical_distance]
            self.task_space_pub.publish(task_space)

            while not globals['arm_done'] and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Arm moved to location')
            globals['arm_done'] = False

            # Raise the arm after picking object
            task_space.data = [int(x_grabber), int(y)+50, int(z_grabber), 2048, jaw, speed]
            self.task_space_pub.publish(task_space)

            while not globals['arm_done'] and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Arm moved to location')
            globals['arm_done'] = False

            ''''# Move to coordinates
            task_space.data = [x_grabber, -60, z_grabber, 2048, jaw, 1]
            self.task_space_pub.publish(task_space)

            while not globals['arm_done'] and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Grabbing object')
            globals['arm_done'] = False
            rospy.sleep(0.7)
            # Move to coordinates
            task_space.data = [x_grabber, -90, z_grabber, 2048, jaw, 1]
            self.task_space_pub.publish(task_space)

            while not globals['arm_done'] and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Grabbing object')
            globals['arm_done'] = False

            rate.sleep()
            # Move to coordinates
            task_space.data = [x_grabber , y, z_grabber, 2048, jaw, quickness]
            self.task_space_pub.publish(task_space)
            while not globals['arm_done'] and not rospy.is_shutdown():
                rate.sleep()'''

            rospy.loginfo('Picking object')
            globals['arm_done'] = False
            rospy.sleep(0.2)
            
        return 'pose_reached'

class PickUp_(smach.State):
    def __init__(self, task_space_pub):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])
        self.task_space_pub = task_space_pub

    def execute(self, userdata):
        task_space = Float32MultiArray()
        task_space.data = [100, 100, 0, 2048, 2100, 10]
        self.task_space_pub.publish(task_space)

        if True:
            rospy.sleep(5)
            return 'packages_picked_up'
        return 'packages_not_picked_up'
  

        
####################################################################################################
#    
####################################################################################################
#                                    O L D  C O D E   B E L O W
####################################################################################################
#    
####################################################################################################
    
# define state PickUpBigPackages
class PickUpBigPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpBigPackages')
        #rospy.sleep(5)
        if True:
            return 'packages_picked_up'
        return 'packages_not_picked_up'  

