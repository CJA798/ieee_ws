#!/usr/bin/env python

import rospy
import smach
import smach_ros
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
    ''' State to initialize the robot'''
    def __init__(self, init_state_pub):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.init_state_pub = init_state_pub

    def execute(self, userdata):
        '''Delay for 5 seconds to simulate the initialization of the robot
        
        Args:
            userdata: The data passed to the state (Not used)
        
        Returns:
            str: The outcome of the state ('succeeded' or 'aborted')
        
        Raises:
            Exception: Any exception that occurs during the state execution'''
        try:
            rospy.loginfo('Executing state Initialize')
            rospy.sleep(5)
            rate = rospy.Rate(50)
            #while not rospy.is_shutdown():
            #    rate.sleep()
            # Publish the initial state
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
            outcome = getattr(self, method_name)()
            
            return outcome
        else:
            # Log that the area is invalid
            rospy.loginfo('Invalid area')
            return 'not_arrived'
    
    def GoToBigPackageWall(self):
        '''State to move the robot to the big package wall'''
        rate = rospy.Rate(20)
        try:
            # Reset the move_done global variable
            globals['move_done'] = False

            # Move to the big package wall
            message = Float32MultiArray()
            message.data = [-235, 0, 0, 20]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()
            
            return 'arrived'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GoToBigPackageWall: {e}")
            return 'not_arrived'
        
    def GoToPushBigPackages(self):
        '''State to move the robot to the push big packages area'''
        rate = rospy.Rate(20)
        rate2 = rospy.Rate(1/1.5)
        try:
            # Reset the move_done global variable
            globals['move_done'] = False

            # Move to the push big packages area
            message = Float32MultiArray()
            message.data = [-235, 1, 0, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            rospy.sleep(1)

            # Reset the move_done global variable
            globals['move_done'] = False

            # Stop
            message.data = [0, 0, 0, 0]
            self.move_pub.publish(message)

            #while not globals['move_done']  and not rospy.is_shutdown():
                #rate.sleep()
            
            globals['move_done'] = False
            
            return 'arrived'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GoToPushBigPackages: {e}")
            return 'not_arrived'
        
    def GoToDropOffArea(self):
        '''State to move the robot to the drop off area'''
        rate = rospy.Rate(20)
        try:
            # Reset the move_done global variable
            globals['move_done'] = False

            # Move to the second slope
            message = Float32MultiArray()
            message.data = [200, 1, 0, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while globals['gravity_vector'] > -25  and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Second slope reached')

            # Keep moving until the robot crosses the second slope
            while globals['gravity_vector'] < -10  and not rospy.is_shutdown():
                rate.sleep()

            # Reset the move_done global variable
            globals['move_done'] = False

            # Move to the drop off area
            message.data = [200, 200, 0, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()
            
            # Reset the move_done global variable
            globals['move_done'] = False
            return 'arrived'
        
        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GoToDropOffArea: {e}")
            return 'not_arrived'


    def GoToFuelTankArea(self):
        '''State to move the robot to the fuel tank area'''
        rate = rospy.Rate(20)
        try:
            # Reset the move_done global variable
            globals['move_done'] = False
            
            # Turn 90 degrees
            message = Float32MultiArray()
            message.data = [0, 0, -90, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()
        
            rospy.loginfo('Turned 90 degrees')

            # Reset the move_done global variable
            globals['move_done'] = False

            # Move to the fuel tank area
            message.data = [150, 270, -90, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()
            #rospy.sleep(4)
            # Reset the move_done global variable
            globals['move_done'] = False
            return 'arrived'

        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GoToFuelTankArea: {e}")
            return 'not_arrived'
        
    def GoToCraterArea(self):
        rate = rospy.Rate(30)
        angle = 180
        try:
            # Reset the move_done global variable
            globals['move_done'] = False

            # Rotate
            message = Float32MultiArray() # create an instance of the Float32MultiArray message
            message.data = [0, 0, angle, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()
            
            rospy.loginfo('Rotated 180 degrees')

            # Reset the move_done global variable 
            globals['move_done'] = False

            # Move to the top second ramp
            message.data = [200, 1, angle, 75]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while globals['gravity_vector'] < 15  and not rospy.is_shutdown():
                rate.sleep()
            
            rospy.loginfo('Half second ramp reached')
            while globals['gravity_vector'] > 2  and not rospy.is_shutdown():
                rate.sleep()


            rate2 = rospy.Rate(1)
            rate3 = rospy.Rate(2)
            rate3.sleep()

            
            # Move to the top second ramp
            message.data = [0, 0, angle, 0]
            self.move_pub.publish(message)

            rospy.loginfo('Top second ramp reached')

            # Reset the move_done global variable
            globals['move_done'] = False

            # Rotate to place bridge
            message.data = [0, 0, 0, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Rotated to place bridge')

            # Reset the move_done global variable
            globals['move_done'] = False

            # Back up until back tof reads over 70
            message.data = [0, -1, 0, 40]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while globals['tof_back'] < 70  and not rospy.is_shutdown():
                rate.sleep()
             

            rospy.loginfo('Backed up')

            globals['move_done'] = False

            # Stop
            message.data = [0, 0, 0, 0]
            self.move_pub.publish(message)
            
            # Reset the misc_done global variable
            globals['misc_done'] = False
            
            # Drop bridge
            bridge_message = Float32MultiArray()
            bridge_message.data = [3200, -1, -1, -1, -1, -1, -1, -1]
            self.misc_angles_pub.publish(bridge_message)

            # Wait for the move to complete
           
            #bridge_rate = rospy.Rate(1/3)
            #bridge_rate.sleep()

            # Wait for bridge to drop
            while not globals['misc_done']  and not rospy.is_shutdown():
                rate.sleep()

            rospy.loginfo('Bridge dropped')

            # Reset the misc_done variable
            globals['misc_done'] = False
            globals['move_done'] = False

            # Forward for ~2 seconds
            message.data = [0, 1, 0, 40]
            self.move_pub.publish(message)
            rate2.sleep()
            rate2.sleep()
            
            


            # Reset the move_done global variable
            globals['move_done'] = False

            # Stop
            message.data = [0, 0, 0, 0]
            self.move_pub.publish(message)
            
            # Reset the move_done global variable
            globals['move_done'] = False

            # Raise the bridge back up
            bridge_message.data = [2048, -1, -1, -1, -1, -1, -1, -1]
            self.misc_angles_pub.publish(bridge_message)

            # Go backwards on the bridge, move to next state

            return 'arrived'

        # Handle any exceptions that occur during the state execution
        except Exception as e:
            rospy.logerr(f"Error in GoToCraterArea: {e}")
            return 'not_arrived'
        
    def GoToButtonArea(self):
        rate = rospy.Rate(20)

        try:
            # Reset the move_done global variable
            globals['move_done'] = False

            # Rotate
            message = Float32MultiArray() # create an instance of the Float32MultiArray message
            #Back until finding the flat area
            message.data = [0, -1, 0, 40]
            self.move_pub.publish(message)

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

            #reset move_done
            globals['move_done'] = False

            # Rotate
            message.data = [0, 0, 90, 100]
            self.move_pub.publish(message)

            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()

            # Reset the move_done global variable
            globals['move_done'] = False

            # Aim camera to thruster assembly
            message.data = [100, 250, 90, 100]
            self.move_pub.publish(message)

            # Wait for the move to complete
            while not globals['move_done']  and not rospy.is_shutdown():
                rate.sleep()

            # Reset the move_done global variable
            globals['move_done'] = False

            #Stop and hit the button
            message.data = [0, 0, 0, 0]
            self.move_pub.publish(message)

            # Reset the move_done global variable
            globals['move_done'] = False

            # next state
            return 'arrived'


        # Handle any exceptions that occur during the state execution    
        except Exception as e:
            rospy.logerr(f"Error in GoToButtonArea: {e}")
            return 'not_arrived'


# Define state PickUp
class PickUp(smach.State):
    # Dictionary mapping board objects to method names
    BOARD_OBJECT_METHODS = {
        BoardObjects.SMALL_PACKAGE: "PickUpSmallPackages",
        BoardObjects.BIG_PACKAGE: "PickUpBigPackages",
        BoardObjects.FUEL_TANK: "PickUpFuelTanks"
    }
    def __init__(self, board_object, camera_pose, arm_angles_publisher, task_space_publisher=None):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])
        self.board_object = board_object
        self.camera_pose = camera_pose
        self.arm_angles_pub = arm_angles_publisher
        self.task_space_pub = task_space_publisher

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
            pose.data = [1940.0, 2125.0, 2120.0, 2443.0, 2179.0, 716.0, 2003.0, 1935.0, speed]
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
        pass


class SpiritCelebration(smach.State):
    def __init__(self, misc_angles_publisher = None,arm_angles_publisher = None):
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
            rate = rospy.Rate(30)
            globals['misc_done'] = False

            flag = Float32MultiArray()
            flag.data = [2048,2048,2048,2048,2048,2048,2048,2048,1]
            self.arm_angles_pub.publish(flag)

            while not globals['misc_done'] and not rospy.is_shutdown():
                rate.sleep()

            globals['misc_done'] = False
            
            
            return 'succeeded'
        
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
        rate = rospy.Rate(100)
        # Reset the arm_done global variable
        globals['arm_done'] = False
        speed = 1
        angles_ = Float32MultiArray()
        angles_.data = [2135, 1627, 1628, 2949, 2019, 629, 2110, 1935, speed]
        self.arm_angles_pub.publish(angles_)
        rospy.loginfo('Moving to scan pose')
        #while not globals['arm_done'] and not rospy.is_shutdown():
        #    rate.sleep()
        rospy.sleep(2)
        globals['arm_done'] = False
        #rospy.sleep(2000)
        return 'pose_reached'
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
        jaw = 1935

        for i in range(len(coordinates)):
            # Reset the arm_done global variable
            globals['arm_done'] = False

            # Go to first coordinate
            target = coordinates[i]
            quickness = 1

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
            task_space.data = [int(x_grabber), int(y), int(z_grabber), 2048, jaw, -40]
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
            rospy.sleep(2)
            
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
        #rospy.sleep(5)
        if True:
            return 'packages_picked_up'
        return 'packages_not_picked_up'

# define state PickUpFuelTanks
class PickUpFuelTanks(smach.State):
    def __init__(self, arm_angles_pub):
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