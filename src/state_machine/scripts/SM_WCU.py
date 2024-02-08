#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import actionlib
import sys
from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
from image_utils.board_objects import BoardObjects

from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback

# Global variables
green_detected = False
arm_done = False

# Create publishers
task_space_pub = rospy.Publisher('Task_Space', Float32MultiArray, queue_size=10)
arm_angles_pub = rospy.Publisher('Arm_Angles', Float32MultiArray, queue_size=10)
#state_SM2Nav_pub = rospy.Publisher('State_SM2Nav', String, queue_size=10)

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
        arm_done = data.data
        rospy.loginfo("Arm Done: %s", arm_done)
    except Exception as e:
        rospy.logerr("Error in state_arm2sm_cb: {}".format(e))

# Create subscribers
#state_Nav2SM_sub = rospy.Subscriber("State_Nav2SM", String, callback=state_nav_cb)
start_led_state_sub = rospy.Subscriber("LED_State", Bool, callback=start_led_callback)
arm_done_sub = rospy.Subscriber("Arm_Done", Int8, callback=state_arm2sm_cb)

# define state Initialize
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.bot_initialized = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Initialize')
        # Run initialization logic
        self.bot_initialized = True
        rospy.sleep(1)

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
            rospy.sleep(1)
            return 'green_led_detected'
        return 'green_led_not_detected'


# define state SetPose
class ScanPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'])
        rospy.loginfo(f'Executing state ScanPose')
        global arm_done
        arm_done = False

    def execute(self, userdata):
        global arm_done
        angles_ = Float32MultiArray()
        
        #pose_.data = [185.0, 217.0, -30.0, 2048.0, 1446.0]
        angles_.data = [2164.0, 1776.0, 1776.0, 2787.0, 2048.0, 556.0, 3147782.0, 1446.0]
        arm_angles_pub.publish(angles_)
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
        goal.timeout.data = 10.0
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
            return 'coords_received'
        return 'coords_not_received'
    
# define state PickUp
class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUp')
        rospy.sleep(2)
        if True:
            return 'packages_picked_up'
        return 'packages_not_picked_up'

# define state VerifyPose
class VerifyPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['pose_reached','pose_not_reached'],
                             input_keys=['coordinates'])
        rospy.loginfo(f'Executing state VerifyPose')
        global arm_done
        arm_done = False

    def execute(self, userdata):
        # print the user data
        coordinates = userdata.coordinates
        rospy.loginfo(f'Coordinates: {coordinates}')
        # wait 5 seconds and go to next state
        rospy.sleep(5)

        return 'pose_reached'


# define state PickUpBigPackages
class PickUpBigPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['packages_picked_up','packages_not_picked_up'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpBigPackages')
        rospy.sleep(2)
        if True:
            return 'packages_picked_up'
        return 'packages_not_picked_up'

def main():
    rospy.init_node('STATE_MACHINE')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'succeeded':'READING_START_LED', 'aborted':'INITIALIZE'})
        smach.StateMachine.add('READING_START_LED', ReadingStartLED(), 
                               transitions={'green_led_detected':'PACKAGE_PICKUP', 'green_led_not_detected':'READING_START_LED'})


        # Create the sub SMACH state machine
        package_pickup_sm = smach.Concurrence(outcomes=['packages_picked_up','aborted'],
                                    default_outcome='aborted',
                                    outcome_map={'packages_picked_up':{
                                        'PICK_BIG_PACKAGES':'packages_picked_up',
                                        'PICK_SMALL_PACKAGES':'packages_picked_up'
                                    }})

        with package_pickup_sm:
            small_packages_sm = smach.StateMachine(outcomes=['packages_picked_up'])

            with small_packages_sm:
                smach.StateMachine.add('SCAN_POSE', ScanPose(),
                                        transitions={'pose_reached':'GET_SP_COORDS', 'pose_not_reached':'SCAN_POSE'})
                smach.StateMachine.add('GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose='SCAN'),
                                        transitions={'coords_received':'VERIFY_POSE', 'coords_not_received':'GET_SP_COORDS'})
                smach.StateMachine.add('VERIFY_POSE', VerifyPose(),
                                        transitions={'pose_reached':'VERIFY_COORDS', 'pose_not_reached':'VERIFY_POSE'})
                smach.StateMachine.add('VERIFY_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose='VERIFY'),
                                        transitions={'coords_received':'PICK_UP', 'coords_not_received':'VERIFY_COORDS'})
                smach.StateMachine.add('PICK_UP', PickUp(),
                                        transitions={'packages_picked_up':'packages_picked_up', 'packages_not_picked_up':'PICK_UP'})
                                        
            smach.Concurrence.add('PICK_BIG_PACKAGES', PickUpBigPackages())
            smach.Concurrence.add('PICK_SMALL_PACKAGES', small_packages_sm)

        smach.StateMachine.add('PACKAGE_PICKUP', package_pickup_sm,
                                transitions={'packages_picked_up':'END',
                                            'aborted':'INITIALIZE'})

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