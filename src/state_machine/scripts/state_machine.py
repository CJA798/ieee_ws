#!/usr/bin/env python

#hello

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import actionlib
import sys
from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray
from image_utils.board_objects import BoardObjects

from vision_system.msg import GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback

arm_pub = rospy.Publisher('Task_Space', Float32MultiArray, queue_size=10)


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


# define state ReadingStartLED
class ReadingStartLED(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.green_detected = False
        self.green_led_sub = rospy.Subscriber("LED_State", Bool, callback=self.start_led_cb)
        
    def start_led_cb(self, data):
        reading = data.data
        #self.green_detected = reading
        self.green_detected = True
        #rospy.loginfo("Green LED: %s", reading)

    def execute(self, userdata):
        rospy.loginfo('Executing state ReadingStartLED')
        rospy.sleep(0.1)

        self.green_detected = True
        if self.green_detected:
            return 'succeeded'
        return 'aborted'

# define state PickUpBigPackages
class PickUpBigPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.done = False

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpBigPackages')
        # Run big package pickup logic
        self.done = True
        rospy.sleep(10)
        if self.done:
            return 'succeeded'
        
        return 'aborted'

# define state PickUpSmallPackages
class PickUpSmallPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.done = False

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpSmallPackages')
        # Run small package pickup logic
        # Place arm in scanning position
        # Get coordinates from camera

        # Pickup boxes 
            # Place arm on top of box
            # Get feedback from camera while lowering to grab box
            # Put box in respective container

        self.done = True
        rospy.sleep(5)

        if self.done:
            return 'succeeded'
        
        return 'aborted'

class SetPose(smach.State):
    def __init__(self, pose):
        self.pose = pose
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        rospy.loginfo(f'Executing state SetPose({pose})')
        self.arm_done = False
        self.arm_done_sub = rospy.Subscriber('Arm_Done', Int8, callback=self.arm_done_cb)

    def execute(self, userdata):
        pose = Float32MultiArray()
        if self.pose == 'SCAN':
            pose.data = [100.0, 250.0, 0.0, 2048.0, 1446.0]

        elif self.pose == 'VERIFY':
            pose.data = [100.0,100.0,150.0,2048,2048]

        arm_pub.publish(pose)

        #rospy.sleep(5)
        #self.arm_done = True
        if self.arm_done:
            return 'succeeded'
        return 'aborted'
    
    def arm_done_cb(self, message):
        self.arm_done = message.data

class GetCoords(smach.State):
    def __init__(self, object_type, pose):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.object_type = object_type
        self.pose = pose

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
        print(result)
        rospy.loginfo(f'Final Coordinates List: {result.coordinates}    |    Total time: {result.elapsed_time.data}')

        
        rospy.sleep(1)
        if True:
            return 'succeeded'
        return 'aborted'

class PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUp')
        rospy.sleep(2)
        if True:
            return 'succeeded'
        return 'aborted'

# define state GoToDropOff
class GoToDropOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToDropOff')
        rospy.sleep(5)

        return 'succeeded'

def feedback_callback(msg):
    pass

def state_status_callback(msg):
    pass

def main():
    rospy.init_node('STATE_MACHINE')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])

    # Create publishers
    state_pub = rospy.Publisher('State', String, queue_size=10)
    misc_pub = rospy.Publisher('Misc_Angles', Float32MultiArray, queue_size=10)
    arm_feedback_pub = rospy.Publisher('Get_Feedback', Int8, queue_size=10)

    # Create subscribers
    feedback_sub = rospy.Subscriber('Feedback', Float32MultiArray, feedback_callback)
    state_status_sub = rospy.Subscriber('Nav_State', String, state_status_callback)
    arm_state_status_sub = rospy.Subscriber('Arm_Done', Int8, state_status_callback)


    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALIZE', Initialize(), 
                               transitions={'succeeded':'READING_START_LED', 'aborted':'INITIALIZE'})
        smach.StateMachine.add('READING_START_LED', ReadingStartLED(), 
                               transitions={'succeeded':'PACKAGE_PICKUP', 'aborted':'READING_START_LED'})

        # Create the sub SMACH state machine
        sm_packages = smach.Concurrence(outcomes=['succeeded','aborted'],
                                    default_outcome='aborted',
                                    outcome_map={'succeeded':{
                                        'PICK_BIG_PACKAGES':'succeeded',
                                        'PICK_SMALL_PACKAGES':'succeeded'
                                    }})

        with sm_packages:
            sm_small_packages = smach.StateMachine(outcomes=['succeeded'])

            with sm_small_packages:
                smach.StateMachine.add('SCAN_POSE', SetPose('scan'),
                                        transitions={'succeeded':'GET_SP_COORDS', 'aborted':'SCAN_POSE'})
                smach.StateMachine.add('GET_SP_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose='SCAN'),
                                        transitions={'succeeded':'VERIFY_POSE', 'aborted':'GET_SP_COORDS'})
                smach.StateMachine.add('VERIFY_POSE', SetPose('verify'),
                                        transitions={'succeeded':'VERIFY_COORDS', 'aborted':'VERIFY_POSE'})
                smach.StateMachine.add('VERIFY_COORDS', GetCoords(object_type=BoardObjects.SMALL_PACKAGE.value, pose='VERIFY'),
                                        transitions={'succeeded':'PICK_UP', 'aborted':'VERIFY_COORDS'})
                smach.StateMachine.add('PICK_UP', PickUp(),
                                        transitions={'succeeded':'succeeded', 'aborted':'PICK_UP'})
                                        
                                        
            smach.Concurrence.add('PICK_BIG_PACKAGES', PickUpBigPackages())
            smach.Concurrence.add('PICK_SMALL_PACKAGES', sm_small_packages)

        smach.StateMachine.add('PACKAGE_PICKUP', sm_packages,
                                transitions={'succeeded':'GO_TO_DROP_OFF',
                                            'aborted':'INITIALIZE'})
        
        smach.StateMachine.add('GO_TO_DROP_OFF', GoToDropOff(),
                                transitions={'succeeded':'END'})

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