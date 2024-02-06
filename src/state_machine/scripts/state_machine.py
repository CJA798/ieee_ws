#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import actionlib
from std_msgs.msg import Int8, Int32, Bool, String, Float32MultiArray

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
        rospy.sleep(10)

        if self.done:
            return 'succeeded'
        
        return 'aborted'

class SetPose(smach.State):
    def __init__(self, pose):
        self.pose = pose
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SetPose(scan)')
        rospy.sleep(5)
        pose = Float32MultiArray()
        pose.data = [1.0,1.0,15.0,10.0,10.0]
        arm_pub.publish(pose)

        if self.pose:
            return 'succeeded'
        return 'aborted'

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
    arm_feedback_pub = rospy.Publisher('Get_Feedback', Float32MultiArray, queue_size=10)

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
                smach.StateMachine.add('GET_SP_COORDS', GetCoords(object_type='small_package', pose='scan'),
                                        transitions={'succeeded':'VERIFY_POSE', 'aborted':'GET_SP_COORDS'})
                smach.StateMachine.add('VERIFY_POSE', SetPose('verify'),
                                        transitions={'succeeded':'VERIFY_COORDS', 'aborted':'VERIFY_POSE'})
                smach.StateMachine.add('VERIFY_COORDS', GetCoords(object_type='small_package', pose='verify'),
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


if __name__ == '__main__':
    main()