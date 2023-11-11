#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Int8, Int32

# define state Initialize
class Initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.test_pub = rospy.Publisher('test_topic', Int8, queue_size=10)
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
        self.green_led_sub = rospy.Subscriber("start_led", Int32, callback=self.start_led_cb)
        
    def start_led_cb(self, data):
        reading = data.data
        rospy.loginfo("Starting LED Reading: %i", reading)
        self.green_detected = reading

    def execute(self, userdata):
        rospy.loginfo('Executing state ReadingStartLED')
        # Run LED reading logic
        rospy.sleep(0.1)
        if self.green_detected < 1000:
            return 'succeeded'
        
        return 'aborted'

# define state PickUpBigPackages
class PickUpBigPackages(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.done = False

    def execute(self, userdata):
        rospy.loginfo('Executing state PickUpBigPackages')
        # Run LED reading logic
        self.done = True
        rospy.sleep(15)

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
        # Run LED reading logic
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
        if self.pose:
            return 'succeeded'
        return 'aborted'

class GetCoords(smach.State):
    def __init__(self, object_type, pose):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetCoords(small_packages, scan)')
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