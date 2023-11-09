#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from std_msgs.msg import Int8

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

    def execute(self, userdata):
        rospy.loginfo('Executing state ReadingStartLED')
        # Run LED reading logic
        self.green_detected = True
        rospy.sleep(5)

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
        # Run LED reading logic
        self.done = True
        rospy.sleep(5)

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
                               transitions={'succeeded':'SM_BOXES', 'aborted':'READING_START_LED'})

        # Create the sub SMACH state machine
        sm_boxes = smach.Concurrence(outcomes=['succeeded','aborted'],
                                    default_outcome='aborted',
                                    outcome_map={'succeeded':{
                                        'PICK_BIG_PACKAGES':'succeeded',
                                        'PICK_SMALL_PACKAGES':'succeeded'
                                    }})

        with sm_boxes:
            smach.Concurrence.add('PICK_BIG_PACKAGES', PickUpBigPackages())
            smach.Concurrence.add('PICK_SMALL_PACKAGES', PickUpSmallPackages())

        smach.StateMachine.add('SM_BOXES', sm_boxes,
                                transitions={'succeeded':'GO_TO_DROP_OFF',
                                            'aborted':'INITIALIZE'})
        
        smach.StateMachine.add('GO_TO_DROP_OFF', GoToDropOff(),
                                transitions={'succeeded':'END'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Stop the introspection server
    sis.stop()
    rospy.loginfo('State machine execution completed.')


if __name__ == '__main__':
    main()