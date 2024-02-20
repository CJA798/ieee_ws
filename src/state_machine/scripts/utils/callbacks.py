import rospy
from utils.globals import globals

def start_led_callback(data):
    '''Callback function to handle start green LED state updates
    
    Args:
        data (std_msgs.Bool): The new state of the green LED

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    try:
        if not globals['green_detected'] == data.data:    
            globals['green_detected'] = data.data
            rospy.loginfo("LED state updated: {}".format(globals['green_detected']))
    except Exception as e:
        rospy.logerr("Error in start_led_callback: {}".format(e))