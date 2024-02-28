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
    # Update the global state of the green LED
    try:
        # Only update the global state if the new state is different from the current state
        if not globals['green_detected'] == data.data:
            globals['green_detected'] = data.data
            rospy.loginfo("LED state updated: {}".format(globals['green_detected']))

    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in start_led_callback: {}".format(e))


def gravity_vector_cb(data):
    '''Callback function to handle gravity vector updates
    
    Args:
        data (std_msgs.Int16): The new gravity vector

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    # Update the global gravity vector
    try:
        globals['gravity_vector'] = data.data
        #rospy.loginfo("Gravity Vector: %s", globals['gravity_vector'])

    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in gravity_vector_cb: {}".format(e))


def bearing_cb(data):
    '''Callback function to handle IMU bearing updates
    
    Args:
        data (std_msgs.Int16): The new bearing value

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    # Update the global bearing value
    try:
        globals['bearing'] = data.data
        #rospy.loginfo("Bearing: %s", globals['bearing'])
    
    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in bearing_cb: {}".format(e))


def move_done_cb(data):
    '''Callback function to handle move done updates
    
    Args:
        data (std_msgs.Int8): The new move done value

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    # Update the global move done value
    try:
        globals['move_done'] = data.data
        rospy.loginfo("Move Done: %s", bool(globals['move_done']))
    
    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in move_done_cb: {}".format(e))


def tof_back_cb(data):
    '''Callback function to handle TOF back updates
    
    Args:
        data (std_msgs.Int16): The new TOF back value

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    
    # Update the global TOF back value
    try:
        globals['tof_back'] = data.data
        #rospy.loginfo("TOF Back: %s", globals['tof_back'])
    
    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in tof_back_cb: {}".format(e))


def state_arm2sm_cb(data):
    '''Callback function to handle arm done updates
    
    Args:
        data (std_msgs.Int8): The new arm done value

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    # Update the global arm done value
    try:
        globals['arm_done'] = data.data
        rospy.loginfo("Arm Done: %s", bool(globals['arm_done']))
    
    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in state_arm2sm_cb: {}".format(e))


def get_coords_fb_cb(feedback):
        rospy.loginfo(f'Current Coordinates List: {feedback.current_coordinates}')

def misc_done_cb(data):
    '''Callback function to handle misc done updates
    
    Args:
        data (std_msgs.Int8): The new misc done value

    Returns:
        None

    Raises:
        Exception: Any exception that occurs during the callback
    '''
    # Update the global misc done value
    try:
        globals['misc_done'] = data.data
        rospy.loginfo("Misc Done: %s", bool(globals['misc_done']))
    
    # Handle any exceptions that occur during the state execution
    except Exception as e:
        rospy.logerr("Error in misc_done_cb: {}".format(e))