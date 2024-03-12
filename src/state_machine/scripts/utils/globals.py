from typing import Dict, Union
from rospy import get_time

globals: Dict[str, Union[bool, int, None]] = {
    'green_detected': False,
    'gravity_vector': 0,
    'bearing': 0,
    # Heartbeat Macros
    'last_heartbeat_time': 0,
    'heartbeat_on':False,
    'move_done': False,
    'misc_done': False,
    'arm_done': False,

    # Vision Macros
    'publish_raw_image': False,
    'img_resize_factor': 2,
    'max_cam_res': (1920, 1080),
    'current_cam_res': None,
    'camera_matrix_path': '/home/pi/ieee_ws/src/vision_system/scripts/image_utils/cameraMatrix.pkl',
    'distortion_coefficients_path': '/home/pi/ieee_ws/src/vision_system/scripts/image_utils/dist.pkl',

    'small_package_Y_arm_offset': -20,
    'fuel_tank_Y_arm_offset': 50,
    'fuel_tank_Z_arm_offset': 75,

    # Arm Macros
    'gripper_bulk_hold': 2000,
    'gripper_bulk_release': 1750,

    # Big Package Macros
    'big_package_Y_offset': -248,

    'set_bulk_top': 1400,
    'set_bulk_bottom': 1050,

    'close_bulk_top': 1150,

    'raise_bulk_offset': 1000,
    
    # 'raise_bulk_top & raise_bulk_bottom declared outside globals dict

    'drop_bulk_top': 1500,
    'drop_bulk_bottom': 1040,
    'big_packages_picked_up': False,
    
    # Fuel Tank Macros
    'fuel_tank_close_top': 1050,
    'fuel_tank_raise_bulk_offset': 2000,


    # Bridge Macros
    'raised_bridge': 2048,
    'mid_bridge': 2400,
    'lowered_bridge': 3250,

    # Flag Macros
    'lowered_flag': 2048,
    'raised_flag': 1024 #was 1800
}

# Macros that depend on other macros
# Leave these outside the dictionary
# Calculate current_cam_res based on max_cam_res and img_resize_factor
globals['current_cam_res'] = (globals['max_cam_res'][0] // globals['img_resize_factor'], globals['max_cam_res'][1] // globals['img_resize_factor'])
globals['raise_bulk_top'] = globals['close_bulk_top'] + globals['raise_bulk_offset']
globals['raise_bulk_bottom'] = globals['set_bulk_bottom'] + globals['raise_bulk_offset']