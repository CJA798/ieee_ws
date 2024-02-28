from typing import Dict, Union

globals: Dict[str, Union[bool, int, None]] = {
    'green_detected': False,
    'gravity_vector': 0,
    'bearing': 0,
    'move_done': False,
    'misc_done': False,
    'arm_done': False,
    'img_resize_factor': 2,
    'max_cam_res': (1920, 1080),
    'current_cam_res': None,
    
    'camera_matrix_path': '/home/pi/ieee_ws/src/vision_system/scripts/image_utils/cameraMatrix.pkl',
    'distortion_coefficients_path': '/home/pi/ieee_ws/src/vision_system/scripts/image_utils/dist.pkl',

    'small_package_Y_arm_offset': -20,

    # Big Package Macros
    'big_package_Y_offset': -249,

    'set_bulk_top': 1300,
    'set_bulk_bottom': 3000,

    'close_bulk_top': 1100,

    'raise_bulk_offset': 1200,
    # 'raise_bulk_top & raise_bulk_bottom declared outside globals dict

    'big_packages_picked_up': False,
    
    'tof_front': None,
    'tof_back': None,
}

# Macros that depend on other macros
# Leave these outside the dictionary
# Calculate current_cam_res based on max_cam_res and img_resize_factor
globals['current_cam_res'] = (globals['max_cam_res'][0] // globals['img_resize_factor'], globals['max_cam_res'][1] // globals['img_resize_factor'])
globals['raise_bulk_top'] = globals['close_bulk_top'] + globals['raise_bulk_offset']
globals['raise_bulk_bottom'] = globals['set_bulk_top'] - globals['raise_bulk_offset']