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

    'tof_front': None,
    'tof_back': None,
}

# Calculate current_cam_res based on max_cam_res and img_resize_factor
globals['current_cam_res'] = (globals['max_cam_res'][0] // globals['img_resize_factor'], globals['max_cam_res'][1] // globals['img_resize_factor'])