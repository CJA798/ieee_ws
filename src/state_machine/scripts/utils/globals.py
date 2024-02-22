from typing import Dict, Union

globals: Dict[str, Union[bool, int, None]] = {
    'green_detected': False,
    'gravity_vector': 0,
    'bearing': 0,
    'move_done': False,
    'misc_done': False,
    'arm_done': False,
    'img_resize_factor': 4,
    # TODO: Add misc_done when implemented
    'tof_front': None,
    'tof_back': None,
    'test_global': 0
}