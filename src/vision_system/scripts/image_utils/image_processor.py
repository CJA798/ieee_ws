#!/usr/bin/env python

from typing import List, Iterable, Tuple, Sized, Union
from geometry_msgs.msg import Point
import cv2
import numpy as np
#from board_objects import BoardObjects 
from utils.globals import globals
from image_utils.board_objects import BoardObjects
from image_utils.poses import Poses
from imutils import grab_contours
from wand.image import Image, Color
from rospy import loginfo, logwarn
from pickle import load


#nidiaes*min_area=514593
#ax_area¡3847
#carlosNOsebaña:True
'''
class Point():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.z = 0
'''

class ImageProcessor_():
    # Dictionary mapping object types to their respective coordinate finder methods
    OBJECT_METHODS = {
        BoardObjects.SMALL_PACKAGE: "find_small_package_coords",
        BoardObjects.FUEL_TANK: "find_fuel_tank_coords",
    }

    POSE_AREA_RANGES = {
        Poses.SMALL_PACKAGE_SCAN: (0.0005, 0.0075)
    }

    def __init__(self) -> None:
        self.image = None
        self.undistorted_image = None

        # TODO: Make the resolution an d everything related to it more flexible
        # Hardcoded values for the camera resolution
        self.image_width = globals['max_cam_res'][0]//globals['img_resize_factor']
        #print("image_width: ", self.image_width)
        self.image_height = globals['max_cam_res'][1]//globals['img_resize_factor']
        #print("image_height: ", self.image_height)

        # Open the camera calibration results using pickle
        self.camera_matrix = load(open(globals['camera_matrix_path'], "rb" ))
        self.dist = load(open(globals['distortion_coefficients_path'], "rb" ))
        loginfo("Camera calibration results loaded")

        # Calculate the color bounds
        self.color_data = {
        "orange": self.get_color_bounds([0, 85, 255], hue_offset=15, value=(100, 255)),
        "copper": self.get_color_bounds([75, 112, 255]),
        "yellow": self.get_color_bounds([0, 255, 255]),
        "magenta": self.get_color_bounds([255, 0, 255], value=(100, 255), saturation=(100, 255)),
        "black": (np.array([0, 0, 0], dtype=np.uint8), np.array([180, 255, 100], dtype=np.uint8)),
        }
        loginfo("Color data calculated")

    def get_pose_from_string(self, pose_str):
        for pose in Poses:
            if pose.value == pose_str:
                return pose
        return None

    def get_coords(self, object_type, pose, *args, **kwargs) -> Tuple[List[Point], np.ndarray]:
        try:
            coordinates = []
            coords_image = []

            # Check if the object type is valid
            if object_type not in BoardObjects:
                raise ValueError(f"Object type {object_type} not recognized.")

            # Get the method to find the coordinates
            method_name = self.OBJECT_METHODS[object_type]
            coordinate_finder = getattr(self, method_name)
            
            # Log the execution of the state
            loginfo(f"Executing {method_name} to find {object_type} coordinates in pose {pose}")

            # Call the method to find the coordinates, passing any additional arguments
            coordinates, coords_image = coordinate_finder(pose, *args, **kwargs)
            return (coordinates, coords_image)
        
        # Handle any exceptions that may occur
        except Exception as e:
            raise ValueError(f"ERROR: {e}")
    
    def get_color_bounds(self, color_code: Iterable[Sized],
                    hue_threshold: Iterable[Sized]=(15,165),
                    hue_offset: int=30,
                    hue: Iterable[Sized]=(0,180),
                    saturation: Iterable[Sized]=(0,255),
                    value: Iterable[Sized]=(0,255)) -> Tuple[np.array, np.array]:
        '''
        Get the lower and upper bounds for the color in the image.
        Arguments:
            color_code: Iterable[Sized] - The BGR color code for the color.
            hue_threshold: Iterable[Sized] - .
            hue_offset: int - The offset for the hue value.
            hue: Iterable[Sized] - Hue range (0-179) = (red -> yellow -> green -> cyan -> blue -> magenta -> pink -> red).
            saturation: Iterable[Sized] - Saturation range (0-255) = (white to color).
            value: Iterable[Sized] - Value range (0-255) = (black to color).
        Returns:
            lower_limit: np.array - The lower limit for the color in the image.
            upper_limit: np.array - The upper limit for the color in the image.
        '''
        c = np.uint8([[color_code]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue_ = hsvC[0][0][0]  # Get the hue value

        upper_hue_threshold = hue_threshold[1]
        lower_hue_threshold = hue_threshold[0]
        min_hue = hue[0]
        max_hue = hue[1]
        min_saturation = saturation[0]
        max_saturation = saturation[1]
        min_value = value[0]
        max_value = value[1]

        # Handle red hue wrap-around
        if hue_ >= upper_hue_threshold:  # Upper limit for divided red hue
            lowerLimit = np.array([hue_ - hue_offset, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([max_hue, max_saturation, max_value], dtype=np.uint8)
        elif hue_ <= lower_hue_threshold:  # Lower limit for divided red hue
            lowerLimit = np.array([min_hue, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue_ + hue_offset, max_saturation, max_value], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue_ - hue_offset//2, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue_ + hue_offset//2, max_saturation, max_value], dtype=np.uint8)

        return (lowerLimit, upperLimit)

    def get_area_range_factor(self, pose: Poses) -> Tuple[float, float]:
        # Get the area range factor for the pose
        return self.POSE_AREA_RANGES[pose]
    
    def find_small_package_coords(self, pose: str) -> Tuple[List[Point], np.ndarray]:
        image = self.image

        # Find the key for the pose value
        pose_ = self.get_pose_from_string(pose)

        # Check if the current image is not None
        if image is None:
            logwarn("find_small_package_coords - Image is None")
            return (None, None)
        
        # Remove distortion from the image
        undistorted_image = self.remove_distortion(image)

        # TODO: resize image if latency is too bad

        # Apply blurs to remove noise
        blur = cv2.GaussianBlur(undistorted_image, (3, 3), 0)
        median = cv2.medianBlur(blur, 3)

        # Convert to HSV
        hsvImage = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # Create binary mask for magenta 
        lower_limit, upper_limit = self.color_data["magenta"]
        magenta_mask = cv2.inRange(hsvImage, lower_limit, upper_limit)
        magenta_mask = cv2.cvtColor(magenta_mask, cv2.COLOR_GRAY2BGR)

        # Convert to grayscale
        gray = cv2.cvtColor(magenta_mask, cv2.COLOR_BGR2GRAY)
        
        # TODO: Apply morphological operations if necessary
        
        area_range_factor = self.get_area_range_factor(pose_)
        #print(f"Area Range Factor: {area_range_factor}")

        coordinates, coords_image = self.find_contours(gray, pose_, area_range_factor=area_range_factor)
        #coords_image = np.hstack([image, blur, median, darkened_frame, self.image])
        
        return (coordinates, coords_image)
    
    def remove_distortion(self, image=None) -> np.ndarray:
        # Check if the current image is not None
        if image is None:
            logwarn("undistort - Image is None")
            return None
    
        h,  w = image.shape[:2]
        newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist, (w,h), 1, (w,h))
        # Undistort
        reprojection = cv2.undistort(image, self.camera_matrix, self.dist, None, newCameraMatrix)
        self.undistorted_image = reprojection

        return reprojection

    def find_contours(self, image: np.ndarray, pose: Poses, area_range_factor: Iterable[Sized] = (0.01,1)) -> Tuple[List[Point], np.ndarray]:
        '''
        Find the contours in the image.
        Arguments:
            image: np.ndarray - The image to find the contours in.
            pose: str - The pose of the camera when the image was taken.
            area_range_factor: Iterable[Sized] - The factor to multiply the image area by to get the area range.
        Returns:
            contour_masks: List[Point] - The coordinates of the contours in the image.
            image: np.ndarray - The image with the contours drawn on it.
        '''
        # Check if image is None
        if image is None:
            logwarn("find_contours - Image is None")
            return (None, None)
        
        coords_list = []
  
        contours = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = grab_contours(contours)

        image_area = image.size
        max_area = image_area * area_range_factor[1]
        min_area = image_area * area_range_factor[0]
        coords_image = self.undistorted_image
        if len(contours) > 0:
            for contour in contours:
                M = cv2.moments(contour)
                area = M["m00"]
                #print(f"Min Area {min_area}    |   Area {area}")
                if area < min_area or area > max_area or area == 0:
                    continue
                cX = int(M["m10"] / area)
                cY = int(M["m01"] / area)

                cv2.drawContours(coords_image, [cv2.convexHull(contour)], -1, (0, 255, 0), 2)
                cv2.circle(coords_image, (cX, cY), 2, (0, 255, 0), -1)
                x_arm, y_arm, z_arm = self.coordinate_frame_conversion(cX, cY, 0, pose)
                coords = Point()
                coords.x = x_arm    # Z-img = X-arm
                coords.z = z_arm    # X-img = Z-arm
                coords.y = y_arm
                coords_list.append(coords)
                coords_text = "(%.1f, %.1f)"  % (z_arm, x_arm) 
                cv2.putText(coords_image, coords_text, (cX - 15, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)    
                #coords_image = np.hstack([coords_image, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)])
        return (coords_list, coords_image)
    
    def coordinate_frame_conversion(self, x: Union[int, float], y: Union[int, float], z: Union[int, float], pose: Poses) -> Tuple[float, float, float]:
        '''
        Convert the coordinates from the image frame to the arm frame.
        Arguments:
            x: Union[int, float] - The x-coordinate in the image frame.
            y: Union[int, float] - The y-coordinate in the image frame.
            z: Union[int, float] - The z-coordinate in the image frame.
            pose: str - The pose of the camera when the image was taken.
        Returns:
            x: float - The x-coordinate in the arm frame.
            y: float - The y-coordinate in the arm frame.
            z: float - The z-coordinate in the arm frame.
        '''
        if pose == Poses.SMALL_PACKAGE_SCAN:
            #print("image_height: ", self.image_height)
            #print("image_width: ", self.image_width)
            KX = 57/47
            PX2MM_Y = 410/self.image_height
            PX2MM_X = 580/self.image_width * KX

            # Offset from bottom of image to arm base
            A = 45

            Xarm_xi_obj = (self.image_height - y) * PX2MM_Y + A
            Yarm_xi_obj = globals['small_package_Y_arm_offset']
            Zarm_xi_obj = (x - self.image_width/2) * PX2MM_X
            #print(f"Y: {y}  |   Max_Cam_Height: {self.image_height}     |   Conversion Factior: {PX2MM_Y}")

            Xarm_offset = 50
            Zarm_offset = 51

            return Xarm_xi_obj + Xarm_offset, Yarm_xi_obj, Zarm_xi_obj + Zarm_offset
        
            #Xarm = y
            #Yarm = globals['small_package_Y_arm_offset']
            #Zarm = x
            #return Xarm, Yarm, Zarm
        
        if pose == Poses.SCAN:
            #print("image_height: ", self.image_height)
            #print("image_width: ", self.image_width)
            KX = 57/47
            PX2MM_Y = 410/self.image_height
            PX2MM_X = 580/self.image_width * KX

            # Offset from bottom of image to arm base
            A = 45

            Xarm_xi_obj = (self.image_height - y) * PX2MM_Y + A
            Yarm_xi_obj = -50
            Zarm_xi_obj = (x - self.image_width/2) * PX2MM_X
            #print(f"Y: {y}  |   Max_Cam_Height: {self.image_height}     |   Conversion Factior: {PX2MM_Y}")

            Xarm_offset = 50
            Zarm_offset = 51

            return Xarm_xi_obj + Xarm_offset, Yarm_xi_obj, Zarm_xi_obj + Zarm_offset
        elif pose == Poses.VERIFY.value:
            # TODO: find and add the conversion factor for the verify pose
            return x*(600/1080*2.5), y*(600/1080*2.5) - self.image_width/2, z
        elif pose == Poses.FRONT.value:
            x_img = x*70/160
            y_img = -20
            z_img = 260 #TODO: make this variable depend on the TOF_Front reading
            return x_img, y_img, z_img
        else:
            raise ValueError(f"Arm pose {pose} not recognized.")
        

class ImageProcessor():
    
    def __init__(self) -> None:
        '''
        Initialize the ImageProcessor class.
        Arguments:
            None
        Returns:
            None
        '''
        #TODO: fix the initial image. It should be None, then initialized in the calling function
        self.image = np.zeros((*globals['current_cam_res'] , 3), dtype=np.uint8)
        self.color_data = {
        "orange": self.get_color_bounds([0, 85, 255], hue_offset=15, value=(100, 255)),
        "copper": self.get_color_bounds([75, 112, 255]),
        "yellow": self.get_color_bounds([0, 255, 255]),
        "magenta": self.get_color_bounds([255, 0, 255], value=(100, 255), saturation=(100, 255)),
        "black": (np.array([0, 0, 0], dtype=np.uint8), np.array([180, 255, 100], dtype=np.uint8)),
        }

        self.image_width = globals['max_cam_res'][0]//globals['img_resize_factor']
        #print("image_width: ", self.image_width)
        self.image_height = globals['max_cam_res'][1]//globals['img_resize_factor']
        #print("image_height: ", self.image_height)

        self.x_conversion_factor = 203.2
        self.z_conversion_factor = 177.8

        self.PX2MMZ = self.z_conversion_factor / self.image_width
        self.PX2MMX = self.x_conversion_factor / self.image_height
        

    def get_coords(self, object_type: str, pose: str, experimental: bool=False) -> Tuple[List[Point], np.ndarray]:
        '''
        Get the coordinates of the object in the image.
        Arguments: 
            object_type: str - The type of object to find in the image.
            pose: str - The pose of the arm when the image was taken.
        Returns:
            coordinates: List[Point] - The coordinates of the objects in the image.
            coords_image: np.ndarray - The image with the coordinates drawn on it.
        Raises:
            ValueError: If the object type is not recognized.
        '''
        coordinates = []
        coords_image = self.image.copy()
        image = self.image.copy()
        #print(f"Finding coordinates for {object_type} in pose {pose}")
        try:
            if object_type == BoardObjects.SMALL_PACKAGE.value:
                #print("Finding small package coordinates")
                if experimental:
                    coordinates, coords_image = self.experimental_find_small_package_coords(image, pose)
                else:
                    coordinates, coords_image = self.find_small_package_coords(image, pose)
            elif object_type == BoardObjects.THRUSTER.value:
                #print("Finding thruster coordinates")
                coordinates, coords_image = self.find_thruster_or_fuel_tank_coords(image, pose)
            elif object_type == BoardObjects.FUEL_TANK.value:
                # Remove noise from the image
                median = cv2.medianBlur(image, 5)
                # Convert to HSV
                hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
                # Create orange mask
                lower_limit, upper_limit = self.color_data["orange"]
                color_mask = cv2.inRange(hsv, lower_limit, upper_limit)
                color_mask = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)
                # Convert to grayscale
                gray = cv2.cvtColor(color_mask, cv2.COLOR_BGR2GRAY)
                # Apply morphological operations to remove noise
                #opening = cv2.morphologyEx(gray, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
                closing = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
                # Find the contours in the image
                coordinates, coords_image = self.find_contours(closing, pose, area_range_factor=(0.1, 0.4))
                output_img = np.hstack([median, color_mask, cv2.cvtColor(closing, cv2.COLOR_GRAY2BGR), coords_image])
                #cv2.imshow('Fuel Tank', output_img)

        except ValueError as e:
            raise ValueError(f"Object type {object_type} not recognized.")
        
        return (coordinates, coords_image)

    def get_color_bounds(self, color_code: Iterable[Sized],
                        hue_threshold: Iterable[Sized]=(15,165),
                        hue_offset: int=30,
                        hue: Iterable[Sized]=(0,180),
                        saturation: Iterable[Sized]=(0,255),
                        value: Iterable[Sized]=(0,255)) -> Tuple[np.array, np.array]:
        '''
        Get the lower and upper bounds for the color in the image.
        Arguments:
            color_code: Iterable[Sized] - The BGR color code for the color.
            hue_threshold: Iterable[Sized] - .
            hue_offset: int - The offset for the hue value.
            hue: Iterable[Sized] - Hue range (0-179) = (red -> yellow -> green -> cyan -> blue -> magenta -> pink -> red).
            saturation: Iterable[Sized] - Saturation range (0-255) = (white to color).
            value: Iterable[Sized] - Value range (0-255) = (black to color).
        Returns:
            lower_limit: np.array - The lower limit for the color in the image.
            upper_limit: np.array - The upper limit for the color in the image.
        '''
        c = np.uint8([[color_code]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue_ = hsvC[0][0][0]  # Get the hue value

        upper_hue_threshold = hue_threshold[1]
        lower_hue_threshold = hue_threshold[0]
        min_hue = hue[0]
        max_hue = hue[1]
        min_saturation = saturation[0]
        max_saturation = saturation[1]
        min_value = value[0]
        max_value = value[1]

        # Handle red hue wrap-around
        if hue_ >= upper_hue_threshold:  # Upper limit for divided red hue
            lowerLimit = np.array([hue_ - hue_offset, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([max_hue, max_saturation, max_value], dtype=np.uint8)
        elif hue_ <= lower_hue_threshold:  # Lower limit for divided red hue
            lowerLimit = np.array([min_hue, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue_ + hue_offset, max_saturation, max_value], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue_ - hue_offset//2, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue_ + hue_offset//2, max_saturation, max_value], dtype=np.uint8)

        return (lowerLimit, upperLimit)

    def apply_pincushion_distortion(self, image, coefficients: Tuple[int, int, int, int]):
        try:
            with Image.from_array(image) as img:
                img.background_color = Color('black')
                img.virtual_pixel = 'background'
                img.distort('barrel', coefficients)
            
                bgr_array = np.array(img)

                return bgr_array
        except Exception as e:
            print(f"Error applying pincushion distortion: {e}")
            return None


    def find_small_package_coords(self, image: np.ndarray, pose: str) -> Tuple[List[Point], np.ndarray]:
        '''
        Find the coordinates of the small packages in the image.
        Arguments:
            image: np.ndarray - The image to find the small packages in.
            pose: str - The pose of the arm when the image was taken.
        Returns:
            coordinates: List[Point] - The coordinates of the small packages in the image.
            coords_image: np.ndarray - The image with the coordinates drawn on it.
        '''

        # Apply blurs to remove noise
        blur = cv2.GaussianBlur(image, (3, 3), 0)
        median = cv2.medianBlur(blur, 3)
        #median = blur

        # Convert to HSV
        hsvImage = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # Create yellow mask
        lower_limit, upper_limit = self.color_data["yellow"]
        yellow_mask = cv2.inRange(hsvImage, lower_limit, upper_limit)

        inverted_mask = cv2.bitwise_not(yellow_mask)
        darkened_frame = cv2.bitwise_and(image, image, mask=inverted_mask)

        # Create black mask
        black_mask = cv2.inRange(darkened_frame, self.color_data["black"][0], self.color_data["black"][1])
        black_inverted_mask = cv2.bitwise_not(black_mask)
        darkened_frame = cv2.bitwise_and(darkened_frame, darkened_frame, mask=black_inverted_mask)
        gray = cv2.cvtColor(darkened_frame, cv2.COLOR_BGR2GRAY)
        coordinates, coords_image = self.find_contours(gray, pose, area_range_factor=(0.0025, 0.025))
        #coords_image = np.hstack([image, blur, median, darkened_frame, self.image])
        return (coordinates, coords_image)
    
    def experimental_find_small_package_coords(self, image: np.ndarray, pose: str) -> Tuple[List[Point], np.ndarray]:
        # Apply pincushion distortion
        #pincushion = self.apply_pincushion_distortion(image, (0.0, 0.0, -1.2, 2.1))

        #if pincushion is None:
        #    print("Pincushion is None")
        #    pincushion = image

        # Apply blurs to remove noise
        blur = cv2.GaussianBlur(image, (3, 3), 0)
        median = cv2.medianBlur(blur, 3)
        
        # Convert to HSV
        hsvImage = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # Create yellow mask
        lower_limit, upper_limit = self.color_data["magenta"]
        magenta_mask = cv2.inRange(hsvImage, lower_limit, upper_limit)
        magenta_mask = cv2.cvtColor(magenta_mask, cv2.COLOR_GRAY2BGR)

        # Convert to grayscale
        gray = cv2.cvtColor(magenta_mask, cv2.COLOR_BGR2GRAY)
        coordinates, coords_image = self.find_contours(gray, pose, area_range_factor=(0.0025, 0.025))
        #coords_image = np.hstack([image, blur, median, darkened_frame, self.image])
        return (coordinates, coords_image)
    
    def find_thruster_or_fuel_tank_coords(self, image: np.ndarray, pose: str, front_offset: int, right_offset: int) -> Tuple[List[Point], np.ndarray]:
        '''
        Find the coordinates of the thrusters or fuel tanks in the image.
        Arguments:
            image: np.ndarray - The image to find the thrusters or fuel tanks in.
            pose: str - The pose of the arm when the image was taken.
            front_offset: int - The offset for the front side of the robot.
            right_offset: int - The offset for the right side of the robot.
        Returns:
            coordinates: List[Point] - The coordinates of the thrusters or fuel tanks in the image.
            coords_image: np.ndarray - The image with the coordinates drawn on it.
        '''
        median = cv2.medianBlur(image, 5)
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        lower_limit, upper_limit = self.color_data["orange"]
        
        color_mask = cv2.inRange(hsv, lower_limit, upper_limit)
        color_mask = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)

        gray = cv2.cvtColor(color_mask, cv2.COLOR_BGR2GRAY)

        coordinates, coords_image = self.find_contours(gray, pose)

        return (coordinates, coords_image)
    
    def find_contours(self, image: np.ndarray, pose: str, area_range_factor: Iterable[Sized] = (0.01,1)) -> Tuple[List[Point], np.ndarray]:
        '''
        Find the contours in the image.
        Arguments:
            image: np.ndarray - The image to find the contours in.
            pose: str - The pose of the camera when the image was taken.
            area_range_factor: Iterable[Sized] - The factor to multiply the image area by to get the area range.
        Returns:
            contour_masks: List[Point] - The coordinates of the contours in the image.
            image: np.ndarray - The image with the contours drawn on it.
        '''
        coords_list = []
  
        contours = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = grab_contours(contours)

        image_area = image.size
        max_area = image_area * area_range_factor[1]
        min_area = image_area * area_range_factor[0]
        output_img = self.image
        if len(contours) > 0:
            for contour in contours:
                M = cv2.moments(contour)
                area = M["m00"]
                #print(f"Min Area {min_area}    |   Area {area}")
                if area < min_area or area > max_area or area == 0:
                    continue
                cX = int(M["m10"] / area)
                cY = int(M["m01"] / area)

                cv2.drawContours(self.image, [cv2.convexHull(contour)], -1, (0, 255, 0), 2)
                cv2.circle(self.image, (cX, cY), 2, (0, 255, 0), -1)
                x_arm, y_arm, z_arm = self.coordinate_frame_conversion(cX, cY, 0, pose)
                coords = Point()
                coords.x = x_arm    # Z-img = X-arm
                coords.z = z_arm    # X-img = Z-arm
                coords.y = y_arm
                coords_list.append(coords)
                coords_text = "(%.1f, %.1f)"  % (z_arm, x_arm) 
                cv2.putText(self.image, coords_text, (cX - 15, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)    
                output_img = np.hstack([self.image, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)])
        return (coords_list, output_img)
    
    def coordinate_frame_conversion(self, x: Union[int, float], y: Union[int, float], z: Union[int, float], pose: str) -> Tuple[float, float, float]:
        '''
        Convert the coordinates from the image frame to the arm frame.
        Arguments:
            x: Union[int, float] - The x-coordinate in the image frame.
            y: Union[int, float] - The y-coordinate in the image frame.
            z: Union[int, float] - The z-coordinate in the image frame.
            pose: str - The pose of the camera when the image was taken.
        Returns:
            x: float - The x-coordinate in the arm frame.
            y: float - The y-coordinate in the arm frame.
            z: float - The z-coordinate in the arm frame.
        '''
        if pose == Poses.SCAN.value:
            #print("image_height: ", self.image_height)
            #print("image_width: ", self.image_width)
            KX = 57/47
            PX2MM_Y = 410/self.image_height
            PX2MM_X = 580/self.image_width * KX

            # Offset from bottom of image to arm base
            A = 45

            Xarm_xi_obj = (self.image_height - y) * PX2MM_Y + A
            Yarm_xi_obj = -50
            Zarm_xi_obj = (x - self.image_width/2) * PX2MM_X
            #print(f"Y: {y}  |   Max_Cam_Height: {self.image_height}     |   Conversion Factior: {PX2MM_Y}")

            Xarm_offset = 50
            Zarm_offset = 51

            return Xarm_xi_obj + Xarm_offset, Yarm_xi_obj, Zarm_xi_obj + Zarm_offset
        elif pose == Poses.VERIFY.value:
            # TODO: find and add the conversion factor for the verify pose
            return x*(600/1080*2.5), y*(600/1080*2.5) - self.image_width/2, z
        elif pose == Poses.FRONT.value:
            x_img = x*70/160
            y_img = -20
            z_img = 260 #TODO: make this variable depend on the TOF_Front reading
            return x_img, y_img, z_img
        else:
            raise ValueError(f"Arm pose {pose} not recognized.")
        

def main():
    device_path = "/dev/v4l/by-id/usb-Arducam_Arducam_5MP_Camera_Module_YL20230518V0-video-index0"
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    #cap.set(cv2.CAP_PROP_SETTINGS, 1)
    img_width = globals['current_cam_res'][0]
    img_height = globals['current_cam_res'][1]    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)
    
    # sET CONTRAST
    cap.set(cv2.CAP_PROP_CONTRAST, 3)
    # SET SATURATION
    cap.set(cv2.CAP_PROP_SATURATION, 150)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # Function to calculate the median blur kernel size
    median_kernel = lambda kernel: kernel if kernel % 2 == 1 else kernel + 1 
    ip = ImageProcessor()
    while cap.isOpened():
        ret, ip.image = cap.read()

        if not ret:
            print("Can't receive frame")
            break

        coords_list, coords_image = ip.get_coords(BoardObjects.SMALL_PACKAGE.value, Poses.SCAN.value, experimental=True)
        
        

        if cv2.waitKey(5) == 27:
            break
        
        '''
        frame = ip.image.copy()
        median_frame = cv2.medianBlur(frame, median_kernel(img_resize_factor))
        hsv_frame = cv2.cvtColor(median_frame, cv2.COLOR_BGR2HSV)
        
        color_mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
        color_mask = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)

        opening = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        median = cv2.medianBlur(closing, 5)

        # Stack images horizontally
        output = np.hstack([frame, median_frame, color_mask, opening, closing, median])
        cv2.imshow('K-Means', output)
        '''

        cv2.imshow('Coordinates', coords_image)
        print(coords_list)
    cap.release()
    cv2.destroyAllWindows()


def main_():
    device_path = "/dev/v4l/by-id/usb-Arducam_Arducam_5MP_Camera_Module_YL20230518V0-video-index0"
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)

    #cap.set(cv2.CAP_PROP_SETTINGS, 1)

    img_width = globals['current_cam_res'][0]
    img_height = globals['current_cam_res'][1]    

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, img_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, img_height)
    
    # Set Contrast
    cap.set(cv2.CAP_PROP_CONTRAST, 3)
    # Set Saturation
    cap.set(cv2.CAP_PROP_SATURATION, 150)

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    ip = ImageProcessor_()
    while cap.isOpened():
        ret, ip.image = cap.read()

        if not ret:
            print("Can't receive frame")
            break

        coords_list, coords_image = ip.get_coords(object_type=BoardObjects.SMALL_PACKAGE, pose=Poses.SMALL_PACKAGE_SCAN.value)        
        
        if coords_image is None:
            logwarn("Coords Image is None")
            continue

        if cv2.waitKey(5) == 27:
            break

        cv2.imshow('Coordinates', coords_image)
        print(coords_list)

    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main_()