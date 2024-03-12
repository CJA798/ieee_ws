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
        Poses.SMALL_PACKAGE_SCAN: (0.0005, 0.0075),
        Poses.FUEL_TANK_SCAN: (0.005, 0.075)
    }

    def __init__(self) -> None:
        self.image = None
        self.undistorted_image = None
        self.undistorted_image_roi = None

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
        "orange": self.get_color_bounds([0, 85, 255], hue_offset=30, value=(100, 255), saturation=(200, 255)),
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

    def get_object_type_from_string(self, object_type_str):
        for obj in BoardObjects:
            if obj.value == object_type_str:
                return obj
        return None
    
    def get_coords(self, object_type, pose, *args, **kwargs) -> Tuple[List[Point], np.ndarray]:
        try:
            coordinates = []
            coords_image = []

            object_type_ = self.get_object_type_from_string(object_type)
            # Check if the object type is valid
            if object_type_ not in BoardObjects:
                raise ValueError(f"Object type {object_type_} not recognized.")

            # Get the method to find the coordinates
            method_name = self.OBJECT_METHODS[object_type_]
            coordinate_finder = getattr(self, method_name)
            
            # Log the execution of the state
            loginfo(f"Executing {method_name} to find {object_type_} coordinates in pose {pose}")

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
        undistorted_image, undistorted_roi = self.remove_distortion(image)
        if undistorted_roi is None:
            logwarn("find_small_package_coords - undistorted_roi is None")
            return (None, None)

        # TODO: resize image if latency is too bad

        # Apply blurs to remove noise
        blur = cv2.GaussianBlur(undistorted_roi, (3, 3), 0)
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
        
        sorted_coordinates_list = sorted(coordinates, key=lambda coord: coord.x, reverse=True)
        return (sorted_coordinates_list, coords_image)
    
    def find_fuel_tank_coords(self, pose: str) -> Tuple[List[Point], np.ndarray]:
        image = self.image

        # Find the key for the pose value
        pose_ = self.get_pose_from_string(pose)

        # Check if the current image is not None
        if image is None:
            logwarn("find_small_package_coords - Image is None")
            return (None, None)
        
        # Remove distortion from the image
        undistorted_image, undistorted_roi = self.remove_distortion(image)

        # TODO: resize image if latency is too bad

        # Get fuel tank roi
        fuel_tank_roi = self.get_fuel_tank_roi(undistorted_roi)

        # Apply blurs to remove noise
        blur = cv2.GaussianBlur(fuel_tank_roi, (3, 3), 0)
        median = cv2.medianBlur(blur, 3)

        # Convert to HSV
        hsvImage = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # Create binary mask for magenta 
        lower_limit, upper_limit = self.color_data["orange"]
        orange_mask = cv2.inRange(hsvImage, lower_limit, upper_limit)
        orange_mask = cv2.cvtColor(orange_mask, cv2.COLOR_GRAY2BGR)

        # Convert to grayscale
        gray = cv2.cvtColor(orange_mask, cv2.COLOR_BGR2GRAY)
        
        # TODO: Apply morphological operations if necessary
        
        area_range_factor = self.get_area_range_factor(pose_)
        #print(f"Area Range Factor: {area_range_factor}")

        coordinates, coords_image = self.find_contours(gray, pose_, area_range_factor=area_range_factor)
        sorted_coordinates_list = sorted(coordinates, key=lambda coord: coord.x)

        coords_image = np.hstack([coords_image, orange_mask])
        
        return (sorted_coordinates_list, coords_image)
    
    def remove_distortion(self, image=None) -> Tuple[np.ndarray, np.ndarray]:
        # Check if the current image is not None
        if image is None or len(image.shape[:2]) != 2:
            logwarn("undistort - Image is None OR bad image shape.")
            return (None, None)

        h, w = image.shape[:2]
        newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist, (w,h), 1, (w,h))
        # Undistort
        reprojection = cv2.undistort(image, self.camera_matrix, self.dist, None, newCameraMatrix)
        self.undistorted_image = reprojection

        x, y, w, h = roi
        reprojection_roi = reprojection[y:y+h, x:x+w]
        self.undistorted_image_roi = reprojection_roi
        self.image_height, self.image_width = reprojection_roi.shape[:2]
        #print(f"Image Height: {self.image_height}    |   Image Width: {self.image_width}")
        #print(reprojection_roi.shape)
        #cv2.imshow('Undistorted Image', reprojection_roi)
        #cv2.waitKey(200)
        return reprojection, reprojection_roi

    def get_fuel_tank_roi(self, image: np.ndarray) -> np.ndarray:
        # Get image dimensions
        height, _, _ = image.shape

        # Define the region to keep
        y_start = int(height * 2 / 5)
        y_end = int(height * 3 / 5)

        # Create a mask with the same dimensions as the image
        mask = np.zeros_like(image)

        # Set the region to keep to white (255)
        mask[y_start:y_end, :] = 255

        # Apply the mask to the image
        return cv2.bitwise_and(image, mask)

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
        coords_image = self.undistorted_image_roi
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
            KX = 59/60
            PX2MM_Y = 470/self.image_height
            PX2MM_X = 610/self.image_width * KX

            # Offset from bottom of image to arm base
            A = -60
            KY = 384/360

            # Convert px to mm
            Xarm_xi_obj = ((self.image_height - y) * PX2MM_Y + A) * KY
            Yarm_xi_obj = globals['small_package_Y_arm_offset']
            Zarm_xi_obj = (x - self.image_width/2) * PX2MM_X * KX
            #print(f"Y: {y}  |   Max_Cam_Height: {self.image_height}     |   Conversion Factior: {PX2MM_Y}")

            # Correction offsets
            Xarm_offset = self.map_offset(y, (50,350), (10,50))
            Zarm_offset = 0

            return Xarm_xi_obj + Xarm_offset, Yarm_xi_obj, Zarm_xi_obj + Zarm_offset
        
        elif pose == Poses.FUEL_TANK_SCAN:
            KX = -1
            PX2MM_Y = 180/self.image_height
            PX2MM_X = 240/self.image_width * KX

            # Offset from bottom of image to arm base
            A = 0
            KY = 1

            # Convert px to mm
            Zarm_xi_obj = globals['fuel_tank_Z_arm_offset']
            Yarm_xi_obj = globals['fuel_tank_Y_arm_offset']
            Xarm_xi_obj = (x - self.image_width/2) * PX2MM_X
            #print(f"Y: {y}  |   Max_Cam_Height: {self.image_height}     |   Conversion Factior: {PX2MM_Y}")

            # Correction offsets
            Xarm_offset = -20
            Zarm_offset = -10

            # Mapped offsets using linear regression
            #Xarm_mapped_offset = -0.2674 * Xarm_xi_obj + 0.3101
            # Mapped offsets using polynomial regression
            Xarm_mapped_offset = 0.0004**2 * Xarm_xi_obj - 0.25 * Xarm_xi_obj * 0.6154

            # Total mm
            Xarm_xi_obj = Xarm_xi_obj + Xarm_offset + Xarm_mapped_offset
            return Xarm_xi_obj, Yarm_xi_obj, Zarm_xi_obj

        else:
            raise ValueError(f"Arm pose {pose} not recognized.")
        
    def map_offset(self, value: float, map_from: Tuple[float, float], map_to: Tuple[float, float]) -> float:
        '''
        Map the value from one range to another.
        Arguments:
            map_from: Iterable[Sized, Sized] - The range to map the value from.
            map_to: Iterable[Sized, Sized] - The range to map the value to.
        Returns:
            mapped_value: Float - The value mapped to the new range.
        '''
        x1 = map_from[0]
        x2 = map_from[1]
        y1 = map_to[0]
        y2 = map_to[1]

        m = (y2 - y1) / (x2 - x1)
        x = value
        b = y1 - m * x1

        if 5 > value > 35:
            return 0
        
        elif 10 <= value < 15:
            b = b - 2.5
        else:
            b = b - 5
            
        mapped_offset = -(m * x + b)
        print(f"Value: {value}    |   Mapped Offset: {mapped_offset}")
        return   mapped_offset
        

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

        coords_list, coords_image = ip.get_coords(object_type=BoardObjects.FUEL_TANK.value, pose=Poses.FUEL_TANK_SCAN.value)        
        
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