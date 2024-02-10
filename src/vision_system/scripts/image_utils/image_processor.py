#!/usr/bin/env python

from time import time
from typing import List, Iterable, Sized, Union
from geometry_msgs.msg import Point
import cv2
import numpy as np
#from board_objects import BoardObjects 
from image_utils.board_objects import BoardObjects
from imutils import grab_contours


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

class ImageProcessor():
    
    def __init__(self) -> None:
        '''
        Initialize the ImageProcessor class.
        Arguments:
            None
        Returns:
            None
        '''
        self.image = np.zeros((160, 120, 3), dtype=np.uint8)
        self.color_data = {
        "orange": self.get_color_bounds([75, 112, 255]),
        "copper": self.get_color_bounds([75, 112, 255]),
        "yellow": self.get_color_bounds([0, 255, 255]),
        "black": (np.array([0, 0, 0], dtype=np.uint8), np.array([180, 255, 100], dtype=np.uint8)),
        }

        self.image_width, self.image_height, _ = self.image.shape

        self.x_conversion_factor = 203.2
        self.z_conversion_factor = 177.8

        self.PX2MMZ = self.z_conversion_factor / self.image_width
        self.PX2MMX = self.x_conversion_factor / self.image_height
        

    def get_coords(self, object_type: str, pose: str) -> (List[Point], np.ndarray):
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
        #print(f"Finding coordinates for {object_type} in pose {pose}")
        try:
            if object_type == BoardObjects.SMALL_PACKAGE.name:
                #print("Finding small package coordinates")
                coordinates, coords_image = self.find_small_package_coords(coords_image, pose)
            elif object_type == BoardObjects.THRUSTER or object_type == BoardObjects.FUEL_TANK:
                #print("Finding thruster or fuel tank coordinates")
                coordinates, coords_image = self.find_thruster_or_fuel_tank_coords(coords_image, pose)
        except ValueError as e:
            raise ValueError(f"Object type {object_type} not recognized.")
        
        return (coordinates, coords_image)

    def get_color_bounds(self, color_code: Iterable[Sized]) -> (np.array, np.array):
        '''
        Get the lower and upper bounds for the color in the image.
        Arguments:
            color_code: Iterable[Sized] - The BGR color code for the color.
        Returns:
            lower_limit: np.array - The lower limit for the color in the image.
            upper_limit: np.array - The upper limit for the color in the image.
        '''
        c = np.uint8([[color_code]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsvC[0][0][0]  # Get the hue value
        min_saturation = 0
        min_value = 0

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for divided red hue
            lowerLimit = np.array([hue - 30, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:  # Lower limit for divided red hue
            lowerLimit = np.array([0, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue + 30, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 30, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue + 30, 255, 255], dtype=np.uint8)

        return (lowerLimit, upperLimit)

    def find_small_package_coords(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        '''
        Find the coordinates of the small packages in the image.
        Arguments:
            image: np.ndarray - The image to find the small packages in.
            pose: str - The pose of the arm when the image was taken.
        Returns:
            coordinates: List[Point] - The coordinates of the small packages in the image.
            coords_image: np.ndarray - The image with the coordinates drawn on it.
        '''
        if image == np.zeros((160, 120, 3), dtype=np.uint8):
            coordinates = []
            return (coordinates, image)
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
        coordinates, coords_image = self.find_contours(gray, pose)
        coords_image = np.hstack([image, blur, median, darkened_frame, self.image])
        return (coordinates, coords_image)
    
    def find_thruster_or_fuel_tank_coords(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        '''
        Find the coordinates of the thrusters or fuel tanks in the image.
        Arguments:
            image: np.ndarray - The image to find the thrusters or fuel tanks in.
            pose: str - The pose of the arm when the image was taken.
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
    
    def find_contours(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        '''
        Find the contours in the image.
        Arguments:
            image: np.ndarray - The image to find the contours in.
            pose: str - The pose of the arm when the image was taken.
        Returns:
            contour_masks: List[Point] - The coordinates of the contours in the image.
            image: np.ndarray - The image with the contours drawn on it.
        '''
        coords_list = []
  
        contours = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = grab_contours(contours)

        image_area = image.size
        max_area = image_area * 0.025
        min_area = image_area * 0.0075
        output_img = self.image
        if len(contours) > 0:
            for contour in contours:
                M = cv2.moments(contour)
                area = M["m00"]
                #print(f"Min Area {min_area}    |   Area {area}")
                if area < min_area or area > max_area:
                    continue
                cX = int(M["m10"] / area)
                cY = int(M["m01"] / area)

                cv2.drawContours(self.image, [cv2.convexHull(contour)], -1, (0, 255, 0), 2)
                cv2.circle(self.image, (cX, cY), 2, (0, 255, 0), -1)
                x_coord, z_coord = self.coordinate_frame_conversion(cX, self.image_height - cY, pose)
                coords = Point()
                coords.x = x_coord
                coords.z = z_coord
                coords.y = -35.0
                coords_list.append(coords)
                coords_text = "(%.1f, %.1f)"  % (x_coord, z_coord) 
                cv2.putText(self.image, coords_text, (cX - 15, cY - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (255, 255, 255), 1)    
                output_img = np.hstack([self.image, cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)])
        return (coords_list, output_img)
    
    def coordinate_frame_conversion(self, x: Union[int, float], y: Union[int, float], pose: str) -> (float, float):
        '''
        Convert the coordinates from the image frame to the arm frame.
        Arguments:
            x: Union[int, float] - The x-coordinate in the image frame.
            y: Union[int, float] - The y-coordinate in the image frame.
            pose: str - The pose of the arm when the image was taken.
        Returns:
            x: float - The x-coordinate in the arm frame.
            y: float - The y-coordinate in the arm frame.
        '''
        if pose == "SCAN":
            return x*self.PX2MMX + 45.0, y*self.PX2MMZ - self.image_width/2
        elif pose == "VERIFY":
            # TODO: find and add the conversion factor for the verify pose
            return x*self.PX2MMX, y*self.PX2MMZ
        else:
            raise ValueError(f"Arm pose {pose} not recognized.")
        

def main():
    cap = cv2.VideoCapture(0)
    #cap.set(cv2.CAP_PROP_SETTINGS, 1)
    img_resize_factor = 4
    img_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    img_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, round(img_width/img_resize_factor))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, round(img_height/img_resize_factor))

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # Function to calculate the median blur kernel size
    median_kernel = lambda kernel: kernel if kernel % 2 == 1 else kernel + 1 
    ip = ImageProcessor()
    while cap.isOpened():
        ret, ip.image = cap.read()
        coords_list, coords_image = ip.get_coords('SMALL_PACKAGE', "SCAN")
        
        if not ret:
            print("Can't receive frame")
            break

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

if __name__ == '__main__':
    main()