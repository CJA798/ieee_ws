#!/usr/bin/env python

from time import time
from typing import List, Iterable, Sized, Union
from geometry_msgs.msg import Point
import cv2
import numpy as np
from image_utils.board_objects import BoardObjects
from imutils import grab_contours


#nidiaes*min_area=514593
#ax_area¡3847
#carlosNOsebaña:True
class ImageProcessor():
    
    def __init__(self) -> None:
        self.image = None
        self.color_data = {
        "orange": self.get_color_bounds([75, 112, 255]),
        "copper": self.get_color_bounds([75, 112, 255]),
        "yellow": self.get_color_bounds([0, 255, 255]),
        "black": (np.array([0, 0, 0], dtype=np.uint8), np.array([180, 255, 100], dtype=np.uint8)),
        }
        
        self.y_conversion_factor = 35.5
        self.x_conversion_factor = 31.5

        self.image_width, self.image_height, _ = self.image.shape()

        self.PX2CMY = self.y_conversion_factor / self.image_width
        self.PX2CMX = self.x_conversion_factor / self.image_height
        

    def get_coords(self, object_type: str, pose: str) -> (List[Point], np.ndarray):
        coordinates = []
        coords_image = self.image.copy()

        try:
            if object_type == BoardObjects.SMALL_PACKAGE.value:
                coordinates, coords_image = self.find_small_package_coords(coords_image, pose)
            elif object_type == BoardObjects.FUEL_TANK or object_type == BoardObjects.THRUSTER:
                coordinates, coords_image = self.find_thruster_or_fuel_tank_coords(coords_image, pose)
        except ValueError as e:
            raise ValueError(f"Object type {object_type} not recognized.")
        
        return (coordinates, coords_image)
    
    def get_color_bounds(self, color_code: Iterable[Sized]) -> (np.array, np.array):
        c = np.uint8([[color_code]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsvC[0][0][0]  # Get the hue value
        min_saturation = 50
        min_value = 10

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for divided red hue
            lowerLimit = np.array([hue - 10, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:  # Lower limit for divided red hue
            lowerLimit = np.array([0, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 10, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

        return (lowerLimit, upperLimit)

    def find_small_package_coords(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        # Apply blurs to remove noise
        blur = cv2.GaussianBlur(image, (7, 7), 0)
        blur = cv2.medianBlur(blur, 15)

        # Convert to HSV
        hsvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        # Create yellow mask
        lower_limit, upper_limit = self.color_data["yellow"]
        yellow_mask = cv2.inRange(hsvImage, lower_limit, upper_limit)
        inverted_mask = cv2.bitwise_not(yellow_mask)
        darkened_frame = cv2.bitwise_and(image, image, mask=inverted_mask)

        # Create black mask
        black_mask = cv2.inRange(darkened_frame, self.color_data["black"][0], self.color_data["black"][1])
        black_inverted_mask = cv2.bitwise_not(black_mask)
        darkened_frame = cv2.bitwise_and(darkened_frame, darkened_frame, mask=black_inverted_mask)

        coordinates, coords_image = self.find_contours(darkened_frame)

        return (coordinates, coords_image)
    
    def find_thruster_or_fuel_tank_coords(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        # ... Find thruster or fuel tank coordinates
        median = cv2.medianBlur(image, 5)
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        lower_limit, upper_limit = self.color_data["orange"]
        
        color_mask = cv2.inRange(hsv, lower_limit, upper_limit)
        color_mask = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)
    
        coordinates, coords_image = self.find_contours(color_mask, pose)

        return (coordinates, coords_image)
    
    def find_contours(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        contour_masks = []
  
        contours = cv2.findContours(image.copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = grab_contours(contours)

        image_area = image.size()
        max_area = image_area * 0.0165
        min_area = image_area * 0.0065

        if len(contours) > 0:
            for contour in contours:
                ()
                M = cv2.moments(contour)
                area = M["m00"]
                #print(f"Min Area {min_area}    |   Area {area}")
                if area < min_area or area > max_area:
                    continue
                cX = int(M["m10"] / area)
                cY = int(M["m01"] / area)

                cv2.drawContours(image, [cv2.convexHull(contour)], -1, (0, 255, 0), 2)
                cv2.circle(image, (cX, cY), 7, (0, 255, 0), -1)
                x_coord, y_coord = self.coordinate_frame_conversion(cX, self.image_height - cY)
                coords_text = "(%.1f, %.1f)"  % (x_coord, y_coord) 
                cv2.putText(image, coords_text, (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)    

        return (contour_masks, image)
    
    def coordinate_frame_conversion(self, x: Union[int, float], y: Union[int, float], pose: str) -> (float, float):
        if pose == "SCAN":
            return x*self.PX2CMX, y*self.PX2CMY
        elif pose == "VERIFY":
            # TODO: find and add the conversion factor for the verify pose
            return x*self.PX2CMX, y*self.PX2CMY
        else:
            raise ValueError(f"Arm pose {pose} not recognized.")
        

def main():
    ip = ImageProcessor()

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

    # Get color bounds
    color = "orange"
    (lower_bound, upper_bound) = ip.get_color_bounds(ip.color_data[color])
    # Function to calculate the median blur kernel size
    median_kernel = lambda kernel: kernel if kernel % 2 == 1 else kernel + 1 
    
    while cap.isOpened():
        ret, ip.image = cap.read()

        if not ret:
            print("Can't receive frame")
            break

        if cv2.waitKey(5) == 27:
            break
        
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
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()