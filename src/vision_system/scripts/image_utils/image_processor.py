#!/usr/bin/env python

from time import time
from typing import List, Iterable, Sized
from geometry_msgs.msg import Point
import cv2
import numpy as np
from image_utils.board_objects import BoardObjects

color_codes = {
    # TODO: Change this dictionary to color_data
    "orange": [75, 112, 255],
    "copper": [75, 112, 255],
    "yellow": [0, 255, 255],
    "black": [0, 0, 0],
}

class CoordinatesList():
    def __init__(self):
        self.coordinates = None

class ImageProcessor():
    def __init__(self) -> None:
        self.image = None

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
    
    def get_limits(self, color_code: Iterable[Sized]) -> (np.array, np.array):
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
        pass

    def find_thruster_or_fuel_tank_coords(self, image: np.ndarray, pose: str) -> (List[Point], np.ndarray):
        # ... Find thruster or fuel tank coordinates
        median = cv2.medianBlur(image, 5)
        hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

        # TODO: Calculate limimts in color_data dictionary.
        # This will avoid doing the same calculation multiple times
        lower_limit, upper_limit = self.get_limits(color_code=color_codes["copper"])
        
        mask_median = cv2.inRange(hsv, lower_limit, upper_limit)
        mask_median_ = cv2.cvtColor(mask_median, cv2.COLOR_GRAY2BGR)

        contour_masks = []
    
        frame_copy = frame.copy()
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)

        if len(contours) > 0:
            for contour in contours:
                M = cv2.moments(contour)
                area = M["m00"]
                if area <= min_area or area >= max_area:
                    continue
                cX = int(M["m10"] / area)
                cY = int(M["m01"] / area)

                cv2.drawContours(frame_copy, [contour], -1, (0, 255, 0), 2)
                cv2.circle(frame_copy, (cX, cY), 7, (0, 255, 0), -1)
                cv2.putText(frame_copy, f"({cX}, {cY})", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(frame_copy, center, radius, (255, 0, 255), 2)
                cv2.circle(frame_copy, center, 5, (255, 0, 255), -1)
        
        contour_masks.append(frame_copy)
    
        return (coordinates, coords_image)

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
    (lower_bound, upper_bound) = ip.get_limits(color_codes[color])
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