#!/usr/bin/env python

from time import time
from typing import List, Iterable, Sized
from geometry_msgs.msg import Point
import cv2
import numpy as np

color_codes = {
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

        # ... Obtain coordinates list
        # ... Obtain image displaying contours and coordinates
        
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