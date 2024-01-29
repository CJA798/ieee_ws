#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from vision_system.msg import CoordinatesList, GetCoordsAction, GetCoordsGoal, GetCoordsResult, GetCoordsFeedback
from time import time
from typing import List
import cv2
import numpy as np
from image_processor import ImageProcessor
from cv_bridge import CvBridge


class ImageProcessor():
    def __init__(self):
        self.image = None

    def get_coords(self) -> (CoordinatesList, np.ndarray):
        coordinates = CoordinatesList()
        coords_image = self.image.copy()

        # Purple:

        # Other than Purple:
        # Get color
        # Get color threshold
        # Apply filters
        # Make mask
        # Find contours


        return (coordinates, coords_image)