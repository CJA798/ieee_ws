#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge


from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from vision_system.srv import get_small_package_coordinates, get_small_package_coordinatesResponse

class CoordServer():
	def __init__(self):
		self.coords_msg = Points()
		self.coords_service = rospy.Service('get_coords_service', , self.get_coords_cb)
				

	def _get_bounds(self, object):
		# Get color limits for the specific object
		return lower_bound, upper_bound

	def _preprocessing(self, image):
		# Apply filters to remove noise and smoothen the image
		return filtered_image	

	def _get_color_mask(self, filtered_image):
		# Get the color mask for the specific object
		return color_mask

	def _contour_filter(self, color_mask):
		# Apply contour filter and return good contours
		return contours

	def _get_coords_from_contour(self, object, contours):
		# Use minimum enclosing circle or rotated rectangle to obtain coords from contours
		# coords = [(),(), ... ,()]
		return coords

	def _get_coords(self, object, image):
		lower, upper = self._get_bounds(object)
		preprocessed_img = self._preprocessing(image)
		color_mask = self._get_color_mask(preprocessed_image)
		contours = self._contour_filter(color_mask)
		coords = self._get_coords_from_contour(object, contours)
		return coords

	def get_coords_cb(self, request):
		pass

if __name__ == "__main__":
    srv_name = "get_small_package_coordinates"
    rospy.init_node("small_package_coordinates_server")
    service = rospy.Service(srv_name, get_small_package_coordinates, get_coordinates_callback)
    rospy.spin()
