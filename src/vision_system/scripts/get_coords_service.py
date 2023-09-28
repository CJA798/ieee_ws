#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from vision_system.msg import CoordinatesList

from vision_system.srv import getCoords, getCoordsResponse


class CoordServer():
	def __init__(self):
		self.coords_service = rospy.Service('get_coords_service', getCoords, self.get_coords_cb)


	def _msg2cv2(self, img_msg):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(img_msg)
		return image


	def _get_bounds(self, object_type):
		if object_type == 'small_package':
			color = [75, 112, 255]
		elif object_type == 'thruster':
			color = [75, 112, 255]
		elif object_type == 'fuel_tank':
			color = [75, 112, 255]
		else:
			color = [75, 112, 255]

		# Get color limits for the specific object_type
		c = np.uint8([[color]])  # BGR values
		hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

		hue = hsvC[0][0][0]  # Get the hue value
		min_saturation = 50
		min_value = 10

		# Handle red hue wrap-around
		if hue >= 165:  # Upper limit for divided red hue
			lower_bound = np.array([hue - 20, min_saturation, min_value], dtype=np.uint8)
			upper_bound = np.array([180, 255, 255], dtype=np.uint8)
		elif hue <= 15:  # Lower limit for divided red hue
			lower_bound = np.array([0, min_saturation, min_value], dtype=np.uint8)
			upper_bound = np.array([hue + 20, 255, 255], dtype=np.uint8)
		else:
			lower_bound = np.array([hue - 20, min_saturation, min_value], dtype=np.uint8)
			upper_bound = np.array([hue + 20, 255, 255], dtype=np.uint8)
		
		return lower_bound, upper_bound


	def _preprocessing(self, image):
		# Apply filters to remove noise and smoothen the image
		median = cv2.medianBlur(image, 25)
		hsv_median = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)
		filtered_image = hsv_median

		return filtered_image	


	def _get_color_mask(self, object_type, filtered_image):
		# Get the color mask for the specific object_type
		lower_bound, upper_bound = self._get_bounds(object_type)
		color_mask = cv2.inRange(filtered_image, lower_bound, upper_bound)

		return color_mask


	def _contour_filter(self, color_mask):
		# Apply contour filter and return good contours
		min_area = 200
		max_area = 30000
		contours = cv2.findContours(color_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = imutils.grab_contours(contours)

		filtered_contours = []
		if len(contours) > 0:
			for contour in contours:
				M = cv2.moments(contour)
				area = M["m00"]
				if min_area <= area <= max_area:
					filtered_contours.append(contour)

		return filtered_contours


	def _get_coords_from_contour(self, object_type, contours):
		# Use minimum enclosing circle or rotated rectangle to obtain coords from contours
		coords_list = CoordinatesList()
		for contour in contours:
			coords = Point()

			if object_type == "small_box":
				coords.x, coords.y = cv2.minAreaRect(contour)[0]
				coords.z = 0.5

				coords_list.coordinates.append(coords)
			else:
				(coords.x, coords.y), _ = cv2.minEnclosingCircle(contour)
				
				if object_type == "fuel_tank":
					coords.z = 1
				else:
					coords.z = 0.5

				coords_list.coordinates.append(coords)
		
		return coords_list


	def _get_coords(self, object_type, image):
		lower, upper = self._get_bounds(object_type)
		preprocessed_image = self._preprocessing(image)
		color_mask = self._get_color_mask(object_type, preprocessed_image)
		contours = self._contour_filter(color_mask)
		coords_list = self._get_coords_from_contour(object_type, contours)
		
		return coords_list


	def get_coords_cb(self, request):
		# Check for valid frame
		if not request.frame:
			rospy.logwarn("Invalid request: Empty image received")
			return getCoordsResponse()
		
		# Check if object type is supported
		if request.object_type not in ["small_box", "thruster", "fuel_tank"]:
			rospy.logwarn("Invalid object type: %s", request.object_type)
			return getCoordsResponse()

		image = self._msg2cv2(request.frame)
		object_type = request.object_type
		coords_list = self._get_coords(object_type, image)
		
		response = getCoordsResponse()
		response.coordinates = coords_list
		rospy.loginfo("Coordinates list returned")

		return response


if __name__ == "__main__":
	node_name = "get_coords_node"
	rospy.init_node(node_name)
	CoordServer()
	rospy.spin()
