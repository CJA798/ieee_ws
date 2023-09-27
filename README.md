# UNCA IEEE Workspace
#### **TODO: insert image here**
#### **TODO: insert info of each category**
### State Machine (Master Node)
### Navigation System
### Arm System
### Vision System
#### TODO:
Create server node for coordinate feedback of different objects

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
