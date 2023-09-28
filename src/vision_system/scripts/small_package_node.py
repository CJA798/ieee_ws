#!/usr/bin/env python
import rospy
from vision_system.srv import getCoords
from vision_system.msg import CoordinatesList

def request_coordinates(object_type):
    # Initialize ROS node
    rospy.init_node('coordinate_requester')

    # Wait for the get_coords_service to become available
    rospy.wait_for_service('get_coords_service')

    try:
        # Create a service proxy for the get_coords_service
        get_coords_proxy = rospy.ServiceProxy('get_coords_service', getCoords)

        # Prepare the request
        request = getCoordsRequest()
        request.object_type = object_type  # Specify the object type you want coordinates for

        # Call the service to request coordinates
        response = get_coords_proxy(request)

        # Process the response
        if response:
            coordinates_list = response.coordinates
            for coords in coordinates_list.coordinates:
                rospy.loginfo("Coordinates: x=%.2f, y=%.2f, z=%.2f", coords.x, coords.y, coords.z)
        else:
            rospy.logwarn("No coordinates received from the service")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    object_type = "small_box"  # Specify the object type you want coordinates for
    request_coordinates(object_type)
