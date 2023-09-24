#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge


from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from vision_system.srv import get_small_package_coordinates, get_small_package_coordinatesResponse


def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value
    min_saturation = 50
    min_value = 10

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 20, min_saturation, min_value], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, min_saturation, min_value], dtype=np.uint8)
        upperLimit = np.array([hue + 20, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 20, min_saturation, min_value], dtype=np.uint8)
        upperLimit = np.array([hue + 20, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit


def get_coordinates_callback(request):
    thruster_color = [75, 112, 255]
    bridge = CvBridge()
    frame_to_cv = bridge.imgmsg_to_cv2(request.frame)

    median = cv2.medianBlur(frame, 25)
    hsv_median = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

    lower_limit, upper_limit = get_limits(color=thruster_color)
    mask = cv2.inRange(hsv_median, lower_limit, upper_limit)

    contour_mask = frame.copy()
    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)

    coords = []

    if len(contours) > 0:
        for contour in contours:
            M = cv2.moments(contour)
            area = M["m00"]
            if area <= min_area or area >= max_area:
                continue
            cX = int(M["m10"] / area)
            cY = int(M["m01"] / area)

            cv2.drawContours(contour_mask, [contour], -1, (0, 255, 0), 2)
            cv2.circle(contour_mask, (cX, cY), 7, (0, 255, 0), -1)
            #cv2.putText(contour_mask, f"({cX}, {cY})", (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            coordinates = (center[0], center[1], 1)
            coords.append(coordinates)
            radius = int(radius)
            cv2.circle(contour_mask, center, radius, (255, 0, 255), 2)
            cv2.circle(contour_mask, center, 5, (255, 0, 255), -1)

    print(coords)
    return get_small_package_coordinatesResponse(coords)

if __name__ == "__main__":
    srv_name = "get_small_package_coordinates"
    rospy.init_node("small_package_coordinates_server")
    service = rospy.Service(srv_name, get_small_package_coordinates, get_coordinates_callback)
    rospy.spin()