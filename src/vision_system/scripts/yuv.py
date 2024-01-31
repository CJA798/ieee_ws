import cv2
import numpy as np
import imutils
from PIL import Image


def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 155:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 30, 0, 0], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 5:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 0, 0], dtype=np.uint8)
        upperLimit = np.array([hue + 30, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 30, 0, 0], dtype=np.uint8)
        upperLimit = np.array([hue + 30, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit

yellow = [0, 255, 255]  # yellow in BGR colorspace
cap = cv2.VideoCapture(0)

frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
frame_area = frame_width * frame_height
max_area = int(frame_area * 0.0165)
min_area = int(frame_area * 0.0065)

PX2CMY = 35.5 / frame_width
PX2CMX = 31.5 / frame_height

# Initialize threshold value and morphological kernel size
threshold_value = 25
erosion_kernel_size = 2
open_kernel_size = 1  # Initial kernel size
closed_kernel_size = 4
dilation_kernel_size = 15

while True:
    ret, frame = cap.read()

    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    blur = cv2.medianBlur(blur, 15)
    yuvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2YUV)
    
    cv2.imshow("YUV", yuvImage)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()