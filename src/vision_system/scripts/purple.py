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

# Function to update the threshold value
def update_threshold(value):
    global threshold_value
    threshold_value = value

# Function to update the kernel sizes for morphological operations
def update_open_kernel_size(value):
    global open_kernel_size
    open_kernel_size = value
# Function to update the kernel sizes for morphological operations
def update_closed_kernel_size(value):
    global closed_kernel_size
    closed_kernel_size = value
# Function to update the kernel sizes for morphological operations
def update_dilation_kernel_size(value):
    global dilation_kernel_size
    dilation_kernel_size = value
# Function to update the kernel sizes for morphological operations
def update_erosion_kernel_size(value):
    global erosion_kernel_size
    erosion_kernel_size = value

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

# Create a window to display trackbars
cv2.namedWindow('Trackbars')
cv2.createTrackbar('Threshold', 'Trackbars', threshold_value, 255, update_threshold)
cv2.createTrackbar('Open Kernel Size', 'Trackbars', open_kernel_size, 10, update_open_kernel_size)
cv2.createTrackbar('Closed Kernel Size', 'Trackbars', closed_kernel_size, 10, update_closed_kernel_size)
cv2.createTrackbar('Dilation Kernel Size', 'Trackbars', dilation_kernel_size, 20, update_dilation_kernel_size)
cv2.createTrackbar('Erosion Kernel Size', 'Trackbars', erosion_kernel_size, 20, update_erosion_kernel_size)



while True:
    ret, frame = cap.read()

    blur = cv2.GaussianBlur(frame, (7, 7), 0)
    blur = cv2.medianBlur(blur, 15)
    hsvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lowerLimit, upperLimit = get_limits(color=yellow)

    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    
     # Invert the mask
    inverted_mask = cv2.bitwise_not(mask)

    # Darken the frame outside the mask by blending with a black background
    darkened_frame = cv2.bitwise_and(frame, frame, mask=inverted_mask)

    black_lower_limit = np.array([0, 0, 0], dtype=np.uint8)
    black_upper_limit = np.array([180, 255, 100], dtype=np.uint8)
    black_mask = cv2.inRange(darkened_frame, black_lower_limit, black_upper_limit)
    # Invert the mask
    black_inverted_mask = cv2.bitwise_not(black_mask)
    # Darken the frame outside the mask by blending with a black background
    darkened_frame = cv2.bitwise_and(darkened_frame, darkened_frame, mask=black_inverted_mask)

    # Convert the image to grayscale
    gray = cv2.cvtColor(darkened_frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian smoothing to the grayscale image
    gaussian_blur = cv2.GaussianBlur(gray, (5, 5), 0)
    median_blur = cv2.medianBlur(gaussian_blur, 15)

    # Calculate the Laplacian of the smoothed image
    laplacian = cv2.Laplacian(median_blur, cv2.CV_64F)

    # Convert the Laplacian image to absolute values
    laplacian_abs = np.absolute(laplacian)

    # Scale the Laplacian values to the range [0, 255]
    laplacian_scaled = cv2.normalize(laplacian_abs, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    # Threshold the Laplacian image to obtain edges
    # You can adjust the threshold value to control edge detection sensitivity
    edges = cv2.threshold(laplacian_scaled, threshold_value, 255, cv2.THRESH_BINARY)[1]
    erode = cv2.erode(edges, np.ones((erosion_kernel_size,erosion_kernel_size)), iterations=1)
    # Perform morphological operations to further clean up the edges
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (open_kernel_size, open_kernel_size))
    kernel_closed = cv2.getStructuringElement(cv2.MORPH_RECT, (closed_kernel_size, closed_kernel_size))
    
    noise_open = cv2.morphologyEx(erode, cv2.MORPH_OPEN, kernel_open)
    noise_close = cv2.morphologyEx(noise_open, cv2.MORPH_CLOSE, kernel_closed)
    dilate = cv2.dilate(noise_close, np.ones((dilation_kernel_size,dilation_kernel_size)), iterations=1)
    
    contours = cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL, cv2. CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
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

            cv2.drawContours(frame, [cv2.convexHull(contour)], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cX, cY), 7, (0, 255, 0), -1)
            coords_text = "(%.1f, %.1f)"  % (cX*PX2CMX, (frame_height - cY)*PX2CMY) 
            cv2.putText(frame, coords_text, (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)    

        #cv2.imshow('Contour Detection', frame)

    row1 = cv2.resize(np.hstack((frame, darkened_frame, cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR), cv2.cvtColor(median_blur, cv2.COLOR_GRAY2BGR))), (960,240))
    row2 = cv2.resize(np.hstack((erode, noise_open, noise_close, dilate)), (960,240))

    cv2.imshow("Frame", row1)
    cv2.imshow("Result", row2)


    # Display the original image and the detected edges
    #cv2.imshow('Edges (LoG)', edges)

    #cv2.imshow('Darkened Frame', darkened_frame)
    #cv2.imshow('Mask', mask)
    #cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()