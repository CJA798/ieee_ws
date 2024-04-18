import cv2
import matplotlib.pyplot as plt
import numpy as np
import imutils

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

# Initialize thruster_color with default values
thruster_color = [75, 112, 255]  # copper in BGR colorspace

# Callback function to update thruster_color based on trackbar values
def update_color(x):
    global thruster_color
    thruster_color = [cv2.getTrackbarPos('Blue', 'Image Preprocessing'),
                      cv2.getTrackbarPos('Green', 'Image Preprocessing'),
                      cv2.getTrackbarPos('Red', 'Image Preprocessing')]

# Create a window to display the sliders
cv2.namedWindow('Image Preprocessing')

# Create trackbars for Red, Green, and Blue components
cv2.createTrackbar('Red', 'Image Preprocessing', thruster_color[2], 255, update_color)
cv2.createTrackbar('Green', 'Image Preprocessing', thruster_color[1], 255, update_color)
cv2.createTrackbar('Blue', 'Image Preprocessing', thruster_color[0], 255, update_color)

# Initialize other variables and capture device
lower_limit, upper_limit = get_limits(color=thruster_color)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#cap.set(cv2.CAP_PROP_SETTINGS, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
min_area = cap.get(cv2.CAP_PROP_FRAME_WIDTH) * cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * 0.001
max_area = cap.get(cv2.CAP_PROP_FRAME_WIDTH) * cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * 0.95

if not cap.isOpened():
    print("Cannot open camera")
    exit()

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame")
        break

    if cv2.waitKey(5) == 27:
        break

    # Apply different smoothing techniques to the frame
    smoothed_frame_gaussian = cv2.GaussianBlur(frame, (15, 15), 0)
    smoothed_frame_median = cv2.medianBlur(frame, 15)
    smoothed_frame_bilateral = cv2.bilateralFilter(frame, 9, 75, 75)

    # Create a horizontal stack of the frames
    first_row = np.hstack((frame, smoothed_frame_gaussian, smoothed_frame_median, smoothed_frame_bilateral))

    # Display respective titles for each frame
    cv2.putText(first_row, 'Original', (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(first_row, 'Gaussian Blur', (frame.shape[1] + 20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(first_row, 'Median Blur', (2 * frame.shape[1] + 20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(first_row, 'Bilateral Filter', (3 * frame.shape[1] + 20, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    hsv_original = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_gaussian = cv2.cvtColor(smoothed_frame_gaussian, cv2.COLOR_BGR2HSV)
    hsv_median = cv2.cvtColor(smoothed_frame_median, cv2.COLOR_BGR2HSV)
    hsv_bilateral = cv2.cvtColor(smoothed_frame_bilateral, cv2.COLOR_BGR2HSV)

    lower_limit, upper_limit = get_limits(color=thruster_color)

    mask_original = cv2.inRange(hsv_original, lower_limit, upper_limit)
    mask_gaussian = cv2.inRange(hsv_gaussian, lower_limit, upper_limit)
    mask_median = cv2.inRange(hsv_median, lower_limit, upper_limit)
    mask_bilateral = cv2.inRange(hsv_bilateral, lower_limit, upper_limit)

    mask_original_ = cv2.cvtColor(mask_original, cv2.COLOR_GRAY2BGR)
    mask_gaussian_ = cv2.cvtColor(mask_gaussian, cv2.COLOR_GRAY2BGR)
    mask_median_ = cv2.cvtColor(mask_median, cv2.COLOR_GRAY2BGR)
    mask_bilateral_ = cv2.cvtColor(mask_bilateral, cv2.COLOR_GRAY2BGR)

    masks = [mask_original, mask_gaussian, mask_median, mask_bilateral]
#robotmove:1 FileExistsError
#robotmove:2 ChildProcessError
#robotmove3 smoothed_frame_gaussian
    # Create a horizontal stack of the frames
    second_row = np.hstack((mask_original_, mask_gaussian_, mask_median_, mask_bilateral_))
    
    contour_masks = []
    for mask in masks:
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
    
    third_row = np.hstack((contour_masks[0], contour_masks[1], contour_masks[2], contour_masks[3]))

    output = np.vstack((first_row, second_row, third_row))
    cv2.imshow('Image Preprocessing', output)

cap.release()
cv2.destroyAllWindows()