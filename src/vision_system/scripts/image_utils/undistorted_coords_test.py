import cv2
import numpy as np
import imutils
from PIL import Image
import pickle
from typing import Union, Tuple, Iterable, Sized


def get_limits(color_code: Iterable[Sized],
                hue_threshold: Iterable[Sized]=(15,165),
                hue_offset: int=30,
                hue: Iterable[Sized]=(0,180),
                saturation: Iterable[Sized]=(0,255),
                value: Iterable[Sized]=(0,255)) -> Tuple[np.array, np.array]:
        '''
        Get the lower and upper bounds for the color in the image.
        Arguments:
            color_code: Iterable[Sized] - The BGR color code for the color.
            hue_threshold: Iterable[Sized] - .
            hue_offset: int - The offset for the hue value.
            hue: Iterable[Sized] - Hue range (0-179) = (red -> yellow -> green -> cyan -> blue -> magenta -> pink -> red).
            saturation: Iterable[Sized] - Saturation range (0-255) = (white to color).
            value: Iterable[Sized] - Value range (0-255) = (black to color).
        Returns:
            lower_limit: np.array - The lower limit for the color in the image.
            upper_limit: np.array - The upper limit for the color in the image.
        '''
        c = np.uint8([[color_code]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue_ = hsvC[0][0][0]  # Get the hue value

        upper_hue_threshold = hue_threshold[1]
        lower_hue_threshold = hue_threshold[0]
        min_hue = hue[0]
        max_hue = hue[1]
        min_saturation = saturation[0]
        max_saturation = saturation[1]
        min_value = value[0]
        max_value = value[1]

        # Handle red hue wrap-around
        if hue_ >= upper_hue_threshold:  # Upper limit for divided red hue
            lowerLimit = np.array([hue_ - hue_offset, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([max_hue, max_saturation, max_value], dtype=np.uint8)
        elif hue_ <= lower_hue_threshold:  # Lower limit for divided red hue
            lowerLimit = np.array([min_hue, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue_ + hue_offset, max_saturation, max_value], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue_ - hue_offset//2, min_saturation, min_value], dtype=np.uint8)
            upperLimit = np.array([hue_ + hue_offset//2, max_saturation, max_value], dtype=np.uint8)

        return (lowerLimit, upperLimit)


def coordinate_frame_conversion(image_height, image_width, x: Union[int, float], y: Union[int, float], z: Union[int, float]) -> Tuple[float, float, float]:
    '''
    Convert the coordinates from the image frame to the arm frame.
    Arguments:
        x: Union[int, float] - The x-coordinate in the image frame.
        y: Union[int, float] - The y-coordinate in the image frame.
        z: Union[int, float] - The z-coordinate in the image frame.
    Returns:
        x: float - The x-coordinate in the arm frame.
        y: float - The y-coordinate in the arm frame.
        z: float - The z-coordinate in the arm frame.
    '''

    KX = 57/47
    PX2MM_Y = 430/image_height
    PX2MM_X = 580/image_width * KX

    # Offset from bottom of image to arm base
    A = 45

    Xarm_xi_obj = (image_height - y) * PX2MM_Y + A
    Yarm_xi_obj = 0
    Zarm_xi_obj = (x - image_width/2) * PX2MM_X
    #print(f"Y: {y}  |   Max_Cam_Height: {self.image_height}     |   Conversion Factior: {PX2MM_Y}")

    Xarm_offset = 0
    Zarm_offset = 0

    return Xarm_xi_obj + Xarm_offset, Yarm_xi_obj, Zarm_xi_obj + Zarm_offset


magenta = [255, 0, 255]  # magenta in BGR colorspace
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Set the resolution to 960x540
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 540)

# Set contrast to 3
cap.set(cv2.CAP_PROP_CONTRAST, 3)
# Set saturation to 150
cap.set(cv2.CAP_PROP_SATURATION, 150)

frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
frame_area = frame_width * frame_height
max_area = int(frame_area) * 0.01
min_area = int(frame_area * 0.0025)

PX2CMY = 58 / frame_width # 58 cm
PX2CMX = 40 / frame_height # 40 cm

# Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
#pickle.dump((cameraMatrix, dist), open( "/home/pi/CameraCalibration/calibration.pkl", "wb" ))
#pickle.dump(cameraMatrix, open( "/home/pi/CameraCalibration/cameraMatrix.pkl", "wb" ))
#pickle.dump(dist, open( "/home/pi/CameraCalibration/dist.pkl", "wb" ))

# Open the camera calibration results using pickle
cameraMatrix = pickle.load(open( "/home/pi/CameraCalibration/cameraMatrix.pkl", "rb" ))
dist = pickle.load(open( "/home/pi/CameraCalibration/dist.pkl", "rb" ))

while True:
    ret, frame = cap.read()

    # Resize the frame to half
    #frame = cv2.resize(frame, (int(frame_width//2), int(frame_height//2)))

    h,  w = frame.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    # Undistort
    dst = cv2.undistort(frame, cameraMatrix, dist, None, newCameraMatrix)
    # crop the image
    #x, y, w, h = roi
    #dst = dst[y:y+h, x:x+w]
    #dst = frame
    contour_frame = dst.copy()

    blur = cv2.GaussianBlur(dst, (7, 7), 0)
    blur = cv2.medianBlur(blur, 15)
    hsvImage = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lowerLimit, upperLimit = get_limits(color_code=magenta)

    mask = cv2.inRange(hsvImage.copy(), lowerLimit, upperLimit)

    # Convert the image to BGR
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Apply Gaussian smoothing to the grayscale image
    gaussian_blur = cv2.GaussianBlur(mask, (5, 5), 0)
    median_blur = cv2.medianBlur(gaussian_blur, 15)

    #cv2.imshow('img', median_blur)
    #contours = cv2.findContours(median_blur.copy(), cv2.RETR_EXTERNAL, cv2. CHAIN_APPROX_SIMPLE)
    contours = cv2.findContours(cv2.cvtColor(median_blur, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        for contour in contours:
            M = cv2.moments(contour)
            area = M["m00"]
            #print(f"Min Area {min_area}    |   Area {area}")
            if area < min_area or area > max_area:
                continue
            cX = int(M["m10"] / area)
            cY = int(M["m01"] / area)

            cv2.drawContours(contour_frame, [cv2.convexHull(contour)], -1, (0, 255, 0), 2)
            cv2.circle(contour_frame, (cX, cY), 7, (0, 255, 0), -1)
            x_arm, y_arm, z_arm = coordinate_frame_conversion(h,w, cX, cY, 0)

            coords_text = "(%.1f, %.1f)"  % (z_arm, x_arm) 
            cv2.putText(contour_frame, coords_text, (cX - 20, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)   

        #cv2.imshow('Contour Detection', frame)

    row1 = cv2.resize(np.hstack((frame, contour_frame)), (960,540))

    cv2.imshow("Frame", row1)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()
