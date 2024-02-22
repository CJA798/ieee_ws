import cv2


# Function to update the camera settings based on trackbar values
def update_settings():
    brightness = cv2.getTrackbarPos('Brightness', 'Settings')
    contrast = cv2.getTrackbarPos('Contrast', 'Settings')
    saturation = cv2.getTrackbarPos('Saturation', 'Settings')
    hue = cv2.getTrackbarPos('Hue', 'Settings')
    gain = cv2.getTrackbarPos('Gain', 'Settings')
    exposure = cv2.getTrackbarPos('Exposure', 'Settings')
    
    cap.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
    cap.set(cv2.CAP_PROP_CONTRAST, contrast)
    cap.set(cv2.CAP_PROP_SATURATION, saturation)
    cap.set(cv2.CAP_PROP_HUE, hue)
    cap.set(cv2.CAP_PROP_GAIN, gain)
    cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

# Function to handle trackbar events
def on_trackbar(value):
    pass

cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
#cap.set(cv2.CAP_PROP_SETTINGS, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920//2)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080//2)

# Create a window to display the camera frame and trackbars
cv2.namedWindow('Camera')
cv2.namedWindow('Settings')

# Get default camera values
default_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
default_contrast = cap.get(cv2.CAP_PROP_CONTRAST)
default_saturation = cap.get(cv2.CAP_PROP_SATURATION)
default_hue = cap.get(cv2.CAP_PROP_HUE)
default_gain = cap.get(cv2.CAP_PROP_GAIN)
default_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)

# Create trackbars for adjusting settings
cv2.createTrackbar('Brightness', 'Settings', int(default_brightness), 100, on_trackbar)
cv2.createTrackbar('Contrast', 'Settings', int(default_contrast), 100, on_trackbar)
cv2.createTrackbar('Saturation', 'Settings', int(default_saturation), 100, on_trackbar)
cv2.createTrackbar('Hue', 'Settings', int(default_hue), 100, on_trackbar)
cv2.createTrackbar('Gain', 'Settings', int(default_gain), 100, on_trackbar)
cv2.createTrackbar('Exposure', 'Settings', int(default_exposure), 100, on_trackbar)

while True:
    ret, frame = cap.read()
    cv2.imshow('Camera', frame)

    # Update settings based on trackbar values
    update_settings()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
