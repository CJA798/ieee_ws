import cv2

device_path = "/dev/v4l/by-id/usb-Arducam_Arducam_5MP_Camera_Module_YL20230518V0-video-index0"
cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
cap.set(3, 960)
cap.set(4, 540)

num = 0

while cap.isOpened():

    succes, img = cap.read()

    k = cv2.waitKey(5)

    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('/home/pi/CameraCalibration/images/img' + str(num) + '.png', img)
        print("image saved!")
        num += 1

    cv2.imshow('Img',img)

# Release and destroy all windows before termination
cap.release()

cv2.destroyAllWindows()