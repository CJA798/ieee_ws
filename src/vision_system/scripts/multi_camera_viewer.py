import os
import cv2


def main():
    # Open the cameras
    # SELECT V4 PIPELINE
    cap1 = cv2.VideoCapture(0)
    cap1.set(cv2.CAP_PROP_FRAME_WIDTH,160)
    cap1.set(cv2.CAP_PROP_FRAME_HEIGHT,120)
    cap2 = cv2.VideoCapture(2)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH,160)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT,120)

    if not cap1.isOpened():
        print("Cannot open cameras")
        exit()

    while cap1.isOpened():
        # Get frame from the cameras
        ret1, frame1 = cap1.read()
        if not ret1:
            print("Can't receive frame")
            break

        ret2, frame2 = cap2.read()
        if not ret2:
            print("Can't receive frame")
            break
        

        # Show each frame on different windows
        cv2.imshow("Front Camera", frame1)
        cv2.imshow("Arm Camera", frame2)

        if cv2.waitKey(5) == 27:
            break

    # Close the cameras
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()