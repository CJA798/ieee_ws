import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920//2.5)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080//2.5)

while True:
    ret, frame = cap.read()
    cv2.imshow('Camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
