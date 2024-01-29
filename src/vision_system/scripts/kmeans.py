from sklearn.cluster import KMeans
import cv2


cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_SETTINGS, 1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640/4)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480/4)
min_area = cap.get(cv2.CAP_PROP_FRAME_WIDTH) * cap.get(cv2.CAP_PROP_FRAME_HEIGHT) * 0.02
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

    X = frame.reshape(-1,3)
    kmeans = KMeans(n_clusters=3, n_init=5)
    kmeans.fit(X)

    segmented_img = kmeans.cluster_centers_[kmeans.labels_]
    segmented_img = segmented_img.reshape(frame.shape)

    segmented_img = segmented_img / 255
    cv2.imshow('K-Means', segmented_img)
cap.release()
cv2.destroyAllWindows()