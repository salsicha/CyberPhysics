import cv2
import numpy as np
import time
# Parameters (adjust as needed)
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
feature_params = dict(maxCorners=500,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)
# Initialize variables
prev_gray = None
prev_pts = None
def stabilize_frame(frame):
    global prev_gray, prev_pts
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if prev_gray is None:
        prev_gray = gray
        prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **feature_params)
        return frame
    # Calculate optical flow
    new_pts, status, errors = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **lk_params)
    # Select good points
    good_new = new_pts[status == 1]
    good_old = prev_pts[status == 1]
    # Calculate homography matrix
    if len(good_new) > 3:
        H, _ = cv2.findHomography(good_old, good_new, cv2.RANSAC, 5.0)
        # Apply transformation
        stabilized_frame = cv2.warpPerspective(frame, H, (frame.shape[1], frame.shape[0]))
    else:
        stabilized_frame = frame
    # Update previous frame and points
    prev_gray = gray
    prev_pts = good_new.reshape(-1, 1, 2)
    return stabilized_frame
# Example usage (replace with your drone's video feed)
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        break
    stabilized_frame = stabilize_frame(frame)
    cv2.imshow('Stabilized Frame', stabilized_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    time.sleep(0.01)
cap.release()
cv2.destroyAllWindows()
