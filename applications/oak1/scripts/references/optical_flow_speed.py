import cv2
import numpy as np

# --- Parameters ---
video_path = "path/to/your/video.mp4"  # Replace with your video path
min_flow_magnitude = 5  # Minimum magnitude of optical flow vectors to consider
time_step = 1  # Time interval between frames in seconds (adjust as needed)
# --- End Parameters ---

# Initialize video capture
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Could not open video")
    exit()

# Get video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

# Calculate the number of pixels per meter (adjust based on your camera setup)
# This is a rough estimation and needs calibration for your specific camera
pixels_per_meter = 100  # Example: 100 pixels per meter

# --- Optical Flow Calculation ---
# Read the first frame
ret, prev_frame = cap.read()
if not ret:
    print("Error: Could not read the first frame")
    cap.release()
    exit()

# Convert to grayscale
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# Define the Lucas-Kanade parameters
lk_params = {
    'winSize': (15, 15),  # Window size for feature tracking
    'maxLevel': 2,  # Pyramid level to use
    'maxIter': 30,  # Maximum number of iterations
    'epsilon': 0.001  # Stop criteria
}

# --- Main Loop ---
while(cap.isOpened()):
    # Read the next frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find corners to track
    corners = cv2.goodFeaturesToTrack(prev_gray, 100, 0.01, 10)  # 100 corners, quality level, min distance

    # Calculate optical flow
    if corners is not None:
        # Find the next positions of the corners
        next_corners, status, error = cv2.calcOpticalFlowPyrLK(prev_gray, gray, corners, None, **lk_params)

        # Filter good points
        good_points = []
        for i, (x, y) in enumerate(corners):
            if status[i] == 1:
                good_points.append((x, y))

        # Calculate the average displacement
        if len(good_points) > 0:
            # Calculate the average displacement in pixels
            avg_displacement_x = np.mean([next_corners[i][0][0] - corners[i][0][0] for i in range(len(good_points))])
            avg_displacement_y = np.mean([next_corners[i][0][1] - corners[i][0][1] for i in range(len(good_points))])

            # Convert to meters per second
            avg_speed_x = (avg_displacement_x / pixels_per_meter) / time_step
            avg_speed_y = (avg_displacement_y / pixels_per_meter) / time_step

            print(f"Average speed (x): {avg_speed_x:.2f} m/s")
            print(f"Average speed (y): {avg_speed_y:.2f} m/s")

    # Display the frame
    cv2.imshow("Optical Flow", frame)

    # Prepare for the next iteration
    prev_gray = gray

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
