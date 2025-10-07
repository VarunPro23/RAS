import cv2
import numpy as np

# ------------------------------
# Open camera
# ------------------------------
cap = cv2.VideoCapture(0)  # change index if needed

# ------------------------------
# Define the region of interest for your 3x3 grid
# Adjust these values to match your camera view of the pallet
# ------------------------------
grid_x = [0, 200, 400, 600]  # x pixel boundaries for columns (left to right)
grid_y = [0, 200, 400, 600]  # y pixel boundaries for rows (top to bottom)

def pixel_to_grid(cx, cy):
    """Convert pixel coordinates to 3x3 grid indices"""
    row = col = 0
    for i in range(3):
        if grid_x[i] <= cx < grid_x[i+1]:
            col = i
        if grid_y[i] <= cy < grid_y[i+1]:
            row = i
    return row, col

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # Blue color
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Detect red blocks
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_red:
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w // 2, y + h // 2
        row, col = pixel_to_grid(cx, cy)
        print(f"Red block detected in cell ({row}, {col})")
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)

    # Detect blue blocks
    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_blue:
        x, y, w, h = cv2.boundingRect(cnt)
        cx, cy = x + w // 2, y + h // 2
        row, col = pixel_to_grid(cx, cy)
        print(f"Blue block detected in cell ({row}, {col})")
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255,0,0), 2)

    # Show frame
    cv2.imshow("Grid Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
