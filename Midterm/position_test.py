import cv2
from pydobot import Dobot
import numpy as np

# ------------------------------
# Open camera
# ------------------------------
cap = cv2.VideoCapture('/dev/video2')  # adjust if needed
Dobot = Dobot(port='/dev/ttyACM0')
print(Dobot.get_pose())  # just to ensure connection is alive

# Base calibration (from previous)
x_min, x_max = 311, 467
y_min, y_max = 230, 434  # from your good vertical tuning

# Adjust horizontal width and position
padding_x_left = 25    # expand left side
padding_x_right = 25   # expand right side

x_min -= padding_x_left
x_max += padding_x_right

# Recalculate divisions
grid_x = [int(x_min + (x_max - x_min) * i / 3) for i in range(4)]
grid_y = [int(y_min + (y_max - y_min) * i / 3) for i in range(4)]

print("Adjusted grid_x:", grid_x)
print("Adjusted grid_y:", grid_y)

# ------------------------------
# Pixel-to-grid mapping
# ------------------------------
def pixel_to_grid(cx, cy):
    """Convert pixel coordinates to 3x3 grid indices"""
    row = col = 0
    for i in range(3):
        if grid_x[i] <= cx < grid_x[i+1]:
            col = i
        if grid_y[i] <= cy < grid_y[i+1]:
            row = i
    return row, col

# ------------------------------
# Main loop
# ------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # ------------------------------
    # Red detection
    # ------------------------------
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # ------------------------------
    # Yellow detection
    # ------------------------------
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Clean noise
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

    # ------------------------------
    # Detect red blocks (X)
    # ------------------------------
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_red:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            row, col = pixel_to_grid(cx, cy)
            print(f"Red (X) block detected in cell ({row}, {col})")
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame, f"X ({row},{col})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # ------------------------------
    # Detect yellow blocks (O)
    # ------------------------------
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours_yellow:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            row, col = pixel_to_grid(cx, cy)
            print(f"Yellow (O) block detected in cell ({row}, {col})")
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
            cv2.putText(frame, f"O ({row},{col})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # ------------------------------
    # Draw grid lines for visual verification
    # ------------------------------
    for x in grid_x:
        cv2.line(frame, (x, y_min), (x, y_max), (255, 255, 255), 1)
    for y in grid_y:
        cv2.line(frame, (x_min, y), (x_max, y), (255, 255, 255), 1)

    cv2.imshow("Calibrated Grid Detection (Red & Yellow)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
