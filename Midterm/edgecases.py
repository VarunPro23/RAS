import cv2
import numpy as np
import time
from pydobot import Dobot

# ---------------------------------------------------
# Camera Setup
# ---------------------------------------------------
home_position = (253,-4,34)  # safe home position
Dobot = Dobot(port='/dev/ttyACM0')
Dobot.move_to(*home_position)
cap = cv2.VideoCapture('/dev/video2')  # adjust camera index if needed

# ---------------------------------------------------
# Grid Calibration
# ---------------------------------------------------
x_min, x_max = 311, 467
y_min, y_max = 230, 434
padding_x_left, padding_x_right = 25, 25
x_min -= padding_x_left
x_max += padding_x_right
grid_x = [int(x_min + (x_max - x_min) * i / 3) for i in range(4)]
grid_y = [int(y_min + (y_max - y_min) * i / 3) for i in range(4)]

# ---------------------------------------------------
# Color Detection (same logic as main code)
# ---------------------------------------------------
def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1, upper_red1 = np.array([0,120,70]), np.array([10,255,255])
    lower_red2, upper_red2 = np.array([170,120,70]), np.array([180,255,255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    lower_yellow, upper_yellow = np.array([20,100,100]), np.array([35,255,255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    red_centers, yellow_centers = [], []
    for mask, centers in [(mask_red, red_centers), (mask_yellow, yellow_centers)]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 300:
                x, y, w, h = cv2.boundingRect(cnt)
                centers.append((x + w // 2, y + h // 2))
    return red_centers, yellow_centers

# ---------------------------------------------------
# Pixel → Grid Mapping
# ---------------------------------------------------
def pixel_to_grid(cx, cy):
    row = col = 0
    for i in range(3):
        if grid_x[i] <= cx < grid_x[i + 1]:
            col = i
        if grid_y[i] <= cy < grid_y[i + 1]:
            row = i
    return 2 - row, col  # flip vertical

# ---------------------------------------------------
# Test Setup
# ---------------------------------------------------
print("⚙️ Starting edge-case detection test...")

# Define players
human_symbol = 1   # 1 = red
robot_symbol = 2   # 2 = yellow
previous_human_blocks = set()
board = [[0 for _ in range(3)] for _ in range(3)]  # track placed cells
font = cv2.FONT_HERSHEY_SIMPLEX

# ---------------------------------------------------
# Main Loop
# ---------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect pieces
    red_centers, yellow_centers = detect_colors(frame)
    red_cells = set(pixel_to_grid(cx, cy) for cx, cy in red_centers)
    yellow_cells = set(pixel_to_grid(cx, cy) for cx, cy in yellow_centers)

    # Draw grid overlay
    for x in grid_x:
        cv2.line(frame, (x, y_min), (x, y_max), (255, 255, 255), 1)
    for y in grid_y:
        cv2.line(frame, (x_min, y), (x_max, y), (255, 255, 255), 1)

    # Draw cell coordinates
    for i in range(3):
        for j in range(3):
            cx = int((grid_x[j] + grid_x[j+1]) / 2)
            cy = int((grid_y[i] + grid_y[i+1]) / 2)
            cv2.putText(frame, f"({2-i},{j})", (cx-25, cy+5), font, 0.5, (255,255,255), 1)

    # Determine which color belongs to human
    current_human_blocks = red_cells if human_symbol == 1 else yellow_cells
    current_robot_blocks = yellow_cells if robot_symbol == 2 else red_cells

    # -------------------------------
    # Detect New Human Move
    # -------------------------------
    new_human_blocks = current_human_blocks - previous_human_blocks

    # Condition 1: Multiple new pieces
    if len(new_human_blocks) > 1:
        msg = "❌ Multiple blocks placed!"
        print(msg)
        cv2.putText(frame, msg, (60, 60), font, 0.7, (0,0,255), 2)
        time.sleep(2)

    # Condition 2 & 3: Single new block
    elif len(new_human_blocks) == 1:
        (row, col) = list(new_human_blocks)[0]

        # Wrong color check
        correct_color = (human_symbol == 1 and (row,col) in red_cells) or \
                        (human_symbol == 2 and (row,col) in yellow_cells)
        if not correct_color:
            msg = "❌ Wrong color block!"
            print(msg)
            cv2.putText(frame, msg, (60, 60), font, 0.7, (0,0,255), 2)
            time.sleep(2)
        # Occupied cell check
        elif board[row][col] != 0:
            msg = f"❌ Cell ({row},{col}) already occupied!"
            print(msg)
            cv2.putText(frame, msg, (60, 60), font, 0.7, (0,0,255), 2)
            time.sleep(2)
        else:
            msg = f"✅ Valid move at ({row},{col})"
            print(msg)
            board[row][col] = human_symbol  # mark it valid on board

        previous_human_blocks = current_human_blocks

    # Draw visual markers
    for (x, y) in red_centers:
        cv2.circle(frame, (x, y), 8, (0, 0, 255), 2)
    for (x, y) in yellow_centers:
        cv2.circle(frame, (x, y), 8, (0, 255, 255), 2)

    cv2.imshow("Edge Case Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("✅ Edge-case detection test complete.")
