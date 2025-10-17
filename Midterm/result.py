import cv2
import numpy as np
import time
from pydobot import Dobot

# ---------------------------------------------------
# Camera Setup
# ---------------------------------------------------
home_position = (253, -4, 34)  # safe home position
Dobot = Dobot(port='/dev/ttyACM0')
Dobot.move_to(*home_position)
cap = cv2.VideoCapture('/dev/video2')  # adjust if needed

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
# Game Logic (from your main code)
# ---------------------------------------------------
def check_winner(b):
    for i in range(3):
        if b[i][0] == b[i][1] == b[i][2] != 0:
            return b[i][0]
        if b[0][i] == b[1][i] == b[2][i] != 0:
            return b[0][i]
    if b[0][0] == b[1][1] == b[2][2] != 0:
        return b[0][0]
    if b[0][2] == b[1][1] == b[2][0] != 0:
        return b[0][2]
    return 0

def is_moves_left(b):
    return any(0 in row for row in b)

# ---------------------------------------------------
# Color Detection (same as main)
# ---------------------------------------------------
def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color mask
    lower_red1, upper_red1 = np.array([0,120,70]), np.array([10,255,255])
    lower_red2, upper_red2 = np.array([170,120,70]), np.array([180,255,255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    # Yellow color mask
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
# Main Loop: Visual Game Result Detection
# ---------------------------------------------------
print("🎥 Starting camera-based board result detection test...")
font = cv2.FONT_HERSHEY_SIMPLEX

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect pieces
    red_centers, yellow_centers = detect_colors(frame)

    # Convert to grid positions
    red_cells = [pixel_to_grid(cx, cy) for cx, cy in red_centers]
    yellow_cells = [pixel_to_grid(cx, cy) for cx, cy in yellow_centers]

    # Build board state
    board = [[0 for _ in range(3)] for _ in range(3)]
    for (r, c) in red_cells:
        board[r][c] = 1  # human
    for (r, c) in yellow_cells:
        board[r][c] = 2  # robot

    # Draw grid lines
    for x in grid_x:
        cv2.line(frame, (x, y_min), (x, y_max), (255, 255, 255), 1)
    for y in grid_y:
        cv2.line(frame, (x_min, y), (x_max, y), (255, 255, 255), 1)

    # Display detected grid state
    for i in range(3):
        for j in range(3):
            cx = int((grid_x[j] + grid_x[j+1]) / 2)
            cy = int((grid_y[i] + grid_y[i+1]) / 2)
            symbol = "X" if board[i][j] == 1 else "O" if board[i][j] == 2 else "."
            cv2.putText(frame, symbol, (cx - 10, cy + 10), font, 0.8, (255,255,255), 2)

    # Evaluate result
    winner = check_winner(board)
    if winner == 1:
        result_text = "🎉 Human Wins!"
    elif winner == 2:
        result_text = "🤖 Robot Wins!"
    elif not is_moves_left(board):
        result_text = "It's a Draw!"
    else:
        result_text = "Game Ongoing..."

    cv2.putText(frame, result_text, (50, 60), font, 1, (0,255,0), 2)
    cv2.imshow("Board Result Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("✅ Test complete.")
