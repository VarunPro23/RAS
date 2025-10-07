import cv2
import numpy as np
import time
from pydobot import Dobot
import math

# ---------------------------------------------------
# Connect to Dobot
# ---------------------------------------------------
dobot = Dobot(port='COM3')  # change to your Dobot port
dobot.speed(100, 100)

# ---------------------------------------------------
# Find working camera
# ---------------------------------------------------
def find_camera():
    for i in range(3):
        cap = cv2.VideoCapture(i)
        if cap.isOpened() and cap.read()[0]:
            print(f"Camera found at index {i}")
            return cap
    raise RuntimeError("No working camera found")

cap = find_camera()

# ---------------------------------------------------
# 3x3 Grid Coordinates (empty placeholders)
# ---------------------------------------------------
grid_positions = [
    [(), (), ()],
    [(), (), ()],
    [(), (), ()]
]

# Source position for robot blocks
source_position = ()  # fill with (x, y, z) of block supply

# ---------------------------------------------------
# Game Board Representation
# 0 = empty, 1 = player (red), 2 = robot (blue)
board = [[0, 0, 0],
         [0, 0, 0],
         [0, 0, 0]]

# ---------------------------------------------------
# Game Logic Functions
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

def evaluate(b):
    winner = check_winner(b)
    if winner == 2:
        return 10
    elif winner == 1:
        return -10
    return 0

def minimax(b, depth, is_max):
    score = evaluate(b)
    if score != 0 or not is_moves_left(b):
        return score
    if is_max:
        best = -math.inf
        for i in range(3):
            for j in range(3):
                if b[i][j] == 0:
                    b[i][j] = 2
                    best = max(best, minimax(b, depth + 1, False))
                    b[i][j] = 0
        return best
    else:
        best = math.inf
        for i in range(3):
            for j in range(3):
                if b[i][j] == 0:
                    b[i][j] = 1
                    best = min(best, minimax(b, depth + 1, True))
                    b[i][j] = 0
        return best

def find_best_move(b):
    best_val = -math.inf
    best_move = (-1, -1)
    for i in range(3):
        for j in range(3):
            if b[i][j] == 0:
                b[i][j] = 2
                move_val = minimax(b, 0, False)
                b[i][j] = 0
                if move_val > best_val:
                    best_move = (i, j)
                    best_val = move_val
    return best_move

# ---------------------------------------------------
# Dobot Movement
# ---------------------------------------------------
def move_to_grid(row, col):
    if not grid_positions[row][col]:
        print(f"No coordinates set for grid cell ({row}, {col})")
        return
    x, y, z = grid_positions[row][col]
    dobot.move_to(x, y, z)
    time.sleep(1)

def pick_and_place_block(source, dest):
    dobot.move_to(*source)
    time.sleep(0.5)
    dobot.suck(True)
    time.sleep(1)
    dobot.move_to(*dest)
    time.sleep(0.5)
    dobot.suck(False)
    time.sleep(1)

# ---------------------------------------------------
# OpenCV Color Detection
# ---------------------------------------------------
def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red color (two ranges)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask_red1 + mask_red2

    # Blue color
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Find centers
    red_centers, blue_centers = [], []

    for mask, centers, val in [(mask_red, red_centers, 1), (mask_blue, blue_centers, 2)]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            centers.append((cx, cy))

    return red_centers, blue_centers

# ---------------------------------------------------
# Map pixel coordinates to grid
# ---------------------------------------------------
def map_to_grid(centers):
    # This is where you map camera pixels to grid indices (0-2)
    # For example, using thresholds or dividing the ROI into 3x3 cells
    grid_cells = []
    for (cx, cy) in centers:
        row, col = 0, 0
        # Example placeholders (replace with your calibration)
        if cx < 200: col = 0
        elif cx < 400: col = 1
        else: col = 2
        if cy < 200: row = 0
        elif cy < 400: row = 1
        else: row = 2
        grid_cells.append((row, col))
    return grid_cells

# ---------------------------------------------------
# Main Loop
# ---------------------------------------------------
print("Starting Tic-Tac-Toe with OpenCV color detection...")
player_turn = True

while True:
    ret, frame = cap.read()
    if not ret:
        break

    red_centers, blue_centers = detect_colors(frame)

    # Update board with player's red blocks
    for row, col in map_to_grid(red_centers):
        board[row][col] = 1

    winner = check_winner(board)
    if winner != 0 or not is_moves_left(board):
        print("Game Over!")
        break

    if not player_turn:
        print("Robot's turn...")
        row, col = find_best_move(board)
        if row != -1 and col != -1:
            print(f"Robot plays at ({row}, {col})")
            pick_and_place_block(source_position, grid_positions[row][col])
            board[row][col] = 2
        player_turn = True
    else:
        print("Waiting for player's move...")
        time.sleep(2)  # give time for player to place a block
        player_turn = False

    cv2.imshow("Tic-Tac-Toe", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
dobot.close()
