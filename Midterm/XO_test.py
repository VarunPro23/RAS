import cv2
import numpy as np
import time
from pydobot import Dobot
import math

# ---------------------------------------------------
# Connect to Dobot
# ---------------------------------------------------
Dobot = Dobot(port='/dev/ttyACM0')  # change to your Dobot port

# ---------------------------------------------------
# Open camera
# ---------------------------------------------------
cap = cv2.VideoCapture('/dev/video2')  # adjust camera index if needed

# ---------------------------------------------------
# GRID CALIBRATION
# ---------------------------------------------------
x_min, x_max = 311, 467
y_min, y_max = 230, 434

padding_x_left = 25
padding_x_right = 25
x_min -= padding_x_left
x_max += padding_x_right

grid_x = [int(x_min + (x_max - x_min) * i / 3) for i in range(4)]
grid_y = [int(y_min + (y_max - y_min) * i / 3) for i in range(4)]

# ---------------------------------------------------
# Dobot physical positions
# ---------------------------------------------------
grid_positions = [
    [(250, -30, -40), (250, -10, -40), (250, 10, -40)],
    [(270, -30, -40), (270, -10, -40), (270, 10, -40)],
    [(290, -30, -40), (290, -10, -40), (290, 10, -40)]
]
source_position = (255, 110, -10)  # starting z of the block stack
current_source_z = source_position[2]
home_position = (225, 5, 20)

# ---------------------------------------------------
# Game Board
# ---------------------------------------------------
board = [[0, 0, 0],
         [0, 0, 0],
         [0, 0, 0]]

# ---------------------------------------------------
# Game Logic
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
def pick_and_place_block(source, dest):
    global current_source_z
    sx, sy, _ = source
    dx, dy, dz = dest

    # Move to home first
    Dobot.move_to(*home_position)
    time.sleep(1)

    # Pick from current stack height
    Dobot.move_to(sx, sy, current_source_z)
    Dobot.suck(True)
    time.sleep(1)

    # Lift slightly before moving
    Dobot.move_to(sx, sy, current_source_z + 30)

    # Move to destination
    Dobot.move_to(dx, dy, dz)
    Dobot.suck(False)
    print(f"🤖 Robot placed block at {dest}")

    # Return to home
    Dobot.move_to(home_position)

    # Decrease source Z for next pick
    current_source_z -= 10
    time.sleep(1)

# ---------------------------------------------------
# Color Detection: Red (X) and Yellow (O)
# ---------------------------------------------------
def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([35, 255, 255])
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
# Game Setup
# ---------------------------------------------------
choice = input("Who plays first? (human/robot): ").strip().lower()
if choice not in ["human", "robot"]:
    choice = "human"

if choice == "human":
    human_symbol = 1  # red
    robot_symbol = 2  # yellow
    player_turn = True
elif choice == "robot":
    human_symbol = 2  # yellow
    robot_symbol = 1  # red
    player_turn = False
else:
    print("Invalid input")

previous_human_blocks = set()
print("Starting Tic Tac Toe...")

# ---------------------------------------------------
# Main Game Loop
# ---------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    red_centers, yellow_centers = detect_colors(frame)
    red_cells = set(pixel_to_grid(cx, cy) for cx, cy in red_centers)
    yellow_cells = set(pixel_to_grid(cx, cy) for cx, cy in yellow_centers)

    # Draw grid overlay
    for x in grid_x:
        cv2.line(frame, (x, y_min), (x, y_max), (255, 255, 255), 1)
    for y in grid_y:
        cv2.line(frame, (x_min, y), (x_max, y), (255, 255, 255), 1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    for i in range(3):
        for j in range(3):
            cx = int((grid_x[j] + grid_x[j+1]) / 2)
            cy = int((grid_y[i] + grid_y[i+1]) / 2)
            cv2.putText(frame, f"({2-i},{j})", (cx-25, cy+5), font, 0.5, (255,255,255), 1)

    current_human_blocks = red_cells if human_symbol == 1 else yellow_cells
    current_robot_blocks = yellow_cells if robot_symbol == 2 else red_cells

    # -------------------
    # Human Move Validation
    # -------------------
    new_human_blocks = current_human_blocks - previous_human_blocks
    if player_turn:
        if len(new_human_blocks) == 0:
            pass  # waiting
        elif len(new_human_blocks) > 1:
            print("⚠️ Multiple new blocks detected! Only one move allowed.")
            time.sleep(2)
            continue
        else:
            (row, col) = list(new_human_blocks)[0]

            # Check if human used the correct color
            correct_color = (human_symbol == 1 and (row,col) in red_cells) or \
                            (human_symbol == 2 and (row,col) in yellow_cells)
            if not correct_color:
                print("⚠️ Wrong block used by player!")
                time.sleep(2)
                continue

            if board[row][col] != 0:
                print("⚠️ Cell already occupied!")
                time.sleep(2)
                continue

            board[row][col] = human_symbol
            previous_human_blocks = current_human_blocks
            player_turn = False
            print(f"✅ Human placed at ({row},{col})")

            # Wait 2-3 seconds before robot moves
            time.sleep(3)

    # -------------------
    # Check Winner / Draw
    # -------------------
    winner = check_winner(board)
    if winner != 0:
        print("🎉 Human wins!" if winner == human_symbol else "🤖 Robot wins!")
        break
    if not is_moves_left(board):
        print("It's a draw!")
        break

    # -------------------
    # Robot Move
    # -------------------
    if not player_turn:
        print("🤖 Robot's turn...")
        row, col = find_best_move(board)
        if row != -1 and col != -1:
            print(f"🤖 Robot plays at ({row},{col})")
            pick_and_place_block(source_position, grid_positions[row][col])
            board[row][col] = robot_symbol
        player_turn = True

    cv2.imshow("Tic Tac Toe", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
Dobot.close()
