import cv2
import numpy as np
import time
from pydobot import Dobot
import math

# ---------------------------------------------------
# Connect to Dobot
# ---------------------------------------------------
Dobot = Dobot(port='/dev/ttyACM0')  # change if needed

# ---------------------------------------------------
# Open camera
# ---------------------------------------------------
cap = cv2.VideoCapture('/dev/video2')  # adjust index if needed

# ---------------------------------------------------
# GRID CALIBRATION
# ---------------------------------------------------
x_min, x_max = 311, 467
y_min, y_max = 230, 434
x_min -= 25
x_max += 25
grid_x = [int(x_min + (x_max - x_min) * i / 3) for i in range(4)]
grid_y = [int(y_min + (y_max - y_min) * i / 3) for i in range(4)]

# ---------------------------------------------------
# Dobot physical positions
# ---------------------------------------------------
grid_positions = [
    [(255, -35, -40), (255, -15, -40), (255, 5, -40)],
    [(275, -35, -40), (275, -15, -40), (275, 5, -40)],
    [(295, -35, -40), (290, -15, -40), (290, 5, -40)]
]
source_position = (255, 104, -10)
current_source_z = source_position[2]
home_position = (253, -4, 34)

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
    w = check_winner(b)
    if w == 2:
        return 10
    elif w == 1:
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
    move = (-1, -1)
    for i in range(3):
        for j in range(3):
            if b[i][j] == 0:
                b[i][j] = 2
                val = minimax(b, 0, False)
                b[i][j] = 0
                if val > best_val:
                    best_val = val
                    move = (i, j)
    return move

# ---------------------------------------------------
# Dobot Movement
# ---------------------------------------------------
def pick_and_place_block(source, dest):
    global current_source_z
    sx, sy, _ = source
    dx, dy, dz = dest

    Dobot.move_to(*home_position)
    time.sleep(0.5)
    Dobot.move_to(255, 105, 10)
    Dobot.move_to(sx, sy, current_source_z)
    Dobot.suck(True)
    time.sleep(1)
    Dobot.move_to(sx, sy, current_source_z + 30)
    Dobot.move_to(*home_position)
    Dobot.move_to(dx, dy, dz)
    Dobot.suck(False)
    Dobot.move_to(*home_position)
    current_source_z -= 10
    print(f" Robot placed at {dest}")

# ---------------------------------------------------
# Color Detection
# ---------------------------------------------------
def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red1, upper_red1 = np.array([0,120,70]), np.array([10,255,255])
    lower_red2, upper_red2 = np.array([170,120,70]), np.array([180,255,255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

    lower_yellow, upper_yellow = np.array([20,100,100]), np.array([35,255,255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    red, yellow = [], []
    for mask, centers in [(mask_red, red), (mask_yellow, yellow)]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > 300:
                x, y, w, h = cv2.boundingRect(c)
                centers.append((x + w // 2, y + h // 2))
    return red, yellow

# ---------------------------------------------------
# Pixel to Grid
# ---------------------------------------------------
def pixel_to_grid(cx, cy):
    row, col = -1, -1
    for i in range(3):
        if grid_x[i] <= cx < grid_x[i + 1]:
            col = i
        if grid_y[i] <= cy < grid_y[i + 1]:
            row = i
    if row == -1 or col == -1:
        return (-1, -1)
    return 2 - row, col  # flip vertical

# ---------------------------------------------------
# Game Setup
# ---------------------------------------------------
choice = input("Who plays first? (human/robot): ").strip().lower()
if choice not in ["human", "robot"]:
    choice = "human"

if choice == "human":
    human_symbol, robot_symbol, player_turn = 1, 2, True
else:
    human_symbol, robot_symbol, player_turn = 2, 1, False

previous_human_blocks = set()
robot_has_played = False

print("Starting Tic Tac Toe...")
Dobot.move_to(*home_position)

# ---------------------------------------------------
# Main Loop (live camera, no threading)
# ---------------------------------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    red, yellow = detect_colors(frame)

    # Map detections to grid
    red_cells, yellow_cells = set(), set()
    for x, y in red:
        rc = pixel_to_grid(x, y)
        if rc != (-1, -1):
            red_cells.add(rc)
    for x, y in yellow:
        rc = pixel_to_grid(x, y)
        if rc != (-1, -1):
            yellow_cells.add(rc)

    # Draw grid
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

    # Determine current blocks
    current_human = red_cells if human_symbol == 1 else yellow_cells
    current_robot = yellow_cells if robot_symbol == 2 else red_cells
    new_human = current_human - previous_human_blocks

    # -----------------------------
    # HUMAN TURN
    # -----------------------------
    if player_turn:
        if len(new_human) == 1:
            row, col = list(new_human)[0]
            if board[row][col] == 0:
                board[row][col] = human_symbol
                previous_human_blocks = current_human
                player_turn = False
                robot_has_played = False
                print(f" Human placed at ({row},{col})")

                # ✅ Check winner immediately after human move
                winner = check_winner(board)
                if winner != 0:
                    print(" Human wins!" if winner == human_symbol else " Robot wins!")
                    cv2.putText(frame, f"{'Human' if winner == human_symbol else 'Robot'} wins!", (60,60),
                                font, 1, (0,255,0), 2)
                    cv2.imshow("Tic Tac Toe", frame)
                    cv2.waitKey(3000)
                    break
                elif not is_moves_left(board):
                    print("It's a draw!")
                    cv2.putText(frame, "Draw!", (150,60), font, 1, (0,255,255), 2)
                    cv2.imshow("Tic Tac Toe", frame)
                    cv2.waitKey(3000)
                    break
            else:
                print("Cell already occupied!")

    # -----------------------------
    # ROBOT TURN
    # -----------------------------
    elif not player_turn and not robot_has_played:
        print(" Robot's turn...")
        row, col = find_best_move(board)
        if row != -1 and col != -1:
            print(f" Robot plays at ({row},{col})")
            pick_and_place_block(source_position, grid_positions[row][col])
            board[row][col] = robot_symbol
        robot_has_played = True
        player_turn = True  # Give control back to human

        # ✅ Check winner immediately after robot move
        winner = check_winner(board)
        if winner != 0:
            print(" Human wins!" if winner == human_symbol else " Robot wins!")
            cv2.putText(frame, f"{'Human' if winner == human_symbol else 'Robot'} wins!", (60,60),
                        font, 1, (0,255,0), 2)
            cv2.imshow("Tic Tac Toe", frame)
            cv2.waitKey(3000)
            break
        elif not is_moves_left(board):
            print("It's a draw!")
            cv2.putText(frame, "Draw!", (150,60), font, 1, (0,255,255), 2)
            cv2.imshow("Tic Tac Toe", frame)
            cv2.waitKey(3000)
            break

    # -----------------------------
    # Show frame live
    # -----------------------------
    cv2.imshow("Tic Tac Toe", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
Dobot.close()
