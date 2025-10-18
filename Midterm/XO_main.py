import cv2
import numpy as np
import time
from pydobot import Dobot
import math

# Connect to Dobot
Dobot = Dobot(port="/dev/ttyACM0") 

# Open camera
cap = cv2.VideoCapture("/dev/video2")

# Grid Calibration
x_min, x_max = 311, 467
y_min, y_max = 230, 434
x_min -= 25
x_max += 25
grid_x = [int(x_min + (x_max - x_min) * i / 3) for i in range(4)]
grid_y = [int(y_min + (y_max - y_min) * i / 3) for i in range(4)]

# Grid Position Coordinates
grid_positions = [
    [(295, 5, -38), (295, -15, -38), (295, -35, -38)],   # row 0 (BOTTOM)
    [(275, 5, -38), (275, -15, -38), (275, -35, -38)],   # row 1 (MIDDLE)
    [(255, 5, -38), (255, -15, -38), (255, -35, -38)]    # row 2 (TOP)
]

# Source positions for block stacks (red / yellow)
source_position_red = (255, 104, -6)
source_position_yellow = (255, -112, -6)

red_current_source_z = source_position_red[2]
yellow_current_source_z = source_position_yellow[2]
home_position = (245, -1.5, 39)

# Game board
board = [[0, 0, 0], 
         [0, 0, 0], 
         [0, 0, 0]]

# Game logic
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

def evaluate(b, robot_sym, human_sym):
    winner = check_winner(b)
    if winner == robot_sym:
        return 10
    elif winner == human_sym:
        return -10
    return 0

# Minimax algorithm to find best move
def minimax(b, depth, is_max, robot_sym, human_sym):
    score = evaluate(b, robot_sym, human_sym)
    # prefer faster wins / slower losses
    if score != 0:
        return score - depth if score > 0 else score + depth
    if not is_moves_left(b):
        return 0

    if is_max:
        best = -math.inf
        for i in range(3):
            for j in range(3):
                if b[i][j] == 0:
                    b[i][j] = robot_sym
                    best = max(best, minimax(b, depth + 1, False, robot_sym, human_sym))
                    b[i][j] = 0
        return best
    else:
        best = math.inf
        for i in range(3):
            for j in range(3):
                if b[i][j] == 0:
                    b[i][j] = human_sym
                    best = min(best, minimax(b, depth + 1, True, robot_sym, human_sym))
                    b[i][j] = 0
        return best

def find_best_move(b, robot_sym, human_sym):
    best_val = -math.inf
    best_move = (-1, -1)
    for i in range(3):
        for j in range(3):
            if b[i][j] == 0:
                b[i][j] = robot_sym
                move_val = minimax(b, 0, False, robot_sym, human_sym)
                b[i][j] = 0
                if move_val > best_val:
                    best_val = move_val
                    best_move = (i, j)
    return best_move

# Dobot movement to pick and place blocks
def pick_and_place_block(source, dest):
    global red_current_source_z, yellow_current_source_z
    sx, sy, _ = source
    dx, dy, dz = dest

    if source == source_position_red:
        current_source_z = red_current_source_z
    else:
        current_source_z = yellow_current_source_z

    Dobot.move_to(*home_position)
    time.sleep(2)

    Dobot.move_to(sx, sy, current_source_z + 30)
    Dobot.move_to(sx, sy, current_source_z)
    Dobot.suck(True)
    time.sleep(2)

    Dobot.move_to(sx, sy, current_source_z + 30)
    Dobot.move_to(*home_position)
    time.sleep(2)

    Dobot.move_to(dx, dy, dz+50)
    time.sleep(1)
    Dobot.move_to(dx, dy, dz)
    Dobot.suck(False)
    Dobot.move_to(*home_position)
    time.sleep(2)

    if source == source_position_red:
        red_current_source_z -= 10       # Reducing the source stack's height by 10mm to make it autonomous
    else:
        yellow_current_source_z -= 10
    time.sleep(1)
    print(f" Robot placed at {dest}")

# Color detection
def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(
        hsv, lower_red2, upper_red2
    )
    lower_yellow, upper_yellow = np.array([20, 100, 100]), np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    red, yellow = [], []
    for mask, centers in [(mask_red, red), (mask_yellow, yellow)]:
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for c in contours:
            if cv2.contourArea(c) > 300:
                x, y, w, h = cv2.boundingRect(c)
                centers.append((x + w // 2, y + h // 2))
    return red, yellow

# Pixel to grid
def pixel_to_grid(cx, cy):
    row, col = -1, -1
    for i in range(3):
        if grid_x[i] <= cx < grid_x[i + 1]:
            col = i
        if grid_y[i] <= cy < grid_y[i + 1]:
            row = i
    if row == -1 or col == -1:
        return (-1, -1)
    return row, col

# Game setup
choice = input("Who plays first? (human/robot): ").strip().lower()
if choice not in ["human", "robot"]:
    choice = "human"

while True:
    color_choice = input("Choose your color (red/yellow): ").strip().lower()
    if color_choice in ["red", "yellow"]:
        break
    print("Invalid choice! Please enter either 'red' or 'yellow'.")

if color_choice == "red":
    human_symbol = 1
    robot_symbol = 2
    human_source = source_position_red
    robot_source = source_position_yellow
else:
    human_symbol = 2
    robot_symbol = 1
    human_source = source_position_yellow
    robot_source = source_position_red

player_turn = True if choice == "human" else False
robot_has_played = False

previous_human_blocks = set()
previous_all_blocks = set()
pending_human_move = None
stable_frames = 0
required_stability = 3
wrong_pending = None
wrong_stable = 0
wrong_required = 3

print(f"\nStarting Tic Tac Toe...")
print(f"You are playing as {'RED' if human_symbol == 1 else 'YELLOW'}.")
print(f"{'You' if player_turn else 'Robot'} will play first.\n")
Dobot.move_to(*home_position)

for _ in range(10):
    ret, frame = cap.read()
    if not ret:
        continue

    #Draw the grid and coordinate labels
    for x in grid_x:
        cv2.line(frame, (x, y_min), (x, y_max), (255, 255, 255), 1)
    for y in grid_y:
        cv2.line(frame, (x_min, y), (x_max, y), (255, 255, 255), 1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    for i in range(3):
        for j in range(3):
            cx = int((grid_x[j] + grid_x[j + 1]) / 2)
            cy = int((grid_y[i] + grid_y[i + 1]) / 2)
            cv2.putText(frame, f"({i},{j})", (cx - 25, cy + 5), font, 0.5, (255, 255, 255), 1)

    # Display the prepared frame
    cv2.imshow("Tic Tac Toe", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Camera ready. Starting game...\n")

# Main loop
while True:
    ret, frame = cap.read()
    if not ret:
        break

    red, yellow = detect_colors(frame)
    red_cells, yellow_cells = set(), set()
    for x, y in red:
        rc = pixel_to_grid(x, y)
        if rc != (-1, -1):
            red_cells.add(rc)
    for x, y in yellow:
        rc = pixel_to_grid(x, y)
        if rc != (-1, -1):
            yellow_cells.add(rc)

    for x in grid_x:
        cv2.line(frame, (x, y_min), (x, y_max), (255, 255, 255), 1)
    for y in grid_y:
        cv2.line(frame, (x_min, y), (x_max, y), (255, 255, 255), 1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    for i in range(3):
        for j in range(3):
            cx = int((grid_x[j] + grid_x[j + 1]) / 2)
            cy = int((grid_y[i] + grid_y[i + 1]) / 2)
            cv2.putText(
                frame, f"({i},{j})", (cx - 25, cy + 5), font, 0.5, (255, 255, 255), 1
            )

    current_human = red_cells if human_symbol == 1 else yellow_cells
    current_robot = yellow_cells if robot_symbol == 2 else red_cells
    current_all = red_cells | yellow_cells
    new_cells = current_all - previous_all_blocks

    # Human Turn
    if player_turn:
        if len(new_cells) == 0:
            pending_human_move = None
            stable_frames = 0
            wrong_pending = None
            wrong_stable = 0

        # Edge Case 1
        elif len(new_cells) > 1:
            print("Multiple new blocks detected! Only one move allowed.")      
            time.sleep(1)
            continue

        else:
            (row, col) = list(new_cells)[0]

            # Determine correct color
            if human_symbol == 1:
                expected_cells = red_cells
                color_name = "RED"
            else:
                expected_cells = yellow_cells
                color_name = "YELLOW"

            # Wrong color case (Edge Case 2)
            if (row, col) not in expected_cells:
                if wrong_pending == (row, col):
                    wrong_stable += 1
                else:
                    wrong_pending = (row, col)
                    wrong_stable = 1

                if wrong_stable >= wrong_required:
                    print(
                        f" Wrong color block used! You are playing {color_name}. "
                        f"Please remove the piece at ({row},{col})."
                    )
                    wrong_stable = 0
                continue

            # Stability check for correct move
            if pending_human_move == (row, col):
                stable_frames += 1
            else:
                pending_human_move = (row, col)
                stable_frames = 1
            if stable_frames < required_stability:
                continue

            if board[row][col] == 0:
                board[row][col] = human_symbol
                previous_human_blocks = current_human
                previous_all_blocks = current_all
                player_turn = False
                robot_has_played = False
                pending_human_move = None
                stable_frames = 0
                wrong_pending = None
                wrong_stable = 0
                print(f"Human placed at ({row},{col})")

                winner = check_winner(board)
                if winner != 0:
                    print(" Human wins!" if winner == human_symbol else " Robot wins!")
                    cv2.putText(
                        frame,
                        f"{'Human' if winner == human_symbol else 'Robot'} wins!",
                        (60, 60),
                        font,
                        1,
                        (0, 255, 0),
                        2,
                    )
                    cv2.imshow("Tic Tac Toe", frame)
                    cv2.waitKey(3000)
                    break
                elif not is_moves_left(board):
                    print("It's a draw!")
                    cv2.putText(
                        frame,
                        "Draw!",
                        (150, 60),
                        font,
                        1,
                        (0, 255, 255),
                        2,
                    )
                    cv2.imshow("Tic Tac Toe", frame)
                    cv2.waitKey(3000)
                    break
            else:
                print(" Cell already occupied!")

    # Robot Turn
    elif not player_turn and not robot_has_played:
        print("🤖 Robot's turn...")
        row, col = find_best_move(board, robot_symbol, human_symbol)
        if row != -1 and col != -1:
            print(f" Robot plays at ({row},{col})")
            pick_and_place_block(robot_source, grid_positions[row][col])
            board[row][col] = robot_symbol
            previous_all_blocks = previous_all_blocks | {(row, col)}
        robot_has_played = True
        player_turn = True

        winner = check_winner(board)
        if winner != 0:
            print(" Human wins!" if winner == human_symbol else " Robot wins!")
            cv2.putText(
                frame,
                f"{'Human' if winner == human_symbol else 'Robot'} wins!",
                (60, 60),
                font,
                1,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Tic Tac Toe", frame)
            cv2.waitKey(3000)
            break
        elif not is_moves_left(board):
            print("It's a draw!")
            cv2.putText(frame, "Draw!", (150, 60), font, 1, (0, 255, 255), 2)
            cv2.imshow("Tic Tac Toe", frame)
            cv2.waitKey(3000)
            break

    # Show frame
    cv2.imshow("Tic Tac Toe", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
Dobot.close()
