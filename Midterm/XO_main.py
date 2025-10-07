"""
Dobot Tic-Tac-Toe controller
- Camera detects red/blue blocks and their centroids.
- Maps centroids to 3x3 grid cells using perspective transform obtained via clicking 4 pallet corners.
- Human = RED (X). Robot = BLUE (O).
- Robot picks a blue block from a blue stack pick point and places it on chosen cell.
- Adapt the DOBOt/gripper function placeholders to match your pydobot API.

Dependencies:
- opencv-python (cv2)
- numpy
- pydobot or your Dobot control library
"""

import cv2
import numpy as np
import time
import copy
from collections import deque

# ---------------------------
# Configuration (EDIT THESE)
# ---------------------------
CAM_INDEX = 0  # OpenCV camera index
COM_PORT = "COM3"  # or '/dev/ttyUSB0' on Linux; change as required

# Physical safe heights (mm) - change according to your Dobot/tooling
Z_APPROACH = 120   # height above pallet to travel safely
Z_PICK = 40        # height to pick a block (touching block)
Z_PLACE = 40       # height to place block
Z_SAFE = 150       # higher safe travel height

# Approximate times to wait for gripper open/close to complete (tweak)
GRIPPER_ACTION_TIME = 0.6

# Supply pick locations (real-world coordinates in mm) for blue (robot) and red (human supply)
# YOU MUST UPDATE THESE to the actual positions near the pallet where stacks are.
BLUE_SUPPLY_POS = (300, 50, Z_APPROACH)   # x,y,z to approach then lower to Z_PICK
RED_SUPPLY_POS  = (300, -50, Z_APPROACH)  # if you want robot to pick red maybe to move them

# Robot travel speed parameters might be needed by your pydobot wrapper — adjust as necessary.

# Minimum contour area to consider a valid block detection
MIN_CONTOUR_AREA = 300

# HSV color ranges (tweak to your lighting)
# Red needs two ranges because hue wraps around 180.
RED_LOWER1 = np.array([0, 120, 70])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([170, 120, 70])
RED_UPPER2 = np.array([180, 255, 255])

BLUE_LOWER = np.array([100, 120, 70])
BLUE_UPPER = np.array([140, 255, 255])

# ---------------------------
# IMPORT/INIT DOBOT (placeholder)
# ---------------------------
# Many pydobot wrappers use:
#   from pydobot import Dobot
#   dobot = Dobot(COM_PORT)
#
# Others use: Dobot(COM_PORT, debug=True)
# Also some use dobot.move_to(x,y,z) or dobot.move(x,y,z)
# Replace the functions in this script accordingly.

try:
    from pydobot import Dobot
    dobot = Dobot(COM_PORT)
except Exception as e:
    print("Warning: can't import or connect to Dobot automatically. Make sure 'pydobot' is installed and COM_PORT is correct.")
    dobot = None

# ---------------------------
# Helper: Dobot command wrappers (ADAPT THESE)
# ---------------------------
def dob_move_abs(x, y, z, r=0, wait=True):
    """
    Move dob to absolute (x,y,z); replace body with exact API call.
    """
    if dobot is None:
        print(f"[SIM MOVE] goto ({x:.1f},{y:.1f},{z:.1f})")
        time.sleep(0.3)
        return
    # Example for some wrappers:
    try:
        # Example: dobot.move_to(x, y, z, r) OR dobot.move(x,y,z) - change as needed
        if hasattr(dobot, "move_to"):
            dobot.move_to(x, y, z, r)
        elif hasattr(dobot, "move"):
            dobot.move(x, y, z)
        elif hasattr(dobot, "go_home"):
            # fallback: set_pose then move
            dobot.move_to(x, y, z)
        else:
            # Last resort: print
            print("Dobot SDK present but wrapper method unknown. Edit dob_move_abs().")
    except Exception as e:
        print("Dobot motion error:", e)
    if wait:
        time.sleep(0.5)

def gripper_close():
    """
    Close the gripper (or engage suction). Replace with your API call.
    """
    if dobot is None:
        print("[SIM GRIP] close")
        time.sleep(GRIPPER_ACTION_TIME)
        return
    # Example: dobot.set_gripper(1) or dobot.suck(1)
    try:
        if hasattr(dobot, "set_gripper"):
            dobot.set_gripper(1)
        elif hasattr(dobot, "vacuum_on"):
            dobot.vacuum_on()
        elif hasattr(dobot, "suck"):
            dobot.suck(1)
        else:
            print("No known gripper method found; edit gripper_close().")
    except Exception as e:
        print("Gripper close error:", e)
    time.sleep(GRIPPER_ACTION_TIME)

def gripper_open():
    """
    Open the gripper (or release suction). Replace with your API call.
    """
    if dobot is None:
        print("[SIM GRIP] open")
        time.sleep(GRIPPER_ACTION_TIME)
        return
    try:
        if hasattr(dobot, "set_gripper"):
            dobot.set_gripper(0)
        elif hasattr(dobot, "vacuum_off"):
            dobot.vacuum_off()
        elif hasattr(dobot, "suck"):
            dobot.suck(0)
        else:
            print("No known gripper method found; edit gripper_open().")
    except Exception as e:
        print("Gripper open error:", e)
    time.sleep(GRIPPER_ACTION_TIME)

# ---------------------------
# Image / calibration utilities
# ---------------------------
def click_points_for_corners(window_name, frame, num_points=4):
    """
    Let user click `num_points` on frame; returns list of (x,y) in pixel coords.
    """
    points = []

    clone = frame.copy()
    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append((x, y))
            cv2.circle(clone, (x, y), 6, (0,255,0), -1)
            cv2.imshow(window_name, clone)

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_cb)

    print(f"Please click {num_points} corners on the image window in order (clockwise or counterclockwise). Press 'c' to confirm when done.")
    while True:
        cv2.imshow(window_name, clone)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('c') and len(points) >= num_points:
            break
        if key == ord('q'):
            break
    cv2.setMouseCallback(window_name, lambda *args: None)
    return points

def compute_homography(pixel_corners, real_corners):
    """
    pixel_corners: list of 4 (x,y) clicked on image (ordered)
    real_corners: list of 4 (X,Y) physical coordinates in mm (ordered same as pixels)
    returns a 3x3 homography matrix to map pixel -> real-world XY
    """
    pts_src = np.array(pixel_corners, dtype=np.float32)
    pts_dst = np.array(real_corners, dtype=np.float32)
    H, status = cv2.findHomography(pts_src, pts_dst)
    return H

def pixel_to_world(homography, px, py):
    """
    Map pixel (px,py) to real world XY using homography.
    """
    pt = np.array([ [px, py, 1.0] ]).T
    world = homography.dot(pt)
    world = world / world[2,0]
    return float(world[0,0]), float(world[1,0])

# ---------------------------
# Grid helpers
# ---------------------------
def grid_index_from_world(x, y, grid_origin, cell_size):
    """
    Convert world XY to grid index (0..8). grid_origin is the world XY of grid cell (0,0) top-left.
    cell_size is (cell_w, cell_h) in mm.
    Returns index (row*3 + col) or None if outside.
    """
    ox, oy = grid_origin
    cx, cy = cell_size
    col = int((x - ox) / cx)
    row = int((y - oy) / cy)
    if 0 <= row < 3 and 0 <= col < 3:
        return row * 3 + col
    return None

def world_coord_for_cell(grid_origin, cell_size, cell_index):
    row = cell_index // 3
    col = cell_index % 3
    cx, cy = cell_size
    x = grid_origin[0] + (col + 0.5) * cx
    y = grid_origin[1] + (row + 0.5) * cy
    return (x, y)

# ---------------------------
# Color detection
# ---------------------------
def detect_color_centroids(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # RED mask (two ranges)
    mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
    mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    # Clean masks
    kernel = np.ones((5,5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

    def mask_to_centroids(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_CONTOUR_AREA:
                continue
            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centers.append(((cx, cy), area, cnt))
        return centers

    red_centers = mask_to_centroids(red_mask)
    blue_centers = mask_to_centroids(blue_mask)
    return red_centers, blue_centers, red_mask, blue_mask

# ---------------------------
# Tic-Tac-Toe game logic (minimax)
# ---------------------------
def check_winner(board):
    wins = [(0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(2,5,8),(0,4,8),(2,4,6)]
    for a,b,c in wins:
        if board[a] and board[a] == board[b] == board[c]:
            return board[a]
    if all(board):
        return "draw"
    return None

def minimax(board, player, ai_player, hu_player):
    winner = check_winner(board)
    if winner == ai_player:
        return 1, None
    if winner == hu_player:
        return -1, None
    if winner == "draw":
        return 0, None

    moves = []
    for i in range(9):
        if board[i] is None:
            newb = board.copy()
            newb[i] = player
            score, _ = minimax(newb, hu_player if player == ai_player else ai_player, ai_player, hu_player)
            moves.append((score, i))
    if player == ai_player:
        # maximize
        best = max(moves, key=lambda x: x[0])
    else:
        best = min(moves, key=lambda x: x[0])
    return best

def ai_choose_move(board, ai_player="O", hu_player="X"):
    _, move = minimax(board, ai_player, ai_player, hu_player)
    return move

# ---------------------------
# Robot pick and place sequence
# ---------------------------
def pick_block_from_supply(supply_pos):
    # supply_pos: (x,y,z_approach)
    x_approach, y_approach, z_app = supply_pos
    # Move above supply
    dob_move_abs(x_approach, y_approach, Z_SAFE)
    dob_move_abs(x_approach, y_approach, z_app)
    dob_move_abs(x_approach, y_approach, Z_PICK)
    gripper_close()
    dob_move_abs(x_approach, y_approach, Z_SAFE)

def place_block_at(x, y):
    # move to approach over (x,y)
    dob_move_abs(x, y, Z_SAFE)
    dob_move_abs(x, y, Z_APPROACH)
    dob_move_abs(x, y, Z_PLACE)  # lower to place
    gripper_open()
    dob_move_abs(x, y, Z_SAFE)

def pick_and_place_from_supply(supply_pos, dest_xy):
    pick_block_from_supply(supply_pos)
    # dest_xy is tuple X,Y in mm; travel low then release
    dx, dy = dest_xy
    place_block_at(dx, dy)

# ---------------------------
# Main flow
# ---------------------------
def main():
    print("Starting Tic-Tac-Toe Dobot Controller")
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        print("Failed to open camera index", CAM_INDEX)
        return

    # Get initial frame for calibration
    ret, frame = cap.read()
    if not ret:
        print("Camera read failed.")
        return

    # Step 1: calibration - get pixel corners of pallet
    window_name = "Calibration - Click 4 pallet corners (press 'c' to confirm)"
    pixel_corners = click_points_for_corners(window_name, frame, 4)
    cv2.destroyWindow(window_name)

    if len(pixel_corners) < 4:
        print("Calibration aborted.")
        return

    # Step 2: ask user to provide real-world coordinates (mm) for those four corners
    print("You clicked 4 pixel corners. Now enter corresponding real-world XY (in mm) for each corner in the SAME ORDER.")
    real_corners = []
    for i, p in enumerate(pixel_corners):
        print(f"Corner {i+1} pixel {p}: enter X Y in mm (e.g. 100 200): ")
        # For convenience, we can ask user to type values
        vals = input(f"Corner {i+1} X Y: ")
        try:
            X, Y = [float(x) for x in vals.strip().split()]
        except:
            print("Invalid input; using 0,0. Please restart and enter correct coordinates.")
            return
        real_corners.append((X, Y))

    # compute homography mapping pixels -> real-world XY
    H = compute_homography(pixel_corners, real_corners)
    if H is None:
        print("Homography failed.")
        return

    # From real_corners we can derive grid origin and cell size if the corners were the outer corners of the grid.
    # User should supply corners of outermost pallet rectangle:
    # We'll compute bounding box and divide into 3x3 for the pallet grid.
    xs = [c[0] for c in real_corners]
    ys = [c[1] for c in real_corners]
    minx, maxx = min(xs), max(xs)
    miny, maxy = min(ys), max(ys)
    grid_origin = (minx, miny)  # top-left
    cell_w = (maxx - minx) / 3.0
    cell_h = (maxy - miny) / 3.0
    cell_size = (cell_w, cell_h)
    print("Grid origin:", grid_origin, "cell size (mm):", cell_size)

    # Initialize game state: None = empty, "X" = human (red), "O" = robot (blue)
    board = [None] * 9
    last_seen_cells = [None] * 9  # memorize last seen occupant to detect change events

    print("Calibration complete. Starting main loop. Press 'q' in the camera window to quit.")
    time.sleep(1.0)

    # Main loop
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame read failed.")
            break

        red_centers, blue_centers, red_mask, blue_mask = detect_color_centroids(frame)

        # visualize detections and compute pixel->world mapping
        display = frame.copy()
        for (c, _, cnt) in red_centers:
            cv2.circle(display, c, 6, (0,0,255), -1)
        for (c, _, cnt) in blue_centers:
            cv2.circle(display, c, 6, (255,0,0), -1)

        # map centroid pixels to world coords and to grid cells
        detected_cells_state = [None] * 9
        # process red
        for (px,py), area, cnt in red_centers:
            wx, wy = pixel_to_world(H, px, py)
            idx = grid_index_from_world(wx, wy, grid_origin, cell_size)
            if idx is not None:
                detected_cells_state[idx] = "X"
                cv2.putText(display, "R", (px+5,py+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255),1)
        # process blue
        for (px,py), area, cnt in blue_centers:
            wx, wy = pixel_to_world(H, px, py)
            idx = grid_index_from_world(wx, wy, grid_origin, cell_size)
            if idx is not None:
                detected_cells_state[idx] = "O"
                cv2.putText(display, "B", (px+5,py+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),1)

        # Show grid overlay for visualization
        for r in range(4):
            # horizontal lines in pixels: map world grid lines back to pixels may require inverse homography
            # We'll simply draw approximate cell boundaries by mapping world grid boundary points to pixels via inverse H
            pass  # keep simple; rely on user calibration

        cv2.imshow("Dobot TicTacToe", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quitting.")
            break

        # detect new human moves by comparing detected_cells_state to last_seen_cells
        for idx in range(9):
            prev = last_seen_cells[idx]
            now = detected_cells_state[idx]
            if prev != now:
                # update last_seen immediately to avoid repeated triggers
                last_seen_cells[idx] = now

        # Update board from detected state
        # Only accept a human move if previously the cell was empty (None) and now contains "X"
        # This helps avoid reacting to partial occlusions.
        # Detect transitions: empty -> X
        for idx in range(9):
            if board[idx] is None and detected_cells_state[idx] == "X":
                # new human move found
                board[idx] = "X"
                print(f"Human moved to cell {idx}")
                # after human move, check win/draw
                w = check_winner(board)
                if w:
                    print("Game ended after human move. Winner:", w)
                    # you might want to handle reset here (not implemented)
                else:
                    # AI turn
                    ai_move = ai_choose_move(board, ai_player="O", hu_player="X")
                    if ai_move is None:
                        print("No move for AI (draw?)")
                    else:
                        print("AI chooses", ai_move)
                        # compute world coordinate for AI placement
                        wx, wy = world_coord_for_cell(grid_origin, cell_size, ai_move)
                        # pick a blue block from supply then place it
                        pick_and_place_from_supply(BLUE_SUPPLY_POS, (wx, wy))
                        # update board
                        board[ai_move] = "O"
                        # small wait and then continue
                        time.sleep(0.5)
                # break to debounce multiple human placements detected same frame
                break

    cap.release()
    cv2.destroyAllWindows()
    # optionally close dobot connection
    if dobot is not None:
        try:
            dobot.close()
        except:
            pass
    print("Program ended.")

if __name__ == "__main__":
    main()
