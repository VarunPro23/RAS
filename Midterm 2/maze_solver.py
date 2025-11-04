"""
Autonomous Maze Solver for Dobot (robust vision version)
--------------------------------------------------------
- Loads homography.npy from calibration.
- Uses color camera feed with adaptive threshold + contour isolation.
- Accurately extracts 4x4 maze grid via cell segmentation.
- Automatically detects entrance/exit.
- If no maze is detected, throws clear error and exits safely.
"""

import cv2
import numpy as np
import time
from pydobot import Dobot
from collections import deque

# 
# Load saved homography
# 
try:
    H = np.load("homography.npy")
    print(" Loaded homography matrix from homography.npy")
except Exception as e:
    raise RuntimeError(" Could not load homography.npy. Run calibration first.") from e

# Dobot parameters
DOBOT_PORT = "/dev/ttyACM0"
HOME = (260.0, 0.0, 80.0, 0.0)
Z_TRAVEL = 60.0
Z_WORK = 5.0

FRAME_W, FRAME_H = 640, 480

# 
# Helper functions
# 
def pixel_to_world(pt_xy):
    """Convert pixel (x, y) to Dobot workspace coordinates using saved H."""
    p = np.array([[[pt_xy[0], pt_xy[1]]]], dtype=np.float32)
    out = cv2.perspectiveTransform(p, H)[0, 0]
    return float(out[0]), float(out[1])

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect

def detect_entrance_exit(binary_grid):
    """Detect openings (0-cells) on outer edges."""
    h, w = binary_grid.shape
    openings = []
    for x in range(w):
        if binary_grid[0, x] == 0: openings.append((x, 0))
        if binary_grid[h-1, x] == 0: openings.append((x, h-1))
    for y in range(h):
        if binary_grid[y, 0] == 0: openings.append((0, y))
        if binary_grid[y, w-1] == 0: openings.append((w-1, y))
    openings = list(dict.fromkeys(openings))
    if len(openings) >= 2:
        start, end = openings[0], openings[1]
        print(f" Detected entrance: {start}, exit: {end}")
        return start, end
    else:
        raise RuntimeError(" Maze not detected — no valid entrance or exit found.")

def solve_maze_from_grid(grid):
    """Solve maze grid (0=path,1=wall) using BFS."""
    start, end = detect_entrance_exit(grid)
    dirs = [(1,0),(-1,0),(0,1),(0,-1)]
    q = deque([start])
    visited = {start: None}

    while q:
        cur = q.popleft()
        if cur == end:
            break
        for dx, dy in dirs:
            nx, ny = cur[0]+dx, cur[1]+dy
            if 0 <= nx < 4 and 0 <= ny < 4 and grid[ny][nx]==0 and (nx,ny) not in visited:
                visited[(nx,ny)] = cur
                q.append((nx,ny))

    if end not in visited:
        raise RuntimeError(" Maze detected but no valid path found between entrance and exit.")

    path, node = [], end
    while node:
        path.append(node)
        node = visited[node]
    path.reverse()
    print(" Maze Path (grid):", path)
    return path

# 
# Main
# 
def main():
    # Step 1: Capture frame
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        raise RuntimeError(" Camera not available.")
    time.sleep(1)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError(" Failed to capture frame.")

    # Step 2: Preprocess image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    binary = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

    # # Step 3: Find largest rectangular contour (maze boundary)
    # contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # if not contours:
    #     raise RuntimeError(" No contours found — no maze detected.")
    # contour = max(contours, key=cv2.contourArea)

    # debug = frame.copy()
    # cv2.drawContours(debug, contours, -1, (0, 255, 0), 2)
    # cv2.imwrite("debug_contours.jpg", debug)
    # print(" Saved contour debug image: debug_contours.jpg")


    # approx = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)
    # if len(approx) != 4:
    #     raise RuntimeError(" Maze boundary not rectangular — reposition camera or check print.")
    # pts = np.array([p[0] for p in approx], dtype=np.float32)
    # pts = order_points(pts)
    # dst = np.array([[0,0],[400,0],[400,400],[0,400]], dtype=np.float32)
    # M = cv2.getPerspectiveTransform(pts, dst)
    # warp = cv2.warpPerspective(binary, M, (400,400))

    # Step 3: Find the maze outer boundary (force largest area)
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError(" No contours found — maze not detected.")

    # Pick the contour with the maximum area
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    print(f" Selected contour with area: {area:.0f}")

    # Approximate to smooth edges
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

   # --- Force a clean 4-corner box from the contour ---
    rect_box = cv2.minAreaRect(contour)      # center, (w,h), angle
    box_pts = cv2.boxPoints(rect_box)        # always returns 4 points
    pts = np.array(box_pts, dtype=np.float32)
    pts = order_points(pts)

    # Draw for debug
    debug = frame.copy()
    cv2.drawContours(debug, [pts.astype(int)], -1, (255, 0, 0), 3)
    cv2.imwrite("debug_maze_outer.jpg", debug)
    print(" Saved debug_maze_outer.jpg (blue = chosen maze boundary)")

    # Warp to a fixed 400×400 square
    dst = np.array([[0, 0], [400, 0], [400, 400], [0, 400]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts, dst)
    warp = cv2.warpPerspective(binary, M, (400, 400))

    # Step 4: Extract 4x4 grid by analyzing cells
    n = 4
    cell_size = warp.shape[0] // n
    grid = np.zeros((n, n), dtype=np.uint8)
    for i in range(n):
        for j in range(n):
            cell = warp[i*cell_size:(i+1)*cell_size, j*cell_size:(j+1)*cell_size]
            white = cv2.countNonZero(cell)
            grid[i, j] = 0 if white > (cell_size**2 * 0.2) else 1

    print(" Maze Grid (0=path,1=wall):\n", grid)

    # Step 5: Solve the maze
    try:
        path = solve_maze_from_grid(grid)
    except RuntimeError as e:
        print(str(e))
        cv2.putText(frame, "NO MAZE / NO PATH", (100, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
        cv2.imshow("Maze Detection Error", frame)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()
        return

    # Step 6: Draw overlay
    cell_w = FRAME_W / n
    cell_h = FRAME_H / n
    pixel_path = [((x + 0.5) * cell_w, (y + 0.5) * cell_h) for (x, y) in path]
    overlay = frame.copy()
    for i in range(1, len(pixel_path)):
        pt1 = tuple(map(int, pixel_path[i-1]))
        pt2 = tuple(map(int, pixel_path[i]))
        cv2.line(overlay, pt1, pt2, (0, 0, 255), 3)
    for (x, y) in pixel_path:
        cv2.circle(overlay, (int(x), int(y)), 6, (0, 255, 0), -1)
    result = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
    cv2.imshow("Maze Solution Path", result)
    print(" Showing maze with path overlay — press any key to continue.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Step 7: Convert pixel → world coords
    world_path = [pixel_to_world(p) for p in pixel_path]

    # Step 8: Move Dobot
    print("🔌 Connecting to Dobot...")
    bot = Dobot(port=DOBOT_PORT)
    time.sleep(1)
    bot.move_to(*HOME)
    time.sleep(1)
    for (X, Y) in world_path:
        print(f" Moving to ({X:.2f}, {Y:.2f})")
        bot.move_to(X, Y, Z_WORK, 0)
        time.sleep(0.5)
    bot.move_to(*HOME)
    bot.close()
    print(" Maze traversal complete.")

# 
if __name__ == "__main__":
    main()
