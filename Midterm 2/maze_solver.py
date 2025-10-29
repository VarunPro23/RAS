"""
Autonomous Maze Solver for Dobot using saved homography (H)
------------------------------------------------------------
- Loads homography.npy from calibration.
- Captures maze inside the camera view.
- Automatically detects maze entrance/exit along edges.
- Solves the maze with BFS.
- Converts pixel → world coordinates using H.
- Moves Dobot along the solved path.

Dependencies:
  pip install opencv-python numpy pydobot
"""

import cv2
import numpy as np
import time
from pydobot import Dobot
from collections import deque

# ===============================================================
# Load saved homography
# ===============================================================
try:
    H = np.load("homography.npy")
    print("✅ Loaded homography matrix from homography.npy")
except Exception as e:
    raise RuntimeError("❌ Could not load homography.npy. Run calibration first.") from e

# Dobot parameters
DOBOT_PORT = "/dev/ttyACM0"
HOME = (260.0, 0.0, 80.0, 0.0)
Z_TRAVEL = 60.0
Z_WORK = 5.0

FRAME_W, FRAME_H = 640, 480

# ===============================================================
# Helper functions
# ===============================================================
def pixel_to_world(pt_xy):
    """Convert pixel (x, y) to Dobot workspace coordinates using saved H."""
    p = np.array([[[pt_xy[0], pt_xy[1]]]], dtype=np.float32)
    out = cv2.perspectiveTransform(p, H)[0, 0]
    return float(out[0]), float(out[1])

def detect_entrance_exit(binary_grid):
    """Detect openings (black cells) on outer edges of the maze grid."""
    h, w = binary_grid.shape
    openings = []

    # top and bottom rows
    for x in range(w):
        if binary_grid[0, x] == 0:  # open path
            openings.append((x, 0))
        if binary_grid[h-1, x] == 0:
            openings.append((x, h-1))

    # left and right columns
    for y in range(h):
        if binary_grid[y, 0] == 0:
            openings.append((0, y))
        if binary_grid[y, w-1] == 0:
            openings.append((w-1, y))

    # Pick first two distinct openings
    openings = list(dict.fromkeys(openings))  # remove duplicates
    if len(openings) >= 2:
        start, end = openings[0], openings[1]
        print(f"✅ Detected entrance: {start}, exit: {end}")
        return start, end
    else:
        print("⚠️ Could not find both entrance and exit — defaulting to corners.")
        return (0, 0), (w-1, h-1)

def solve_maze(binary):
    """Solve maze via BFS (black path = open, white wall = blocked)."""
    maze_resized = cv2.resize(binary, (4, 4), interpolation=cv2.INTER_AREA)
    grid = (maze_resized > 128).astype(np.uint8)

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

    # Backtrack
    path, node = [], end
    while node:
        path.append(node)
        node = visited[node]
    path.reverse()
    print("✅ Maze Path (grid):", path)
    return path

# ===============================================================
# Main
# ===============================================================
def main():
    # -------------------------------
    # Step 1: Capture maze image
    # -------------------------------
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        raise RuntimeError("❌ Camera not available.")
    time.sleep(1)

    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("❌ Failed to capture frame.")
    cap.release()

    # -------------------------------
    # Step 2: Preprocess for maze detection
    # -------------------------------
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    _, binary = cv2.threshold(blur, 127, 255, cv2.THRESH_BINARY_INV)

    cv2.imshow("Maze Binary", binary)
    cv2.waitKey(1000)
    cv2.destroyAllWindows()

    # -------------------------------
    # Step 3: Solve maze automatically
    # -------------------------------
    path = solve_maze(binary)

    # -------------------------------
    # Step 4: Convert grid path → pixels
    # -------------------------------
    cell_w = FRAME_W / 4
    cell_h = FRAME_H / 4
    pixel_path = [((x + 0.5) * cell_w, (y + 0.5) * cell_h) for (x, y) in path]

    # -------------------------------
    # Step 5: Convert pixels → world coords
    # -------------------------------
    world_path = [pixel_to_world(p) for p in pixel_path]

    # -------------------------------
    # Step 6: Move Dobot along the path
    # -------------------------------
    print("🔌 Connecting to Dobot...")
    bot = Dobot(port=DOBOT_PORT)
    time.sleep(1)

    bot.move_to(*HOME)
    time.sleep(1)

    for (X, Y) in world_path:
        print(f"🤖 Moving to ({X:.2f}, {Y:.2f})")
        bot.move_to(X, Y, Z_WORK, 0)
        time.sleep(0.5)

    bot.move_to(*HOME)
    bot.close()
    print("🏁 Maze traversal complete.")

if __name__ == "__main__":
    main()
