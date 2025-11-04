import cv2
import numpy as np
import time
from pydobot import Dobot
from collections import deque

# Load saved homography
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

def pixel_to_world(pt_xy):
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

def detect_color_points(frame):
    """Detect green and red points in the image and return their pixel centers."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define HSV ranges
    green_lower = np.array([35, 80, 80])
    green_upper = np.array([85, 255, 255])
    red_lower1 = np.array([0, 120, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 120, 70])
    red_upper2 = np.array([180, 255, 255])

    mask_green = cv2.inRange(hsv, green_lower, green_upper)
    mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    def find_center(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] == 0:
            return None
        cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
        return (cx, cy)

    green_pt = find_center(mask_green)
    red_pt = find_center(mask_red)

    if green_pt is None or red_pt is None:
        raise RuntimeError("❌ Could not detect both red and green markers. Ensure they are visible.")

    print(f"✅ Detected GREEN at {green_pt}, RED at {red_pt}")
    return green_pt, red_pt

def choose_start_color():
    print("\nChoose start color:")
    print("1. Green ➜ Red")
    print("2. Red ➜ Green")
    choice = input("Enter 1 or 2: ").strip()
    if choice == "1":
        return "green"
    elif choice == "2":
        return "red"
    else:
        raise ValueError("Invalid choice. Please enter 1 or 2.")

def solve_maze_from_grid(grid, start, end):
    """Solve maze grid (0=path,1=wall) using BFS from start→end."""
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
        raise RuntimeError("❌ Maze detected but no valid path found between selected colors.")

    path, node = [], end
    while node:
        path.append(node)
        node = visited[node]
    path.reverse()
    print("✅ Maze Path (grid):", path)
    return path

def main():
    # Step 1: Capture frame
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        raise RuntimeError("Camera not available.")
    time.sleep(1)
    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError("Failed to capture frame.")

    # Step 2: Detect colored points
    green_pt, red_pt = detect_color_points(frame)
    start_color = choose_start_color()
    if start_color == "green":
        start_pixel, end_pixel = green_pt, red_pt
    else:
        start_pixel, end_pixel = red_pt, green_pt

    # Step 3: Preprocess image
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    binary = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)
    kernel = np.ones((3,3), np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

    # Step 4: Find outer boundary
    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise RuntimeError("No contours found — maze not detected.")
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    print(f"✅ Selected contour with area: {area:.0f}")
    rect_box = cv2.minAreaRect(contour)
    box_pts = cv2.boxPoints(rect_box)
    pts = np.array(box_pts, dtype=np.float32)
    pts = order_points(pts)
    dst = np.array([[0,0],[400,0],[400,400],[0,400]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts, dst)
    warp = cv2.warpPerspective(binary, M, (400,400))

    # Step 5: Map colors into grid
    n = 4
    cell_size = warp.shape[0] // n
    grid = np.zeros((n, n), dtype=np.uint8)
    for i in range(n):
        for j in range(n):
            cell = warp[i*cell_size:(i+1)*cell_size, j*cell_size:(j+1)*cell_size]
            white = cv2.countNonZero(cell)
            grid[i, j] = 0 if white > (cell_size**2 * 0.2) else 1

    print("🧩 Maze Grid (0=path,1=wall):\n", grid)

    # Convert colored points to grid positions
    invM = np.linalg.inv(M)
    def pixel_to_grid(px, py):
        p = np.array([[[px, py]]], dtype=np.float32)
        warped = cv2.perspectiveTransform(p, M)[0,0]
        gx, gy = int(warped[0] // cell_size), int(warped[1] // cell_size)
        return (gx, gy)

    start_grid = pixel_to_grid(*start_pixel)
    end_grid = pixel_to_grid(*end_pixel)
    print(f"Start grid: {start_grid}, End grid: {end_grid}")

    # Step 6: Solve path
    path = solve_maze_from_grid(grid, start_grid, end_grid)

    # Step 7: Draw overlay
    overlay = frame.copy()
    cell_w, cell_h = FRAME_W / n, FRAME_H / n
    pixel_path = [((x + 0.5) * cell_w, (y + 0.5) * cell_h) for (x, y) in path]
    for i in range(1, len(pixel_path)):
        cv2.line(overlay, tuple(map(int, pixel_path[i-1])), tuple(map(int, pixel_path[i])), (0, 0, 255), 3)
    for (x, y) in pixel_path:
        cv2.circle(overlay, (int(x), int(y)), 6, (0, 255, 0), -1)
    result = cv2.addWeighted(frame, 0.7, overlay, 0.3, 0)
    cv2.imshow("Maze Solution Path", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Step 8: Move Dobot
    print("🔌 Connecting to Dobot...")
    bot = Dobot(port=DOBOT_PORT)
    time.sleep(1)
    bot.move_to(*HOME)
    time.sleep(1)
    for (X, Y) in [pixel_to_world(p) for p in pixel_path]:
        print(f"🤖 Moving to ({X:.2f}, {Y:.2f})")
        bot.move_to(X, Y, Z_WORK, 0)
        time.sleep(0.5)
    bot.move_to(*HOME)
    bot.close()
    print("✅ Maze traversal complete.")

if __name__ == "__main__":
    main()
