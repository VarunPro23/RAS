import cv2
import numpy as np
import time
from pydobot import Dobot

# ==============================================================
# 1️⃣ CAMERA INITIALIZATION
# ==============================================================
def initialize_camera():
    cap = cv2.VideoCapture('/dev/video0') 
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("✅ Camera initialized.")
    return cap


# ==============================================================
# 2️⃣ ARUCO MARKER DETECTION + HOMOGRAPHY
# ==============================================================
def detect_aruco_markers(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return corners, ids
    else:
        return None, None


def get_maze_topdown(frame, corners, ids):
    """Warp maze to a 400x400 top-down view using 4 ArUco markers (IDs 0–3)."""
    ids = ids.flatten()
    try:
        # Get corners in correct ID order
        ordered = [corners[np.where(ids == i)[0][0]][0][0] for i in range(4)]
        pts_src = np.array(ordered, dtype=np.float32)
        pts_dst = np.array([[0,0], [400,0], [0,400], [400,400]], dtype=np.float32)

        H, _ = cv2.findHomography(pts_src, pts_dst)
        maze_warped = cv2.warpPerspective(frame, H, (400, 400))
        return maze_warped, H
    except Exception as e:
        raise ValueError("❌ Could not warp maze. Check marker placement.") from e


# ==============================================================
# 3️⃣ MAZE SOLVER (4x4 GRID BFS)
# ==============================================================
def solve_maze(maze_img):
    gray = cv2.cvtColor(maze_img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

    # Downsample to 4×4 grid
    maze_resized = cv2.resize(binary, (4, 4), interpolation=cv2.INTER_AREA)
    maze_binary = (maze_resized > 128).astype(np.uint8)

    start = (0, 0)
    end = (3, 3)
    queue = [start]
    visited = {start: None}
    directions = [(1,0),(-1,0),(0,1),(0,-1)]

    while queue:
        current = queue.pop(0)
        if current == end:
            break
        for dx, dy in directions:
            nx, ny = current[0]+dx, current[1]+dy
            if 0 <= nx < 4 and 0 <= ny < 4 and maze_binary[ny, nx] == 0:
                if (nx, ny) not in visited:
                    visited[(nx, ny)] = current
                    queue.append((nx, ny))

    # Backtrack path
    path = []
    node = end
    while node:
        path.append(node)
        node = visited[node]
    path.reverse()
    print(f"✅ Maze Path: {path}")
    return path


# ==============================================================
# 4️⃣ PIXEL → DOBOT TRANSFORM (from ArUco markers)
# ==============================================================
def compute_affine_from_aruco(corners, ids, robot_points_dict):
    """Use ArUco marker centers + known robot coordinates to compute affine map."""
    pixel_points = []
    robot_points = []
    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in robot_points_dict:
            cx = np.mean(corners[i][0][:, 0])
            cy = np.mean(corners[i][0][:, 1])
            pixel_points.append([cx, cy])
            robot_points.append(robot_points_dict[marker_id])

    if len(pixel_points) >= 3:
        M = cv2.getAffineTransform(np.float32(pixel_points[:3]), np.float32(robot_points[:3]))
        print("✅ Affine transform computed from ArUco markers.")
        return M
    else:
        raise ValueError("Need at least 3 valid markers for affine transform.")


def pixel_to_dobot(x, y, M):
    p = np.array([[x, y, 1]], dtype=np.float32).T
    result = np.dot(M, p)
    return result[0][0], result[1][0]


# ==============================================================
# 5️⃣ DOBOT MOVEMENT
# ==============================================================
def move_dobot_along_path(dobot, path, maze_img, M):
    """Convert maze path (grid cells) to Dobot moves using transform M."""
    h, w, _ = maze_img.shape
    cell_w, cell_h = w / 4, h / 4

    for (cx, cy) in path:
        px = int((cx + 0.5) * cell_w)
        py = int((cy + 0.5) * cell_h)
        x_dobot, y_dobot = pixel_to_dobot(px, py, M)

        print(f"🤖 Moving to: X={x_dobot:.2f}, Y={y_dobot:.2f}")
        dobot.move_to(x_dobot, y_dobot, -20, r=0)
        time.sleep(0.5)

    print("✅ Maze traversal complete!")


# ==============================================================
# 6️⃣ MAIN PROGRAM
# ==============================================================
def main():
    # --- Known robot coordinates for each ArUco marker (mm) ---
    robot_points_dict = {
        0: [192, -118],  # top-left
        1: [206, 64],  # top-right
        2: [348, 48],  # bottom-left
        3: [338, -124]   # bottom-right
    }

    # --- Step 1: Initialize camera ---
    cap = initialize_camera()
    time.sleep(1)

    # --- Step 2: Capture frame and detect markers ---
    print("📸 Detecting ArUco markers...")
    ret, frame = cap.read()
    if not ret:
        raise IOError("Failed to capture camera frame.")

    corners, ids = detect_aruco_markers(frame)
    if ids is None or len(ids) < 4:
        raise ValueError("❌ Not all 4 ArUco markers detected. Check alignment.")

    # --- Step 3: Compute transform + warp maze ---
    M = compute_affine_from_aruco(corners, ids, robot_points_dict)
    maze_warped, _ = get_maze_topdown(frame, corners, ids)
    cv2.imshow("Warped Maze", maze_warped)
    cv2.waitKey(1000)

    # --- Step 4: Solve maze ---
    path = solve_maze(maze_warped)

    # --- Step 5: Connect to Dobot ---
    print("🔌 Connecting to Dobot...")
    dobot = Dobot(port="COM3")  # Adjust to your Dobot port

    # --- Step 6: Execute path ---
    move_dobot_along_path(dobot, path, maze_warped, M)

    # --- Step 7: Cleanup ---
    cap.release()
    cv2.destroyAllWindows()
    dobot.close()
    print("🏁 Task complete.")


if __name__ == "__main__":
    main()
