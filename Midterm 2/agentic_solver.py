import cv2
import numpy as np
import time
from pydobot import Dobot
import sys
import traceback

# ========== CONFIGURATION ==========
CAMERA_ID = 0
PORT = "/dev/ttyACM0"
Z_WORK = -25  # Working height in mm
HOME_POS = (250, 0, 50, 0)
X_STEP, Y_STEP = 40, 40  # mm step sizes if needed for traversal
H_PATH = "homography.npy"

# ===================================
# ---------- AGENT CLASSES ----------
# ===================================

class VisionAgent:
    def __init__(self, cam_id=0):
        self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            raise Exception("Camera could not be opened.")
        print("[VisionAgent] Camera initialized.")

    def capture_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            raise Exception("Failed to capture frame from camera.")
        return frame

    def detect_maze(self, frame):
        """Detect maze lines and extract a binary maze map."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        # morphological closing to connect gaps
        kernel = np.ones((3,3), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            raise Exception("No maze detected.")

        # largest contour assumed to be maze boundary
        contour = max(contours, key=cv2.contourArea)
        approx = cv2.approxPolyDP(contour, 0.02*cv2.arcLength(contour, True), True)
        if len(approx) != 4:
            raise Exception("Maze boundary not rectangular.")

        # warp to top-down view
        pts = np.array([p[0] for p in approx], dtype=np.float32)
        pts = order_points(pts)
        dst_pts = np.array([[0,0],[400,0],[400,400],[0,400]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(pts, dst_pts)
        warp = cv2.warpPerspective(gray, M, (400,400))

        # threshold for binary maze
        _, binary = cv2.threshold(warp, 128, 255, cv2.THRESH_BINARY_INV)
        grid = cv2.resize(binary, (4,4), interpolation=cv2.INTER_AREA)
        grid = (grid > 128).astype(int)

        return warp, grid

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


class SolverAgent:
    """Simple BFS solver for 4x4 maze grid (0 = free, 1 = wall)."""
    def __init__(self):
        pass

    def solve(self, grid):
        start, goal = (0,0), (3,3)  # top-left → bottom-right
        q = [(start, [start])]
        visited = set()

        while q:
            (x,y), path = q.pop(0)
            if (x,y) == goal:
                return path
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = x+dx, y+dy
                if 0 <= nx < 4 and 0 <= ny < 4 and grid[nx,ny]==0 and (nx,ny) not in visited:
                    visited.add((nx,ny))
                    q.append(((nx,ny), path+[(nx,ny)]))
        raise Exception("No valid path found.")


class RobotAgent:
    def __init__(self, port, H_path):
        self.device = Dobot(port=port)
        self.H = np.load(H_path)
        print("[RobotAgent] Dobot connected and homography loaded.")

    def pixel_to_world(self, pixel):
        px = np.array([[pixel]], dtype=np.float32)
        world = cv2.perspectiveTransform(px, self.H)
        return world[0][0]

    def move_to(self, x, y, z=Z_WORK, r=0):
        self.device.move_to(x, y, z, r, wait=True)

    def follow_path(self, path, warp_shape):
        h, w = warp_shape
        for (gx, gy) in path:
            px = int((gy + 0.5) * w/4)
            py = int((gx + 0.5) * h/4)
            wx, wy = self.pixel_to_world((px, py))
            print(f"[RobotAgent] Moving to ({wx:.1f}, {wy:.1f})")
            self.move_to(wx, wy)
        print("[RobotAgent] Path execution complete.")

    def go_home(self):
        self.move_to(*HOME_POS)

    def disconnect(self):
        self.device.close()


class SupervisorAgent:
    def reflect(self, e):
        print("[SupervisorAgent] Error detected:", str(e))
        traceback.print_exc()
        print("Retry or adjust lighting/maze position.")


# ===================================
# ----------- MAIN LOOP -------------
# ===================================

def main():
    vision = VisionAgent(CAMERA_ID)
    solver = SolverAgent()
    robot = RobotAgent(PORT, H_PATH)
    supervisor = SupervisorAgent()

    try:
        frame = vision.capture_frame()
        warp, grid = vision.detect_maze(frame)
        print("[VisionAgent] Maze grid:\n", grid)

        path = solver.solve(grid)
        print("[SolverAgent] Path found:", path)

        robot.follow_path(path, warp.shape[:2])
        robot.go_home()

    except Exception as e:
        supervisor.reflect(e)

    finally:
        vision.release()
        robot.disconnect()


if __name__ == "__main__":
    main()
