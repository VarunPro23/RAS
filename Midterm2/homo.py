"""
Dobot + camera runner (matrix traversal, NOT diagonal)
------------------------------------------------------
- Fixed port: /dev/ttyACM0
- Live camera feed ON
- Traverse the 640x480 image like a matrix (boustrophedon/zig-zag scan)
- Map each pixel (via homography H) to real-world XY and move there at Z_WORK
- Return to HOME, turn camera OFF, exit

Tunable:
  X_STEP, Y_STEP   -> pixel stride for the scan
  Z_TRAVEL, Z_WORK -> safe heights
"""

import cv2
import numpy as np
import time
from pydobot import Dobot

# Calibration / Homography
DOBOT_PORT = "/dev/ttyACM0"
FRAME_W, FRAME_H = 640, 480

img_pts = np.array([
    [153, 63],
    [484, 58],
    [218, 128],
    [358, 128],
    [226, 192],
    [351, 191],
    [288, 255],
], dtype=np.float64)

world_pts = np.array([
    [332.8875427246094,  50.575565338134766],
    [331.2615661621094, -94.13094329833984],
    [308.04815673828125,  27.38081169128418],
    [307.2852783203125, -34.91622543334961],
    [278.18890380859375,  24.748804092407227],
    [277.52886962890625, -31.535066604614258],
    [248.09780883789062,  -3.083724021911621]
], dtype=np.float64)

H, _ = cv2.findHomography(img_pts, world_pts)
HINV = np.linalg.inv(H)
np.save("homography.npy", H)

# Robot parameters
HOME      = (260.0, 0.0, 80.0, 0.0)
Z_TRAVEL  = 60.0
Z_WORK    = 5.0
VELOCITY  = 150
ACCEL     = 150

# Step size for traversal
X_STEP    = 60
Y_STEP    = 60

# Helper functions
def pixel_to_world(pt_xy):
    p = np.array([[[pt_xy[0], pt_xy[1]]]], dtype=np.float32)
    out = cv2.perspectiveTransform(p, H)[0, 0]
    return float(out[0]), float(out[1])

def world_to_pixel(xy):
    p = np.array([[[xy[0], xy[1]]]], dtype=np.float32)
    q = cv2.perspectiveTransform(p, HINV)[0, 0]
    return int(q[0]), int(q[1])

def boustrophedon_path(width, height, x_step, y_step):
    """Generate a zig-zag (row-wise) pixel path covering the image."""
    xs = list(range(0, width, x_step))
    if xs[-1] != width - 1:
        xs.append(width - 1)
    ys = list(range(0, height, y_step))
    if ys[-1] != height - 1:
        ys.append(height - 1)

    path = []
    for j, y in enumerate(ys):
        row = xs if j % 2 == 0 else list(reversed(xs))
        for x in row:
            path.append((x, y))
    return path

def draw_overlay(frame, current_world, progress, total):
    """Draw current progress and target location."""
    cv2.putText(frame, f"Matrix scan 640x480 | step=({X_STEP},{Y_STEP}) | {progress}/{total}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    if current_world is not None:
        qx, qy = world_to_pixel(current_world)
        cv2.circle(frame, (qx, qy), 6, (0, 0, 255), -1)

def wait_with_camera(bot, target_idx, cap, get_current=None, progress=(0,0)):
    """Keep the camera live while Dobot moves toward target."""
    has_index = hasattr(bot, "_get_queued_cmd_current_index")
    while True:
        ok, frame = cap.read()
        if ok:
            cur_world = get_current() if get_current else None
            draw_overlay(frame, cur_world, *progress)
            cv2.imshow("Dobot Live", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if has_index:
            cur = bot._get_queued_cmd_current_index()
            if cur >= target_idx:
                break
        else:
            if hasattr(bot, "wait_for_cmd"):
                bot.wait_for_cmd(target_idx)
            else:
                time.sleep(0.05)
            break
        time.sleep(0.01)

# Main Program
def main():
    # Camera ON
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        raise RuntimeError("Camera not available.")

    # Connect Dobot
    bot = Dobot(port=DOBOT_PORT)

    # Configure speed (use any available API)
    if hasattr(bot, "speed"):
        bot.speed(VELOCITY, ACCEL)
    elif hasattr(bot, "set_ptp_common_params"):
        bot.set_ptp_common_params(VELOCITY, ACCEL)

    # Build traversal path
    path_px = boustrophedon_path(FRAME_W, FRAME_H, X_STEP, Y_STEP)
    total_pts = len(path_px)
    current_world = None

    # 1. Go HOME
    idx = bot.move_to(*HOME)
    wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(0, total_pts))

    # 2. Move above first point
    Xs, Ys = pixel_to_world(path_px[0])
    idx = bot.move_to(Xs, Ys, Z_TRAVEL, 0.0)
    wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(0, total_pts))

    # 3. Lower to work height
    idx = bot.move_to(Xs, Ys, Z_WORK, 0.0)
    wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(0, total_pts))

    # 4. Zig-zag scan
    for i, (px, py) in enumerate(path_px, start=1):
        X, Y = pixel_to_world((px, py))
        current_world = (X, Y)
        idx = bot.move_to(X, Y, Z_WORK, 0.0)
        wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(i, total_pts))

    # 5. Lift and return HOME
    if current_world is not None:
        idx = bot.move_to(current_world[0], current_world[1], Z_TRAVEL, 0.0)
        wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(total_pts, total_pts))

    idx = bot.move_to(*HOME)
    wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(total_pts, total_pts))

    # 6. Close
    cap.release()
    cv2.destroyAllWindows()
    bot.close()

if __name__ == "__main__":
    main()
