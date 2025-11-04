"""
Dobot + camera runner (matrix traversal, NOT diagonal)
- Fixed port: /dev/ttyACM0
- Live camera feed ON
- Traverse the 640x480 image like a matrix (boustrophedon/zig-zag scan)
- Map each pixel (via homography H) to real-world XY and move there at Z_WORK
- Return to HOME, turn camera OFF, exit

Tunable:
  X_STEP, Y_STEP   -> pixel stride for the scan (keep modest to avoid huge queues)
  Z_TRAVEL, Z_WORK -> safe heights

Deps: opencv-python, numpy, pyserial, pydobot
"""

import cv2
import numpy as np
import time
from pydobot import Dobot

# ----------- Fixed serial port -----------
DOBOT_PORT = "/dev/ttyACM0"

# ----------- Given calibration -----------
FRAME_W, FRAME_H = 640, 480
img_pts = np.array([[0, 0], [FRAME_W, 0], [FRAME_W, FRAME_H], [0, FRAME_H]], dtype=np.float32)
world_pts = np.array([
    [211, -113],   # img (0,   0)
    [211, 51],   # img (640, 0)
    [360, 49],  # img (0,   480)
    [348, -110],    # img (640, 480)
    
], dtype=np.float32)
H, _ = cv2.findHomography(img_pts, world_pts)
HINV = np.linalg.inv(H)
np.save("homography.npy", H)


# ----------- Motion params -----------
HOME      = (260.0,   0.0,  80.0, 0.0)  # (x, y, z, r)
Z_TRAVEL  = 60.0
Z_WORK    =  5.0
VELOCITY  = 150
ACCEL     = 150

# Keep these reasonable; (20,20) ~ 31x24 = 744 waypoints
# Smaller steps mean more points/time.
X_STEP    = 60
Y_STEP    = 60

# ----------- Helpers -----------
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
    # draw current point and progress
    cv2.putText(frame, f"Matrix scan 640x480 | step=({X_STEP},{Y_STEP}) | {progress}/{total}",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    if current_world is not None:
        qx, qy = world_to_pixel(current_world)
        cv2.circle(frame, (qx, qy), 6, (0, 0, 255), -1)

def wait_with_camera(bot, target_idx, cap, get_current=None, progress=(0,0)):
    """
    Keep the camera live while waiting for Dobot to reach 'target_idx'.
    Uses _get_queued_cmd_current_index if available; falls back to wait_for_cmd.
    """
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
            try:
                cur = bot._get_queued_cmd_current_index()
                if cur >= target_idx:
                    break
            except Exception:
                has_index = False
        else:
            if hasattr(bot, "wait_for_cmd"):
                bot.wait_for_cmd(target_idx)
            else:
                time.sleep(0.05)
            break
        time.sleep(0.01)

# ----------- Main -----------
def main():
    # Camera ON
    cap = cv2.VideoCapture('/dev/video0')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        raise RuntimeError("Camera not available.")

    # Robot ON (fixed port)
    bot = Dobot(port=DOBOT_PORT)

    # Speed setup across pydobot variants
    try:
        bot.speed(VELOCITY, ACCEL)
    except Exception:
        try:
            bot.set_ptp_common_params(VELOCITY, ACCEL)
        except Exception:
            pass

    # Build the boustrophedon (matrix) path in pixel space
    path_px = boustrophedon_path(FRAME_W, FRAME_H, X_STEP, Y_STEP)
    total_pts = len(path_px)

    current_world = None
    try:
        # Go HOME first
        idx = bot.move_to(*HOME)
        wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(0, total_pts))

        # Move above first point, descend to work height
        Xs, Ys = pixel_to_world(path_px[0])
        idx = bot.move_to(Xs, Ys, Z_TRAVEL, 0.0)
        wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(0, total_pts))

        idx = bot.move_to(Xs, Ys, Z_WORK, 0.0)
        wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(0, total_pts))

        # Scan the grid row-by-row at Z_WORK
        for i, (px, py) in enumerate(path_px, start=1):
            X, Y = pixel_to_world((px, py))
            current_world = (X, Y)
            idx = bot.move_to(X, Y, Z_WORK, 0.0)
            wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(i, total_pts))

        # Exit plane and return HOME
        if current_world is not None:
            idx = bot.move_to(current_world[0], current_world[1], Z_TRAVEL, 0.0)
            wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(total_pts, total_pts))

        idx = bot.move_to(*HOME)
        wait_with_camera(bot, idx, cap, get_current=lambda: current_world, progress=(total_pts, total_pts))

    finally:
        # Camera OFF; disconnect
        try: cap.release()
        except Exception: pass
        cv2.destroyAllWindows()
        try: bot.close()
        except Exception: pass

if __name__ == "__main__":
    main()


