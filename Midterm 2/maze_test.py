import cv2
import time
import numpy as np

def initialize_camera():
    """Initialize camera as in the Tic-Tac-Toe project."""
    cap = cv2.VideoCapture('/dev/video0') 
    if not cap.isOpened():
        raise IOError("❌ Cannot open camera. Check connection.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    print("✅ Camera initialized.")
    return cap


def get_camera_frame(cap):
    """Capture a single frame from the camera (same as TicTacToe capture logic)."""
    ret, frame = cap.read()
    if not ret:
        raise IOError("❌ Failed to read from camera.")
    return frame


def detect_maze(frame, debug=True):
    """Detects whether a maze-like pattern is visible in the camera frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Find outer rectangular shapes (possible maze borders)
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    maze_contours = []

    for c in contours:
        approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
        area = cv2.contourArea(c)
        if len(approx) == 4 and area > 5000:
            maze_contours.append(approx)

    if debug:
        output = frame.copy()
        cv2.drawContours(output, maze_contours, -1, (0, 255, 0), 3)
        cv2.imshow("Maze Detection", output)

    return len(maze_contours) > 0


def main():
    cap = initialize_camera()

    print("📷 Press 'q' to quit.")
    while True:
        frame = get_camera_frame(cap)
        maze_found = detect_maze(frame, debug=True)

        if maze_found:
            print("🟢 Maze detected!")
            # time.sleep(30)
            # break
        else:
            print("🔴 No maze detected.")
            # time.sleep(30)
            # break

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
