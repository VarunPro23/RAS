import cv2
import numpy as np
import os

# ==================== USER CONFIG ====================
IMAGE_PATH = "/home/varunpro/RAS/Midterm2/charuco_test2.jpg"   # path to your saved board image
ARUCO_DICT = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH = 30   # mm (or any consistent unit)
MARKER_LENGTH = 22   # mm
# =====================================================

# Create ChArUco board
board = cv2.aruco.CharucoBoard_create(SQUARES_X, SQUARES_Y,
                                      SQUARE_LENGTH, MARKER_LENGTH,
                                      ARUCO_DICT)
params = cv2.aruco.DetectorParameters_create()

def detect_markers(gray):
    """Compatible with old and new OpenCV versions."""
    if hasattr(cv2.aruco, "ArucoDetector"):  # OpenCV ≥ 4.7
        detector = cv2.aruco.ArucoDetector(ARUCO_DICT, params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, ARUCO_DICT, parameters=params
        )
    return corners, ids, rejected


# =====================================================
# === LOAD IMAGE ===
# =====================================================
if not os.path.exists(IMAGE_PATH):
    raise FileNotFoundError(f"Image file '{IMAGE_PATH}' not found!")

print(f"[INFO] Loading saved image: {IMAGE_PATH}")
frame = cv2.imread(IMAGE_PATH)
if frame is None:
    raise RuntimeError("Could not read image file.")

gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# =====================================================
# === DETECTION ===
# =====================================================
corners, ids, rejected = detect_markers(gray)
if ids is None or len(ids) == 0:
    raise RuntimeError("No ArUco markers detected!")

cv2.aruco.drawDetectedMarkers(frame, corners, ids)

retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
    markerCorners=corners,
    markerIds=ids,
    image=gray,
    board=board
)

if retval is None or retval < 4:
    raise RuntimeError("Not enough ChArUco corners detected.")

# Draw corners and bounding box
for pt in charuco_corners:
    cv2.circle(frame, tuple(pt[0].astype(int)), 4, (0, 255, 0), -1)

x, y, w, h = cv2.boundingRect(charuco_corners)
cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

print("\nDetected ChArUco corners:")
for i, c in enumerate(charuco_corners):
    cx, cy = c[0]
    print(f"  Corner {i}: ({cx:.2f}, {cy:.2f})")
    cv2.putText(frame, str(i), (int(cx) + 5, int(cy) - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

# =====================================================
# === SAVE OUTPUTS (no GUI required) ===
# =====================================================
out_image = "charuco_detected.png"
out_corners = "charuco_corners.npy"

cv2.imwrite(out_image, frame)
np.save(out_corners, charuco_corners)

print(f"\n✅ Saved results:")
print(f" - Detected image with overlays: {os.path.abspath(out_image)}")
print(f" - Corner coordinates: {os.path.abspath(out_corners)}")

print("\nDone — no GUI used, so no Qt crash possible.")
