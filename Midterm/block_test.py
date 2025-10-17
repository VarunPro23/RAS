import time
from pydobot import Dobot
import cv2

# ---------------------------------------------------
# Dobot Setup
# ---------------------------------------------------
Dobot = Dobot(port='/dev/ttyACM0')
cap = cv2.VideoCapture('/dev/video2') 

# ---------------------------------------------------
# Coordinates
# ---------------------------------------------------
grid_positions = [
    [(255, -35, -40), (255, -15, -40), (255, 5, -40)],
    # [(270, -30, -40), (270, -10, -40), (270, 10, -40)],
    # [(290, -30, -40), (290, -10, -40), (290, 10, -40)]
]

source_position = (255, 105, -10)
current_source_z = source_position[2]
home_position = (253, -4, 34)

# ---------------------------------------------------
# Pick & Place Logic
# ---------------------------------------------------
def pick_and_place_block(source, dest):
    global current_source_z
    sx, sy, _ = source
    dx, dy, dz = dest

    # Move to home first
    Dobot.move_to(*home_position)
    time.sleep(2)

    # Pick from current stack height
    Dobot.move_to(255,105,10)
    Dobot.move_to(sx, sy, current_source_z)
    Dobot.suck(True)
    time.sleep(2)

    # Lift slightly before moving
    Dobot.move_to(sx, sy, current_source_z + 30)

    # Move to destination
    Dobot.move_to(dx, dy, dz)
    Dobot.suck(False)
    time.sleep(2)
    print(f"🤖 Placed block at {dest}")

    # Return home
    Dobot.move_to(*home_position)
    current_source_z -= 10
    time.sleep(1)


# ---------------------------------------------------
# Display Loop Function
# ---------------------------------------------------
def show_camera_feed():
    """Continuously shows camera feed with grid overlay while Dobot operates."""
    ret, frame = cap.read()
    if not ret:
        return False

    # Draw a simple 3x3 overlay grid
    h, w, _ = frame.shape
    third_w, third_h = w // 3, h // 3
    for i in range(1, 3):
        cv2.line(frame, (i * third_w, 0), (i * third_w, h), (255, 255, 255), 1)
        cv2.line(frame, (0, i * third_h), (w, i * third_h), (255, 255, 255), 1)

    cv2.imshow("Pick and Place View", frame)
    return cv2.waitKey(1) & 0xFF != ord('q')

# ---------------------------------------------------
# Run Test
# ---------------------------------------------------
print("🔧 Starting pick-and-place loop test...")
show_camera_feed()
for i in range(3):
    for j in range(3):
        print(f"Placing at grid cell ({i},{j}) -> {grid_positions[i][j]}")
        pick_and_place_block(source_position, grid_positions[i][j])
        time.sleep(2)
print("✅ Pick-and-place test completed.")
Dobot.close()
