import time
from pydobot import Dobot

# ---------------------------------------------------
# Dobot Setup
# ---------------------------------------------------
Dobot = Dobot(port='/dev/ttyACM0')

# ---------------------------------------------------
# Coordinates
# ---------------------------------------------------
grid_positions = [
    [(250, -30, -40), (250, -10, -40), (250, 10, -40)],
    [(270, -30, -40), (270, -10, -40), (270, 10, -40)],
    [(290, -30, -40), (290, -10, -40), (290, 10, -40)]
]
source_position = (255, 110, -10)
current_source_z = source_position[2]
home_position = (225, 5, 20)

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
    Dobot.move_to(home_position)
    current_source_z -= 10
    time.sleep(1)

# ---------------------------------------------------
# Run Test
# ---------------------------------------------------
print("🔧 Starting pick-and-place loop test...")
for i in range(3):
    for j in range(3):
        print(f"Placing at grid cell ({i},{j}) -> {grid_positions[i][j]}")
        pick_and_place_block(source_position, grid_positions[i][j])
        time.sleep(2)
print("✅ Pick-and-place test completed.")
Dobot.close()
