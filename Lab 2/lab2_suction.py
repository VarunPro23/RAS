from pydobot import Dobot
import time

# --- Connect to Dobot ---
PORT = "COM7"
device = Dobot(port=PORT)

# --- Safe lift height and rotation ---
z_lift = 50  # safe height for horizontal moves
r = 0        # rotation angle of the end-effector

# --- Define pick and drop positions for 4 blocks ---
# Fill these coordinates with your actual positions
blocks = [
    {"pick": {"x": 270, "y": -10, "z": -45}, "drop": {"x": 270, "y": -98, "z": -45}},
    {"pick": {"x": 270, "y": 46, "z": -45}, "drop": {"x": 270, "y": -40, "z": -45}},
    {"pick": {"x": 330, "y": 46, "z": -45}, "drop": {"x": 330, "y": -40, "z": -45}},
    {"pick": {"x": 330.0, "y": -10, "z": -45}, "drop": {"x": 330, "y": -98.0, "z": -45}},

    {"pick": {"x": 270, "y": -98, "z": -45}, "drop": {"x": 270, "y": -10, "z": -45}},
    {"pick": {"x": 270, "y": -40, "z": -45}, "drop": {"x": 270, "y": 46, "z": -45}},
    {"pick": {"x": 330, "y": -40, "z": -45}, "drop": {"x": 330, "y": 46, "z": -45}},
    {"pick": {"x": 330.0, "y": -98, "z": -45}, "drop": {"x": 330, "y": -10.0, "z": -45}}

]

# --- Loop through each block ---
for i, block in enumerate(blocks, start=1):
    pick = block["pick"]
    drop = block["drop"]

    print(f"\nHandling Block {i}:")

    # Move above pick point
    device.move_to(pick["x"], pick["y"], z_lift, r, wait=True)
    time.sleep(1)

    # Move down to pick
    device.move_to(pick["x"], pick["y"], pick["z"], r, wait=True)
    time.sleep(1)

    # Enable suction
    print("Picking up pallet...")
    device.suck(True)
    time.sleep(2)

    # Lift up
    device.move_to(pick["x"], pick["y"], z_lift, r, wait=True)
    time.sleep(1)

    # Move above drop point
    device.move_to(drop["x"], drop["y"], z_lift, r, wait=True)
    time.sleep(1)

    # Move down to drop
    device.move_to(drop["x"], drop["y"], drop["z"], r, wait=True)
    time.sleep(1)

    # Disable suction
    print("Dropping pallet...")
    device.suck(False)
    time.sleep(2)

    # Lift up after drop
    device.move_to(drop["x"], drop["y"], z_lift, r, wait=True)
    time.sleep(1)

# --- Return to home / safe position ---
device.close()
print("\nAll blocks handled successfully!")
