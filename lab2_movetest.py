from serial.tools import list_ports
from pydobot import Dobot

# List ports to confirm correct one
for p in list_ports.comports():
    print(p.device, p.description)

# Connect to the correct port (replace with your actual COM port, e.g., COM6)
device = Dobot(port="/dev/ttyACM0")

# Get pose (tuple)
pose = device.pose()
print("Raw pose tuple:", pose)

# Unpack the tuple (first 4 values are x, y, z, r)
x, y, z, r, *joints = pose

# Move 20 mm in x direction
device.move_to(x + 20, y, z, r, wait=False)
device.move_to(x, y, z, r, wait=True)

device.close()
