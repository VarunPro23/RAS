import time
import math
import numpy as np
from pydobot import Dobot
from ht import forward_kinematics, inverse_kinematics

robot = Dobot(port="/dev/ttyACM0") 
time.sleep(2)

# Robot link dimensions (in mm)
a1 = 53.5
a2 = 150
a3 = 150
z_offset = 53.5

# Test joint angles (degrees)
joint_tests = [
    (0, 0, 0),
    (90, 20, 30),
    (-2, 25, 23),
    (31, 52, 42),
    (27, 52, 24),
]

# Test Cartesian positions (mm)
position_tests = [
    (300, 50, 100),
    (280, -195, 15),
    (35, 270, -33)
]

# Forward and Inverse Kinematics Test – Joint Inputs
for i, (j1, j2, j3) in enumerate(joint_tests, 1):
    print(f"\n{'='*60}")
    print(f"TEST {i}: Joint Input Mode")
    print(f"{'='*60}")

    # Convert to radians
    t1, t2, t3 = np.deg2rad([j1, j2, j3])

    # Forward kinematics calculation
    px, py, pz = forward_kinematics(t1, t2, t3, a1, a2, a3)

    print(f"Given Joint Angles: [{j1}, {j2}, {j3}]")
    print(f"Computed Position (mm): ({px:.3f}, {py:.3f}, {pz:.3f})")

    # Move robot using joint mode
    robot.move_to(j1, j2, j3, mode=4)
    time.sleep(2)

    # Retrieve actual pose and joint data
    pose, joints = robot.get_pose()
    rx, ry, rz, rrot = pose
    rj1, rj2, rj3, rj4 = joints

    print(f"Robot Feedback Position (mm): ({rx:.2f}, {ry:.2f}, {rz:.2f})")
    print(f"Robot Joint Angles : ({rj1:.2f}, {rj2:.2f}, {rj3:.2f}, {rj4:.2f})")

    # Inverse kinematics test using robot feedback
    sol = inverse_kinematics(rx, ry, rz)
    th1, th2, th3 = np.rad2deg(sol)

    print(f"Inverse Kinematics Solution : ({th1:.3f}, {th2:.3f}, {th3:.3f})")

    # Compare with measured joint values
    err = np.sqrt((rj1 - th1)**2 + (rj2 - th2)**2 + (rj3 - th3)**2)
    print(f"Joint Error Magnitude: {err:.4f}°")

    print(f"{'='*60}\n")
    time.sleep(4)

# Forward and Inverse Kinematics Test – Position Inputs
for i, (px, py, pz) in enumerate(position_tests, 1):
    print(f"\n{'='*60}")
    print(f"TEST {i}: Cartesian Input Mode")
    print(f"{'='*60}")

    print(f"Target Position (mm): ({px}, {py}, {pz})")

    # Move robot directly to position
    robot.move_to(px, py, pz, 0)
    time.sleep(2)

    # Retrieve robot pose
    pose, joints = robot.get_pose()
    rx, ry, rz, rrot = pose
    rj1, rj2, rj3, rj4 = joints

    print(f"Robot Feedback Position (mm): ({rx:.3f}, {ry:.3f}, {rz:.3f})")
    print(f"Joint Angles : ({rj1:.2f}, {rj2:.2f}, {rj3:.2f}, {rj4:.2f})")

    # Inverse kinematics from measured position
    sol = inverse_kinematics(rx, ry, rz)
    th1, th2, th3 = np.rad2deg(sol)
    print(f"Inverse Kinematics : ({th1:.3f}, {th2:.3f}, {th3:.3f})")

    # Compute angular error
    err = np.sqrt((rj1 - th1)**2 + (rj2 - th2)**2 + (rj3 - th3)**2)
    print(f"Joint Error Magnitude: {err:.4f}°")

    print(f"{'='*60}\n")
    time.sleep(4)

print("All tests completed successfully.")
robot.close()
