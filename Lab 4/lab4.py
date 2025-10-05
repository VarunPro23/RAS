import math
import time
import numpy as np
from pydobot import Dobot  # if you're using pydobot2, adjust import accordingly
from ht import forward_kinematics, inverse_kinematics


# ---------------- Setup ----------------
device = Dobot(port="/dev/ttyACM0")  # adjust port if needed
time.sleep(2)

# Test joint sets (in degrees)
# Format: (theta1, theta2, theta3)
test_joint_sets = [
    (0, 0, 0),
    (90, 20, 30),
    (-2, 25, 23),
    (31, 52, 42),
    (27, 52, 24),
]


test_position_sets = [ (300, 50, 100),
    (280, -195, 15),
    (35, 270, -33)]

# Robot parameters
a1 = 53.5
a2 = 150
a3 = 150
z_offset = 53.5

# ---------------- Test Loop ----------------
for idx, joints_deg in enumerate(test_joint_sets, 1):
    print(f"\n{'='*60}")
    print(f"Test {idx}")
    print(f"{'='*60}")
    
    j1, j2, j3 = joints_deg
    print(f"\nInput Joint Angles:")
    print(f"  Theta1 = {j1}°")
    print(f"  Theta2 = {j2}°")
    print(f"  Theta3 = {j3}°")
    
    # Convert degrees to radians for forward kinematics
    t1_rad = np.deg2rad(j1)
    t2_rad = np.deg2rad(j2)
    t3_rad = np.deg2rad(j3)
    
    # Forward kinematics - returns (px, py, pz)
    result = forward_kinematics(t1_rad, t2_rad, t3_rad, a1, a2, a3)
    
    # Unpack the result
    px, py, pz = result
    
    # Convert to integers for robot command
    px_mm = int(px)
    py_mm = int(py)
    pz_mm = int(pz)
    rot = 0  # Rotation parameter (adjust as needed)
    
    print(f"\nForward Kinematics Result:")
    print(f"  Position (mm): ({px_mm}, {py_mm}, {pz_mm})")
    print(f"  Position (raw): ({px:.6f}, {py:.6f}, {pz:.6f})")
    # print(help(device.move_to))
    # Send command to robot
    print(f"\nSending to robot: move_to_j({j1}, {j2}, {j3})")
    device.move_to(j1, j2, j3, mode=4)
    time.sleep(2)
    
    # Get robot feedback
    (pose, joint) = device.get_pose()
    [rx, ry, rz, rrot] = pose
    [rj1, rj2, rj3, rj4] = joint
    
    print(f"\nRobot Feedback:")
    print(f"  Actual Position (mm): ({rx}, {ry}, {rz}, {rrot})")
    print(f"  Actual Joints (deg): ({rj1:.2f}, {rj2:.2f}, {rj3:.2f}, {rj4:.2f})")
    
    # Inverse kinematics - find ALL solutions
    print(f"Target: ({rx:.6f}, {ry:.6f}, {rz:.6f} mm)")
    try:
        # Run inverse kinematics
        solution_rad = inverse_kinematics(rx, ry, rz)
        
        # Convert to degrees
        Theta1 = np.rad2deg(solution_rad[0])
        Theta2 = np.rad2deg(solution_rad[1])
        Theta3 = np.rad2deg(solution_rad[2])
        
        print(f"\nInverse Kinematics Solution:")
        print(f"  Theta1 = {Theta1:.3f}°")
        print(f"  Theta2 = {Theta2:.3f}°")
        print(f"  Theta3 = {Theta3:.3f}°")
        
        # Verify with forward kinematics
        # verification = verify_solution(solution_rad, (rx, ry, rz))
        # px_v, py_v, pz_v = verification['verified_pos']
        # print(f"\nVerification Forward Kinematics:")
        # print(f"  Verified Pos (mm): ({px_v:.3f}, {py_v:.3f}, {pz_v:.3f})")
        # print(f"  Error: {verification['error']:.6f} mm")
        
        # Compare joint errors with actual robot
        joint_error = np.sqrt((rj1 - Theta1)**2 + (rj2 - Theta2)**2 + (rj3 - Theta3)**2)
        print(f"\nJoint Error vs Robot: {joint_error:.4f}°")
        
    except Exception as e:
        print(f"IK failed: {e}")
    
    # Compare commanded vs actual position
    # pos_error = np.sqrt((px - rx)**2 + (py - ry)**2 + (pz - rz)**2)
    # print(f"\nPosition Error (commanded vs actual): {pos_error:.4f} mm")
    
    print(f"\n{'='*60}\n")
    time.sleep(4)
         

# ---------------- Test Loop ----------------
for idx, pval in enumerate(test_position_sets, 1):
    
    print(f"Test {idx}")
   
    
    j1, j2, j3 = joints_deg
    print(f"\nInput Joint Angles:")
    print(f"  Theta1 = {j1}°")
    print(f"  Theta2 = {j2}°")
    print(f"  Theta3 = {j3}°")
    
    # Convert to radians
    t1_rad = np.deg2rad(j1)
    t2_rad = np.deg2rad(j2)
    t3_rad = np.deg2rad(j3)
    
    # Forward kinematics
    px, py, pz = forward_kinematics(t1_rad, t2_rad, t3_rad, a1, a2, a3)
    
    # Convert to integers (for Dobot commands)
    px_mm = pval[0]
    py_mm = pval[1]
    pz_mm = pval[2]
    rot = 0  # Wrist rotation (if applicable)
    
    print(f"\nForward Kinematics Result:")
    print(f"  Position (mm): ({px_mm}, {py_mm}, {pz_mm})")
    #print(f"  Position (raw): ({px:.3f}, {py:.3f}, {pz:.3f})")
    
    # Send move command to robot
    print(f"\nSending to robot: move_to({px_mm}, {py_mm}, {pz_mm}, {rot})")
    device.move_to(px_mm, py_mm, pz_mm, rot)
    time.sleep(2)
    
    # Get actual feedback
    pose, joint = device.get_pose()
    [rx, ry, rz, rrot] = pose
    [rj1, rj2, rj3, rj4] = joint
    
    print(f"\nRobot Feedback:")
    print(f"  Actual Position (mm): ({rx:.3f}, {ry:.3f}, {rz:.3f}, {rrot:.3f})")
    print(f"  Actual Joints (deg): ({rj1:.2f}, {rj2:.2f}, {rj3:.2f}, {rj4:.2f})")
    
    # ---------------- Inverse Kinematics ----------------
    
    print(f"INVERSE KINEMATICS - Single Solution Check")
  
    print(f"Target (from robot): ({rx:.3f}, {ry:.3f}, {rz:.3f} mm)")
    
    try:
        # Run inverse kinematics
        solution_rad = inverse_kinematics(rx, ry, rz)
        
        # Convert to degrees
        Theta1 = np.rad2deg(solution_rad[0])
        Theta2 = np.rad2deg(solution_rad[1])
        Theta3 = np.rad2deg(solution_rad[2])
        
        print(f"\nInverse Kinematics Solution:")
        print(f"  Theta1 = {Theta1:.3f}°")
        print(f"  Theta2 = {Theta2:.3f}°")
        print(f"  Theta3 = {Theta3:.3f}°")
        
        # Verify with forward kinematics
        # verification = verify_solution(solution_rad, (rx, ry, rz))
        # px_v, py_v, pz_v = verification['verified_pos']
        # print(f"\nVerification Forward Kinematics:")
        # print(f"  Verified Pos (mm): ({px_v:.3f}, {py_v:.3f}, {pz_v:.3f})")
        # print(f"  Error: {verification['error']:.6f} mm")
        
        # Compare joint errors with actual robot
        joint_error = np.sqrt((rj1 - Theta1)**2 + (rj2 - Theta2)**2 + (rj3 - Theta3)**2)
        print(f"\nJoint Error vs Robot: {joint_error:.4f}°")
        
    except Exception as e:
        print(f"IK failed: {e}")
    
    # Compare commanded vs actual position
    # pos_error = np.sqrt((px - rx)**2 + (py - ry)**2 + (pz - rz)**2)
    # print(f"\nPosition Error (commanded vs actual): {pos_error:.4f} mm")
    
    print(f"\n{'='*60}\n")
    time.sleep(4)


print("Test sequence completed!")
device.close()  # Clean up connection
