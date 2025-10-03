import sympy as sp
import numpy as np
import pydobot
import time

# -------------------------------
# Robot parameters (mm)
# -------------------------------
d1 = 150       # base height
a1 = 257 / 2   # horizontal offset
a2 = 150
a3 = 40
a4 = 50

# -------------------------------
# DH Transformation function
# -------------------------------
def DH(theta, d, a, alpha):
    """Standard DH transformation matrix"""
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0, 0, 0, 1]
    ])

# -------------------------------
# Forward Kinematics
# -------------------------------
def forward_kinematics(theta_vals_deg):
    """Compute T04 given [theta1, theta2, theta3, theta4] in degrees"""
    t1, t2, t3, t4 = [sp.rad(i) for i in theta_vals_deg]
    T01 = DH(t1, d1, 0, 0)
    T12 = DH(t2, 0, a2, 0)
    T23 = DH(t3, 0, a3, 0)
    T34 = DH(t4, 0, a4, 0)
    T04 = sp.simplify(T01 * T12 * T23 * T34)
    return sp.N(T04, 4)

# -------------------------------
# Inverse Kinematics
# -------------------------------
def inverse_kinematics(x, y, z):
    """
    Compute theta1, theta2, theta3, theta4 (deg) for target (x,y,z)
    Simple planar IK, assumes theta4=0
    """
    theta1 = np.arctan2(y, x)

    wx = np.sqrt(x**2 + y**2) - a4   # wrist offset
    wz = z - d1

    r = np.sqrt(wx**2 + wz**2)

    cos_theta3 = (r**2 - a2**2 - a3**2) / (2 * a2 * a3)
    theta3 = np.arccos(np.clip(cos_theta3, -1.0, 1.0))

    phi = np.arctan2(wz, wx)
    psi = np.arctan2(a3*np.sin(theta3), a2 + a3*np.cos(theta3))
    theta2 = phi - psi

    theta4 = 0  # fixed orientation

    return [np.degrees(theta1), np.degrees(theta2), np.degrees(theta3), theta4]

# -------------------------------
# Main Program
# -------------------------------
if __name__ == "__main__":
    # --- FK Test Angles ---
    theta_input = [0, 0, 0, 0]       # degrees
    T_fk = forward_kinematics(theta_input)
    print("FK result for", theta_input, ":\n", T_fk, "\n")

    # --- IK Target Position ---
    x_target, y_target, z_target = 240, 0, 150  # example position
    theta_calc = inverse_kinematics(x_target, y_target, z_target)
    print("IK joint angles to reach ({},{},{}):".format(x_target, y_target, z_target))
    print(theta_calc)

    # --- Connect to Dobot ---
    port = "/dev/ttyACM0"  # <-- hardcode your port here
    print("Using port:", port)
    device = pydobot.Dobot(port=port, verbose=True)

    # -------------------------------
    # Choose movement method (FK or IK)
    # -------------------------------

    # ----- FK mode -----
    # x, y, z = float(T_fk[0,3]), float(T_fk[1,3]), float(T_fk[2,3])
    # r = theta_input[3]

    # ----- IK mode -----
    T_ik_fk = forward_kinematics(theta_calc)   # convert IK angles to FK position
    x, y, z = float(T_ik_fk[0,3]), float(T_ik_fk[1,3]), float(T_ik_fk[2,3])
    r = theta_calc[3]

    print("Moving Dobot to position (X,Y,Z,R):", x, y, z, r)
    device.move_to(x, y, z, r, wait=True)

    time.sleep(2)

    # Read actual pose from Dobot
    pose = device.pose()
    print("Actual Dobot Pose (x, y, z, r):", pose)

    # Cleanup
    device.close()
