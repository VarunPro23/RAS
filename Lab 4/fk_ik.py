import sympy as sp

# -------------------------------
# Robot parameters (mm)
# -------------------------------
d1 = 150   # base height
a2 = 150   # link 2 length
a3 = 40    # link 3 length
a4 = 50    # link 4 length

# -------------------------------
# DH Transformation function
# -------------------------------
def DH(theta, d, a, alpha):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),               sp.cos(alpha),               d],
        [0,              0,                           0,                           1]
    ])

# -------------------------------
# Forward Kinematics
# -------------------------------
def forward_kinematics(theta_vals_deg):
    """
    Compute T04 given [theta1, theta2, theta3, theta4] in degrees
    """
    # convert degrees to radians
    t1, t2, t3, t4 = [sp.rad(i) for i in theta_vals_deg]

    # Correct DH table (real Dobot geometry)
    T01 = DH(t1, d1, 0, sp.pi/2)   # α1 = +90°
    T12 = DH(t2, 0, a2, 0)
    T23 = DH(t3, 0, a3, 0)
    T34 = DH(t4, 0, a4, 0)

    T04 = sp.simplify(T01 * T12 * T23 * T34)
    return sp.N(T04, 4)

# -------------------------------
# Example usage
# -------------------------------
if __name__ == "__main__":
    print("FK for [90,20,30,0]:")
    sp.pprint(forward_kinematics([90, 20, 30, 0]))
