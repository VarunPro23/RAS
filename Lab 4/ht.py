import numpy as np
import sympy as sp

# Robot link dimensions (millimeters)
z_offset = 53.5
a1 = 53.5
a2 = 150
a3 = 150

# Forward Kinematics (Symbolic + Numeric)
def forward_kinematics(theta1, theta2, theta3,
                       a1_param=None, a2_param=None, a3_param=None,
                       z_offset_param=None, simplify=True, numeric=False):
    """
    Compute forward kinematics for a 3-DOF manipulator using homogeneous matrices.
    Returns symbolic or numeric end-effector position (Px, Py, Pz).
    """
    # Assign parameters or use defaults
    A1 = a1_param if a1_param is not None else a1
    A2 = a2_param if a2_param is not None else a2
    A3 = a3_param if a3_param is not None else a3
    ZOFF = z_offset_param if z_offset_param is not None else z_offset

    # Base - Joint 1
    H01 = sp.Matrix([
        [sp.cos(theta1), 0, -sp.sin(theta1), 0],
        [sp.sin(theta1), 0,  sp.cos(theta1), 0],
        [0,             -1,  0,              A1],
        [0,              0,  0,              1]
    ])

    # Joint 1 - Joint 2
    H12 = sp.Matrix([
        [-sp.sin(theta2), -sp.cos(theta2), 0, 90 + A2 * sp.sin(theta2)],
        [ sp.cos(theta2), -sp.sin(theta2), 0, -A2 * sp.cos(theta2)],
        [0,                0,              1,  0],
        [0,                0,              0,  1]
    ])

    # Joint 2 - Joint 3
    H23 = sp.Matrix([
        [sp.cos(theta3 - theta2), -sp.sin(theta3 - theta2), 0,  A3 * sp.sin(theta3 - theta2)],
        [sp.sin(theta3 - theta2),  sp.cos(theta3 - theta2), 0, -A3 * sp.cos(theta3 - theta2)],
        [0,                        0,                        1,  0],
        [0,                        0,                        0,  1]
    ])

    # Full transformation
    H03 = H01 * H12 * H23

    # Extract coordinates
    Px, Py, Pz = H03[0, 3], H03[1, 3], H03[2, 3] - ZOFF

    # Optional simplification and numeric evaluation
    if simplify:
        H03 = sp.simplify(H03)
        Px, Py, Pz = [sp.simplify(x) for x in (Px, Py, Pz)]

    if numeric:
        H03 = H03.evalf()
        Px, Py, Pz = [sp.N(x) for x in (Px, Py, Pz)]

    return Px, Py, Pz

def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Symbolic setup for inverse kinematics
q1, q2, q3 = sp.symbols('q1 q2 q3', real=True)
A1, A2, A3, ZOFF = sp.symbols('A1 A2 A3 ZOFF', real=True)

# Symbolic equations (represent FK equations)
R = A3 * sp.cos(q3) + A2 * sp.sin(q2) + 90
Px_s = sp.cos(q1) * R
Py_s = sp.sin(q1) * R
Pz_s = A1 + A2 * sp.cos(q2) - A3 * sp.sin(q3) - ZOFF

# Substitute numeric constants
subs_dict = {A1: a1, A2: a2, A3: a3, ZOFF: z_offset}
Px_s, Py_s, Pz_s = [expr.subs(subs_dict) for expr in (Px_s, Py_s, Pz_s)]

# Inverse Kinematics using SymPy nsolve
def inverse_kinematics(px_target, py_target, pz_target, guess=None):
    """
    Solves for (q1, q2, q3) given desired end-effector coordinates.
    Uses SymPy's nsolve for numerical solutions.
    """
    if guess is None:
        guess = (0.0, 0.0, 0.0)

    # Define equations Px_s = px_target, etc.
    equations = (
        Px_s - px_target,
        Py_s - py_target,
        Pz_s - pz_target
    )

    # Solve system numerically
    result = sp.nsolve(
        equations,
        (q1, q2, q3),
        tuple(map(float, guess)),
        tol=1e-14,
        maxsteps=200,
        prec=50
    )

    # Convert to floats and wrap angles
    numeric_q = np.array([float(r) for r in result])
    wrapped_q = np.array([wrap_angle(val) for val in numeric_q])

    return tuple(wrapped_q)

if __name__ == "__main__":
    # Define test joint angles (degrees - radians)
    t1, t2, t3 = np.deg2rad([90, 20, 30])

    print("Input Joint Angles:")
    print(f"Theta1 = {np.rad2deg(t1):.2f}°")
    print(f"Theta2 = {np.rad2deg(t2):.2f}°")
    print(f"Theta3 = {np.rad2deg(t3):.2f}°\n")

    # Compute forward kinematics
    px, py, pz = forward_kinematics(t1, t2, t3)
    print("Forward Kinematics Result:")
    print(f"px = {px:.6f}")
    print(f"py = {py:.6f}")
    print(f"pz = {pz:.6f}\n")

    # Solve inverse kinematics from computed position
    print("Computing Inverse Kinematics...")
    q_solution = inverse_kinematics(px, py, pz, guess=(0, 0, 0))

    q_deg = np.rad2deg(q_solution)
    print("\nInverse Kinematics Solution:")
    print(f"Theta1 = {q_deg[0]:.6f}°")
    print(f"Theta2 = {q_deg[1]:.6f}°")
    print(f"Theta3 = {q_deg[2]:.6f}°")

    # Recheck by running FK again on the found angles
    px_check, py_check, pz_check = forward_kinematics(*q_solution)
    print("\nVerification:")
    print(f"px = {px_check:.6f} (expected {px:.6f})")
    print(f"py = {py_check:.6f} (expected {py:.6f})")
    print(f"pz = {pz_check:.6f} (expected {pz:.6f})")

    error = np.sqrt((px_check - px)**2 + (py_check - py)**2 + (pz_check - pz)**2)
    print(f"\nPosition Error: {error:.10f} mm")
