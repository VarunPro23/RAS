import numpy as np
import sympy as sp

# -------------------
# Parameters (same as yours)
# -------------------
z_offset = 53.5
a1 = 53.5
a2 = 150
a3 = 150

# -------------------
# Numeric FK (unchanged): for verification/printing
# -------------------
# def forward_kinematics(the1, the2, the3, a1_param=None, a2_param=None, a3_param=None):
#     """
#     Calculate end-effector position from joint angles (in radians) - NUMPY
#     """
#     a1_val = a1_param if a1_param is not None else a1
#     a2_val = a2_param if a2_param is not None else a2
#     a3_val = a3_param if a3_param is not None else a3

#     Px = np.cos(the1) * (a3_val * np.cos(the3) + a2_val * np.sin(the2) + 90)
#     Py = np.sin(the1) * (a3_val * np.cos(the3) + a2_val * np.sin(the2) + 90)
#     Pz = a1_val + a2_val * np.cos(the2) - a3_val * np.sin(the3) - z_offset
#     return Px, Py, Pz

# -------------------
# SymPy FK matrices (matches your MATLAB H01/H12/H23)
# -------------------
def forward_kinematics(theta1, theta2, theta3,
                      a1_param=None, a2_param=None, a3_param=None, z_offset_param=None,
                      do_simplify=True, eval_numeric=False):
    """
    Build homogeneous transforms H01, H12, H23 and H03 (= H01*H12*H23) using SymPy,
    mirroring your MATLAB matrices. Also returns Px, Py, Pz with Pz -= z_offset.

    Args:
        theta1, theta2, theta3: joint angles (can be SymPy symbols or numeric values)
        a1_param, a2_param, a3_param, z_offset_param: link params (defaults to globals a1,a2,a3,z_offset)
        do_simplify: run sympy.simplify on results
        eval_numeric: if True, evalf() the matrices/positions (nice when all inputs are numeric)

    Returns:
        H01, H12, H23, H03, Px, Py, Pz  (all SymPy objects)
    """
    A1 = a1_param if a1_param is not None else a1
    A2 = a2_param if a2_param is not None else a2
    A3 = a3_param if a3_param is not None else a3
    ZOFF = z_offset_param if z_offset_param is not None else z_offset

    # 0H1
    H01 = sp.Matrix([
        [sp.cos(theta1), 0, -sp.sin(theta1), 0],
        [sp.sin(theta1), 0,  sp.cos(theta1), 0],
        [0,             -1,  0,              A1],
        [0,              0,  0,               1]
    ])

    # 1H2
    H12 = sp.Matrix([
        [-sp.sin(theta2), -sp.cos(theta2), 0,  90 + A2*sp.sin(theta2)],
        [ sp.cos(theta2), -sp.sin(theta2), 0, -A2*sp.cos(theta2)],
        [0,                0,              1,  0],
        [0,                0,              0,  1]
    ])

    # 2H3
    H23 = sp.Matrix([
        [sp.cos(theta3 - theta2), -sp.sin(theta3 - theta2), 0,  A3*sp.sin(theta3 - theta2)],
        [sp.sin(theta3 - theta2),  sp.cos(theta3 - theta2), 0, -A3*sp.cos(theta3 - theta2)],
        [0,                        0,                        1,  0],
        [0,                        0,                        0,  1]
    ])

    H03 = H01 * H12 * H23
    Px, Py, Pz = H03[0, 3], H03[1, 3], H03[2, 3] - ZOFF

    if do_simplify:
        H03 = sp.simplify(H03)
        Px, Py, Pz = sp.simplify(Px), sp.simplify(Py), sp.simplify(Pz)

    if eval_numeric:
        H01, H12, H23, H03 = H01.evalf(), H12.evalf(), H23.evalf(), H03.evalf()
        Px, Py, Pz = sp.N(Px), sp.N(Py), sp.N(Pz)

    return Px, Py, Pz


def wrap_angle(a):
    """wrap to (-pi, pi]"""
    return (a + np.pi) % (2*np.pi) - np.pi

# -------------------
# Symbolic setup (SymPy) for IK
# -------------------
q1, q2, q3 = sp.symbols('q1 q2 q3', real=True)
# Use SymPy reals for parameters
A1, A2, A3, ZOFF = sp.symbols('A1 A2 A3 ZOFF', real=True)

# Symbolic FK expressions that mirror your numeric FK exactly
R = A3*sp.cos(q3) + A2*sp.sin(q2) + 90  # radial term used in Px/Py
Px_s = sp.cos(q1) * R
Py_s = sp.sin(q1) * R
Pz_s = A1 + A2*sp.cos(q2) - A3*sp.sin(q3) - ZOFF

# Substitute numeric parameters once to get concrete expressions
subs_params = {A1: a1, A2: a2, A3: a3, ZOFF: z_offset}
Px_s = Px_s.subs(subs_params)
Py_s = Py_s.subs(subs_params)
Pz_s = Pz_s.subs(subs_params)

def inverse_kinematics(px_target, py_target, pz_target, initial_guess=None):
    """
    IK with SymPy nsolve (MATLAB 'solve'-style numeric solving).
    initial_guess: iterable of 3 radians (q1,q2,q3).
    """
    if initial_guess is None:
        initial_guess = (0.0, 0.0, 0.0)

    # Build the system: Px_s == px, Py_s == py, Pz_s == pz
    eqs = (Px_s - px_target, Py_s - py_target, Pz_s - pz_target)

    # nsolve requires a tuple of symbols and an initial guess
    sol = sp.nsolve(eqs, (q1, q2, q3), tuple(map(float, initial_guess)),
                    tol=1e-14, maxsteps=200, prec=50)

    # Convert to floats and wrap
    q_numeric = np.array([float(sol[0]), float(sol[1]), float(sol[2])], dtype=float)
    q_numeric = np.array([wrap_angle(x) for x in q_numeric])
    return tuple(q_numeric)

# OPTIONAL: exact symbolic solve (can be heavy/slow for trig systems)
# def inverse_kinematics_symbolic(px_target, py_target, pz_target):
#     sol_syms = sp.solve(
#         (sp.Eq(Px_s, px_target), sp.Eq(Py_s, py_target), sp.Eq(Pz_s, pz_target)),
#         (q1, q2, q3), dict=True
#     )
#     return sol_syms

# -------------------
# Example usage
# -------------------
if __name__ == "__main__":
    # Joint angles in radians
    t1 = np.deg2rad(90)
    t2 = np.deg2rad(20)
    t3 = np.deg2rad(30)

    print("Given Joint Angles:")
    print(f"Theta1 = {np.rad2deg(t1):.2f}°")
    print(f"Theta2 = {np.rad2deg(t2):.2f}°")
    print(f"Theta3 = {np.rad2deg(t3):.2f}°\n")

    # Forward Kinematics - Calculate positions
    px_new, py_new, pz_new = forward_kinematics(t1, t2, t3)
    print("Forward Kinematics Results:")
    print(f"px_new = {px_new:.6f}")
    print(f"py_new = {py_new:.6f}")
    print(f"pz_new = {pz_new:.6f}\n")

    # Solve inverse kinematics with SymPy nsolve (MATLAB-like)
    print("Solving Inverse Kinematics with SymPy nsolve...")
    initial_guess = (0.0, 0.0, 0.0)  # radians
    q_sol = inverse_kinematics(px_new, py_new, pz_new, initial_guess)

    Theta1 = np.rad2deg(q_sol[0])
    Theta2 = np.rad2deg(q_sol[1])
    Theta3 = np.rad2deg(q_sol[2])

    print("\nInverse Kinematics Solution:")
    print(f"Theta1 = {Theta1:.6f}°")
    print(f"Theta2 = {Theta2:.6f}°")
    print(f"Theta3 = {Theta3:.6f}°")
    print(f"\nSolution: [{Theta1:.6f}, {Theta2:.6f}, {Theta3:.6f}]")

    # Verify the solution
    px_verify, py_verify, pz_verify = forward_kinematics(q_sol[0], q_sol[1], q_sol[2])
    print("\nVerification (Forward Kinematics with solved angles):")
    print(f"px = {px_verify:.6f} (target: {px_new:.6f})")
    print(f"py = {py_verify:.6f} (target: {py_new:.6f})")
    print(f"pz = {pz_verify:.6f} (target: {pz_new:.6f})")

    error = np.sqrt((px_verify - px_new)**2 + (py_verify - py_new)**2 + (pz_verify - pz_new)**2)
    print(f"\nError: {error:.10f} mm")
