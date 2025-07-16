import math

LENGTH_J1 = 205
LENGTH_J2 = 205
LENGTH_J3 = 200

LIMIT_J1_MAX = 109
LIMIT_J1_MIN = -109
LIMIT_J2_MAX = 146
LIMIT_J2_MIN = -146
LIMIT_J3_MAX = 200
LIMIT_J3_MIN = 6
LIMIT_J4_MAX = 180
LIMIT_J4_MIN = -180

def check_joint_limits(j1, j2, j3, j4):
    if not (LIMIT_J1_MIN <= j1 <= LIMIT_J1_MAX):
        raise ValueError(f"J1 angle out of range: {j1:.2f}")
    if not (LIMIT_J2_MIN <= j2 <= LIMIT_J2_MAX):
        raise ValueError(f"J2 angle out of range: {j2:.2f}")
    if j3 is not None and not (LIMIT_J3_MIN <= j3 <= LIMIT_J3_MAX):
        raise ValueError(f"J3 value out of range: {j3}")
    if j4 is not None and not (LIMIT_J4_MIN <= j4 <= LIMIT_J4_MAX):
        raise ValueError(f"J4 value out of range: {j4}")

def cartesian_to_angles(x, y, z=None, j4=None, L1=LENGTH_J1, L2=LENGTH_J2, elbow='up'):
    r2 = x**2 + y**2
    r = math.sqrt(r2)

    if r > (L1 + L2):
        raise ValueError("Target is out of reach")

    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_theta2) > 1:
        raise ValueError("Unreachable: cos(theta2) out of bounds")

    theta2 = math.acos(cos_theta2)

    if elbow == 'up':
        theta2 = -theta2

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    print(f"(x: {x}, y: {y}) --> [J1: {theta1_deg:.2f}º, J2: {theta2_deg:.2f}º]")

    check_joint_limits(theta1_deg, theta2_deg, z, j4)
    return theta1_deg, theta2_deg

def angles_to_cartesian(j1_deg, j2_deg, L1=LENGTH_J1, L2=LENGTH_J2):
    j1 = math.radians(j1_deg)
    j2 = math.radians(j2_deg)
    x = L1 * math.cos(j1) + L2 * math.cos(j1 + j2)
    y = L1 * math.sin(j1) + L2 * math.sin(j1 + j2)
    print(f"[J1: {j1_deg:.2f}º, J2: {j2_deg:.2f}º] --> (x: {x:.2f}, y: {y:.2f})")
    return x, y

cartesian_to_angles(277.14, 17.14, 200, 0)
angles_to_cartesian(50.91,-94.74)