import math

def cartesian_to_angles(x, y, L1=205, L2=205):
    # distance to target
    r2 = x**2 + y**2
    r = math.sqrt(r2)

    # check if the target is reachable
    if r > (L1 + L2):
        raise ValueError("Target is out of reach")

    # law of cosines for angle at elbow (theta2)
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)

    # shoulder angle (theta1)
    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    # convert to degrees
    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    return theta1_deg, theta2_deg

print(cartesian_to_angles(385.82,38.26))