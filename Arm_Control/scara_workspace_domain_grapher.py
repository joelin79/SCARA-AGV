import math
import matplotlib.pyplot as plt

# SCARA arm parameters
L1 = 205
L2 = 205
J1_min = -109
J1_max = 109
J2_min = -146
J2_max = 146

# Extension arm parameters
EXTENSION_SUCTION_LENGTH = 45   # mm from J4 to suction cup center
EXTENSION_CAMERA_LENGTH = 140   # mm from J4 to camera center

def calculate_j4_for_cartesian_direction(j1: float, j2: float, cartesian_angle: float) -> float:
    """Calculate J4 angle needed to point extension arm camera in specified cartesian direction."""
    current_arm_orientation = j1 + j2
    target_standard_angle = cartesian_angle
    j4_angle = target_standard_angle - current_arm_orientation
    
    # Normalize angle to [-180, 180]
    while j4_angle > 180:
        j4_angle -= 360
    while j4_angle <= -180:
        j4_angle += 360
    
    return j4_angle

def angles_to_cartesian(j1_deg, j2_deg, L1=L1, L2=L2):
    """Convert joint angles to cartesian coordinates."""
    j1 = math.radians(j1_deg)
    j2 = math.radians(j2_deg)
    x = L1 * math.cos(j1) + L2 * math.cos(j1 + j2)
    y = L1 * math.sin(j1) + L2 * math.sin(j1 + j2)
    return x, y

def get_extension_positions(end_x, end_y, j1_deg, j2_deg, j4_deg):
    """Calculate camera and suction cup positions for given arm configuration."""
    # Calculate extension arm absolute angle
    extension_absolute_angle = j1_deg + j2_deg + j4_deg
    extension_rad = math.radians(extension_absolute_angle)
    
    # Camera position (points in the direction of the extension arm)
    camera_x = end_x + EXTENSION_CAMERA_LENGTH * math.cos(extension_rad)
    camera_y = end_y + EXTENSION_CAMERA_LENGTH * math.sin(extension_rad)
    
    # Suction cup position (opposite side of camera, 180° offset)
    suction_angle_rad = extension_rad + math.pi  # 180° offset
    suction_x = end_x + EXTENSION_SUCTION_LENGTH * math.cos(suction_angle_rad)
    suction_y = end_y + EXTENSION_SUCTION_LENGTH * math.sin(suction_angle_rad)
    
    return (camera_x, camera_y), (suction_x, suction_y)

# Original workspace calculation
x_vals = []
y_vals = []

# Left-handed mode (original domain)
for j1_deg in range(J1_min, J1_max + 1):
    for j2_deg in range(J2_min, J2_max + 1):
        j1_rad = math.radians(j1_deg)
        j2_rad = math.radians(j2_deg)
        x = L1 * math.cos(j1_rad) + L2 * math.cos(j1_rad + j2_rad)
        y = L1 * math.sin(j1_rad) + L2 * math.sin(j1_rad + j2_rad)
        if x < 0 and -100 < y < 100:
            continue
        x_vals.append(x)
        y_vals.append(y)

# Right-handed mode: J2 only negative values
x_vals_right = []
y_vals_right = []
for j1_deg in range(J1_min, J1_max + 1):
    for j2_deg in range(-146, 1):  # J2 in [-146, 0]
        j1_rad = math.radians(j1_deg)
        j2_rad = math.radians(j2_deg)
        x = L1 * math.cos(j1_rad) + L2 * math.cos(j1_rad + j2_rad)
        y = L1 * math.sin(j1_rad) + L2 * math.sin(j1_rad + j2_rad)
        if x < 0 and -100 < y < 100:
            continue
        x_vals_right.append(x)
        y_vals_right.append(y)

# Calculate extension arm workspace with camera pointing at -90 degrees (down)
camera_x_vals = []
camera_y_vals = []
suction_x_vals = []
suction_y_vals = []
end_effector_x_vals = []
end_effector_y_vals = []

target_camera_direction = -90.0  # Camera pointing downward (-Y direction)

print("Calculating extension arm workspace with camera at -90°...")

# Use right-handed mode for extension calculations (more practical)
for j1_deg in range(J1_min, J1_max + 1, 5):  # Sample every 5 degrees for performance
    for j2_deg in range(-146, 1, 5):  # J2 in [-146, 0]
        try:
            # Get end effector position
            end_x, end_y = angles_to_cartesian(j1_deg, j2_deg)
            
            # Skip collision zones
            if end_x < 0 and -100 < end_y < 100:
                continue
            
            # Calculate required J4 for -90° camera direction
            j4_deg = calculate_j4_for_cartesian_direction(j1_deg, j2_deg, target_camera_direction)
            
            # Check J4 limits
            if not (-180 <= j4_deg <= 180):
                continue
            
            # Calculate extension positions
            (camera_x, camera_y), (suction_x, suction_y) = get_extension_positions(
                end_x, end_y, j1_deg, j2_deg, j4_deg
            )
            
            # Store positions
            end_effector_x_vals.append(end_x)
            end_effector_y_vals.append(end_y)
            camera_x_vals.append(camera_x)
            camera_y_vals.append(camera_y)
            suction_x_vals.append(suction_x)
            suction_y_vals.append(suction_y)
            
        except Exception:
            continue

print(f"Generated {len(camera_x_vals)} valid extension arm configurations")

# Create two plots side by side
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# Plot 1: Original workspace
ax1.scatter(x_vals, y_vals, s=0.5, color='blue', label='Left-handed Mode', alpha=0.6)
ax1.scatter(x_vals_right, y_vals_right, s=0.5, color='red', label='Right-handed Mode', alpha=0.6)
ax1.plot(0, 0, 'kx', markersize=8, label='Origin (0,0)')
ax1.plot(-105, 0, 'ko', markersize=5)
ax1.set_title("SCARA Arm Reachable Workspace\n(End Effector Only)")
ax1.set_xlabel("X (mm)")
ax1.set_ylabel("Y (mm)")
ax1.axis("equal")
ax1.grid(True)
ax1.legend()
ax1.invert_xaxis()

# Plot 2: Extension arm workspace with camera at -90°
ax2.scatter(end_effector_x_vals, end_effector_y_vals, s=1, color='gray', label='End Effector', alpha=0.3)
ax2.scatter(camera_x_vals, camera_y_vals, s=1, color='blue', label='Camera (at -90°)', alpha=0.7)
ax2.scatter(suction_x_vals, suction_y_vals, s=1, color='red', label='Suction Cup (at +90°)', alpha=0.7)
ax2.plot(0, 0, 'kx', markersize=8, label='Origin (0,0)')
ax2.plot(-105, 0, 'ko', markersize=5)
ax2.set_title("Extension Arm Workspace\n(Camera pointing at -90°)")
ax2.set_xlabel("X (mm)")
ax2.set_ylabel("Y (mm)")
ax2.axis("equal")
ax2.grid(True)
ax2.legend()
ax2.invert_xaxis()

plt.tight_layout()
plt.show()

# Print workspace statistics
print("\n=== Workspace Statistics ===")
print(f"End Effector Range:")
print(f"  X: {min(end_effector_x_vals):.1f} to {max(end_effector_x_vals):.1f} mm")
print(f"  Y: {min(end_effector_y_vals):.1f} to {max(end_effector_y_vals):.1f} mm")

if camera_x_vals:
    print(f"Camera Range (at -90°):")
    print(f"  X: {min(camera_x_vals):.1f} to {max(camera_x_vals):.1f} mm")
    print(f"  Y: {min(camera_y_vals):.1f} to {max(camera_y_vals):.1f} mm")

if suction_x_vals:
    print(f"Suction Cup Range (at +90°):")
    print(f"  X: {min(suction_x_vals):.1f} to {max(suction_x_vals):.1f} mm")
    print(f"  Y: {min(suction_y_vals):.1f} to {max(suction_y_vals):.1f} mm")