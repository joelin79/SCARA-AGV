import serial
import time
import pygame
import math

ser = serial.Serial(port='/dev/tty.usbserial-10', baudrate=115200, timeout=1)

RIGHT_HANDED_MODE = True

HOME_FEEDRATE = 3000
MAX_FEEDRATE = 2000
MAX_ACC = 100

PULSE_J1 = 88.889
PULSE_J2 = 88.889
PULSE_J3 = 320
PULSE_J4 = 88.889
PULSE_J5 = 88.889
PULSE_J6 = 88.889

ORIGIN_X = 96.979
ORIGIN_Y = 70.459
ORIGIN_Z = 200

# Positive is CW
ORIGIN_J1 = 109
ORIGIN_J2 = -146
ORIGIN_J3 = 200
ORIGIN_J4 = 0

CUR_X = ORIGIN_X
CUR_Y = ORIGIN_Y
CUR_Z = ORIGIN_Z
CUR_J1 = ORIGIN_J1
CUR_J2 = ORIGIN_J2
CUR_J3 = ORIGIN_J3
CUR_J4 = ORIGIN_J4

LENGTH_J1 = 205
LENGTH_J2 = 205
LENGTH_J3 = 200

# Extension arm dimensions (from J4 center)
EXTENSION_SUCTION_LENGTH = 45   # mm from J4 to suction cup center
EXTENSION_CAMERA_LENGTH = 140   # mm from J4 to camera center

LIMIT_J1_MAX = 109
LIMIT_J1_MIN = -109
LIMIT_J2_MAX = 146
LIMIT_J2_MIN = -146
LIMIT_J3_MAX = 200
LIMIT_J3_MIN = 6
LIMIT_J4_MAX = 180
LIMIT_J4_MIN = -180

def calculate_j4_for_cartesian_direction(j1: float, j2: float, cartesian_angle: float) -> float:
    """
    Calculate J4 angle needed to point extension arm in specified cartesian direction.
    
    Args:
        j1, j2: Current joint angles (degrees)
        cartesian_angle: Target direction in cartesian coordinates (degrees)
                        0° = +X direction, 90° = +Y direction, -90° = -Y direction, ±180° = -X direction
    
    Returns:
        J4 angle relative to J2 (degrees)
    """
    # Current arm orientation (J1 + J2) in standard coordinates where 0° = +X
    current_arm_orientation = j1 + j2
    
    # User's cartesian system: 0° = +X, 90° = +Y, -90° = -Y, ±180° = -X
    # Standard system: 0° = +X, 90° = +Y, 180° = -X, 270° = -Y
    # For this system, the angles are already aligned with standard coordinates
    target_standard_angle = cartesian_angle
    
    # Calculate required J4 angle
    j4_angle = target_standard_angle - current_arm_orientation
    
    # Normalize angle to [-180, 180]
    while j4_angle > 180:
        j4_angle -= 360
    while j4_angle <= -180:
        j4_angle += 360
    
    return j4_angle

def set_extension_direction(cartesian_angle: float):
    """
    Set extension arm to point in specified cartesian direction.
    
    Args:
        cartesian_angle: Target direction (0° = +X, 90° = +Y, -90° = -Y, ±180° = -X)
    """
    global CUR_J1, CUR_J2, CUR_J4
    
    # Calculate required J4 angle
    required_j4 = calculate_j4_for_cartesian_direction(CUR_J1, CUR_J2, cartesian_angle)
    
    # Check if J4 is within limits
    if not (LIMIT_J4_MIN <= required_j4 <= LIMIT_J4_MAX):
        raise ValueError(f"Required J4 angle {required_j4:.2f}° is out of range [{LIMIT_J4_MIN}, {LIMIT_J4_MAX}]")
    
    # Send J4 command in angle mode
    angle_mode()
    send_commands([f"G0 I{CUR_J1:.3f} J{CUR_J2:.3f} K{CUR_J3:.3f} L{required_j4:.3f}"])
    CUR_J4 = required_j4
    coordinate_mode()
    
    print(f"Extension arm set to {cartesian_angle:.1f}° (J4 = {required_j4:.2f}°)")

def get_suction_cup_position() -> tuple[float, float, float]:
    """
    Get current suction cup position in world coordinates.
    
    Returns:
        (x, y, z) position of suction cup center
    """
    global CUR_X, CUR_Y, CUR_Z, CUR_J4
    
    # Extension arm absolute angle (assuming it points +Y = 90°)
    extension_angle_rad = math.radians(90)  # +Y direction
    
    # Suction cup position
    suction_x = CUR_X + EXTENSION_SUCTION_LENGTH * math.cos(extension_angle_rad)
    suction_y = CUR_Y + EXTENSION_SUCTION_LENGTH * math.sin(extension_angle_rad)
    suction_z = CUR_Z  # Same height as end effector
    
    return (suction_x, suction_y, suction_z)

def get_camera_position() -> tuple[float, float, float]:
    """
    Get current camera position in world coordinates.
    
    Returns:
        (x, y, z) position of camera center
    """
    global CUR_X, CUR_Y, CUR_Z, CUR_J4
    
    # Extension arm absolute angle (assuming it points +Y = 90°)
    extension_angle_rad = math.radians(90)  # +Y direction
    
    # Camera position
    camera_x = CUR_X + EXTENSION_CAMERA_LENGTH * math.cos(extension_angle_rad)
    camera_y = CUR_Y + EXTENSION_CAMERA_LENGTH * math.sin(extension_angle_rad)
    camera_z = CUR_Z  # Same height as end effector
    
    return (camera_x, camera_y, camera_z)

def check_extension_arm_collision(j1: float, j2: float, j4: float) -> None:
    """
    Check if extension arm will collide with base or arm structure.
    
    Args:
        j1, j2: Arm joint angles (degrees)
        j4: Extension arm angle relative to J2 (degrees)
    """
    # Get end effector position
    end_x, end_y = angles_to_cartesian(j1, j2, LENGTH_J1, LENGTH_J2)
    
    # Calculate extension arm absolute angle
    extension_absolute_angle = j1 + j2 + j4
    extension_rad = math.radians(extension_absolute_angle)
    
    # Check suction cup position
    suction_x = end_x + EXTENSION_SUCTION_LENGTH * math.cos(extension_rad)
    suction_y = end_y + EXTENSION_SUCTION_LENGTH * math.sin(extension_rad)
    
    # Check camera position  
    camera_x = end_x + EXTENSION_CAMERA_LENGTH * math.cos(extension_rad)
    camera_y = end_y + EXTENSION_CAMERA_LENGTH * math.sin(extension_rad)
    
    # Check collision with base (enlarged safety zone due to extension)
    base_collision_zone = 120  # mm, increased from 100mm
    
    # Check suction cup collision
    if suction_x < 0 and abs(suction_y) < base_collision_zone:
        raise ValueError(f"Suction cup will hit base: position ({suction_x:.1f}, {suction_y:.1f})")
    
    # Check camera collision  
    if camera_x < 0 and abs(camera_y) < base_collision_zone:
        raise ValueError(f"Camera will hit base: position ({camera_x:.1f}, {camera_y:.1f})")
    
    # Check if extension goes too far from base (safety limit)
    max_reach = LENGTH_J1 + LENGTH_J2 + EXTENSION_CAMERA_LENGTH + 50  # 50mm safety margin
    
    suction_distance = math.sqrt(suction_x**2 + suction_y**2)
    camera_distance = math.sqrt(camera_x**2 + camera_y**2)
    
    if suction_distance > max_reach:
        raise ValueError(f"Suction cup too far from base: {suction_distance:.1f}mm > {max_reach}mm")
    if camera_distance > max_reach:
        raise ValueError(f"Camera too far from base: {camera_distance:.1f}mm > {max_reach}mm")

def check_joint_limits(j1: float, j2: float, j3: float | None, j4: float | None, L1: float = LENGTH_J1, L2: float = LENGTH_J2) -> None:

    x, y = angles_to_cartesian(j1, j2, L1, L2)

    if x < 0 and -100 < y < 100:
        raise ValueError("會打到機身 (-X, +100~-100)")
    r = math.sqrt(x**2 + y**2)
    if r > (L1 + L2):
        raise ValueError("Target is out of reach")
    r2 = x**2 + y**2
    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_theta2) > 1:
        raise ValueError("Unreachable: cos(theta2) out of bounds")

    if not (LIMIT_J1_MIN <= j1 <= LIMIT_J1_MAX):
        raise ValueError(f"J1 angle out of range: {j1:.2f}")
    if not (LIMIT_J2_MIN <= j2 <= LIMIT_J2_MAX):
        raise ValueError(f"J2 angle out of range: {j2:.2f}")
    if j3 is not None and not (LIMIT_J3_MIN <= j3 <= LIMIT_J3_MAX):
        raise ValueError(f"J3 value out of range: {j3}")
    if j4 is not None and not (LIMIT_J4_MIN <= j4 <= LIMIT_J4_MAX):
        raise ValueError(f"J4 value out of range: {j4}")
    
    # Check extension arm collisions if J4 is specified
    if j4 is not None:
        check_extension_arm_collision(j1, j2, j4)

def cartesian_to_angles(x, y, z=None, j4=None, L1=LENGTH_J1, L2=LENGTH_J2, elbow='up'):
    r2 = x**2 + y**2
    r = math.sqrt(r2)

    cos_theta2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)
    theta2 = math.acos(cos_theta2)

    if elbow == 'up':
        theta2 = -theta2

    k1 = L1 + L2 * math.cos(theta2)
    k2 = L2 * math.sin(theta2)
    theta1 = math.atan2(y, x) - math.atan2(k2, k1)

    theta1_deg = math.degrees(theta1)
    theta2_deg = math.degrees(theta2)

    print(f"(x: {x}, y: {y}) --> [J1: {theta1_deg:.2f}º, J2: {theta2_deg:.2f}º]")

    return theta1_deg, theta2_deg

def angles_to_cartesian(j1_deg, j2_deg, L1=LENGTH_J1, L2=LENGTH_J2):
    j1 = math.radians(j1_deg)
    j2 = math.radians(j2_deg)
    x = L1 * math.cos(j1) + L2 * math.cos(j1 + j2)
    y = L1 * math.sin(j1) + L2 * math.sin(j1 + j2)
    print(f"[J1: {j1_deg:.2f}º, J2: {j2_deg:.2f}º] --> (x: {x:.2f}, y: {y:.2f})")
    return x, y

# 設定脈衝數
def set_steps_per_unit(x, y, z, i, j, k):
    send_commands([f"M92 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# ----- HOME -----
# 設定回原點馬達電機方向
def set_home_dir(x, y, z, i, j, k):
    send_commands([f"M352 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# 設定回原點速度
def set_home_feedrate(x, y, z, i, j, k):
    send_commands([f"M359 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# 設定回原點順序
def set_home_order(x, y, z, i, j, k):
    send_commands([f"M365 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# ----- General -----
# 設定機械臂模式
def set_scara_mode(right: bool):
    if right: send_commands(["M358 V1"])
    else: send_commands(["M358 V0"])
    send_commands(["M366 T2"])      # Type 2 Robot

# 設定機械臂尺寸
def set_arm_dimensions(x, y, z):
    send_commands([f"M363 X{x} Y{y} Z{z}"])

# 設定馬達電機方向
def set_axis_dir(x, y, z, i, j, k):
    send_commands([f"M364 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# 設定最大運行速度
def set_max_feedrate(x, y, z, i, j, k):
    send_commands([f"M203 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# 設定最大加速度
def set_max_acc(x, y, z, i, j, k):
    send_commands([f"M201 X{x} Y{y} Z{z} I{i} J{j} K{k}"])

# 硬體儲存
def save_settings():
    send_commands(["M500"])

# 設定所在位置為原點
def set_origin(x, y, z, i=0, j=0, k=0):
    send_commands([f"G92 X{x:.3f} Y{y:.3f} Z{z:.3f} I{i:.3f} J{j:.3f} K{k:.3f}"])

# 取消座標零點設定 (undo set_origin G92)
# POST-REQ: set_origin()
def reset_origin():
    send_commands(["M368"])

# 開啟/關閉限位檢查
def set_limit_detection(mode: bool):
    if mode: send_commands(["M120"])
    else: send_commands(["M121"])       # （僅在回原點有效）

# 設定座標模式 X Y Z I
def coordinate_mode():
    send_commands(["M361"])

# 設定角度模式 J1 J2 J3 J4
def angle_mode():
    send_commands(["M362"])


# ----- ACTIONS -----
# 快速移動 (with automatic extension arm orientation)
def quick(x, y, z, f=3000, maintain_extension_direction=True, extension_angle=90.0):
    """
    Quick movement with automatic extension arm control.
    
    Args:
        x, y, z: Target position
        f: Feedrate
        maintain_extension_direction: If True, automatically maintain extension direction
        extension_angle: Target extension direction in cartesian coordinates (default: +Y = 90°)
    """
    global CUR_X, CUR_Y, CUR_Z, CUR_J1, CUR_J2, CUR_J4
    
    if maintain_extension_direction:
        # Calculate joint angles for target position
        j1, j2 = cartesian_to_angles(x, y)
        
        # Calculate required J4 for maintaining extension direction
        required_j4 = calculate_j4_for_cartesian_direction(j1, j2, extension_angle)
        
        # Check limits including extension arm collision
        check_joint_limits(j1, j2, z, required_j4)
        
        # Move with coordinated J4
        send_commands([f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])
        
        # Update J4 if needed
        if abs(required_j4 - CUR_J4) > 0.1:  # Only update if significant change
            angle_mode()
            send_commands([f"G0 L{required_j4:.3f}"])
            coordinate_mode()
            CUR_J4 = required_j4
        
        # Update current position
        CUR_X, CUR_Y, CUR_Z = x, y, z
        CUR_J1, CUR_J2 = j1, j2
        
    else:
        # Standard movement without extension arm control
        send_commands([f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])
        CUR_X, CUR_Y, CUR_Z = x, y, z

# 線性移動 (with automatic extension arm orientation)
def linear(x, y, z, f=3000, maintain_extension_direction=True, extension_angle=90.0):
    """
    Linear movement with automatic extension arm control.
    
    Args:
        x, y, z: Target position
        f: Feedrate
        maintain_extension_direction: If True, automatically maintain extension direction
        extension_angle: Target extension direction in cartesian coordinates (default: +Y = 90°)
    """
    global CUR_X, CUR_Y, CUR_Z, CUR_J1, CUR_J2, CUR_J4
    
    if maintain_extension_direction:
        # Calculate joint angles for target position
        j1, j2 = cartesian_to_angles(x, y)
        
        # Calculate required J4 for maintaining extension direction
        required_j4 = calculate_j4_for_cartesian_direction(j1, j2, extension_angle)
        
        # Check limits including extension arm collision
        check_joint_limits(j1, j2, z, required_j4)
        
        # Move with coordinated J4
        send_commands([f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])
        
        # Update J4 if needed
        if abs(required_j4 - CUR_J4) > 0.1:  # Only update if significant change
            angle_mode()
            send_commands([f"G1 L{required_j4:.3f}"])
            coordinate_mode()
            CUR_J4 = required_j4
        
        # Update current position
        CUR_X, CUR_Y, CUR_Z = x, y, z
        CUR_J1, CUR_J2 = j1, j2
        
    else:
        # Standard movement without extension arm control
        send_commands([f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])
        CUR_X, CUR_Y, CUR_Z = x, y, z

# 延遲
def delay(s):
    send_commands([f"G4 T{s}"])

def section_power(v: bool):
    if v: send_commands(["G130 V1"])
    else: send_commands(["G130 V0"])

def section_trigger(v: bool):
    if v: send_commands(["G131 V1"])
    else: send_commands(["G131 V0"])

# 回原點      NOTE: 會重設原點設置 (相當於執行 M368)
def home(x=None, y=None, z=None):
    cmd = "G28"
    if x is not None: cmd += f" X{x:.3f}"
    if y is not None: cmd += f" Y{y:.3f}"
    if z is not None: cmd += f" Z{z:.3f}"
    send_commands([cmd])


def calibrate():
    global CUR_J4
    
    set_steps_per_unit(PULSE_J1, PULSE_J2, PULSE_J3, PULSE_J4,PULSE_J5, PULSE_J6)
    set_home_dir(1,-1,1,1,1,1)
    set_home_order(0,1,2,3,4,5)
    set_home_feedrate(HOME_FEEDRATE,HOME_FEEDRATE,HOME_FEEDRATE,HOME_FEEDRATE,600,600)
    set_axis_dir(1,-1,-1,1,1,1)
    set_max_feedrate(MAX_FEEDRATE-500,MAX_FEEDRATE,MAX_FEEDRATE,MAX_FEEDRATE,600,600)
    set_max_acc(MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC, MAX_ACC)
    send_commands(["M356 F0"])
    set_scara_mode(RIGHT_HANDED_MODE)
    set_arm_dimensions(LENGTH_J1, LENGTH_J2, LENGTH_J3)
    save_settings()
    reset_origin()
    set_origin(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    set_limit_detection(True)

    home(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    quick(ORIGIN_X, ORIGIN_Y, ORIGIN_Z, maintain_extension_direction=False)  # Move without extension control first
    set_origin(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    set_limit_detection(True)
    
    print("Extension arm calibration...")
    print("Ensure extension arm is parallel to J2 arm before proceeding.")
    input("Press Enter when extension arm is properly aligned...")
    
    # Set J4 origin (extension parallel to J2 arm = 0 degrees)
    angle_mode()
    send_commands([f"G92 L0"])  # Set current J4 position as 0 degrees
    CUR_J4 = 0
    coordinate_mode()
    
    # Set extension arm to point in +Y direction (90 degrees cartesian)
    print("Setting extension arm to point in +Y direction...")
    set_extension_direction(90.0)  # +Y direction
    
    print("Calibration complete. Extension arm now points in +Y direction.")


def send_commands(commands):
    for cmd in commands:
        ser.write((cmd + '\n').encode())
        print(f"Sent: {cmd}")
        time.sleep(0.02)


calibrate()
# quick(410,0,100,6000)
# # quick(ORIGIN_X,ORIGIN_Y,ORIGIN_Z)
#
# angle_mode()
# set_origin(0,0,200)
# quick(ORIGIN_J1,ORIGIN_J2,ORIGIN_J3,1000)
coordinate_mode()

STEP_CART = 1  # mm step for cartesian movements
STEP_ANG = 1   # degree step for joint angle movements

def keyboard_control_pygame():
    global CUR_X, CUR_Y, CUR_Z, CUR_J1, CUR_J2, CUR_J3, CUR_J4

    pygame.init()
    screen = pygame.display.set_mode((200, 200))
    pygame.display.set_caption("SCARA Control")
    clock = pygame.time.Clock()

    mode = 'coord'  # 'coord' or 'angle'
    coordinate_mode()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                if event.key == pygame.K_0:
                    calibrate()
                    print("Calibration triggered")
                if event.key == pygame.K_j:
                    if mode == 'coord':
                        mode = 'angle'
                        angle_mode()
                        CUR_J1, CUR_J2 = cartesian_to_angles(CUR_X, CUR_Y)
                        CUR_J3 = CUR_Z
                        print("Switched to angle mode")
                    else:
                        mode = 'coord'
                        coordinate_mode()
                        CUR_X, CUR_Y = angles_to_cartesian(CUR_J1, CUR_J2)
                        CUR_Z = CUR_J3
                        print("Switched to coordinate mode")
                    continue

        keys = pygame.key.get_pressed()

        if mode == 'coord':
            dx = 0
            dy = 0
            if keys[pygame.K_w]:
                dx += STEP_CART
            if keys[pygame.K_s]:
                dx -= STEP_CART
            if keys[pygame.K_a]:
                dy += STEP_CART
            if keys[pygame.K_d]:
                dy -= STEP_CART
            if dx != 0 or dy != 0:
                new_x = CUR_X + dx
                new_y = CUR_Y + dy
                try:
                    j1, j2 = cartesian_to_angles(new_x, new_y, L1=LENGTH_J1, L2=LENGTH_J2)
                    check_joint_limits(j1, j2, CUR_Z, CUR_J4)
                    quick(new_x, new_y, CUR_Z)
                    CUR_X, CUR_Y = new_x, new_y
                except ValueError as e:
                    print(f"Limit error: {e}")

        elif mode == 'angle':
            dj1 = 0
            dj2 = 0
            dj3 = 0
            dj4 = 0
            if keys[pygame.K_q]:
                dj1 += STEP_ANG
            if keys[pygame.K_a]:
                dj1 -= STEP_ANG
            if keys[pygame.K_w]:
                dj2 += STEP_ANG
            if keys[pygame.K_s]:
                dj2 -= STEP_ANG
            if keys[pygame.K_e]:
                dj3 += STEP_ANG
            if keys[pygame.K_d]:
                dj3 -= STEP_ANG
            if keys[pygame.K_r]:
                dj4 += STEP_ANG
            if keys[pygame.K_f]:
                dj4 -= STEP_ANG
            if dj1 != 0 or dj2 != 0 or dj3 != 0 or dj4 != 0:
                new_j1 = CUR_J1 + dj1
                new_j2 = CUR_J2 + dj2
                new_j3 = CUR_J3 + dj3
                new_j4 = CUR_J4 + dj4
                try:
                    check_joint_limits(new_j1, new_j2, new_j3, new_j4)
                    angle_mode()
                    quick(new_j1, new_j2, new_j3)
                    CUR_J1, CUR_J2, CUR_J3, CUR_J4 = new_j1, new_j2, new_j3, new_j4
                except ValueError as e:
                    print(f"Limit error: {e}")

        clock.tick(30)

    pygame.quit()
