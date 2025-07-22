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

LIMIT_J1_MAX = 109
LIMIT_J1_MIN = -109
LIMIT_J2_MAX = 146
LIMIT_J2_MIN = -146
LIMIT_J3_MAX = 200
LIMIT_J3_MIN = 6
LIMIT_J4_MAX = 180
LIMIT_J4_MIN = -180

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
# 快速移動
def quick(x, y, z, f=3000):
    send_commands([f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])

# 線性移動
def linear(x, y, z, f=3000):
    send_commands([f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])

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
    quick(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    set_origin(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
    set_limit_detection(True)


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
