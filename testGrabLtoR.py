import serial
import time
import pygame

ser = serial.Serial(port='/dev/tty.usbserial-10', baudrate=115200, timeout=1)

RIGHT_HANDED_MODE = True

HOME_FEEDRATE = 3000
MAX_FEEDRATE = 2000
MAX_ACC = 500

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

LENGTH_J1 = 205
LENGTH_J2 = 205
LENGTH_J3 = 200

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
    send_commands([f"M201 X{x} Y{y} Z{z} I{i} J{i} K{k}"])

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
    send_commands([f"G0 X{x:.3f} Y{y:.3f} Z{z:.3f} F{f}"])

# 延遲
def delay(s):
    send_commands([f"G4 T{s}"])

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
        time.sleep(0.1)


STEP = 5     # mm or degrees per key press
FEED = 1000  # movement speed

def keyboard_control_pygame():
    pygame.init()
    screen = pygame.display.set_mode((300, 200))
    pygame.display.set_caption("SCARA Arm Keyboard Control")

    mode = 'cartesian'
    pos = {'x': ORIGIN_X, 'y': ORIGIN_Y, 'z': ORIGIN_Z}
    joint = {'j1': ORIGIN_J1, 'j2': ORIGIN_J2, 'j3': ORIGIN_J3}
    current_joint = 'j1'

    running = True
    clock = pygame.time.Clock()

    print("Keyboard control started. Press ESC to quit.")

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        if keys[pygame.K_ESCAPE]:
            running = False

        # Toggle mode
        if keys[pygame.K_TAB]:
            mode = 'joint' if mode == 'cartesian' else 'cartesian'
            print(f"Switched to {mode} mode")
            pygame.time.wait(300)

        if mode == 'cartesian':
            moved = False
            if keys[pygame.K_w]:
                pos['y'] += STEP
                moved = True
            elif keys[pygame.K_s]:
                pos['y'] -= STEP
                moved = True
            elif keys[pygame.K_a]:
                pos['x'] -= STEP
                moved = True
            elif keys[pygame.K_d]:
                pos['x'] += STEP
                moved = True
            elif keys[pygame.K_q]:
                pos['z'] += STEP
                moved = True
            elif keys[pygame.K_e]:
                pos['z'] -= STEP
                moved = True

            if moved:
                quick(pos['x'], pos['y'], pos['z'], FEED)
                print(f"Moved to: {pos}")
                pygame.time.wait(150)

        else:
            if keys[pygame.K_1]: current_joint = 'j1'
            if keys[pygame.K_2]: current_joint = 'j2'
            if keys[pygame.K_3]: current_joint = 'j3'

            moved = False
            if keys[pygame.K_LEFT]:
                joint[current_joint] -= STEP
                moved = True
            elif keys[pygame.K_RIGHT]:
                joint[current_joint] += STEP
                moved = True

            if moved:
                quick(joint['j1'], joint['j2'], joint['j3'], FEED)
                print(f"Joints: {joint}")
                pygame.time.wait(150)

        clock.tick(20)  # limit FPS

    pygame.quit()





calibrate()
# quick(410,0,100,6000)
# # quick(ORIGIN_X,ORIGIN_Y,ORIGIN_Z)
#
# angle_mode()
# set_origin(0,0,200)
# quick(ORIGIN_J1,ORIGIN_J2,ORIGIN_J3,1000)
coordinate_mode()
keyboard_control_pygame()

ser.close()
