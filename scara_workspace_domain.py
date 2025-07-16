import math
import matplotlib.pyplot as plt

L1 = 205
L2 = 205
J1_min = -109
J1_max = 109
J2_min = -146
J2_max = 146

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

plt.figure(figsize=(8, 8))
plt.scatter(x_vals, y_vals, s=0.5, color='blue', label='Left-handed Mode')
plt.scatter(x_vals_right, y_vals_right, s=0.5, color='red', label='Right-handed Mode')
plt.plot(0, 0, 'kx', markersize=8, label='Origin (0,0)')
plt.plot(-105, 0, 'ko', markersize=5, label='_nolegend_')  # suppress legend for second point to avoid duplicate labels
plt.title("SCARA Arm Reachable Workspace")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.gca().invert_xaxis()
plt.show()