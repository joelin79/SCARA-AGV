import time

import SCARA as sc

# quick(300, -0,330, maintain_extension_direction=True, extension_angle=-90.0)
# quick_suction(253.85, 28.08, 101-5)
x=207.33024649118786
y=-62.85586909548598
z=108.0

sc.calibrate()
sc.linear(300,0,330, f=1000)
sc.await_arrival()
sc.linear(150,0,330,f=1000)