import time

from SCARA import *

# quick(300, -0,330, maintain_extension_direction=True, extension_angle=-90.0)
# quick_suction(253.85, 28.08, 101-5)
x=207.33024649118786
y=-62.85586909548598
z=108.0


# quick_suction(x,y,z)
suck_object(x, y, z, 1000)

#