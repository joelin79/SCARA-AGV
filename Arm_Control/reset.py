from SCARA import *

quick(ORIGIN_X, ORIGIN_Y, ORIGIN_Z, maintain_extension_direction=True, extension_angle=-90.0)
CUR_X, CUR_Y, CUR_Z = ORIGIN_X, ORIGIN_Y, ORIGIN_Z
CUR_J1, CUR_J2, CUR_J3, CUR_J4 = ORIGIN_J1, ORIGIN_J2, ORIGIN_J3, 0
