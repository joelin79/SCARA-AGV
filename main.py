import time

import Arm_Control.SCARA as sc
sc.calibrate()

import Detection_Models.ObjectDetection as detection
detection.main()

import object_mover
object_mover.main()