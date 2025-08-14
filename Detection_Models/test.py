import ObjectDetection as detector

arm_xyz = detector.debug_pixel_to_arm_coordinates(
    75, -92.7, 150, -72.3,
    (150, -100, 150),
    468, 305, 193
)
print("Arm coords:", arm_xyz)