# -*- coding: utf-8 -*-
# distance_detection.py

import cv2
from realsense_depth import DepthCamera

# Initialize camera
dc = DepthCamera()
point = (400, 300)

# Mouse callback function
def show_distance(event, x, y, flags, param):
    global point
    if event == cv2.EVENT_MOUSEMOVE:
        point = (x, y)

cv2.namedWindow("Color frame")
cv2.setMouseCallback("Color frame", show_distance)

while True:
    ret, depth_frame, color_frame = dc.get_frame()
    if not ret:
        continue

    # Draw circle and display distance
    cv2.circle(color_frame, point, 5, (0, 0, 255), -1)
    distance = depth_frame[point[1], point[0]]
    cv2.putText(color_frame, "{}mm".format(distance), (point[0] + 10, point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.imshow("Color frame", color_frame)

    key = cv2.waitKey(1)
    if key == 27:  # Press ESC to exit
        break

dc.release()
cv2.destroyAllWindows()
