import os
output_dir = "../detections_output"
os.makedirs(output_dir, exist_ok=True)
import sys
import argparse
import glob
import time

import cv2
import numpy as np
from ultralytics import YOLO

# python yolo_detection.py --model yolo/my_model/my_model.pt --source usb0 --resolution 1280x720

# Define and parse user input arguments

parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                    required=True)
parser.add_argument('--source', nargs='+', help='Image source(s): can be image file(s), folder, video, USB camera index, or Picamera index', required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                    default=0.5)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                    otherwise, match source resolution',
                    default=None)
parser.add_argument('--record',
                    help='Record results from video or webcam and save it as "demo1.avi". Must specify --resolution argument to record.',
                    action='store_true')

args = parser.parse_args()

# Parse user inputs
model_path = args.model
img_source = args.source

# Expand wildcard(s) if any provided
img_ext_list = ['.jpg', '.JPG', '.jpeg', '.JPEG', '.png', '.PNG', '.bmp', '.BMP']
expanded_sources = []
for s in img_source:
    if '*' in s or '?' in s:
        expanded_sources.extend([f for f in glob.glob(s) if os.path.splitext(f)[1] in img_ext_list])
    elif os.path.isfile(s) and os.path.splitext(s)[1] in img_ext_list:
        expanded_sources.append(s)
if not expanded_sources:
    print('No valid image files found for given --source input.')
    sys.exit(0)
img_source = expanded_sources
min_thresh = args.thresh
user_res = args.resolution
record = args.record

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

# Parse input to determine if image source is a file, folder, video, or USB camera
img_ext_list = ['.jpg', '.JPG', '.jpeg', '.JPEG', '.png', '.PNG', '.bmp', '.BMP']
vid_ext_list = ['.avi', '.mov', '.mp4', '.mkv', '.wmv']

# Determine source type
if isinstance(img_source, list) and all(os.path.isfile(src) for src in img_source):
    source_type = 'image_list'
    imgs_list = img_source
elif len(img_source) == 1 and os.path.isdir(img_source[0]):
    source_type = 'folder'
    imgs_list = []
    filelist = glob.glob(img_source[0] + '/*')
    for file in filelist:
        _, file_ext = os.path.splitext(file)
        if file_ext in img_ext_list:
            imgs_list.append(file)
elif len(img_source) == 1 and os.path.isfile(img_source[0]):
    _, ext = os.path.splitext(img_source[0])
    if ext in img_ext_list:
        source_type = 'image'
        imgs_list = [img_source[0]]
    elif ext in vid_ext_list:
        source_type = 'video'
        cap_arg = img_source[0]
elif len(img_source) == 1 and 'usb' in img_source[0]:
    source_type = 'usb'
    usb_idx = int(img_source[0][3:])
elif len(img_source) == 1 and 'picamera' in img_source[0]:
    source_type = 'picamera'
    picam_idx = int(img_source[0][8:])
else:
    print(f'Input {img_source} is invalid. Please try again.')
    sys.exit(0)

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Check if recording is valid and set up recording
if record:
    if source_type not in ['video', 'usb']:
        print('Recording only works for video and camera sources. Please try again.')
        sys.exit(0)
    if not user_res:
        print('Please specify resolution to record video at.')
        sys.exit(0)

    # Set up recording
    record_name = 'demo1.avi'
    record_fps = 30
    recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW, resH))

# Load or initialize image source
if source_type == 'image' or source_type == 'image_list':
    pass  # imgs_list already set above
elif source_type == 'folder':
    pass  # imgs_list already set above
elif source_type == 'video' or source_type == 'usb':
    if source_type == 'video':
        # cap_arg was set above if video
        pass
    elif source_type == 'usb':
        cap_arg = 1 if sys.platform == "darwin" else usb_idx
    cap = cv2.VideoCapture(cap_arg)

    # Set camera or video resolution if specified by user
    if user_res:
        ret = cap.set(3, resW)
        ret = cap.set(4, resH)

# elif source_type == 'picamera':
#     from picamera2 import Picamera2
#
#     cap = Picamera2()
#     cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
#     cap.start()

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164, 120, 87), (68, 148, 228), (93, 97, 209), (178, 182, 133), (88, 159, 106),
               (96, 202, 231), (159, 124, 168), (169, 162, 241), (98, 118, 150), (172, 176, 184)]

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

# Begin inference loop over images in imgs_list
for img_filename in imgs_list:
    t_start = time.perf_counter()

    frame = cv2.imread(img_filename)
    if frame is None:
        print(f'Failed to load image {img_filename}. Skipping.')
        continue

    # Resize frame to desired display resolution
    if resize == True:
        frame = cv2.resize(frame, (resW, resH))

    # Determine dynamic label size based on resolution
    font_scale = 0.5
    font_thickness = 1
    if frame.shape[1] >= 1600 or frame.shape[0] >= 1200:  # e.g. iPhone resolution or larger
        font_scale = 1.2
        font_thickness = 2
    elif frame.shape[1] >= 1200:
        font_scale = 0.9
        font_thickness = 2

    # Run inference on frame
    results = model(frame, verbose=False)

    # Extract results
    detections = results[0].boxes

    # Initialize variable for basic object counting example
    object_count = 0

    # Go through each detection and get bbox coords, confidence, and class
    for i in range(len(detections)):

        # Get bounding box coordinates
        # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
        xyxy_tensor = detections[i].xyxy.cpu()  # Detections in Tensor format in CPU memory
        xyxy = xyxy_tensor.numpy().squeeze()  # Convert tensors to Numpy array
        xmin, ymin, xmax, ymax = xyxy.astype(int)  # Extract individual coordinates and convert to int

        # Calculate center of bounding box
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)

        # Draw a pink dot at the center
        cv2.circle(frame, (center_x, center_y), 4, (255, 0, 255), -1)

        # Draw the center coordinates as label
        center_label = f'({center_x}, {center_y})'
        cv2.putText(frame, center_label, (center_x + 5, center_y - 5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 0, 255), font_thickness)

        # Get bounding box class ID and name
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]

        # Get bounding box confidence
        conf = detections[i].conf.item()

        # Draw box if confidence threshold is high enough
        if conf > 0.4:
            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)

            label = f'{classname}: {int(conf * 100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)  # Get font size
            label_ymin = max(ymin, labelSize[1] + 10)  # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin - labelSize[1] - 10),
                          (xmin + labelSize[0], label_ymin + baseLine - 10), color,
                          cv2.FILLED)  # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0),
                        font_thickness)  # Draw label text

            # Basic example: count the number of objects in the image
            object_count = object_count + 1

    # Display detection results
    cv2.putText(frame, f'Number of objects: {object_count}', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, font_scale if font_scale > 0.7 else 0.7, (0, 255, 255),
                font_thickness if font_thickness > 1 else 2)  # Draw total number of detected objects

    # Save the output image to the output folder
    output_path = os.path.join(output_dir, os.path.basename(img_filename))
    cv2.imwrite(output_path, frame)
    print(f"Saved: {output_path}")
    if record: recorder.write(frame)

    # Calculate FPS for this frame
    t_stop = time.perf_counter()
    frame_rate_calc = float(1 / (t_stop - t_start))

    # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
    if len(frame_rate_buffer) >= fps_avg_len:
        temp = frame_rate_buffer.pop(0)
        frame_rate_buffer.append(frame_rate_calc)
    else:
        frame_rate_buffer.append(frame_rate_calc)

    # Calculate average FPS for past frames
    avg_frame_rate = np.mean(frame_rate_buffer)



# Clean up
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')
if source_type == 'video' or source_type == 'usb':
    cap.release()
elif source_type == 'picamera':
    cap.stop()
if record: recorder.release()
cv2.destroyAllWindows()