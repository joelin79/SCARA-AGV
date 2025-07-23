# SCARA AGV Object Detection System

A complete computer vision system for detecting objects and transforming their pixel coordinates to SCARA arm coordinates for automated manipulation.

## Overview

This system integrates:
- **SCARA arm control** (`SCARA.py`) - Right-handed mode robotic arm
- **Camera calibration** (`camera_calib.py`) - Intel RealSense D435i calibration
- **YOLO object detection** (`yolo_detection.py`) - Custom trained YOLOv11 model
- **Coordinate transformation** (`ObjectDetection.py`) - Pixel to arm coordinate conversion
- **Automated scanning** - Multi-position workspace coverage

## Quick Start

### 1. Test Your Setup
```bash
# Test camera and detection without moving the arm
python demo_detection.py
```

### 2. Camera Calibration (First Time Setup)
```bash
# Calibrate camera with checkerboard pattern
python camera_calib.py
```

### 3. Run Complete Object Detection
```bash
# Automated workspace scan and object detection
python execute.py
```

## System Requirements

- **Hardware**: SCARA arm with D435i camera mounted facing down
- **Python Dependencies**: 
  - `opencv-python`
  - `ultralytics` (YOLO)
  - `numpy`
  - `PyYAML`
  - `pygame` (for manual arm control)

### Installation
```bash
pip install opencv-python ultralytics numpy PyYAML pygame
```

## Camera Setup

The D435i camera should be:
- Mounted on the SCARA arm
- Facing downward (-Z direction)
- Aligned to the -X direction of the arm
- Height: ~30cm (300mm) above work surface

## Usage Guide

### Option 1: Full Automated Scan
```python
from ObjectDetection import SCARAObjectDetection

detector = SCARAObjectDetection()
objects = detector.scan_workspace_and_detect_objects()
coordinates = detector.get_object_coordinates_list()
```

### Option 2: Single Position Detection
```python
detector = SCARAObjectDetection()
image = detector.capture_image_at_position(200, 0, 200)
detections = detector.detect_objects_in_image(image, (200, 0, 200))
```

## Output Files

- `object_coordinates.txt` - Human-readable coordinate list
- `object_coordinates.py` - Python importable coordinates
- `detections_output/` - Annotated detection images
- `calib_params.yaml` - Camera calibration parameters

## Coordinate System

**SCARA Arm Coordinates:**
- Origin: (96.979, 70.459, 200) mm
- X-axis: Forward/backward
- Y-axis: Left/right  
- Z-axis: Up/down
- Workspace: ~410mm radius from origin

**Camera Coordinates:**
- Facing down, aligned to -X direction
- 1280x720 resolution
- Objects detected at Z=0 (work surface)

## Calibration

### Camera Intrinsic Calibration
1. Print a checkerboard pattern (9x6 corners, 25mm squares)
2. Run `python camera_calib.py`
3. Capture 15-20 images at different angles
4. Calibration parameters saved to `calib_params.yaml`

### Default Parameters
If no calibration file exists, default D435i parameters are used:
- Focal length: ~910 pixels
- Principal point: (640, 360)
- No lens distortion assumed

## Configuration Options

### SCARAObjectDetection Parameters
```python
detector = SCARAObjectDetection(
    model_path="yolo/my_model/my_model.pt",  # YOLO model path
    confidence_threshold=0.5,                # Detection confidence
    camera_height=300.0,                     # Camera height (mm)
    calib_file="calib_params.yaml"          # Calibration file
)
```

### Scanning Parameters
```python
objects = detector.scan_workspace_and_detect_objects(
    camera_index=0,    # Camera device index
    grid_size=3        # 3x3 = 9 scan positions
)
```

## Troubleshooting

### Camera Issues
- **Camera not found**: Check `camera_index` (try 0, 1, 2)
- **Poor image quality**: Adjust lighting, clean camera lens
- **Wrong resolution**: D435i supports up to 1920x1080

### Detection Issues
- **No objects detected**: Lower `confidence_threshold`
- **Wrong coordinates**: Re-run camera calibration
- **Duplicate detections**: Adjust `distance_threshold` in duplicate removal

### Arm Movement Issues
- **Unreachable positions**: Check workspace limits in `scara_workspace_domain_grapher.py`
- **Communication errors**: Verify serial connection to arm controller

## File Structure

```
SCARA-AGV/
├── ObjectDetection.py      # Main detection system
├── execute.py              # Easy run script
├── demo_detection.py       # Test individual components
├── camera_calib.py         # Camera calibration
├── SCARA.py               # Arm control
├── yolo_detection.py      # Object detection
├── yolo/my_model/         # Trained YOLO model
├── detections_output/     # Output images
└── README.md              # This file
```

## Advanced Usage

### Custom Object Classes
Modify your YOLO model training to detect specific objects for your application.

### Multiple Camera Heights
Adjust `camera_height` parameter based on your mounting configuration.

### Custom Scan Patterns
Override `generate_scan_positions()` to create custom scanning paths.

### Integration with Pick-and-Place
Use the generated coordinates with your existing arm control system:

```python
from object_coordinates import object_coordinates
from SCARA import quick

for x, y, z in object_coordinates:
    quick(x, y, z)  # Move to object
    # Add suction/gripper control here
```

## Safety Notes

- Always verify arm workspace limits before operation
- Ensure emergency stop is accessible during automated scanning
- Test with demo mode before full automation
- Check camera mount stability during arm movement

## Support

For issues related to:
- **YOLO detection**: Check model path and training data
- **Coordinate accuracy**: Re-calibrate camera
- **Arm control**: Verify SCARA.py serial connection
- **System integration**: Use demo_detection.py for debugging

