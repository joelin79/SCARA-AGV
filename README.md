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

### 1. Test Extension Arm (Hardware Setup)
```bash
# Test extension arm functionality and calibration
python test_extension_arm.py
```

### 2. Test Vision System
```bash
# Test camera and detection without moving the arm
python demo_detection.py
```

### 3. Camera Calibration (First Time Setup)
```bash
# Calibrate camera with checkerboard pattern
python camera_calib.py
```

### 4. Run Complete Object Detection
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

## Hardware Setup

### Extension Arm Configuration
- **Extension arm** attached to J4 joint
- **Suction cup** at 45mm from J4 center
- **D435i camera** at 140mm from J4 center
- **Extension arm orientation**: Always points in +X direction during operation

### Camera Setup
The D435i camera is now mounted on the extension arm:
- Mounted 140mm from J4 center on extension arm
- Facing downward (-Z direction)
- Extension arm maintains +X orientation (90° cartesian)
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

**Extension Arm Coordinates:**
- Cartesian directions: 0° = +X, 90° = +Y, -90° = -Y, ±180° = -X
- Extension arm automatically maintains +Y direction (90°) during operation (default)
- Suction cup: 45mm from J4 center along extension arm
- Camera: 140mm from J4 center along extension arm

**Camera Coordinates:**
- Mounted on extension arm pointing +X direction
- Facing down, 1280x720 resolution
- Objects detected at Z=0 (work surface)
- Camera position automatically calculated from end effector + 140mm in +X

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

## Extension Arm Functions

### New SCARA.py Functions
```python
# Extension arm control
set_extension_direction(90.0)  # Point extension arm in +Y direction (default)
calculate_j4_for_cartesian_direction(j1, j2, angle)  # Calculate required J4

# Position utilities  
get_suction_cup_position()  # Returns (x, y, z) of suction cup
get_camera_position()       # Returns (x, y, z) of camera

# Enhanced movement with automatic extension control
quick(x, y, z, maintain_extension_direction=True, extension_angle=90.0)  # Default +Y
linear(x, y, z, maintain_extension_direction=True, extension_angle=90.0)  # Default +Y
```

### Enhanced Collision Checking
- Extension arm collision detection with base
- Safety margins for suction cup and camera positions
- Automatic workspace boundary checking

## File Structure

```
SCARA-AGV/
├── ObjectDetection.py      # Main detection system (updated for extension arm)
├── execute.py              # Easy run script
├── demo_detection.py       # Test individual components
├── test_extension_arm.py   # Extension arm functionality tests
├── camera_calib.py         # Camera calibration
├── SCARA.py               # Arm control (updated with extension arm)
├── yolo_detection.py      # Object detection
├── yolo/my_model/         # Trained YOLO model
├── detections_output/     # Output images
└── README.md              # This file
```

## Advanced Usage

### Extension Arm Calibration Process
1. **Physical Setup**: Manually align extension arm parallel to J2 arm segment
2. **Run Calibration**: Execute `python SCARA.py` or call `calibrate()` function
3. **Automatic Setup**: System sets J4=0° and moves extension to +Y direction (default)
4. **Verification**: Use `test_extension_arm.py` to verify proper operation

### Custom Extension Arm Orientations
```python
# Move with different extension arm directions
quick(x, y, z, maintain_extension_direction=True, extension_angle=90)   # +Y direction (default)
quick(x, y, z, maintain_extension_direction=True, extension_angle=0)    # +X direction
quick(x, y, z, maintain_extension_direction=True, extension_angle=180)  # -X direction
quick(x, y, z, maintain_extension_direction=True, extension_angle=-90)  # -Y direction
```

### Pick-and-Place with Extension Arm
```python
from object_coordinates import object_coordinates
from SCARA import quick, get_suction_cup_position, EXTENSION_SUCTION_LENGTH

for obj_x, obj_y, obj_z in object_coordinates:
    # Calculate end effector position to place suction cup at object
    # Extension arm points +Y by default, so suction cup is 45mm in +Y direction
    end_x = obj_x
    end_y = obj_y - EXTENSION_SUCTION_LENGTH  # Account for 45mm offset in +Y
    end_z = 200  # Working height
    
    quick(end_x, end_y, end_z)  # Extension arm automatically points +Y
    # Suction cup is now positioned over the object
    # Add suction control here
```

### Custom Object Classes
Modify your YOLO model training to detect specific objects for your application.

### Custom Scan Patterns
Override `generate_scan_positions()` to create custom scanning paths that account for the camera's 140mm offset.

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

