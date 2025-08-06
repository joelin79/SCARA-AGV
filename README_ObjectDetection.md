# SCARA AGV Object Detection System

This system uses the RealSense D435i camera mounted on the SCARA arm to scan the entire workspace and detect objects using a trained YOLO model.

## Features

- **Automated Workspace Scanning**: Plans and executes camera movements to cover the entire reachable workspace
- **YOLO Object Detection**: Uses custom trained model to detect non-stacking blocks
- **3D Coordinate Conversion**: Converts pixel detections to arm coordinate system using depth information
- **Duplicate Filtering**: Removes duplicate detections from overlapping camera views
- **3D Visualization**: Creates comprehensive 3D plots of detected objects
- **Camera Calibration Support**: Uses calibrated camera parameters for accurate coordinate conversion

## System Requirements

- Python 3.8+
- RealSense D435i camera
- SCARA arm with extension arm
- Required Python packages:
  - ultralytics (YOLO)
  - pyrealsense2
  - opencv-python
  - numpy
  - matplotlib

## Installation

1. Install required packages:
```bash
pip install ultralytics pyrealsense2 opencv-python numpy matplotlib
```

2. Ensure the YOLO model exists at `yolo/my_model/my_model.pt`

3. Calibrate the camera (optional but recommended):
```bash
python RealSense/camera_calibration.py
```

## Usage

### Quick Start

Run with default settings:
```bash
python run_object_detection.py
```

### Advanced Usage

Customize scanning parameters:
```bash
python run_object_detection.py --height 120 --spacing 60 --confidence 0.4
```

### Command Line Options

- `--model`: Path to YOLO model (default: yolo/my_model/my_model.pt)
- `--height`: Scan height in mm (default: 150.0)
- `--spacing`: Grid spacing between scan points in mm (default: 80.0)
- `--confidence`: YOLO confidence threshold (default: 0.3)
- `--no-calibration`: Disable camera calibration
- `--no-images`: Don't save captured images
- `--simulate`: Run in simulation mode (no arm movement)

### Programmatic Usage

```python
from Detection_Models.ObjectDetection import ObjectDetectionSystem

# Initialize system
detector = ObjectDetectionSystem(
    model_path="yolo/my_model/my_model.pt",
    use_calibration=True,
    save_images=True
)

# Initialize components
if detector.initialize():
    # Plan scanning positions
    detector.plan_scanning_positions(
        scan_height=150.0,
        grid_spacing=80.0,
        camera_direction=-90.0
    )
    
    # Run the scan
    if detector.scan_workspace(confidence_threshold=0.3):
        # Save results
        detector.save_results("detected_objects.json")
        
        # Create visualization
        detector.visualize_3d()
        
        print(f"Detected {len(detector.detected_objects)} objects")

# Clean up
detector.cleanup()
```

## System Architecture

### Core Components

1. **ObjectDetectionSystem**: Main class that orchestrates the entire detection process
2. **SCARA Integration**: Uses the SCARA.py module for arm control and movement
3. **Camera Interface**: Uses RealSense depth camera for image capture
4. **YOLO Detection**: Custom trained model for object detection
5. **Coordinate Conversion**: Converts 2D pixel + depth to 3D arm coordinates

### Workflow

1. **Initialization**: Set up camera, load YOLO model, load calibration
2. **Scan Planning**: Generate grid of camera positions covering the workspace
3. **Position Validation**: Check each position is reachable by the SCARA arm
4. **Scanning Loop**:
   - Move camera to position
   - Capture color and depth images
   - Run YOLO detection
   - Convert detections to arm coordinates
5. **Duplicate Filtering**: Remove objects detected multiple times
6. **Results**: Save coordinates and create visualizations

### Coordinate Systems

- **Pixel Coordinates**: (u, v) in camera image
- **Camera Coordinates**: (x, y, z) relative to camera
- **Arm Coordinates**: (x, y, z) in SCARA base coordinate system

## Configuration

### Workspace Bounds

The system automatically calculates reachable camera positions based on:
- SCARA arm limits (J1: ±109°, J2: ±146°)
- Extension arm length (140mm camera, 45mm suction cup)
- Collision avoidance zones

### Scanning Parameters

- **Scan Height**: Typically 150mm above table surface
- **Grid Spacing**: 80mm provides good coverage with overlap
- **Camera Direction**: -90° (pointing down) is standard

### Detection Parameters

- **Confidence Threshold**: 0.7 for high-confidence detections only
- **Duplicate Distance**: 30mm threshold for considering objects as duplicates

## Output Files

The system creates a `detection_output/` directory containing:

- `detected_objects.json`: Complete detection results with metadata
- `3d_detection_results.png`: 3D visualization plot
- `captured_images/`: Individual scan images (if enabled)

### JSON Format

```json
{
  "scan_metadata": {
    "timestamp": "2024-01-01 12:00:00",
    "total_objects": 5,
    "scan_positions": 25,
    "model_path": "yolo/my_model/my_model.pt"
  },
  "detected_objects": [
    {
      "object_id": 0,
      "class_name": "block",
      "confidence": 0.85,
      "bbox": [100, 150, 200, 250],
      "center_pixel": [150, 200],
      "depth_mm": 580.5,
      "arm_coordinates": {
        "x": 250.3,
        "y": -45.7,
        "z": 25.8
      },
      "camera_position": {
        "x": 250.0,
        "y": -50.0,
        "z": 150.0
      }
    }
  ]
}
```

## Troubleshooting

### Camera Issues
- Ensure RealSense camera is connected and recognized
- Check camera permissions and drivers
- Verify camera calibration file exists

### SCARA Arm Issues
- Ensure SCARA control module is available
- Check arm is calibrated and at home position
- Verify joint limits and collision detection

### Detection Issues
- Check YOLO model path and file exists
- Adjust confidence threshold for better detection
- Ensure proper lighting conditions
- Verify object classes match model training

### Performance Issues
- Reduce grid spacing for faster scanning
- Disable image saving for better performance
- Use lower resolution camera settings

## Technical Notes

### Camera Calibration

Camera calibration improves coordinate conversion accuracy:
- Intrinsic calibration: Camera focal length, principal point, distortion
- Extrinsic calibration: Camera to arm transformation matrix

### Coordinate Conversion

The system uses the following transformation chain:
1. Pixel (u,v) + depth → Camera coordinates (Xc, Yc, Zc)
2. Camera coordinates → Arm coordinates (Xa, Ya, Za)

Without calibration, approximate camera parameters are used.

### Duplicate Filtering

Objects detected in multiple overlapping images are filtered by:
1. Grouping by object class
2. Sorting by confidence (highest first)
3. Removing objects within distance threshold of higher-confidence detections

## Future Enhancements

- Real-time detection mode
- Multi-class object support
- Adaptive scanning based on detection density
- Integration with pick-and-place operations
- Improved coordinate transformation accuracy 