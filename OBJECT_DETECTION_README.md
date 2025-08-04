# SCARA AGV Object Detection System

This system enables the SCARA arm to automatically detect objects in its workspace using a RealSense camera mounted on the extension arm. The system captures multiple images from different positions, detects objects using YOLO, and converts pixel coordinates to real-world arm coordinates.

## System Overview

The system consists of several key components:

1. **SCARA Arm Control** - Controls arm movement and positioning
2. **RealSense Camera** - Captures color and depth images
3. **YOLO Object Detection** - Detects objects in images
4. **Coordinate Transformation** - Converts pixel coordinates to arm coordinates
5. **Duplicate Removal** - Eliminates duplicate detections from overlapping images

## Key Features

- **Grid-based scanning**: Systematically scans workspace in a configurable grid pattern
- **Overlap handling**: Images overlap to ensure complete coverage, duplicates are removed
- **Real-time coordinate conversion**: Pixel coordinates converted to arm coordinate system
- **Multiple output formats**: Results saved in JSON, text, and Python formats
- **Simulation mode**: Test system without hardware dependencies

## File Structure

```
SCARA-AGV/
├── Detection_Models/
│   ├── ObjectDetection.py          # Main detection system
│   ├── yolo_detection.py           # Original YOLO detection
│   ├── YoloE.py                    # YOLO-E model
│   └── YoloWorld.py                # YOLO-World model
├── Arm_Control/
│   ├── SCARA.py                    # Real SCARA arm control
│   ├── SCARA_Simulator.py          # Simulated arm for testing
│   └── scara_workspace_domain_grapher.py
├── realsense_depth.py              # RealSense camera interface
├── test_object_detection.py        # Test script
├── execute.py                      # Original execution script
└── OBJECT_DETECTION_README.md      # This file
```

## Installation

### Prerequisites

```bash
pip install ultralytics opencv-python pyrealsense2 numpy
```

### Hardware Requirements

- SCARA arm with extension arm
- RealSense camera (D415/D435 recommended)
- YOLO model file (trained for your objects)

## Usage

### Basic Usage

```python
from Detection_Models.ObjectDetection import SCARAObjectDetection

# Initialize the system
detector = SCARAObjectDetection(
    model_path="yolo/my_model/my_model.pt",
    confidence_threshold=0.4,
    camera_height=300.0,  # mm
    grid_size=3,          # 3x3 grid
    overlap_percentage=0.3 # 30% overlap
)

# Perform detection
detected_objects = detector.scan_workspace_and_detect_objects()

# Print results
detector.print_detection_results()

# Save results
detector.save_results_to_file("results.json")
```

### Test Script

Run the test script to see the system in action:

```bash
python test_object_detection.py
```

Choose between:
1. **Full test** - Requires camera and model
2. **Simulation test** - No hardware required

## Configuration

### System Parameters

| Parameter | Description | Default | Units |
|-----------|-------------|---------|-------|
| `model_path` | Path to YOLO model file | `"yolo/my_model/my_model.pt"` | - |
| `confidence_threshold` | Minimum detection confidence | `0.4` | - |
| `camera_height` | Camera height above workspace | `300.0` | mm |
| `grid_size` | Number of positions in grid | `3` | - |
| `overlap_percentage` | Image overlap percentage | `0.3` | - |

### Camera Parameters

| Parameter | Description | Default | Units |
|-----------|-------------|---------|-------|
| `camera_fov_horizontal` | Horizontal field of view | `87` | degrees |
| `camera_fov_vertical` | Vertical field of view | `58` | degrees |
| `camera_resolution` | Image resolution | `(640, 480)` | pixels |

### Workspace Parameters

| Parameter | Description | Default | Units |
|-----------|-------------|---------|-------|
| `workspace_width` | Workspace width | `400` | mm |
| `workspace_height` | Workspace height | `400` | mm |

## Workflow

### 1. Initialization
- Load YOLO model
- Initialize RealSense camera
- Connect to SCARA arm
- Set up workspace parameters

### 2. Grid Calculation
- Calculate camera positions based on grid size and overlap
- Positions are centered on workspace

### 3. Scanning Process
For each position:
- Move arm to position
- Capture color and depth images
- Run YOLO detection
- Convert pixel coordinates to arm coordinates
- Save images (optional)

### 4. Post-Processing
- Remove duplicate objects based on spatial proximity
- Sort by confidence (highest first)
- Generate final object list

### 5. Output Generation
- Print formatted results
- Save to multiple file formats
- Generate coordinate lists for arm control

## Coordinate Systems

### Camera Coordinates
- Origin: Camera center
- X: Right direction
- Y: Down direction  
- Z: Forward direction (depth)

### Arm Coordinates
- Origin: Arm base
- X: Right direction
- Y: Forward direction
- Z: Up direction

### Transformation Process
1. Normalize pixel coordinates to [-1, 1]
2. Convert to angles using FOV
3. Calculate 3D point in camera coordinates
4. Transform to arm coordinate system

## Output Formats

### JSON Format
```json
{
  "timestamp": 1234567890.123,
  "total_objects": 5,
  "objects": [
    {
      "class_name": "object",
      "confidence": 0.85,
      "arm_coordinates": [100.5, 200.3, 50.0],
      "pixel_coordinates": [320, 240],
      "depth_mm": 250.0,
      "camera_position": [0.0, 0.0, 300.0],
      "camera_angle": -90.0,
      "timestamp": 1234567890.123
    }
  ]
}
```

### Text Format
```
# Object coordinates in SCARA arm coordinate system (mm)
# Format: x, y, z
100.5, 200.3, 50.0  # Object 1
150.2, 180.7, 45.0  # Object 2
```

### Python Format
```python
# Object coordinates for SCARA arm
# Automatically generated by ObjectDetection system

object_coordinates = [
    (100.5, 200.3, 50.0),
    (150.2, 180.7, 45.0),
]
```

## Error Handling

The system includes comprehensive error handling:

- **Model loading errors**: Check model file path
- **Camera errors**: Check RealSense connection
- **Arm errors**: Check arm communication
- **Import errors**: Install required packages

## Simulation Mode

For testing without hardware:

```python
# The system automatically falls back to simulation mode
# when hardware is not available
detector = SCARAObjectDetection()
```

Simulation mode provides:
- Mock camera images
- Simulated arm movements
- Full workflow demonstration

## Troubleshooting

### Common Issues

1. **Model not found**
   - Check model file path
   - Ensure model file exists

2. **Camera not detected**
   - Check RealSense connection
   - Install pyrealsense2 package
   - Check USB permissions

3. **Arm communication error**
   - Check serial connection
   - Verify baud rate settings
   - Check arm power

4. **No objects detected**
   - Lower confidence threshold
   - Check lighting conditions
   - Verify model training

### Debug Mode

Enable debug output by modifying the confidence threshold:

```python
detector = SCARAObjectDetection(confidence_threshold=0.1)
```

## Performance Optimization

### Grid Size
- Larger grids provide better coverage but take longer
- Smaller grids are faster but may miss objects
- Recommended: 3x3 for most applications

### Overlap Percentage
- Higher overlap reduces missed objects
- Lower overlap is faster
- Recommended: 0.3 (30%)

### Confidence Threshold
- Higher threshold reduces false positives
- Lower threshold catches more objects
- Recommended: 0.4 for general use

## Integration with Arm Control

The detected coordinates can be used directly with the SCARA arm:

```python
# Get coordinates for arm control
coords = detector.get_object_coordinates_list()

# Move to each object
for x, y, z in coords:
    arm.quick_suction(x, y, z)
    # Perform pick operation
```

## Future Enhancements

- **Multi-camera support**: Multiple cameras for better coverage
- **Dynamic grid sizing**: Adaptive grid based on object density
- **Real-time tracking**: Track objects as they move
- **3D object modeling**: Generate 3D models of detected objects
- **Path planning**: Optimize arm movement between objects

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review error messages carefully
3. Test in simulation mode first
4. Verify all hardware connections 