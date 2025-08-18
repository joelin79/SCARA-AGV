# ArUco Plate Detection System

This module adds ArUco marker detection capabilities to the SCARA AGV object detection system, allowing it to define plate areas and workspace boundaries based on edge markers.

## Overview

The ArUco plate detection system:
- Detects ArUco markers at the edges of plates during workspace scanning
- Groups markers to define rectangular plate areas
- Calculates plate dimensions, height, and orientation
- Provides workspace information for path planning and object placement

## Features

### ArUco Marker Detection
- Uses OpenCV's ArUco detector with configurable dictionary types
- Supports 6x6 ArUco markers (DICT_6X6_250)
- Configurable marker size and expected count per plate
- Robust depth measurement at marker centers

### Plate Definition
- Automatically groups nearby markers to form plates
- Calculates plate dimensions from marker positions
- Determines plate height from average depth at markers
- Computes plate orientation relative to arm coordinate system
- Filters plates by minimum area and marker proximity

### Workspace Information
- Identifies main workspace plate (largest detected plate)
- Calculates workspace bounds from all detected plates
- Provides plate center coordinates and dimensions
- Supports multiple plates for complex workspace layouts

## Usage

### Basic Integration

The ArUco detection is automatically integrated into the main `ObjectDetectionSystem`:

```python
from Detection_Models.ObjectDetection import ObjectDetectionSystem

# Initialize system (ArUco detection is automatic)
detector = ObjectDetectionSystem()
detector.initialize()

# Plan and run scanning
detector.plan_scanning_positions()
detector.scan_workspace()

# Access plate information
workspace_info = detector.get_workspace_info()
if workspace_info:
    main_plate = workspace_info['main_plate']
    print(f"Main plate: {main_plate['dimensions'][0]}x{main_plate['dimensions'][1]}mm")
    print(f"Height: {main_plate['height']}mm")
```

### Standalone ArUco Detection

You can also use the ArUco detector independently:

```python
from Detection_Models.aruco_plate_detector import ArUcoPlateDetector

# Create detector
detector = ArUcoPlateDetector(
    aruco_dict_type=cv2.aruco.DICT_6X6_250,
    marker_size_mm=30.0,
    expected_markers=4
)

# Detect markers in an image
markers = detector.detect_markers(
    color_image, depth_image, camera_position, pixel_to_arm_converter
)

# Define plates from markers
plates = detector.define_plates_from_markers()

# Get workspace information
workspace_info = detector.get_workspace_info()
```

## Configuration

### ArUco Detector Parameters

- `aruco_dict_type`: OpenCV ArUco dictionary type (default: DICT_6X6_250)
- `marker_size_mm`: Physical size of markers in millimeters (default: 30.0)
- `expected_markers`: Number of markers per plate (default: 4)

### Detection Settings

- `min_marker_confidence`: Minimum confidence for marker detection (default: 0.7)
- `max_marker_distance_mm`: Maximum distance between markers for same plate (default: 500.0)
- `min_plate_area_mm2`: Minimum plate area to consider valid (default: 10000.0)

## Marker Placement Guidelines

### For Rectangular Plates
- Place 4 ArUco markers at the corners of the plate
- Use markers with IDs 0, 1, 2, 3 for easy identification
- Ensure markers are visible from multiple camera angles
- Maintain consistent marker size and orientation

### For Workspace Definition
- Place markers at workspace boundaries
- Use larger plates for main workspace areas
- Consider camera field of view when positioning markers
- Ensure adequate lighting for marker detection

## Output Data

### Detected Objects JSON
The system now includes plate information in the output:

```json
{
  "scan_metadata": {
    "total_objects": 5,
    "total_plates": 2,
    "scan_positions": 12
  },
  "detected_objects": [...],
  "detected_plates": [
    {
      "plate_id": "plate_0_1_2_3",
      "center_arm_coordinates": {"x": 0.0, "y": 0.0, "z": 150.0},
      "dimensions": {"width_mm": 200.0, "length_mm": 200.0},
      "height_mm": 150.0,
      "area_mm2": 40000.0,
      "orientation_deg": 0.0,
      "edge_marker_ids": [0, 1, 2, 3]
    }
  ]
}
```

### Workspace Information
```python
workspace_info = detector.get_workspace_info()
# Returns:
{
  "main_plate": {
    "center": (0.0, 0.0, 150.0),
    "dimensions": (200.0, 200.0),
    "height": 150.0,
    "area": 40000.0,
    "orientation": 0.0
  },
  "total_plates": 2,
  "workspace_bounds": {
    "x_min": -100.0, "x_max": 100.0,
    "y_min": -100.0, "y_max": 100.0,
    "z_min": 150.0, "z_max": 150.0
  }
}
```

## Testing

Run the integration test to verify functionality:

```bash
cd Detection_Models
python test_aruco_integration.py
```

## Dependencies

- OpenCV (with ArUco support)
- NumPy
- The existing SCARA AGV object detection system

## Troubleshooting

### Common Issues

1. **No markers detected**: Check lighting, marker size, and camera calibration
2. **Incorrect plate dimensions**: Verify marker placement and coordinate conversion
3. **Missing plates**: Ensure markers are within detection distance limits

### Debug Information

The system provides detailed logging during detection:
- Marker detection counts per scan position
- Plate definition results
- Workspace calculation details

## Future Enhancements

- Support for non-rectangular plate shapes
- Dynamic marker size detection
- Multi-camera marker fusion
- Real-time plate tracking during operation 