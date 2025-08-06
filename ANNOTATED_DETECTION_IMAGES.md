# Annotated Detection Images

## Overview

The ObjectDetection system now saves annotated detection images similar to `yolo_detection.py`, but with enhanced information including both camera pixel coordinates (with depth) and arm coordinates, plus camera position information.

## Features

### 1. Camera Position Information
- **Location**: Top left of the image
- **Content**: Camera position number and coordinates (x, y, z)
- **Format**: `Camera Pos 1: (150.0, 100.0, 150.0)`

### 2. Timestamp
- **Location**: Below camera position info
- **Content**: Current date and time
- **Format**: `2024-01-15 14:30:25`

### 3. Bounding Boxes
- **Style**: Colored rectangles around detected objects
- **Colors**: Tableau 10 color scheme (different color per class)
- **Thickness**: 2 pixels

### 4. Class Labels
- **Content**: Class name and confidence percentage
- **Style**: Colored background with black text
- **Format**: `object_name: 85%`

### 5. Center Point Markers
- **Style**: Pink dots at object centers
- **Size**: 4 pixel radius
- **Color**: (255, 0, 255) - Magenta

### 6. Coordinate Labels
Each detected object shows two coordinate labels:

#### Pixel Coordinates with Depth
- **Format**: `Pix:(320,240) D:150mm`
- **Content**: 
  - Pixel coordinates (x, y)
  - Depth in millimeters
- **Position**: Above center point

#### Arm Coordinates
- **Format**: `Arm:(150.5,100.2,50.1)`
- **Content**: 3D coordinates in arm coordinate system (x, y, z)
- **Position**: Below pixel coordinates

### 7. Object Count
- **Location**: Top left, below timestamp
- **Format**: `Objects detected: 3`

## File Naming Convention

When `save_images=True`, the system saves two files per scan position:

1. **Annotated Image**: `detection_MMDDHHmm_001.jpg`
2. **Raw Image**: `raw_MMDDHHmm_001.jpg`

Where `MMDDHHmm` is the timestamp when the program was initialized:
- `MM`: Month (01-12)
- `DD`: Day (01-31) 
- `HH`: Hour (00-23)
- `mm`: Minute (00-59)

Example: If initialized on January 15th at 14:30, files would be:
- `detection_01151430_001.jpg`
- `raw_01151430_001.jpg`

## Example Output

```
detection_output/
├── captured_images/
│   ├── detection_01151430_001.jpg  # Annotated with bounding boxes and labels
│   ├── raw_01151430_001.jpg        # Original captured image
│   ├── detection_01151430_002.jpg
│   ├── raw_01151430_002.jpg
│   └── ...
└── detected_objects.json           # Detection results data
```

## Implementation Details

### Method: `_create_annotated_detection_image()`

```python
def _create_annotated_detection_image(self, color_image: np.ndarray, 
                                    detections: List[DetectedObject],
                                    camera_position: Tuple[float, float, float],
                                    scan_position: int) -> np.ndarray:
```

**Parameters:**
- `color_image`: Original captured image
- `detections`: List of detected objects
- `camera_position`: Current camera position (x, y, z)
- `scan_position`: Scan position number

**Returns:**
- Annotated image with all visual elements

### Dynamic Font Scaling

The system automatically adjusts font size based on image resolution:
- **High resolution** (≥1600x1200): Font scale 1.2, thickness 2
- **Medium resolution** (≥1200x): Font scale 0.9, thickness 2
- **Standard resolution**: Font scale 0.5, thickness 1

### Color Scheme

Uses the Tableau 10 color palette for consistent, distinguishable colors:
```python
bbox_colors = [
    (164, 120, 87),   # Brown
    (68, 148, 228),   # Blue
    (93, 97, 209),    # Purple
    (178, 182, 133),  # Olive
    (88, 159, 106),   # Green
    (96, 202, 231),   # Light Blue
    (159, 124, 168),  # Pink
    (169, 162, 241),  # Light Purple
    (98, 118, 150),   # Gray
    (172, 176, 184)   # Light Gray
]
```

## Usage

### Basic Usage
```python
from Detection_Models.ObjectDetection import ObjectDetectionSystem

# Initialize with image saving enabled
detector = ObjectDetectionSystem(
    model_path="yolo/my_model/my_model.pt",
    save_images=True  # Enable annotated image saving
)

# Initialize and scan
detector.initialize()
detector.plan_scanning_positions()
detector.scan_workspace()
```

### Testing
Run the test script to see the functionality:
```bash
python test_annotated_detections.py
```

## Benefits

1. **Visual Verification**: Easy to verify detection results visually
2. **Coordinate Reference**: Both pixel and arm coordinates displayed
3. **Position Tracking**: Camera position clearly marked
4. **Debugging**: Raw images saved for comparison
5. **Documentation**: Timestamps for tracking when scans were performed
6. **Analysis**: Object counts and confidence levels visible

## Comparison with yolo_detection.py

| Feature | yolo_detection.py | ObjectDetection.py |
|---------|------------------|-------------------|
| Bounding boxes | ✓ | ✓ |
| Class labels | ✓ | ✓ |
| Center points | ✓ | ✓ |
| Pixel coordinates | ✓ | ✓ |
| Depth information | ✗ | ✓ |
| Arm coordinates | ✗ | ✓ |
| Camera position | ✗ | ✓ |
| Timestamp | ✗ | ✓ |
| Object count | ✓ | ✓ |
| Raw image backup | ✗ | ✓ |

## Troubleshooting

### No Annotated Images Saved
1. Check that `save_images=True` in constructor
2. Verify `images_dir` exists and is writable
3. Ensure detections are found (confidence threshold may be too high)

### Poor Text Visibility
1. Check image resolution - font scaling is automatic
2. Verify image contrast - text is in yellow (0, 255, 255)
3. Ensure image is not too dark or bright

### Missing Coordinate Information
1. Verify depth camera is working
2. Check that objects have valid depth values
3. Ensure camera calibration is loaded (if using calibration) 