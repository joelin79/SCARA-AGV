# Camera Calibration Guide for SCARA AGV

This guide explains how to properly calibrate the RealSense camera for accurate object detection and coordinate transformation.

## Why Calibration is Important

The current system uses **hardcoded camera parameters** which can lead to:
- ❌ Inaccurate coordinate transformations
- ❌ Lens distortion not corrected
- ❌ Poor depth accuracy
- ❌ Incorrect object positioning

Proper calibration provides:
- ✅ Accurate pixel-to-world coordinate conversion
- ✅ Lens distortion correction
- ✅ Precise depth measurements
- ✅ Reliable object positioning

## Calibration Types

### 1. Intrinsic Calibration
**What it does:** Calibrates the camera's internal parameters
- Focal length (fx, fy)
- Principal point (cx, cy)
- Lens distortion coefficients
- Image resolution

**When needed:** 
- When using a new camera
- After camera firmware updates
- If coordinate accuracy is poor

### 2. Extrinsic Calibration
**What it does:** Calibrates camera-to-arm transformation
- Rotation matrix (3x3)
- Translation vector (3x1)
- Camera mounting position and orientation

**When needed:**
- After camera remounting
- When changing camera position
- For precise coordinate transformation

## Prerequisites

### Hardware Requirements
- RealSense camera (D415/D435 recommended)
- SCARA arm with extension arm
- Chessboard pattern (9x6 internal corners, 20mm squares)
- Flat, well-lit workspace

### Software Requirements
```bash
pip install opencv-python pyrealsense2 numpy
```

## Step-by-Step Calibration Process

### Step 1: Prepare Chessboard Pattern

1. **Print or create a chessboard pattern:**
   - Size: 9x6 internal corners (10x7 total squares)
   - Square size: 20mm
   - Material: Rigid, flat surface
   - Pattern: Black and white squares

2. **Mount the pattern:**
   - Attach to a flat, rigid surface
   - Ensure it's not warped or bent
   - Make it easily visible to the camera

### Step 2: Run Calibration

```bash
python camera_calibration.py
```

Choose calibration mode:
- **Option 1:** Intrinsic calibration only
- **Option 2:** Full calibration (intrinsic + extrinsic)
- **Option 3:** Load existing calibration

### Step 3: Intrinsic Calibration

1. **Start intrinsic calibration:**
   - Hold chessboard in front of camera
   - Move to different angles and distances
   - Ensure pattern is fully visible

2. **Capture images:**
   - Press 'c' to capture when pattern is detected
   - Capture 15-20 images from different angles
   - Include various orientations (tilted, rotated)

3. **Best practices:**
   - Cover different areas of the image
   - Include different distances (200-500mm)
   - Vary the angle (0-45 degrees)
   - Ensure good lighting

### Step 4: Extrinsic Calibration (Optional)

1. **Position chessboard:**
   - Place chessboard in workspace
   - Ensure it's visible from multiple arm positions
   - Keep it stationary during calibration

2. **Arm movement:**
   - System moves arm to 5 different positions
   - Captures chessboard from each position
   - Calculates camera-to-arm transformation

3. **Verification:**
   - Check transformation error (should be < 5mm)
   - Review rotation matrix and translation vector

## Calibration Quality Assessment

### Good Calibration Indicators
- ✅ Reprojection error < 1.0 pixels
- ✅ Transformation error < 5mm
- ✅ Consistent results across multiple runs
- ✅ No extreme distortion coefficients

### Poor Calibration Indicators
- ❌ Reprojection error > 2.0 pixels
- ❌ Transformation error > 10mm
- ❌ Inconsistent results
- ❌ Extreme distortion values

## Calibration Files

### Generated Files
```
camera_calibration/
├── camera_calibration.json    # Main calibration file
├── calib_image_1.jpg         # Captured calibration images
├── calib_image_2.jpg
└── ...
```

### Calibration Data Structure
```json
{
  "intrinsics": {
    "fx": 1234.56,           # Focal length x
    "fy": 1234.56,           # Focal length y
    "cx": 640.0,             # Principal point x
    "cy": 360.0,             # Principal point y
    "width": 1280,           # Image width
    "height": 720,           # Image height
    "distortion_coeffs": [0.1, -0.05, 0.0, 0.0, 0.0]
  },
  "camera_matrix": [[...]],  # 3x3 camera matrix
  "dist_coeffs": [[...]],    # Distortion coefficients
  "extrinsics": {
    "rotation_matrix": [[...]],     # 3x3 rotation matrix
    "translation_vector": [...]     # 3x1 translation vector
  }
}
```

## Troubleshooting

### Common Issues

#### 1. Chessboard Not Detected
**Symptoms:** Pattern not recognized in camera view
**Solutions:**
- Check lighting conditions
- Ensure pattern is fully visible
- Clean camera lens
- Verify pattern size and quality

#### 2. Poor Calibration Quality
**Symptoms:** High reprojection error
**Solutions:**
- Capture more images (20-30)
- Vary angles and distances more
- Ensure pattern is flat and rigid
- Check for motion blur

#### 3. Extrinsic Calibration Fails
**Symptoms:** High transformation error
**Solutions:**
- Ensure chessboard is stationary
- Check arm positioning accuracy
- Verify depth measurements
- Use more calibration positions

#### 4. Camera Connection Issues
**Symptoms:** Camera not detected
**Solutions:**
- Check USB connection
- Install pyrealsense2 package
- Verify camera permissions
- Test with RealSense Viewer

### Debug Mode

Enable verbose output for troubleshooting:
```python
# In camera_calibration.py, add debug prints
print(f"Camera matrix: {camera_matrix}")
print(f"Distortion: {dist_coeffs}")
print(f"Reprojection error: {mean_error}")
```

## Integration with Object Detection

### Automatic Loading
The object detection system automatically loads calibration:
```python
detector = SCARAObjectDetection()
# Calibration is loaded automatically if available
```

### Manual Loading

```python
from RealSense.camera_calibration import CameraCalibrator

calibrator = CameraCalibrator()
if calibrator.load_calibration():
    # Use calibrated parameters
    arm_x, arm_y, arm_z = calibrator.pixel_to_arm_coordinates(pixel_x, pixel_y, depth)
```

### Verification
Test calibration accuracy:
1. Place known objects in workspace
2. Run object detection
3. Measure actual vs. detected positions
4. Adjust calibration if needed

## Maintenance

### When to Recalibrate
- After camera firmware updates
- When changing camera position
- If coordinate accuracy degrades
- After significant temperature changes
- Every 6 months (recommended)

### Calibration Validation
Regular validation process:
1. Place test objects at known positions
2. Run detection system
3. Compare detected vs. actual coordinates
4. Document accuracy metrics

## Advanced Calibration

### Multi-Camera Calibration
For systems with multiple cameras:
1. Calibrate each camera individually
2. Use overlapping views for relative calibration
3. Establish common coordinate system

### Dynamic Calibration
For changing environments:
1. Use fiducial markers
2. Implement online calibration
3. Adaptive parameter adjustment

## Support

For calibration issues:
1. Check this guide first
2. Review error messages carefully
3. Verify hardware setup
4. Test with RealSense Viewer
5. Check calibration file format

## Example Workflow

```bash
# 1. Prepare workspace
# - Mount chessboard pattern
# - Ensure good lighting
# - Position camera

# 2. Run calibration
python camera_calibration.py
# Choose option 2 (full calibration)

# 3. Follow on-screen instructions
# - Capture 20 images
# - Move arm to 5 positions
# - Verify calibration quality

# 4. Test calibration
python test_object_detection.py
# Check coordinate accuracy

# 5. Use in production
python example_usage.py
# Calibration loaded automatically
```

This calibration process ensures accurate object detection and precise coordinate transformation for your SCARA AGV system. 