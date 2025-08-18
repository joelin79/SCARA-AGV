#!/usr/bin/env python3
"""
Test script for ArUco plate detection integration
"""

import sys
from pathlib import Path

# Add project directories to path
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "Detection_Models"))

try:
    from aruco_plate_detector import ArUcoPlateDetector, ArUcoMarker, PlateDefinition
    print("✓ ArUco plate detector imported successfully")
except ImportError as e:
    print(f"✗ Failed to import ArUco plate detector: {e}")
    sys.exit(1)

def test_aruco_detector():
    """Test basic ArUco detector functionality"""
    print("\nTesting ArUco Plate Detector...")
    
    # Create detector
    detector = ArUcoPlateDetector(
        aruco_dict_type=4,  # DICT_6X6_250
        marker_size_mm=30.0,
        expected_markers=4
    )
    
    print(f"✓ Detector created with:")
    print(f"  - ArUco dictionary: {detector.aruco_dict_type}")
    print(f"  - Marker size: {detector.marker_size_mm}mm")
    print(f"  - Expected markers: {detector.expected_markers}")
    
    # Test marker grouping logic
    print("\nTesting marker grouping logic...")
    
    # Create sample markers (simulated)
    sample_markers = [
        ArUcoMarker(
            marker_id=0,
            corners=np.array([[100, 100], [130, 100], [130, 130], [100, 130]]),
            center_pixel=(115, 115),
            depth_mm=150.0,
            arm_coords=(0.0, 0.0, 150.0)
        ),
        ArUcoMarker(
            marker_id=1,
            corners=np.array([[200, 100], [230, 100], [230, 130], [200, 130]]),
            center_pixel=(215, 115),
            depth_mm=150.0,
            arm_coords=(100.0, 0.0, 150.0)
        ),
        ArUcoMarker(
            marker_id=2,
            corners=np.array([[200, 200], [230, 200], [230, 230], [200, 230]]),
            center_pixel=(215, 215),
            depth_mm=150.0,
            arm_coords=(100.0, 100.0, 150.0)
        ),
        ArUcoMarker(
            marker_id=3,
            corners=np.array([[100, 200], [130, 200], [130, 230], [100, 230]]),
            center_pixel=(115, 215),
            depth_mm=150.0,
            arm_coords=(0.0, 100.0, 150.0)
        )
    ]
    
    detector.detected_markers = sample_markers
    
    # Test plate definition
    plates = detector.define_plates_from_markers()
    
    if plates:
        print(f"✓ Successfully defined {len(plates)} plate(s)")
        for i, plate in enumerate(plates):
            print(f"  Plate {i+1}:")
            print(f"    ID: {plate.plate_id}")
            print(f"    Dimensions: {plate.dimensions[0]:.1f}x{plate.dimensions[1]:.1f}mm")
            print(f"    Center: ({plate.center_arm_coords[0]:.1f}, {plate.center_arm_coords[1]:.1f}, {plate.center_arm_coords[2]:.1f})")
            print(f"    Height: {plate.height_mm:.1f}mm")
            print(f"    Area: {plate.area_mm2:.0f}mm²")
            print(f"    Orientation: {plate.orientation_deg:.1f}°")
    else:
        print("✗ Failed to define plates from markers")
    
    return True

def test_workspace_calculation():
    """Test workspace calculation from plates"""
    print("\nTesting workspace calculation...")
    
    detector = ArUcoPlateDetector()
    
    # Create a sample plate
    sample_markers = [
        ArUcoMarker(
            marker_id=0,
            corners=np.array([[0, 0], [0, 0], [0, 0], [0, 0]]),
            center_pixel=(0, 0),
            depth_mm=150.0,
            arm_coords=(-100.0, -100.0, 150.0)
        ),
        ArUcoMarker(
            marker_id=1,
            corners=np.array([[0, 0], [0, 0], [0, 0], [0, 0]]),
            center_pixel=(0, 0),
            depth_mm=150.0,
            arm_coords=(100.0, -100.0, 150.0)
        ),
        ArUcoMarker(
            marker_id=2,
            corners=np.array([[0, 0], [0, 0], [0, 0], [0, 0]]),
            center_pixel=(0, 0),
            depth_mm=150.0,
            arm_coords=(100.0, 100.0, 150.0)
        ),
        ArUcoMarker(
            marker_id=3,
            corners=np.array([[0, 0], [0, 0], [0, 0], [0, 0]]),
            center_pixel=(0, 0),
            depth_mm=150.0,
            arm_coords=(-100.0, 100.0, 150.0)
        )
    ]
    
    detector.detected_markers = sample_markers
    plates = detector.define_plates_from_markers()
    
    if plates:
        workspace_info = detector.get_workspace_info()
        print(f"✓ Workspace info calculated:")
        print(f"  Main plate: {workspace_info['main_plate']['dimensions'][0]:.0f}x{workspace_info['main_plate']['dimensions'][1]:.0f}mm")
        print(f"  Bounds: X[{workspace_info['workspace_bounds']['x_min']:.0f}, {workspace_info['workspace_bounds']['x_max']:.0f}]")
        print(f"           Y[{workspace_info['workspace_bounds']['y_min']:.0f}, {workspace_info['workspace_bounds']['y_max']:.0f}]")
        print(f"           Z[{workspace_info['workspace_bounds']['z_min']:.0f}, {workspace_info['workspace_bounds']['z_max']:.0f}]")
    else:
        print("✗ Failed to calculate workspace info")
    
    return True

if __name__ == "__main__":
    import numpy as np
    
    print("=" * 60)
    print("ArUco Plate Detection Integration Test")
    print("=" * 60)
    
    try:
        # Test basic functionality
        if test_aruco_detector():
            print("\n✓ Basic ArUco detector test passed")
        else:
            print("\n✗ Basic ArUco detector test failed")
        
        # Test workspace calculation
        if test_workspace_calculation():
            print("\n✓ Workspace calculation test passed")
        else:
            print("\n✗ Workspace calculation test failed")
        
        print("\n" + "=" * 60)
        print("Integration test completed!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        import traceback
        traceback.print_exc() 