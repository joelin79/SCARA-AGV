#!/usr/bin/env python3
"""
Test script to verify the fix for the detections variable issue
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_detections_fix():
    """Test that the detections variable is properly defined"""
    print("Testing detections variable fix...")
    
    try:
        from Detection_Models.ObjectDetection import ObjectDetectionSystem
        
        # Initialize system
        detector = ObjectDetectionSystem(
            model_path="yolo/my_model/my_model.pt",
            use_calibration=True,
            save_images=True
        )
        
        print("✓ ObjectDetectionSystem imported successfully")
        
        # Check if the scan_workspace method exists and has the correct structure
        import inspect
        source = inspect.getsource(detector.scan_workspace)
        
        # Look for the pattern where detections is used before being defined
        if "detections" in source and "self._detect_objects" in source:
            # Find the line numbers where detections is first used and first defined
            lines = source.split('\n')
            detections_used_line = None
            detections_defined_line = None
            
            for i, line in enumerate(lines):
                if "detections" in line and "self._create_annotated_detection_image" in line:
                    detections_used_line = i + 1
                if "detections = self._detect_objects" in line:
                    detections_defined_line = i + 1
            
            if detections_used_line and detections_defined_line:
                if detections_defined_line < detections_used_line:
                    print("✓ Fix verified: detections is defined before being used")
                    print(f"  Defined at line {detections_defined_line}")
                    print(f"  Used at line {detections_used_line}")
                else:
                    print("❌ Issue still exists: detections used before being defined")
                    print(f"  Used at line {detections_used_line}")
                    print(f"  Defined at line {detections_defined_line}")
            else:
                print("⚠ Could not verify the fix - pattern not found")
        
        print("✓ Test completed")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_detections_fix() 