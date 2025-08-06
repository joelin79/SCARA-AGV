#!/usr/bin/env python3
"""
Test script to verify the new timestamp-based file naming convention
"""

import sys
import os
import time
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_timestamp_naming():
    """Test the new timestamp-based file naming"""
    print("=" * 60)
    print("TIMESTAMP-BASED FILE NAMING TEST")
    print("=" * 60)
    
    try:
        from Detection_Models.ObjectDetection import ObjectDetectionSystem
        
        print("\n1. Creating ObjectDetectionSystem...")
        detector = ObjectDetectionSystem(
            model_path="yolo/my_model/my_model.pt",
            use_calibration=True,
            save_images=True
        )
        
        print(f"   Timestamp generated: {detector.timestamp}")
        print(f"   Format: MMDDHHMM = {detector.timestamp}")
        
        # Parse the timestamp to show what it means
        if len(detector.timestamp) == 8:
            mm = detector.timestamp[0:2]
            dd = detector.timestamp[2:4]
            hh = detector.timestamp[4:6]
            mm_time = detector.timestamp[6:8]
            print(f"   Month: {mm}")
            print(f"   Day: {dd}")
            print(f"   Hour: {hh}")
            print(f"   Minute: {mm_time}")
        
        print("\n2. Expected file naming pattern:")
        print(f"   Annotated images: detection_{detector.timestamp}_001.jpg")
        print(f"   Raw images: raw_{detector.timestamp}_001.jpg")
        
        print("\n3. Example filenames for scan positions:")
        for i in range(1, 4):
            detection_name = f"detection_{detector.timestamp}_{i:03d}.jpg"
            raw_name = f"raw_{detector.timestamp}_{i:03d}.jpg"
            print(f"   Position {i}: {detection_name}, {raw_name}")
        
        print("\n" + "=" * 60)
        print("TIMESTAMP NAMING TEST COMPLETED!")
        print("=" * 60)
        print("\nBenefits of new naming:")
        print("  ✓ Files are organized by initialization time")
        print("  ✓ Easy to identify different scanning sessions")
        print("  ✓ No file conflicts between different runs")
        print("  ✓ Clear distinction between detection and raw images")
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure all required modules are available")
    except Exception as e:
        print(f"❌ Error during test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_timestamp_naming() 