#!/usr/bin/env python3
"""
Simple Example: SCARA Object Detection System
This script demonstrates the basic usage of the object detection system
"""

import os
import sys

# Add Detection_Models to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'Detection_Models'))

def main():
    """Main example function"""
    print("SCARA Object Detection - Simple Example")
    print("=" * 50)
    
    try:
        # Import the detection system
        from ObjectDetection import SCARAObjectDetection
        
        print("\nStep 1: Initialize the detection system")
        print("-" * 40)
        
        # Create detector with default settings
        detector = SCARAObjectDetection(
            model_path="yolo/my_model/my_model.pt",
            confidence_threshold=0.4,
            camera_height=300.0,
            grid_size=3,
            overlap_percentage=0.3
        )
        
        print("✓ Detection system initialized")
        print(f"  - Model: {detector.model_path}")
        print(f"  - Confidence threshold: {detector.confidence_threshold}")
        print(f"  - Camera height: {detector.camera_height} mm")
        print(f"  - Grid size: {detector.grid_size}x{detector.grid_size}")
        print(f"  - Overlap: {detector.overlap_percentage*100}%")
        
        print("\nStep 2: Calculate scanning positions")
        print("-" * 40)
        
        # Get the positions where the arm will move
        positions = detector.calculate_grid_positions()
        print(f"✓ Calculated {len(positions)} scanning positions:")
        for i, (x, y) in enumerate(positions):
            print(f"  Position {i+1}: ({x:.1f}, {y:.1f}) mm")
        
        print("\nStep 3: Perform workspace scan")
        print("-" * 40)
        print("This will:")
        print("  - Move the arm to each position")
        print("  - Capture color and depth images")
        print("  - Detect objects in each image")
        print("  - Convert coordinates to arm system")
        print("  - Remove duplicate objects")
        print("\nStarting scan... (Press Ctrl+C to stop)")
        
        # Perform the detection
        detected_objects = detector.scan_workspace_and_detect_objects(save_images=True)
        detector.detected_objects = detected_objects
        
        print("\nStep 4: Display results")
        print("-" * 40)
        
        if detected_objects:
            print(f"✓ Found {len(detected_objects)} unique objects:")
            detector.print_detection_results()
        else:
            print("✗ No objects detected")
            print("  - Check if objects are in the workspace")
            print("  - Try lowering the confidence threshold")
            print("  - Verify the YOLO model is trained for your objects")
        
        print("\nStep 5: Save results")
        print("-" * 40)
        
        # Save in multiple formats
        detector.save_results_to_file("example_results.json")
        
        # Get coordinates for arm control
        coords = detector.get_object_coordinates_list()
        if coords:
            print("✓ Object coordinates for arm control:")
            for i, (x, y, z) in enumerate(coords):
                print(f"  Object {i+1}: ({x:.1f}, {y:.1f}, {z:.1f}) mm")
            
            # Save as text file
            with open("example_coordinates.txt", "w") as f:
                f.write("# Example object coordinates\n")
                f.write("# Format: x, y, z (mm)\n")
                for i, (x, y, z) in enumerate(coords):
                    f.write(f"{x:.1f}, {y:.1f}, {z:.1f}  # Object {i+1}\n")
            
            print("✓ Coordinates saved to 'example_coordinates.txt'")
        
        print("\nStep 6: Cleanup")
        print("-" * 40)
        
        # Clean up resources
        detector.cleanup()
        print("✓ Resources cleaned up")
        
        print("\n" + "=" * 50)
        print("Example completed successfully!")
        print("=" * 50)
        print("\nFiles created:")
        print("  - example_results.json (detailed results)")
        print("  - example_coordinates.txt (coordinates for arm)")
        print("  - detections_output/ (captured images)")
        
        print("\nNext steps:")
        print("  1. Review the detected objects")
        print("  2. Use coordinates with your arm control system")
        print("  3. Adjust parameters if needed")
        print("  4. Run again to detect new objects")
        
    except KeyboardInterrupt:
        print("\n\nExample interrupted by user (Ctrl+C)")
        print("Partial results may be available")
        
    except ImportError as e:
        print(f"\nImport error: {e}")
        print("Make sure all required packages are installed:")
        print("  pip install ultralytics opencv-python pyrealsense2 numpy")
        
    except FileNotFoundError as e:
        print(f"\nFile not found: {e}")
        print("Make sure the YOLO model file exists at: yolo/my_model/my_model.pt")
        
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        print("Check the error message above for details")


def quick_test():
    """Quick test without hardware"""
    print("Quick Test (Simulation Mode)")
    print("=" * 30)
    
    try:
        # Create mock camera for testing
        class MockCamera:
            def __init__(self):
                print("Mock camera initialized")
            
            def get_frame(self):
                import numpy as np
                color_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                depth_image = np.random.randint(100, 500, (480, 640), dtype=np.uint16)
                return True, depth_image, color_image
            
            def release(self):
                print("Mock camera released")
        
        # Patch the camera import
        import Detection_Models.ObjectDetection as od
        od.DepthCamera = MockCamera
        
        # Import and test
        from Detection_Models.ObjectDetection import SCARAObjectDetection
        
        detector = SCARAObjectDetection(
            model_path="yolo/my_model/my_model.pt",
            grid_size=2  # Smaller grid for quick test
        )
        
        print("Running quick test...")
        detected_objects = detector.scan_workspace_and_detect_objects(save_images=False)
        
        print(f"Test completed. Found {len(detected_objects)} objects (simulated)")
        detector.cleanup()
        
    except Exception as e:
        print(f"Quick test error: {e}")


if __name__ == "__main__":
    print("Choose example mode:")
    print("1. Full example (requires camera and model)")
    print("2. Quick test (simulation mode)")
    
    try:
        choice = input("Enter choice (1 or 2): ").strip()
        
        if choice == "1":
            main()
        elif choice == "2":
            quick_test()
        else:
            print("Invalid choice. Running quick test...")
            quick_test()
            
    except KeyboardInterrupt:
        print("\nExample cancelled by user")
    except Exception as e:
        print(f"Error: {e}")
        print("Running quick test as fallback...")
        quick_test() 