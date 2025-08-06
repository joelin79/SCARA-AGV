#!/usr/bin/env python3
"""
Test script for annotated detection images
Demonstrates the new functionality to save detection images with annotations
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def test_annotated_detections():
    """Test the annotated detection image functionality"""
    print("=" * 60)
    print("ANNOTATED DETECTION IMAGES TEST")
    print("=" * 60)
    
    try:
        from Detection_Models.ObjectDetection import ObjectDetectionSystem
        
        print("\n1. Initializing Object Detection System...")
        detector = ObjectDetectionSystem(
            model_path="yolo/my_model/my_model.pt",
            use_calibration=True,
            save_images=True
        )
        
        if not detector.initialize():
            print("❌ Failed to initialize detection system")
            return
        
        print("\n2. Planning scanning positions...")
        detector.plan_scanning_positions(
            scan_height=150.0,
            grid_spacing=100.0,
            camera_direction=-90.0
        )
        
        print(f"   Planned {len(detector.scan_positions)} scan positions")
        
        print("\n3. Testing single position scan with annotated images...")
        
        # Test with just the first position to see the annotated images
        if detector.scan_positions:
            cam_x, cam_y, cam_z = detector.scan_positions[0]
            print(f"   Testing position: ({cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f})")
            
            # Move camera to position
            if detector._move_camera_to_position(cam_x, cam_y, cam_z):
                print("   ✓ Camera moved successfully")
                
                # Wait for stabilization
                import time
                time.sleep(2)
                
                # Capture image
                image_data = detector._capture_image()
                if image_data is not None:
                    color_image, depth_image = image_data
                    print("   ✓ Image captured successfully")
                    
                    # Run detection
                    detections = detector._detect_objects(
                        color_image, depth_image, 
                        (cam_x, cam_y, cam_z), 
                        confidence_threshold=0.4
                    )
                    
                    print(f"   ✓ Found {len(detections)} objects")
                    
                    # Create and save annotated image
                    annotated_image = detector._create_annotated_detection_image(
                        color_image, detections, (cam_x, cam_y, cam_z), 1
                    )
                    
                    # Save annotated image
                    output_path = detector.images_dir / "test_annotated_detection.jpg"
                    import cv2
                    cv2.imwrite(str(output_path), annotated_image)
                    print(f"   ✓ Annotated image saved: {output_path}")
                    
                    # Also save raw image for comparison
                    raw_path = detector.images_dir / "test_raw_image.jpg"
                    cv2.imwrite(str(raw_path), color_image)
                    print(f"   ✓ Raw image saved: {raw_path}")
                    
                    # Show detection details
                    for i, obj in enumerate(detections):
                        print(f"     Object {i+1}: {obj.class_name} ({obj.confidence:.2f})")
                        print(f"       Pixel: ({obj.center_pixel[0]}, {obj.center_pixel[1]})")
                        print(f"       Depth: {obj.depth_mm:.1f} mm")
                        print(f"       Arm: ({obj.arm_coords[0]:.1f}, {obj.arm_coords[1]:.1f}, {obj.arm_coords[2]:.1f})")
                else:
                    print("   ❌ Failed to capture image")
            else:
                print("   ❌ Failed to move camera")
        else:
            print("   ❌ No scan positions planned")
        
        print("\n" + "=" * 60)
        print("ANNOTATED DETECTION TEST COMPLETED!")
        print("=" * 60)
        print("\nExpected output files:")
        print("  - test_annotated_detection.jpg: Image with bounding boxes, labels, and coordinates")
        print("  - test_raw_image.jpg: Original captured image")
        print("\nAnnotation features:")
        print("  ✓ Camera position info in top left")
        print("  ✓ Timestamp")
        print("  ✓ Bounding boxes with class names and confidence")
        print("  ✓ Center points (pink dots)")
        print("  ✓ Pixel coordinates with depth")
        print("  ✓ Arm coordinates")
        print("  ✓ Object count")
        
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure all required modules are available")
    except Exception as e:
        print(f"❌ Error during test: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_annotated_detections() 