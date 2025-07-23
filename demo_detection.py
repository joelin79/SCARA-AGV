#!/usr/bin/env python3
"""
Demo script for testing object detection components without moving the SCARA arm
This is useful for testing camera setup, YOLO detection, and coordinate transformation
"""

import cv2
import numpy as np
from ObjectDetection import SCARAObjectDetection

def test_camera(camera_index=0):
    """Test camera connection and capture"""
    print("Testing camera connection...")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera {camera_index}")
        return False
    
    # Set resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    print("Camera opened successfully!")
    print("Press 's' to save test image, 'q' to quit")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break
        
        cv2.imshow('Camera Test - Press s to save, q to quit', frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('test_image.jpg', frame)
            print("Test image saved as test_image.jpg")
    
    cap.release()
    cv2.destroyAllWindows()
    return True

def test_yolo_detection():
    """Test YOLO detection on a static image"""
    print("Testing YOLO detection...")
    
    # Check if test image exists
    try:
        test_image = cv2.imread('test_image.jpg')
        if test_image is None:
            print("No test image found. Run camera test first to capture one.")
            return False
    except:
        print("Error loading test image")
        return False
    
    # Initialize detector
    detector = SCARAObjectDetection(
        model_path="yolo/my_model/my_model.pt",
        confidence_threshold=0.3
    )
    
    # Test detection on static position (don't move arm)
    test_arm_position = (200, 0, 200)  # Example position
    detections = detector.detect_objects_in_image(test_image, test_arm_position)
    
    print(f"Found {len(detections)} objects")
    for i, det in enumerate(detections):
        print(f"  {i+1}. {det['class_name']} ({det['confidence']:.2f}) at {det['arm_coordinates']}")
    
    # Save annotated image
    if detections:
        detector.save_detection_image(test_image, detections, "test_detection_result.jpg")
        print("Annotated image saved as test_detection_result.jpg")
    
    return True

def test_coordinate_transformation():
    """Test pixel to world coordinate transformation"""
    print("Testing coordinate transformation...")
    
    detector = SCARAObjectDetection()
    
    # Test with some example pixels
    test_cases = [
        (640, 360, "Image center"),
        (320, 180, "Upper left quadrant"),
        (960, 540, "Lower right quadrant")
    ]
    
    arm_position = (200, 0, 200)  # Example arm position
    
    for pixel_x, pixel_y, description in test_cases:
        world_x, world_y, world_z = detector.pixel_to_world_coordinates(
            pixel_x, pixel_y, *arm_position
        )
        print(f"{description}: pixel({pixel_x}, {pixel_y}) -> world({world_x:.1f}, {world_y:.1f}, {world_z:.1f})")

def demo_single_image_detection():
    """Demo detection on a single image without moving the arm"""
    print("\nSingle Image Detection Demo")
    print("="*40)
    
    # Test camera first
    if not test_camera():
        return
    
    # Test YOLO detection
    if not test_yolo_detection():
        return
    
    # Test coordinate transformation
    test_coordinate_transformation()
    
    print("\nDemo complete! Check the output images:")
    print("  - test_image.jpg (captured image)")
    print("  - test_detection_result.jpg (with detections)")

def main():
    """Main demo menu"""
    print("SCARA Object Detection Demo")
    print("="*30)
    print("1. Test camera connection")
    print("2. Test YOLO detection on static image")
    print("3. Test coordinate transformation")
    print("4. Run complete single-image demo")
    print("5. Exit")
    
    while True:
        try:
            choice = input("\nEnter your choice (1-5): ").strip()
            
            if choice == '1':
                test_camera()
            elif choice == '2':
                test_yolo_detection()
            elif choice == '3':
                test_coordinate_transformation()
            elif choice == '4':
                demo_single_image_detection()
            elif choice == '5':
                print("Goodbye!")
                break
            else:
                print("Invalid choice. Please enter 1-5.")
                
        except KeyboardInterrupt:
            print("\nGoodbye!")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main() 