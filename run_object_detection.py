#!/usr/bin/env python3
"""
Simple script to run the object detection system with annotated images
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

def main():
    """Run the object detection system"""
    print("=" * 60)
    print("SCARA AGV Object Detection System")
    print("=" * 60)
    
    try:
        from Detection_Models.ObjectDetection import ObjectDetectionSystem
        
        # Initialize system with annotated images enabled
        detector = ObjectDetectionSystem(
            model_path="yolo/my_model/my_model.pt",
            use_calibration=True,
            save_images=True  # Enable annotated image saving
        )
        
        # Initialize all components
        if not detector.initialize():
            print("❌ Failed to initialize system")
            return
        
        # Plan scanning positions
        print("\nPlanning scanning positions...")
        detector.plan_scanning_positions(
            scan_height=150.0,     # 150mm above table
            grid_spacing=100.0,    # 100mm between scan points
            camera_direction=-90.0  # Camera pointing down
        )
        
        print(f"✅ Planned {len(detector.scan_positions)} scanning positions")
        
        # Ask for confirmation before starting
        response = input("Start scanning? This will move the SCARA arm. (y/n): ")
        if response.lower() != 'y':
            print("Scanning cancelled.")
            return
        
        # Run the scan
        print("\nStarting workspace scan...")
        if not detector.scan_workspace(confidence_threshold=0.5):
            print("❌ Scanning failed")
            return
        
        # Save results
        print("\nSaving results...")
        detector.save_results("detected_objects.json")
        
        # Create visualization
        print("\nCreating 3D visualization...")
        detector.visualize_3d()
        
        print(f"\n✅ Detection completed successfully!")
        print(f"Total objects detected: {len(detector.detected_objects)}")
        print(f"Results saved to: {detector.output_dir}")
        print(f"Annotated images saved to: {detector.images_dir}")
        
    except KeyboardInterrupt:
        print("\n⚠️ Detection interrupted by user")
    except Exception as e:
        print(f"❌ Error during detection: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        if 'detector' in locals():
            detector.cleanup()

if __name__ == "__main__":
    main() 