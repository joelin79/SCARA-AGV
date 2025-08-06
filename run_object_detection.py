#!/usr/bin/env python3
"""
Simple script to run the object detection system
"""

import sys
import argparse
from pathlib import Path

# Add the Detection_Models directory to path
sys.path.append(str(Path(__file__).parent / "Detection_Models"))

from ObjectDetection import ObjectDetectionSystem

def main():
    parser = argparse.ArgumentParser(description="Run SCARA AGV Object Detection")
    parser.add_argument("--model", default="yolo/my_model/my_model.pt", 
                       help="Path to YOLO model (default: yolo/my_model/my_model.pt)")
    parser.add_argument("--height", type=float, default=150.0,
                       help="Scan height in mm (default: 150.0)")
    parser.add_argument("--spacing", type=float, default=80.0,
                       help="Grid spacing in mm (default: 80.0)")
    parser.add_argument("--confidence", type=float, default=0.7,
                       help="YOLO confidence threshold (default: 0.7)")
    parser.add_argument("--no-calibration", action="store_true",
                       help="Don't use camera calibration")
    parser.add_argument("--no-images", action="store_true",
                       help="Don't save captured images")
    parser.add_argument("--simulate", action="store_true",
                       help="Run in simulation mode (no arm movement)")
    
    args = parser.parse_args()
    
    print("SCARA AGV Object Detection System")
    print("=" * 50)
    print(f"Model: {args.model}")
    print(f"Scan height: {args.height}mm")
    print(f"Grid spacing: {args.spacing}mm")
    print(f"Confidence threshold: {args.confidence}")
    print(f"Use calibration: {not args.no_calibration}")
    print(f"Save images: {not args.no_images}")
    print(f"Simulation mode: {args.simulate}")
    print("=" * 50)
    
    # Initialize detection system
    detector = ObjectDetectionSystem(
        model_path=args.model,
        use_calibration=not args.no_calibration,
        save_images=not args.no_images
    )
    
    try:
        # Initialize components
        if not detector.initialize():
            print("❌ Failed to initialize system")
            return 1
        
        # Plan scanning positions
        positions = detector.plan_scanning_positions(
            scan_height=args.height,
            grid_spacing=args.spacing,
            camera_direction=-90.0
        )
        
        if len(positions) == 0:
            print("❌ No valid scanning positions found")
            return 1
        
        print(f"✅ Planned {len(positions)} scanning positions")
        
        # Ask for confirmation
        if not args.simulate:
            response = input("Start scanning? This will move the SCARA arm. (y/n): ")
            if response.lower() != 'y':
                print("Scanning cancelled")
                return 0
        
        # Run the scan
        if detector.scan_workspace(confidence_threshold=args.confidence):
            print("✅ Scanning completed successfully")
            
            # Save results
            detector.save_results("detected_objects.json")
            print("✅ Results saved")
            
            # Create visualization
            detector.visualize_3d()
            print("✅ 3D visualization created")
            
            print(f"\n🎯 Detection Summary:")
            print(f"   Total objects detected: {len(detector.detected_objects)}")
            print(f"   Results saved to: {detector.output_dir}")
            
            # Print object details
            if detector.detected_objects:
                print(f"\n📋 Detected Objects:")
                for i, obj in enumerate(detector.detected_objects):
                    x, y, z = obj.arm_coords
                    print(f"   {i+1}. {obj.class_name} (conf: {obj.confidence:.2f}) at ({x:.1f}, {y:.1f}, {z:.1f})")
            
        else:
            print("❌ Scanning failed")
            return 1
            
    except KeyboardInterrupt:
        print("\n⚠️ Detection interrupted by user")
        return 1
    except Exception as e:
        print(f"❌ Error during detection: {e}")
        return 1
    finally:
        detector.cleanup()
    
    return 0

if __name__ == "__main__":
    exit(main()) 