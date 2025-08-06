#!/usr/bin/env python3
"""
Object Detection System for SCARA AGV
Scans the entire workspace using the D435i camera and detects non-stacking blocks using YOLO
"""

import os
import sys
import time
import numpy as np
import cv2
import json
import math
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from dataclasses import dataclass
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Add project directories to path
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "Arm_Control"))
sys.path.append(str(project_root / "RealSense"))

# Import project modules
try:
    import Arm_Control.SCARA as scara_control
    from RealSense.realsense_depth import DepthCamera
    from RealSense.camera_calibration import CameraCalibrator
except ImportError as e:
    print(f"Warning: Could not import modules: {e}")
    scara_control = None

# Import YOLO
try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics not installed. Please install with: pip install ultralytics")
    sys.exit(1)

@dataclass
class DetectedObject:
    """Represents a detected object with its properties"""
    object_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    center_pixel: Tuple[int, int]    # pixel coordinates of center
    depth_mm: float                  # depth in millimeters
    arm_coords: Tuple[float, float, float]  # x, y, z in arm coordinate system
    camera_position: Tuple[float, float, float]  # camera position when detected

class ObjectDetectionSystem:
    """
    Complete object detection system for SCARA AGV
    """
    
    def __init__(self, model_path: str = "yolo/my_model/my_model.pt", 
                 use_calibration: bool = True, save_images: bool = True):
        """
        Initialize the object detection system
        
        Args:
            model_path: Path to the YOLO model
            use_calibration: Whether to use camera calibration for coordinate conversion
            save_images: Whether to save captured images for debugging
        """
        self.model_path = Path(project_root / model_path)
        self.use_calibration = use_calibration
        self.save_images = save_images
        
        # Initialize components
        self.camera = None
        self.calibrator = None
        self.yolo_model = None
        
        # Detection data
        self.detected_objects: List[DetectedObject] = []
        self.scan_positions: List[Tuple[float, float, float]] = []
        self.current_scan_index = 0
        
        # Create output directories
        self.output_dir = Path("detection_output")
        self.output_dir.mkdir(exist_ok=True)
        if save_images:
            self.images_dir = self.output_dir / "captured_images"
            self.images_dir.mkdir(exist_ok=True)
    
    def initialize(self) -> bool:
        """Initialize all components"""
        print("Initializing Object Detection System...")
        
        # Initialize camera
        try:
            print("Initializing RealSense camera...")
            self.camera = DepthCamera()
            print("✓ Camera initialized successfully")
        except Exception as e:
            print(f"✗ Camera initialization failed: {e}")
            return False
        
        # Initialize camera calibration
        if self.use_calibration:
            try:
                print("Loading camera calibration...")
                self.calibrator = CameraCalibrator()
                if self.calibrator.load_calibration():
                    print("✓ Camera calibration loaded successfully")
                else:
                    print("⚠ Camera calibration not found, using basic conversion")
                    self.use_calibration = False
            except Exception as e:
                print(f"⚠ Camera calibration failed: {e}, using basic conversion")
                self.use_calibration = False
        
        # Initialize YOLO model
        try:
            print(f"Loading YOLO model from {self.model_path}...")
            if not self.model_path.exists():
                raise FileNotFoundError(f"Model file not found: {self.model_path}")
            
            self.yolo_model = YOLO(str(self.model_path))
            print("✓ YOLO model loaded successfully")
        except Exception as e:
            print(f"✗ YOLO model initialization failed: {e}")
            return False
        
        # Initialize SCARA arm
        if scara_control is None:
            print("⚠ SCARA control not available - running in simulation mode")
        else:
            print("✓ SCARA control available")
        
        return True
    
    def plan_scanning_positions(self, scan_height: float = 150.0, 
                              grid_spacing: float = 80.0,
                              camera_direction: float = -90.0) -> List[Tuple[float, float, float]]:
        """
        Plan camera scanning positions to cover the entire workspace
        
        Args:
            scan_height: Z height for scanning (mm)
            grid_spacing: Distance between scan points (mm)
            camera_direction: Camera direction in degrees (-90 = downward)
            
        Returns:
            List of (x, y, z) camera positions
        """
        print("Planning scanning positions...")
        
        # Define workspace bounds (based on SCARA capabilities)
        # From the workspace analysis, typical reachable area for camera
        x_min, x_max = 50, 450   # Conservative bounds to avoid collisions
        y_min, y_max = -200, 200
        
        positions = []
        
        # Generate grid of camera positions
        x = x_min
        while x <= x_max:
            y = y_min
            while y <= y_max:
                # Check if this camera position is reachable
                if self._is_camera_position_reachable(x, y, scan_height, camera_direction):
                    positions.append((x, y, scan_height))
                y += grid_spacing
            x += grid_spacing
        
        print(f"Generated {len(positions)} scanning positions")
        self.scan_positions = positions
        return positions
    
    def _is_camera_position_reachable(self, cam_x: float, cam_y: float, cam_z: float, 
                                    camera_direction: float) -> bool:
        """
        Check if a camera position is reachable by the SCARA arm
        
        Args:
            cam_x, cam_y, cam_z: Camera position
            camera_direction: Camera direction in degrees
            
        Returns:
            True if position is reachable
        """
        try:
            # Calculate required end effector position
            extension_angle_rad = math.radians(camera_direction)
            end_x = cam_x - scara_control.EXTENSION_CAMERA_LENGTH * math.cos(extension_angle_rad)
            end_y = cam_y - scara_control.EXTENSION_CAMERA_LENGTH * math.sin(extension_angle_rad)
            
            # Check if end effector position is reachable
            j1, j2 = scara_control.cartesian_to_angles(end_x, end_y)
            
            # Calculate required J4
            j4 = scara_control.calculate_j4_for_cartesian_direction(j1, j2, camera_direction)
            
            # Check all joint limits
            scara_control.check_joint_limits(j1, j2, cam_z, j4)
            
            return True
            
        except Exception:
            return False
    
    def scan_workspace(self, confidence_threshold: float = 0.7) -> bool:
        """
        Scan the entire workspace and detect objects
        
        Args:
            confidence_threshold: Minimum confidence for YOLO detections
            
        Returns:
            True if scanning completed successfully
        """
        if not self.scan_positions:
            print("No scan positions planned. Call plan_scanning_positions() first.")
            return False
        
        print(f"Starting workspace scan with {len(self.scan_positions)} positions...")
        
        total_detections = 0
        
        for i, (cam_x, cam_y, cam_z) in enumerate(self.scan_positions):
            self.current_scan_index = i
            print(f"\nScanning position {i+1}/{len(self.scan_positions)}: ({cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f})")
            
            # Move camera to position and wait for completion
            print(f"  Moving to position...")
            if not self._move_camera_to_position(cam_x, cam_y, cam_z):
                print(f"  ❌ Failed to reach position {i+1}, skipping...")
                continue
            
            # Movement completion is now handled in _move_camera_to_position
            # Additional settling time for camera stabilization
            time.sleep(2)
            
            # Capture image
            image_data = self._capture_image()
            if image_data is None:
                print(f"Failed to capture image at position {i+1}, skipping...")
                continue
            
            color_image, depth_image = image_data
            
            # Save image if requested
            if self.save_images:
                img_filename = self.images_dir / f"scan_{i+1:03d}.jpg"
                cv2.imwrite(str(img_filename), color_image)
            
            # Run YOLO detection
            detections = self._detect_objects(color_image, depth_image, 
                                            (cam_x, cam_y, cam_z), 
                                            confidence_threshold)
            
            print(f"Found {len(detections)} objects at position {i+1}")
            total_detections += len(detections)
            
            # Add to global detection list
            self.detected_objects.extend(detections)
        
        print(f"\nScanning completed! Total detections: {total_detections}")
        
        # Filter duplicates
        self._filter_duplicate_objects()
        
        return True
    
    def _move_camera_to_position(self, cam_x: float, cam_y: float, cam_z: float, 
                               camera_direction: float = -90.0) -> bool:
        """Move camera to specified position and wait for completion"""
        try:
            if scara_control is not None:
                # Send movement command
                scara_control.quick_camera(cam_x, cam_y, cam_z, 
                                         maintain_extension_direction=True,
                                         extension_angle=camera_direction)
                
                # Wait for movement to complete
                return self._wait_for_movement_completion(cam_x, cam_y, cam_z)
            else:
                # Simulation mode - just wait
                time.sleep(2.0)  # Simulate movement time
                return True
        except Exception as e:
            print(f"Error moving camera: {e}")
            return False
    
    def _wait_for_movement_completion(self, target_x: float, target_y: float, target_z: float, 
                                    timeout: float = 30.0, tolerance: float = 2.0) -> bool:
        """
        Wait for arm to reach target position within tolerance
        
        Args:
            target_x, target_y, target_z: Target position
            timeout: Maximum wait time in seconds
            tolerance: Position tolerance in mm
            
        Returns:
            True if position reached within timeout
        """
        if scara_control is None:
            return True  # Simulation mode
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                # Get current camera position
                current_camera_pos = scara_control.get_camera_position()
                
                # Calculate distance to target
                distance = math.sqrt(
                    (current_camera_pos[0] - target_x) ** 2 +
                    (current_camera_pos[1] - target_y) ** 2 +
                    (current_camera_pos[2] - target_z) ** 2
                )
                
                if distance <= tolerance:
                    print(f"  ✓ Position reached (distance: {distance:.1f}mm)")
                    return True
                
                # Wait a bit before checking again
                time.sleep(0.1)
                
            except Exception as e:
                print(f"  ⚠ Warning: Could not check position: {e}")
                # If we can't check position, wait a conservative amount
                time.sleep(2.0)
                return True  # Assume movement completed
        
        print(f"  ⚠ Timeout waiting for position (distance: {distance:.1f}mm)")
        return False
    
    def _capture_image(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Capture color and depth images"""
        try:
            success, depth_image, color_image = self.camera.get_frame()
            if success:
                return color_image, depth_image
            else:
                return None
        except Exception as e:
            print(f"Error capturing image: {e}")
            return None
    
    def _detect_objects(self, color_image: np.ndarray, depth_image: np.ndarray,
                       camera_position: Tuple[float, float, float],
                       confidence_threshold: float) -> List[DetectedObject]:
        """
        Detect objects in the image using YOLO
        
        Args:
            color_image: RGB image
            depth_image: Depth image
            camera_position: Current camera position
            confidence_threshold: Minimum confidence for detections
            
        Returns:
            List of detected objects
        """
        try:
            # Run YOLO inference
            results = self.yolo_model(color_image, conf=confidence_threshold)
            
            detections = []
            object_id = len(self.detected_objects)  # Continue numbering
            
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        # Extract detection data
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        class_name = self.yolo_model.names[class_id]
                        
                        # Calculate center point
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        # Get depth at center point
                        depth_mm = self._get_depth_at_point(depth_image, center_x, center_y)
                        
                        if depth_mm > 0:  # Valid depth
                            # Convert to arm coordinates
                            arm_coords = self._pixel_to_arm_coordinates(
                                center_x, center_y, depth_mm, camera_position
                            )
                            
                            detected_obj = DetectedObject(
                                object_id=object_id,
                                class_name=class_name,
                                confidence=confidence,
                                bbox=(x1, y1, x2, y2),
                                center_pixel=(center_x, center_y),
                                depth_mm=depth_mm,
                                arm_coords=arm_coords,
                                camera_position=camera_position
                            )
                            
                            detections.append(detected_obj)
                            object_id += 1
                            
                            print(f"  Object {object_id}: {class_name} ({confidence:.2f}) at {arm_coords}")
            
            return detections
            
        except Exception as e:
            print(f"Error in object detection: {e}")
            return []
    
    def _get_depth_at_point(self, depth_image: np.ndarray, x: int, y: int, 
                           window_size: int = 5) -> float:
        """
        Get depth value at a point, using median of surrounding pixels
        
        Args:
            depth_image: Depth image
            x, y: Pixel coordinates
            window_size: Size of window for median calculation
            
        Returns:
            Depth in millimeters (0 if invalid)
        """
        h, w = depth_image.shape
        
        # Clamp coordinates
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        
        # Extract window around point
        half_window = window_size // 2
        x1 = max(0, x - half_window)
        x2 = min(w, x + half_window + 1)
        y1 = max(0, y - half_window)
        y2 = min(h, y + half_window + 1)
        
        depth_window = depth_image[y1:y2, x1:x2]
        valid_depths = depth_window[depth_window > 0]
        
        if len(valid_depths) > 0:
            return float(np.median(valid_depths))
        else:
            return 0.0
    
    def _pixel_to_arm_coordinates(self, pixel_x: int, pixel_y: int, depth_mm: float,
                                 camera_position: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to arm coordinate system
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            depth_mm: Depth in millimeters
            camera_position: Current camera position
            
        Returns:
            (x, y, z) in arm coordinate system
        """
        if self.use_calibration and self.calibrator and self.calibrator.intrinsics:
            # Use calibrated transformation
            return self.calibrator.pixel_to_arm_coordinates(pixel_x, pixel_y, depth_mm)
        else:
            # Use basic geometric transformation
            # This is a simplified version - assumes camera intrinsics
            # Typical RealSense D435i parameters (approximate)
            fx = fy = 615.0  # Approximate focal length for 640x480
            cx, cy = 320.0, 240.0  # Image center
            
            # Convert to camera coordinates
            camera_x = (pixel_x - cx) * depth_mm / fx
            camera_y = (pixel_y - cy) * depth_mm / fy
            camera_z = depth_mm
            
            # Transform to arm coordinates (simplified - assumes camera pointing down)
            cam_x, cam_y, cam_z = camera_position
            arm_x = cam_x + camera_x
            arm_y = cam_y + camera_y
            arm_z = cam_z - camera_z  # Camera pointing down
            
            return (arm_x, arm_y, arm_z)
    
    def _filter_duplicate_objects(self, distance_threshold: float = 30.0):
        """
        Filter duplicate objects that are close to each other
        
        Args:
            distance_threshold: Maximum distance (mm) to consider objects as duplicates
        """
        print("Filtering duplicate objects...")
        
        if len(self.detected_objects) <= 1:
            return
        
        # Group objects by class
        objects_by_class = {}
        for obj in self.detected_objects:
            if obj.class_name not in objects_by_class:
                objects_by_class[obj.class_name] = []
            objects_by_class[obj.class_name].append(obj)
        
        filtered_objects = []
        total_removed = 0
        
        for class_name, objects in objects_by_class.items():
            if len(objects) == 1:
                filtered_objects.extend(objects)
                continue
            
            # Sort by confidence (highest first)
            objects.sort(key=lambda x: x.confidence, reverse=True)
            
            # Filter duplicates
            kept_objects = []
            for obj in objects:
                is_duplicate = False
                
                for kept_obj in kept_objects:
                    # Calculate 3D distance
                    distance = math.sqrt(
                        (obj.arm_coords[0] - kept_obj.arm_coords[0]) ** 2 +
                        (obj.arm_coords[1] - kept_obj.arm_coords[1]) ** 2 +
                        (obj.arm_coords[2] - kept_obj.arm_coords[2]) ** 2
                    )
                    
                    if distance < distance_threshold:
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    kept_objects.append(obj)
                else:
                    total_removed += 1
            
            filtered_objects.extend(kept_objects)
        
        print(f"Removed {total_removed} duplicate objects")
        print(f"Final object count: {len(filtered_objects)}")
        
        self.detected_objects = filtered_objects
        
        # Renumber objects
        for i, obj in enumerate(self.detected_objects):
            obj.object_id = i
    
    def save_results(self, filename: str = "detected_objects.json") -> bool:
        """
        Save detection results to file
        
        Args:
            filename: Output filename
            
        Returns:
            True if save successful
        """
        try:
            results = {
                "scan_metadata": {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "total_objects": len(self.detected_objects),
                    "scan_positions": len(self.scan_positions),
                    "model_path": str(self.model_path)
                },
                "detected_objects": []
            }
            
            for obj in self.detected_objects:
                obj_data = {
                    "object_id": obj.object_id,
                    "class_name": obj.class_name,
                    "confidence": obj.confidence,
                    "bbox": obj.bbox,
                    "center_pixel": obj.center_pixel,
                    "depth_mm": obj.depth_mm,
                    "arm_coordinates": {
                        "x": obj.arm_coords[0],
                        "y": obj.arm_coords[1],
                        "z": obj.arm_coords[2]
                    },
                    "camera_position": {
                        "x": obj.camera_position[0],
                        "y": obj.camera_position[1],
                        "z": obj.camera_position[2]
                    }
                }
                results["detected_objects"].append(obj_data)
            
            output_file = self.output_dir / filename
            with open(output_file, 'w') as f:
                json.dump(results, f, indent=2)
            
            print(f"Results saved to {output_file}")
            return True
            
        except Exception as e:
            print(f"Error saving results: {e}")
            return False
    
    def visualize_3d(self, show_scan_positions: bool = True, 
                    show_workspace: bool = True) -> None:
        """
        Create 3D visualization of detected objects
        
        Args:
            show_scan_positions: Whether to show scan positions
            show_workspace: Whether to show workspace bounds
        """
        print("Creating 3D visualization...")
        
        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot detected objects
        if self.detected_objects:
            object_coords = np.array([obj.arm_coords for obj in self.detected_objects])
            
            # Color by class
            unique_classes = list(set(obj.class_name for obj in self.detected_objects))
            colors = plt.cm.Set1(np.linspace(0, 1, len(unique_classes)))
            color_map = dict(zip(unique_classes, colors))
            
            for obj in self.detected_objects:
                color = color_map[obj.class_name]
                ax.scatter(obj.arm_coords[0], obj.arm_coords[1], obj.arm_coords[2],
                          c=[color], s=100, alpha=0.8, label=obj.class_name)
                
                # Add object ID annotation
                ax.text(obj.arm_coords[0], obj.arm_coords[1], obj.arm_coords[2] + 10,
                       f'ID{obj.object_id}', fontsize=8)
        
        # Plot scan positions
        if show_scan_positions and self.scan_positions:
            scan_coords = np.array(self.scan_positions)
            ax.scatter(scan_coords[:, 0], scan_coords[:, 1], scan_coords[:, 2],
                      c='gray', s=20, alpha=0.3, marker='^', label='Scan Positions')
        
        # Plot workspace bounds
        if show_workspace:
            # SCARA base
            ax.scatter([0], [0], [0], c='red', s=200, marker='x', label='SCARA Base')
            
            # Workspace circle (approximate)
            theta = np.linspace(0, 2*np.pi, 100)
            max_reach = 410  # L1 + L2
            circle_x = max_reach * np.cos(theta)
            circle_y = max_reach * np.sin(theta)
            circle_z = np.zeros_like(circle_x)
            ax.plot(circle_x, circle_y, circle_z, 'k--', alpha=0.3, label='Max Reach')
        
        # Customize plot
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(f'3D Object Detection Results\n{len(self.detected_objects)} objects detected')
        
        # Remove duplicate labels
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper left', bbox_to_anchor=(1.05, 1))
        
        # Set aspect ratio
        ax.set_box_aspect([1,1,0.5])
        
        plt.tight_layout()
        
        # Save plot
        plot_file = self.output_dir / "3d_detection_results.png"
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        print(f"3D plot saved to {plot_file}")
        
        plt.show()
    
    def cleanup(self):
        """Clean up resources"""
        if self.camera:
            self.camera.release()
        print("Object detection system cleaned up")


def main():
    """Main function to run the object detection system"""
    print("=" * 60)
    print("SCARA AGV Object Detection System")
    print("=" * 60)
    
    # Initialize system
    detector = ObjectDetectionSystem()
    
    try:
        # Initialize all components
        if not detector.initialize():
            print("Failed to initialize system")
            return
        
        # Plan scanning positions
        detector.plan_scanning_positions(
            scan_height=150.0,     # 150mm above table
            grid_spacing=80.0,     # 80mm between scan points
            camera_direction=-90.0  # Camera pointing down
        )
        
        # Run the scan
        if not detector.scan_workspace(confidence_threshold=0.7):
            print("Scanning failed")
            return
        
        # Save results
        detector.save_results("detected_objects.json")
        
        # Create visualization
        detector.visualize_3d()
        
        print(f"\nDetection completed successfully!")
        print(f"Total objects detected: {len(detector.detected_objects)}")
        print(f"Results saved to: {detector.output_dir}")
        
    except KeyboardInterrupt:
        print("\nDetection interrupted by user")
    except Exception as e:
        print(f"Error during detection: {e}")
    finally:
        detector.cleanup()


if __name__ == "__main__":
    main()
