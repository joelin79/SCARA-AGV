#!/usr/bin/env python3
"""
SCARA Object Detection and Coordinate Transformation System
Integrates RealSense camera, YOLO detection, and SCARA arm control
"""

import os
import sys
import time
import numpy as np
import cv2
from ultralytics import YOLO
from typing import List, Tuple
import json
from dataclasses import dataclass
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from Arm_Control.SCARA import SCARA
except ImportError:
    from Arm_Control.SCARA_Simulator import SCARA
from RealSense.realsense_depth import DepthCamera

@dataclass
class DetectedObject:
    """Represents a detected object with all relevant information"""
    class_name: str
    confidence: float
    pixel_x: int
    pixel_y: int
    depth_mm: float
    arm_x: float
    arm_y: float
    arm_z: float
    camera_position: Tuple[float, float, float]
    camera_angle: float
    timestamp: float

class SCARAObjectDetection:
    """
    Main class for SCARA arm object detection and coordinate transformation
    """
    
    def __init__(self, 
                 model_path: str = "yolo/my_model/my_model.pt",
                 confidence_threshold: float = 0.4,
                 camera_height: float = 300.0,
                 grid_size: int = 3,
                 overlap_percentage: float = 0.3):
        """
        Initialize the SCARA object detection system
        
        Args:
            model_path: Path to YOLO model file
            confidence_threshold: Minimum confidence for object detection
            camera_height: Height of camera above workspace (mm)
            grid_size: Number of positions in grid (e.g., 3 = 3x3 grid)
            overlap_percentage: Percentage of overlap between images (0.0-1.0)
        """
        self.model_path = model_path
        self.confidence_threshold = confidence_threshold
        self.camera_height = camera_height
        self.grid_size = grid_size
        self.overlap_percentage = overlap_percentage
        
        # Initialize components
        self.model = None
        self.camera = None
        self.arm = None
        self.detected_objects: List[DetectedObject] = []
        
        # Camera parameters (will be calibrated)
        self.camera_fov_horizontal = 87  # degrees
        self.camera_fov_vertical = 58    # degrees
        self.camera_resolution = (1280, 720)
        
        # Calibration data
        self.calibration_loaded = False
        self.camera_matrix = None
        self.dist_coeffs = None
        self.extrinsics_rotation = None
        self.extrinsics_translation = None
        
        # Workspace parameters
        self.workspace_width = 400   # mm
        self.workspace_height = 400  # mm
        
        # Initialize components
        self._initialize_model()
        self._initialize_camera()
        self._initialize_arm()
        
        # Load calibration if available
        self._load_calibration()
        
        # Create output directory
        self.output_dir = Path("detections_output")
        self.output_dir.mkdir(exist_ok=True)
        
    def _initialize_model(self):
        """Initialize YOLO model"""
        try:
            print(f"Loading YOLO model from: {self.model_path}")
            self.model = YOLO(self.model_path)
            print(f"Model loaded successfully. Classes: {list(self.model.names.values())}")
        except Exception as e:
            print(f"Error loading model: {e}")
            raise
    
    def _initialize_camera(self):
        """Initialize RealSense camera"""
        try:
            print("Initializing RealSense camera...")
            self.camera = DepthCamera()
            print("Camera initialized successfully")
        except Exception as e:
            print(f"Error initializing camera: {e}")
            raise
    
    def _initialize_arm(self):
        """Initialize SCARA arm connection"""
        try:
            print("Initializing SCARA arm...")
            # Note: SCARA class is imported but connection depends on hardware
            # For now, we'll create a mock arm for testing
            self.arm = SCARA()
            print("Arm initialized successfully")
        except Exception as e:
            print(f"Warning: Arm initialization failed: {e}")
            print("Running in simulation mode")
            self.arm = None
    
    def _load_calibration(self):
        """Load camera calibration if available"""
        try:
            calibration_file = Path("camera_calibration/camera_calibration.json")
            if calibration_file.exists():
                import json
                with open(calibration_file, 'r') as f:
                    data = json.load(f)
                
                # Load camera matrix and distortion coefficients
                self.camera_matrix = np.array(data["camera_matrix"])
                self.dist_coeffs = np.array(data["dist_coeffs"])
                
                # Load extrinsics if available
                if "extrinsics" in data:
                    extrinsics_data = data["extrinsics"]
                    self.extrinsics_rotation = np.array(extrinsics_data["rotation_matrix"])
                    self.extrinsics_translation = np.array(extrinsics_data["translation_vector"])
                
                self.calibration_loaded = True
                print("Camera calibration loaded successfully")
                
                # Update camera parameters from calibration
                intrinsics = data["intrinsics"]
                self.camera_resolution = (intrinsics["width"], intrinsics["height"])
                
            else:
                print("No camera calibration found. Using default parameters.")
                
        except Exception as e:
            print(f"Warning: Failed to load calibration: {e}")
            print("Using default camera parameters")
    
    def calculate_grid_positions(self) -> List[Tuple[float, float]]:
        """
        Calculate camera positions for grid scanning
        
        Returns:
            List of (x, y) positions in arm coordinates
        """
        positions = []
        
        # Calculate step size with overlap
        step_x = self.workspace_width / (self.grid_size - 1) * (1 - self.overlap_percentage)
        step_y = self.workspace_height / (self.grid_size - 1) * (1 - self.overlap_percentage)
        
        # Calculate start positions (center the grid)
        start_x = -self.workspace_width / 2 + step_x / 2
        start_y = -self.workspace_height / 2 + step_y / 2
        
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                x = start_x + i * step_x
                y = start_y + j * step_y
                positions.append((x, y))
        
        return positions
    
    def capture_image_at_position(self, x: float, y: float, camera_angle: float = -90.0) -> Tuple[bool, np.ndarray, np.ndarray]:
        """
        Move arm to position and capture image with depth
        
        Args:
            x, y: Arm coordinates (mm)
            camera_angle: Camera direction angle (degrees)
            
        Returns:
            (success, color_image, depth_image)
        """
        if self.arm:
            try:
                # Move arm to position
                self.arm.quick_camera(x, y, self.camera_height, extension_angle=camera_angle)
                time.sleep(1.0)  # Wait for movement to complete
            except Exception as e:
                print(f"Warning: Arm movement failed: {e}")
        
        # Capture image
        success, depth_image, color_image = self.camera.get_frame()
        if not success:
            print(f"Failed to capture image at position ({x}, {y})")
            return False, None, None
        
        return True, color_image, depth_image
    
    def detect_objects_in_image(self, color_image: np.ndarray, depth_image: np.ndarray,
                               camera_position: Tuple[float, float, float],
                               camera_angle: float) -> List[DetectedObject]:
        """
        Detect objects in image and convert to arm coordinates
        
        Args:
            color_image: RGB image from camera
            depth_image: Depth image from camera
            camera_position: (x, y, z) position of camera in arm coordinates
            camera_angle: Camera direction angle
            
        Returns:
            List of detected objects
        """
        # Run YOLO detection
        results = self.model(color_image, verbose=False)
        detections = results[0].boxes
        
        detected_objects = []
        
        for i in range(len(detections)):
            # Get detection info
            xyxy = detections[i].xyxy.cpu().numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)
            
            class_idx = int(detections[i].cls.item())
            class_name = self.model.names[class_idx]
            confidence = detections[i].conf.item()
            
            # Skip if confidence is too low
            if confidence < self.confidence_threshold:
                continue
            
            # Calculate center pixel
            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)
            
            # Get depth at center point
            depth_mm = depth_image[center_y, center_x]
            
            # Convert pixel coordinates to arm coordinates
            arm_x, arm_y, arm_z = self.pixel_to_arm_coordinates(
                center_x, center_y, depth_mm, camera_position, camera_angle
            )
            
            # Create detected object
            obj = DetectedObject(
                class_name=class_name,
                confidence=confidence,
                pixel_x=center_x,
                pixel_y=center_y,
                depth_mm=depth_mm,
                arm_x=arm_x,
                arm_y=arm_y,
                arm_z=arm_z,
                camera_position=camera_position,
                camera_angle=camera_angle,
                timestamp=time.time()
            )
            
            detected_objects.append(obj)
        
        return detected_objects
    
    def pixel_to_arm_coordinates(self, pixel_x: int, pixel_y: int, depth_mm: float,
                                camera_position: Tuple[float, float, float],
                                camera_angle: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to arm coordinates
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            depth_mm: Depth in millimeters
            camera_position: Camera position in arm coordinates
            camera_angle: Camera direction angle
            
        Returns:
            (arm_x, arm_y, arm_z) in millimeters
        """
        if self.calibration_loaded and self.camera_matrix is not None:
            # Use calibrated camera parameters
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            
            # Convert to camera coordinates using calibrated intrinsics
            camera_x = (pixel_x - cx) * depth_mm / fx
            camera_y = (pixel_y - cy) * depth_mm / fy
            camera_z = depth_mm
            
            # Apply extrinsics transformation if available
            if self.extrinsics_rotation is not None and self.extrinsics_translation is not None:
                camera_point = np.array([camera_x, camera_y, camera_z])
                arm_point = self.extrinsics_rotation @ camera_point + self.extrinsics_translation
                return arm_point[0], arm_point[1], arm_point[2]
            else:
                # Fall back to simple transformation
                arm_x = camera_position[0] + camera_x
                arm_y = camera_position[1] + camera_y
                arm_z = camera_position[2] - camera_z  # Subtract because camera looks down
                return arm_x, arm_y, arm_z
        else:
            # Fall back to FOV-based method (original approach)
            # Normalize pixel coordinates to [-1, 1]
            norm_x = (pixel_x - self.camera_resolution[0] / 2) / (self.camera_resolution[0] / 2)
            norm_y = (pixel_y - self.camera_resolution[1] / 2) / (self.camera_resolution[1] / 2)
            
            # Convert to angles using FOV
            angle_x = norm_x * (self.camera_fov_horizontal / 2) * (np.pi / 180)
            angle_y = norm_y * (self.camera_fov_vertical / 2) * (np.pi / 180)
            
            # Calculate 3D point in camera coordinates
            # Camera looks down (-Z direction)
            camera_x = depth_mm * np.tan(angle_x)
            camera_y = depth_mm * np.tan(angle_y)
            camera_z = depth_mm
            
            # Transform to arm coordinates
            # Camera is mounted on extension arm pointing down
            arm_x = camera_position[0] + camera_x
            arm_y = camera_position[1] + camera_y
            arm_z = camera_position[2] - camera_z  # Subtract because camera looks down
            
            return arm_x, arm_y, arm_z
    
    def remove_duplicate_objects(self, objects: List[DetectedObject], 
                                distance_threshold: float = 50.0) -> List[DetectedObject]:
        """
        Remove duplicate objects based on spatial proximity and confidence
        
        Args:
            objects: List of detected objects
            distance_threshold: Minimum distance between objects (mm)
            
        Returns:
            List of objects with duplicates removed
        """
        if not objects:
            return []
        
        # Sort by confidence (highest first)
        sorted_objects = sorted(objects, key=lambda x: x.confidence, reverse=True)
        
        unique_objects = []
        
        for obj in sorted_objects:
            is_duplicate = False
            
            for unique_obj in unique_objects:
                # Calculate distance between objects
                distance = np.sqrt(
                    (obj.arm_x - unique_obj.arm_x) ** 2 +
                    (obj.arm_y - unique_obj.arm_y) ** 2 +
                    (obj.arm_z - unique_obj.arm_z) ** 2
                )
                
                # Check if same class and close enough to be duplicate
                if (obj.class_name == unique_obj.class_name and 
                    distance < distance_threshold):
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique_objects.append(obj)
        
        return unique_objects
    
    def scan_workspace_and_detect_objects(self, save_images: bool = True) -> List[DetectedObject]:
        """
        Perform complete workspace scan and object detection
        
        Args:
            save_images: Whether to save captured images
            
        Returns:
            List of unique detected objects
        """
        print("Starting workspace scan...")
        
        # Calculate grid positions
        positions = self.calculate_grid_positions()
        print(f"Scanning {len(positions)} positions in {self.grid_size}x{self.grid_size} grid")
        
        all_detected_objects = []
        
        for i, (x, y) in enumerate(positions):
            print(f"Position {i+1}/{len(positions)}: ({x:.1f}, {y:.1f})")
            
            # Capture image at position
            success, color_image, depth_image = self.capture_image_at_position(x, y)
            
            if not success:
                print(f"Failed to capture at position {i+1}")
                continue
            
            # Save images if requested
            if save_images:
                timestamp = int(time.time())
                cv2.imwrite(str(self.output_dir / f"color_pos_{i+1}_{timestamp}.jpg"), color_image)
                cv2.imwrite(str(self.output_dir / f"depth_pos_{i+1}_{timestamp}.png"), depth_image)
            
            # Detect objects
            camera_position = (x, y, self.camera_height)
            detected_objects = self.detect_objects_in_image(
                color_image, depth_image, camera_position, -90.0
            )
            
            print(f"  Detected {len(detected_objects)} objects")
            all_detected_objects.extend(detected_objects)
            
            # Small delay between positions
            time.sleep(0.5)
        
        # Remove duplicates
        print("Removing duplicate objects...")
        unique_objects = self.remove_duplicate_objects(all_detected_objects)
        
        print(f"Total objects detected: {len(all_detected_objects)}")
        print(f"Unique objects after deduplication: {len(unique_objects)}")
        
        return unique_objects
    
    def print_detection_results(self):
        """Print detection results in a formatted way"""
        if not self.detected_objects:
            print("No objects detected")
            return
        
        print("\n" + "="*60)
        print("DETECTION RESULTS")
        print("="*60)
        
        for i, obj in enumerate(self.detected_objects):
            print(f"Object {i+1}:")
            print(f"  Class: {obj.class_name}")
            print(f"  Confidence: {obj.confidence:.3f}")
            print(f"  Arm Coordinates: ({obj.arm_x:.1f}, {obj.arm_y:.1f}, {obj.arm_z:.1f}) mm")
            print(f"  Pixel Coordinates: ({obj.pixel_x}, {obj.pixel_y})")
            print(f"  Depth: {obj.depth_mm:.1f} mm")
            print()
    
    def get_object_coordinates_list(self) -> List[Tuple[float, float, float]]:
        """Get list of object coordinates for arm control"""
        return [(obj.arm_x, obj.arm_y, obj.arm_z) for obj in self.detected_objects]
    
    def save_results_to_file(self, filename: str = "detection_results.json"):
        """Save detection results to JSON file"""
        results = {
            "timestamp": time.time(),
            "total_objects": len(self.detected_objects),
            "objects": []
        }
        
        for obj in self.detected_objects:
            obj_dict = {
                "class_name": obj.class_name,
                "confidence": obj.confidence,
                "arm_coordinates": [obj.arm_x, obj.arm_y, obj.arm_z],
                "pixel_coordinates": [obj.pixel_x, obj.pixel_y],
                "depth_mm": obj.depth_mm,
                "camera_position": list(obj.camera_position),
                "camera_angle": obj.camera_angle,
                "timestamp": obj.timestamp
            }
            results["objects"].append(obj_dict)
        
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"Results saved to {filename}")
    
    def cleanup(self):
        """Clean up resources"""
        if self.camera:
            self.camera.release()
        print("Cleanup completed")


def main():
    """Main function for testing"""
    detector = SCARAObjectDetection()
    
    try:
        # Perform detection
        detected_objects = detector.scan_workspace_and_detect_objects()
        detector.detected_objects = detected_objects
        
        # Print results
        detector.print_detection_results()
        
        # Save results
        detector.save_results_to_file()
        
    except KeyboardInterrupt:
        print("\nScan interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        detector.cleanup()


if __name__ == "__main__":
    main() 