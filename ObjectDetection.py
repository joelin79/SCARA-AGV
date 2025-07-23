import cv2
import numpy as np
import yaml
import os
import math
import time
from typing import List, Tuple, Dict, Any
from ultralytics import YOLO
import tempfile

# Import SCARA functions
from SCARA import (
    quick, coordinate_mode, angles_to_cartesian, cartesian_to_angles,
    check_joint_limits, LENGTH_J1, LENGTH_J2, ORIGIN_X, ORIGIN_Y, ORIGIN_Z,
    CUR_X, CUR_Y, CUR_Z
)

class SCARAObjectDetection:
    def __init__(self, 
                 model_path: str = "yolo/my_model/my_model.pt",
                 calib_file: str = "calib_params.yaml",
                 camera_height: float = 300.0,  # 30cm in mm
                 confidence_threshold: float = 0.5):
        """
        Initialize SCARA Object Detection System
        
        Args:
            model_path: Path to YOLO model
            calib_file: Path to camera calibration parameters
            camera_height: Height of camera above work surface (mm)
            confidence_threshold: YOLO detection confidence threshold
        """
        self.model_path = model_path
        self.calib_file = calib_file
        self.camera_height = camera_height
        self.confidence_threshold = confidence_threshold
        
        # Load YOLO model
        self.model = YOLO(model_path)
        self.labels = self.model.names
        
        # Camera parameters (will be loaded from calibration file)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.T_cam_to_robot = None
        
        # Load or create calibration parameters
        self.load_calibration_params()
        
        # Detection results
        self.detected_objects = []
        
        # Set coordinate mode
        coordinate_mode()
    
    def load_calibration_params(self):
        """Load camera calibration parameters from YAML file"""
        if os.path.exists(self.calib_file):
            with open(self.calib_file, 'r') as f:
                params = yaml.safe_load(f)
                self.camera_matrix = np.array(params['camera_matrix'])
                self.dist_coeffs = np.array(params['dist_coeffs'])
                self.T_cam_to_robot = np.array(params['T_cam_to_robot'])
            print(f"Loaded calibration parameters from {self.calib_file}")
        else:
            print(f"Calibration file {self.calib_file} not found. Using default parameters.")
            self.create_default_calibration()
    
    def create_default_calibration(self):
        """Create default calibration parameters for D435i camera"""
        # Default intrinsic parameters for Intel RealSense D435i at 1280x720
        # These are approximate values - real calibration is recommended
        self.camera_matrix = np.array([
            [910.0, 0.0, 640.0],
            [0.0, 910.0, 360.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((1, 5))  # Assume no distortion for now
        
        # Camera to robot transform (camera facing down, aligned to -x direction)
        # Camera is mounted on the arm at current position
        self.T_cam_to_robot = np.eye(4)
        
        # Save default parameters
        params = {
            'camera_matrix': self.camera_matrix.tolist(),
            'dist_coeffs': self.dist_coeffs.tolist(),
            'T_cam_to_robot': self.T_cam_to_robot.tolist()
        }
        with open(self.calib_file, 'w') as f:
            yaml.dump(params, f)
        print(f"Created default calibration parameters in {self.calib_file}")
    
    def pixel_to_world_coordinates(self, pixel_x: int, pixel_y: int, 
                                 arm_x: float, arm_y: float, arm_z: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in image
            arm_x, arm_y, arm_z: Current arm position when image was taken
            
        Returns:
            World coordinates (x, y, z) in arm coordinate system
        """
        # Convert pixel to normalized image coordinates
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Normalized image coordinates
        x_norm = (pixel_x - cx) / fx
        y_norm = (pixel_y - cy) / fy
        
        # Assume objects are on the work surface (z = 0 in world coordinates)
        # Camera is at height camera_height above the work surface
        z_world = 0.0  # Objects on work surface
        z_camera = self.camera_height
        
        # Back-project to world coordinates using similar triangles
        # Camera coordinate system: X right, Y down, Z forward (into scene)
        x_camera = x_norm * z_camera
        y_camera = y_norm * z_camera
        
        # Transform from camera coordinates to robot coordinates
        # Camera is aligned to -x direction and faces down
        # Camera X axis = Robot -Y axis
        # Camera Y axis = Robot +Z axis (but we're looking down, so this is inverted)
        # Camera Z axis = Robot -X axis
        
        # Object position relative to camera mount point
        dx = -z_camera  # Camera looks in -X direction from arm position
        dy = -x_camera  # Camera X maps to -Y in robot coordinates
        dz = y_camera   # Camera Y maps to Z, but objects are on surface so dz ≈ 0
        
        # Transform to world coordinates
        world_x = arm_x + dx
        world_y = arm_y + dy
        world_z = z_world  # Objects on work surface
        
        return world_x, world_y, world_z
    
    def is_point_in_workspace(self, x: float, y: float) -> bool:
        """Check if a point is within the SCARA arm's reachable workspace"""
        try:
            # Check if we can calculate valid joint angles for this position
            j1, j2 = cartesian_to_angles(x, y)
            check_joint_limits(j1, j2, ORIGIN_Z, 0)
            return True
        except (ValueError, Exception):
            return False
    
    def generate_scan_positions(self, grid_size: int = 5) -> List[Tuple[float, float, float]]:
        """
        Generate a grid of positions for the arm to move to for comprehensive scanning
        
        Args:
            grid_size: Number of positions along each axis
            
        Returns:
            List of (x, y, z) positions for scanning
        """
        positions = []
        
        # Define scanning area based on arm workspace
        # Right-handed mode workspace is roughly a circle with some restrictions
        x_min, x_max = 50, 400  # Conservative range
        y_min, y_max = -300, 300
        z_scan = ORIGIN_Z  # Keep Z at origin height
        
        x_step = (x_max - x_min) / (grid_size - 1)
        y_step = (y_max - y_min) / (grid_size - 1)
        
        for i in range(grid_size):
            for j in range(grid_size):
                x = x_min + i * x_step
                y = y_min + j * y_step
                
                # Check if position is reachable and safe
                if self.is_point_in_workspace(x, y):
                    positions.append((x, y, z_scan))
        
        print(f"Generated {len(positions)} scan positions")
        return positions
    
    def capture_image_at_position(self, x: float, y: float, z: float, 
                                camera_index: int = 0) -> np.ndarray:
        """
        Move arm to position and capture image
        
        Args:
            x, y, z: Target position for arm
            camera_index: Camera device index
            
        Returns:
            Captured image as numpy array
        """
        try:
            # Move arm to position
            print(f"Moving to position ({x:.1f}, {y:.1f}, {z:.1f})")
            quick(x, y, z)
            time.sleep(1.0)  # Wait for arm to stabilize
            
            # Capture image
            cap = cv2.VideoCapture(camera_index)
            if not cap.isOpened():
                raise RuntimeError(f"Cannot open camera {camera_index}")
            
            # Set camera resolution (D435i supports up to 1920x1080)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Capture frame
            ret, frame = cap.read()
            cap.release()
            
            if not ret:
                raise RuntimeError("Failed to capture frame")
            
            return frame
            
        except Exception as e:
            print(f"Error capturing image at position ({x}, {y}, {z}): {e}")
            return None
    
    def detect_objects_in_image(self, image: np.ndarray, arm_position: Tuple[float, float, float]) -> List[Dict]:
        """
        Detect objects in a single image and return their arm coordinates
        
        Args:
            image: Input image
            arm_position: (x, y, z) position of arm when image was taken
            
        Returns:
            List of detection dictionaries with arm coordinates
        """
        detections = []
        
        # Run YOLO inference
        results = self.model(image, verbose=False)
        yolo_detections = results[0].boxes
        
        if yolo_detections is None:
            return detections
        
        arm_x, arm_y, arm_z = arm_position
        
        # Process each detection
        for i in range(len(yolo_detections)):
            # Get bounding box
            xyxy_tensor = yolo_detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)
            
            # Calculate center of bounding box
            center_x = int((xmin + xmax) / 2)
            center_y = int((ymin + ymax) / 2)
            
            # Get class and confidence
            class_idx = int(yolo_detections[i].cls.item())
            class_name = self.labels[class_idx]
            confidence = yolo_detections[i].conf.item()
            
            if confidence >= self.confidence_threshold:
                # Convert pixel coordinates to arm coordinates
                world_x, world_y, world_z = self.pixel_to_world_coordinates(
                    center_x, center_y, arm_x, arm_y, arm_z
                )
                
                detection = {
                    'class_name': class_name,
                    'class_idx': class_idx,
                    'confidence': confidence,
                    'pixel_center': (center_x, center_y),
                    'bbox': (xmin, ymin, xmax, ymax),
                    'arm_coordinates': (world_x, world_y, world_z),
                    'arm_position_when_detected': arm_position
                }
                detections.append(detection)
        
        return detections
    
    def remove_duplicate_detections(self, all_detections: List[Dict], 
                                  distance_threshold: float = 20.0) -> List[Dict]:
        """
        Remove duplicate detections from multiple images
        
        Args:
            all_detections: List of all detections from all images
            distance_threshold: Minimum distance (mm) to consider detections as separate
            
        Returns:
            Filtered list of unique detections
        """
        unique_detections = []
        
        for detection in all_detections:
            x, y, z = detection['arm_coordinates']
            is_duplicate = False
            
            # Check against existing unique detections
            for unique_det in unique_detections:
                ux, uy, uz = unique_det['arm_coordinates']
                distance = math.sqrt((x - ux)**2 + (y - uy)**2 + (z - uz)**2)
                
                if distance < distance_threshold:
                    # This is a duplicate, keep the one with higher confidence
                    if detection['confidence'] > unique_det['confidence']:
                        unique_detections.remove(unique_det)
                        unique_detections.append(detection)
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique_detections.append(detection)
        
        return unique_detections
    
    def scan_workspace_and_detect_objects(self, camera_index: int = 0, 
                                        grid_size: int = 4) -> List[Dict]:
        """
        Perform complete workspace scan and object detection
        
        Args:
            camera_index: Camera device index
            grid_size: Grid resolution for scanning
            
        Returns:
            List of detected objects with arm coordinates
        """
        print("Starting workspace scan...")
        
        # Generate scan positions
        scan_positions = self.generate_scan_positions(grid_size)
        
        if not scan_positions:
            print("No valid scan positions generated!")
            return []
        
        all_detections = []
        
        # Scan each position
        for i, (x, y, z) in enumerate(scan_positions):
            print(f"Scanning position {i+1}/{len(scan_positions)}: ({x:.1f}, {y:.1f}, {z:.1f})")
            
            # Capture image at this position
            image = self.capture_image_at_position(x, y, z, camera_index)
            
            if image is not None:
                # Detect objects in this image
                detections = self.detect_objects_in_image(image, (x, y, z))
                all_detections.extend(detections)
                
                # Save image with detections for debugging
                self.save_detection_image(image, detections, f"scan_position_{i+1}.jpg")
        
        # Remove duplicates
        unique_detections = self.remove_duplicate_detections(all_detections)
        
        print(f"Scan complete. Found {len(unique_detections)} unique objects.")
        return unique_detections
    
    def save_detection_image(self, image: np.ndarray, detections: List[Dict], filename: str):
        """Save image with detection annotations"""
        output_dir = "detections_output"
        os.makedirs(output_dir, exist_ok=True)
        
        annotated_image = image.copy()
        
        for detection in detections:
            xmin, ymin, xmax, ymax = detection['bbox']
            center_x, center_y = detection['pixel_center']
            world_x, world_y, world_z = detection['arm_coordinates']
            
            # Draw bounding box
            cv2.rectangle(annotated_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(annotated_image, (center_x, center_y), 5, (255, 0, 255), -1)
            
            # Add labels
            label = f"{detection['class_name']}: {detection['confidence']:.2f}"
            coord_label = f"({world_x:.1f}, {world_y:.1f}, {world_z:.1f})"
            
            cv2.putText(annotated_image, label, (xmin, ymin - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(annotated_image, coord_label, (xmin, ymax + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
        
        output_path = os.path.join(output_dir, filename)
        cv2.imwrite(output_path, annotated_image)
    
    def get_object_coordinates_list(self) -> List[Tuple[float, float, float]]:
        """
        Get list of arm coordinates for all detected objects
        
        Returns:
            List of (x, y, z) coordinates in arm coordinate system
        """
        return [det['arm_coordinates'] for det in self.detected_objects]
    
    def print_detection_results(self):
        """Print summary of detection results"""
        print("\n" + "="*60)
        print("OBJECT DETECTION RESULTS")
        print("="*60)
        
        if not self.detected_objects:
            print("No objects detected.")
            return
        
        print(f"Total objects detected: {len(self.detected_objects)}")
        print("\nObject Details:")
        print("-" * 60)
        
        for i, obj in enumerate(self.detected_objects):
            x, y, z = obj['arm_coordinates']
            print(f"{i+1:2d}. {obj['class_name']:15s} "
                  f"Confidence: {obj['confidence']:.2f} "
                  f"Position: ({x:6.1f}, {y:6.1f}, {z:4.1f}) mm")
        
        print("\nArm Coordinates List:")
        print("-" * 60)
        coords = self.get_object_coordinates_list()
        for i, (x, y, z) in enumerate(coords):
            print(f"Object {i+1}: ({x:.1f}, {y:.1f}, {z:.1f})")

def main():
    """Main function to run the object detection system"""
    # Initialize the detection system
    detector = SCARAObjectDetection(
        model_path="yolo/my_model/my_model.pt",
        confidence_threshold=0.5
    )
    
    try:
        # Perform workspace scan and detection
        detected_objects = detector.scan_workspace_and_detect_objects(
            camera_index=0,  # USB camera index
            grid_size=3      # 3x3 grid for faster scanning
        )
        
        # Store results
        detector.detected_objects = detected_objects
        
        # Print results
        detector.print_detection_results()
        
        # Save results to file
        if detected_objects:
            coords = detector.get_object_coordinates_list()
            with open("detected_object_coordinates.txt", "w") as f:
                f.write("# Detected Object Coordinates (x, y, z) in mm\n")
                for i, (x, y, z) in enumerate(coords):
                    f.write(f"{x:.1f}, {y:.1f}, {z:.1f}\n")
            print(f"\nCoordinates saved to detected_object_coordinates.txt")
        
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    except Exception as e:
        print(f"Error during detection: {e}")

if __name__ == "__main__":
    main()
