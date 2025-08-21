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

import shortest_path_optimizer

# Add project directories to path
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "Arm_Control"))
sys.path.append(str(project_root / "RealSense"))
sys.path.append(str(project_root / "Detection_Models"))


# Import project modules (lazy where needed to avoid serial on import)
try:
    from RealSense.realsense_depth import DepthCamera
except ImportError as e:
    print(f"Warning: Could not import DepthCamera: {e}")
    DepthCamera = None  # type: ignore

# Lazy SCARA/Calibrator import to avoid opening serial on module import
scara_control = None

def _import_scara():
    """Lazy import of SCARA control to avoid serial connection on module import"""
    global scara_control
    if scara_control is None:
        try:
            print("Attempting to import SCARA control...")
            import Arm_Control.SCARA as scara_control
            print("✓ SCARA control imported successfully")
            
            # Test if we can access the serial connection
            try:
                if hasattr(scara_control, 'is_connected') and scara_control.is_connected():
                    print("✓ Serial connection is open and working")
                elif hasattr(scara_control, 'ser') and scara_control.ser and scara_control.ser.is_open:
                    print("✓ Serial connection is open")
                else:
                    print("⚠ Serial connection is not open")
                    scara_control = None
            except Exception as e:
                print(f"⚠ Serial connection test failed: {e}")
                scara_control = None
                
        except Exception as e:
            print(f"⚠ Failed to import SCARA control: {e}")
            print("  This usually means the serial connection failed")
            print("  Check if your SCARA robot is connected to COM3")
            print("  Or update the COM port in Arm_Control/SCARA.py")
            scara_control = None
    return scara_control

# Import YOLO
try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics not installed. Please install with: pip install ultralytics")
    sys.exit(1)

# Import ArUco plate detection
try:
    from aruco_plate_detector import ArUcoPlateDetector
except ImportError as e:
    print(f"Warning: Could not import ArUcoPlateDetector: {e}")
    ArUcoPlateDetector = None  # type: ignore

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
        self.aruco_detector = None
        
        # Detection data
        self.detected_objects: List[DetectedObject] = []
        self.detected_plates: List = []  # Will store ArUco plate definitions
        self.scan_positions: List[Tuple[float, float, float]] = []
        self.current_scan_index = 0
        # Accumulate ArUco markers across the entire scan
        self.collected_aruco_markers: List = []
        
        # Generate timestamp for file naming (MMDDHHmm format)
        self.timestamp = time.strftime("%m%d%H%M")
        
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
            if DepthCamera is None:
                raise ImportError("DepthCamera unavailable")
            self.camera = DepthCamera()
            print("✓ Camera initialized successfully")
        except Exception as e:
            print(f"✗ Camera initialization failed: {e}")
            return False
        
        # Initialize camera calibration
        if self.use_calibration:
            try:
                print("Loading camera calibration from factory file...")
                from RealSense.camera_calibration import CameraCalibrator
                self.calibrator = CameraCalibrator()
                calibration_file = project_root / "RealSense" / "camera_calibration" / "camera_calibration_factory.json"
                if self.calibrator.load_calibration(calibration_file):
                    print("✓ Camera calibration loaded successfully from factory file")
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
        
        # Initialize ArUco plate detector
        if ArUcoPlateDetector is not None:
            try:
                print("Initializing ArUco plate detector...")
                self.aruco_detector = ArUcoPlateDetector(
                    aruco_dict_type=cv2.aruco.DICT_4X4_250,
                    marker_size_mm=30.0,
                    expected_markers=4
                )
                print("✓ ArUco plate detector initialized successfully")
            except Exception as e:
                print(f"⚠ ArUco detector initialization failed: {e}")
                self.aruco_detector = None
        else:
            print("⚠ ArUco plate detection not available")
        
        # Initialize SCARA arm
        print("Initializing SCARA arm control...")
        if _import_scara() is not None:
            print("✓ SCARA control available")
        else:
            print("⚠ SCARA control not available - running in simulation mode")
        
        return True
    
    def plan_scanning_positions(self, scan_height: float = 150.0, 
                              grid_spacing: float = 80.0,
                              camera_direction: float = -90.0,
                              x_min: float = -250, x_max: float = 450,
                              y_min: float = -420, y_max: float = 420) -> List[Tuple[float, float, float]]:
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
        self.scan_positions = shortest_path_optimizer.optimize_scanning_order(positions)
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
        # Check if SCARA control is available
        if scara_control is None:
            print("⚠ SCARA control not available, assuming all positions are reachable")
            return True
            
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
        # Reset ArUco accumulation for this scan
        self.collected_aruco_markers = []
        
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
            if self.current_scan_index == 0:
                time.sleep(3)
            else:
                time.sleep(0.9)
            
            # Capture image
            image_data = self._capture_image()
            if image_data is None:
                print(f"Failed to capture image at position {i+1}, skipping...")
                continue
            
            color_image, depth_image = image_data
            
            # Run YOLO detection first
            detections = self._detect_objects(color_image, depth_image, 
                                            (cam_x, cam_y, cam_z), 
                                            confidence_threshold)
            
            # Run ArUco detection if available
            if self.aruco_detector is not None:
                try:
                    print(f"  Detecting ArUco markers...")
                    aruco_markers = self.aruco_detector.detect_markers(
                        color_image, depth_image, (cam_x, cam_y, cam_z),
                        self._pixel_to_arm_coordinates
                    )
                    print(f"  Found {len(aruco_markers)} ArUco markers")
                    # Accumulate markers across all scan positions
                    if aruco_markers:
                        self.collected_aruco_markers.extend(aruco_markers)
                except Exception as e:
                    print(f"  ⚠ ArUco detection failed: {e}")
                    aruco_markers = []
            else:
                aruco_markers = []
            
            # Save annotated detection image if requested
            if self.save_images:
                annotated_image = self._create_annotated_detection_image(
                    color_image, detections, (cam_x, cam_y, cam_z), i+1, aruco_markers
                )
                img_filename = self.images_dir / f"detection_{self.timestamp}_{i+1:03d}.jpg"
                cv2.imwrite(str(img_filename), annotated_image)
                
                # Also save raw image for reference
                raw_img_filename = self.images_dir / f"raw_{self.timestamp}_{i+1:03d}.jpg"
                cv2.imwrite(str(raw_img_filename), color_image)
            
            print(f"Found {len(detections)} objects at position {i+1}")
            total_detections += len(detections)
            
            # Add to global detection list
            self.detected_objects.extend(detections)
        
        print(f"\nScanning completed! Total detections: {total_detections}")
        
        # Process ArUco markers to define plates
        if self.aruco_detector is not None:
            # Feed accumulated markers into the detector before processing plates
            if self.collected_aruco_markers:
                deduped_markers = self._deduplicate_aruco_markers(self.collected_aruco_markers)
                if len(deduped_markers) != len(self.collected_aruco_markers):
                    print(f"Consolidated ArUco markers: {len(self.collected_aruco_markers)} → {len(deduped_markers)}")
                self.aruco_detector.detected_markers = deduped_markers
            self._process_aruco_plates()
        
        # Filter duplicates
        self._filter_duplicate_objects()
        
        return True
    
    def _move_camera_to_position(self, cam_x: float, cam_y: float, cam_z: float, 
                               camera_direction: float = -90.0) -> bool:
        """Move camera to specified position and wait for completion"""
        try:
            if scara_control is not None:
                # Send movement command
                scara_control.quick_camera(cam_x, cam_y, cam_z, 20000,
                                         maintain_extension_direction=True,
                                         extension_angle=camera_direction)
                
                # Wait for movement to complete
                return True
            else:
                # Simulation mode - just wait
                time.sleep(2.0)  # Simulate movement time
                return True
        except Exception as e:
            print(f"Error moving camera: {e}")
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
    
    def _create_annotated_detection_image(self, color_image: np.ndarray, 
                                        detections: List[DetectedObject],
                                        camera_position: Tuple[float, float, float],
                                        scan_position: int,
                                        aruco_markers: List = None) -> np.ndarray:
        """
        Create annotated detection image similar to yolo_detection.py
        
        Args:
            color_image: Original color image
            detections: List of detected objects
            camera_position: Current camera position (x, y, z)
            scan_position: Current scan position number
            
        Returns:
            Annotated image with bounding boxes, labels, and camera position info
        """
        # Create a copy of the image to annotate
        annotated_image = color_image.copy()
        
        # Determine dynamic label size based on resolution
        font_scale = 0.5
        font_thickness = 1
        if annotated_image.shape[1] >= 1600 or annotated_image.shape[0] >= 1200:
            font_scale = 1.2
            font_thickness = 2
        elif annotated_image.shape[1] >= 1200:
            font_scale = 0.9
            font_thickness = 2
        
        # Set bounding box colors (using the Tableau 10 color scheme)
        bbox_colors = [(164, 120, 87), (68, 148, 228), (93, 97, 209), (178, 182, 133), (88, 159, 106),
                      (96, 202, 231), (159, 124, 168), (169, 162, 241), (98, 118, 150), (172, 176, 184)]
        
        # Add camera position info in top left
        camera_info = f"Camera Pos {scan_position}: ({camera_position[0]:.1f}, {camera_position[1]:.1f}, {camera_position[2]:.1f})"
        cv2.putText(annotated_image, camera_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale if font_scale > 0.7 else 0.7, (0, 255, 255), 
                   font_thickness if font_thickness > 1 else 2)
        
        # Add timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(annotated_image, timestamp, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale if font_scale > 0.7 else 0.7, (0, 255, 255), 
                   font_thickness if font_thickness > 1 else 2)
        
        # Process each detection
        for i, obj in enumerate(detections):
            x1, y1, x2, y2 = obj.bbox
            center_x, center_y = obj.center_pixel
            class_name = obj.class_name
            confidence = obj.confidence
            depth_mm = obj.depth_mm
            arm_x, arm_y, arm_z = obj.arm_coords
            
            # Get color for this class
            class_idx = list(self.yolo_model.names.values()).index(class_name) if class_name in self.yolo_model.names.values() else i
            color = bbox_colors[class_idx % len(bbox_colors)]
            
            # Draw bounding box
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
            
            # Draw class label with confidence
            label = f'{class_name}: {int(confidence * 100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, font_thickness)
            label_ymin = max(y1, labelSize[1] + 10)
            cv2.rectangle(annotated_image, (x1, label_ymin - labelSize[1] - 10),
                         (x1 + labelSize[0], label_ymin + baseLine - 10), color, cv2.FILLED)
            cv2.putText(annotated_image, label, (x1, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 
                       font_scale, (0, 0, 0), font_thickness)
            
            # Draw center point (pink dot)
            cv2.circle(annotated_image, (center_x, center_y), 4, (255, 0, 255), -1)
            
            # Create detailed center label with both pixel coordinates and arm coordinates
            pixel_label = f'Pix:({center_x},{center_y}) D:{depth_mm:.0f}mm'
            arm_label = f'Arm:({arm_x:.1f},{arm_y:.1f},{arm_z:.1f})'
            
            # Draw pixel coordinates label
            cv2.putText(annotated_image, pixel_label, (center_x + 5, center_y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (255, 0, 255), font_thickness)
            
            # Draw arm coordinates label (below pixel coordinates)
            cv2.putText(annotated_image, arm_label, (center_x + 5, center_y + 15), 
                       cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (255, 0, 255), font_thickness)
        
        # Add total object count
        object_count = len(detections)
        count_text = f'Objects detected: {object_count}'
        cv2.putText(annotated_image, count_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale if font_scale > 0.7 else 0.7, (0, 255, 255), 
                   font_thickness if font_thickness > 1 else 2)
        
        # Draw ArUco markers if available
        if aruco_markers:
            aruco_count = len(aruco_markers)
            aruco_text = f'ArUco markers: {aruco_count}'
            cv2.putText(annotated_image, aruco_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 
                       font_scale if font_scale > 0.7 else 0.7, (0, 255, 0), 
                       font_thickness if font_thickness > 1 else 2)
            
            # Draw each ArUco marker
            for marker in aruco_markers:
                # Draw marker corners
                corners = marker.corners.astype(int)
                cv2.polylines(annotated_image, [corners], True, (0, 255, 0), 2)
                
                # Draw marker ID
                center_x, center_y = marker.center_pixel
                cv2.putText(annotated_image, f"ArUco:{marker.marker_id}", 
                           (center_x + 10, center_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(annotated_image, (center_x, center_y), 4, (255, 0, 255), -1)
        
        return annotated_image
    
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
        print("[_get_depth_at_point] Inputs:")
        print(f"  requested (x, y) = ({x}, {y}), window_size = {window_size}")
        print(f"  depth_image shape = (h={h}, w={w})")
        
        # Clamp coordinates
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        print(f"  clamped (x, y) = ({x}, {y})")
        
        # Extract window around point
        half_window = window_size // 2
        x1 = max(0, x - half_window)
        x2 = min(w, x + half_window + 1)
        y1 = max(0, y - half_window)
        y2 = min(h, y + half_window + 1)
        print(f"  window bounds x:[{x1}, {x2}) y:[{y1}, {y2})")
        
        depth_window = depth_image[y1:y2, x1:x2]
        valid_depths = depth_window[depth_window > 0]
        print(f"  window size = {depth_window.size}, valid count = {valid_depths.size}")
        if valid_depths.size > 0:
            print(f"  valid depths sample (up to 10): {valid_depths.flatten()[:10]}")
        
        if len(valid_depths) > 0:
            median_depth = float(np.median(valid_depths))
            print(f"  median(valid_depths) = {median_depth}")
            return median_depth
        else:
            print("  no valid depths; returning 0.0")
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
        # Compute intrinsics and normalized pixel like the visualizer
        fx = fy = 606.0
        cx = cy = 0.0
        have_intr = self.calibrator is not None and getattr(self.calibrator, "intrinsics", None) is not None
        if have_intr:
            fx = float(self.calibrator.intrinsics.fx)
            fy = float(self.calibrator.intrinsics.fy)
            cx = float(self.calibrator.intrinsics.cx)
            cy = float(self.calibrator.intrinsics.cy)
        else:
            cx = 320.0
            cy = 240.0
        
        x_norm = y_norm = 0.0
        if (
            self.use_calibration
            and self.calibrator is not None
            and getattr(self.calibrator, "camera_matrix", None) is not None
            and getattr(self.calibrator, "dist_coeffs", None) is not None
        ):
            try:
                pts = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
                und = cv2.undistortPoints(pts, self.calibrator.camera_matrix, self.calibrator.dist_coeffs, P=None)
                x_norm = float(und[0, 0, 0])        # und[0,0] = undistorted pts (x,y)
                y_norm = float(und[0, 0, 1])
            except Exception:
                x_norm = (float(pixel_x) - cx) / fx
                y_norm = (float(pixel_y) - cy) / fy
        else:
            x_norm = (float(pixel_x) - cx) / fx
            y_norm = (float(pixel_y) - cy) / fy
        
        # Form camera-space point at depth and reflect camera X (match visualizer)
        cam_point = np.array([x_norm * depth_mm, y_norm * depth_mm, depth_mm], dtype=float)
        cam_point = np.diag([-1.0, 1.0, 1.0]) @ cam_point
        
        # Determine R_bc and t_bc per visualizer
        R_bc = None
        t_bc = None
        handeye_ok = (self.use_calibration and self.calibrator is not None and getattr(self.calibrator, "handeye_R_ee2cam", None) is not None and getattr(self.calibrator, "handeye_t_ee2cam", None) is not None)
        extr_ok = (self.use_calibration and self.calibrator is not None and getattr(self.calibrator, "extrinsics", None) is not None)
        
        if handeye_ok:
            try:
                # Build R_be using the same yaw convention as the visualizer (J1+J2+J4)
                yaw_deg = 0.0
                t_be = None
                if scara_control is not None:
                    try:
                        # If SCARA exposes current joints and EE pose, use them
                        yaw_deg = float(scara_control.CUR_J1 + scara_control.CUR_J2 + scara_control.CUR_J4)
                        t_be = np.array([float(scara_control.CUR_X), float(scara_control.CUR_Y), float(scara_control.CUR_Z)], dtype=float)
                    except Exception:
                        # Fallback to camera_direction for yaw only
                        try:
                            yaw_deg = float(scara_control.get_camera_direction())
                        except Exception:
                            yaw_deg = 0.0
                theta = math.radians(yaw_deg)
                R_be = np.array([[math.cos(theta), -math.sin(theta), 0.0],
                                 [math.sin(theta),  math.cos(theta), 0.0],
                                 [0.0,              0.0,             1.0]], dtype=float)
                R_bc = R_be @ self.calibrator.handeye_R_ee2cam  # MARK: Axis correction is included in the calibrator
                # Compose translation per visualizer: t_bc = R_be @ t_ee2cam + t_be
                if t_be is None:
                    # If we don't have EE pose, fall back to the provided camera_position
                    t_bc = np.array([float(camera_position[0]), float(camera_position[1]), float(camera_position[2])], dtype=float)
                else:
                    t_ee2cam = self.calibrator.handeye_t_ee2cam # MARK: Axis correction is included in the calibrator
                    t_bc = R_be @ t_ee2cam + t_be
            except Exception:
                R_bc = None
        if R_bc is None and extr_ok:
            R_bc = self.calibrator.extrinsics.rotation_matrix
            # Use the live camera_position as translation to avoid stale/static t
            t_bc = np.array([float(camera_position[0]), float(camera_position[1]), float(camera_position[2])], dtype=float)
        
        if R_bc is not None and t_bc is not None:
            # Quick diagnostic: check Z orientation (expect near -1 for down-looking camera)
            try:
                print(f"[_pixel_to_arm_coordinates] R_bc[2,2] = {R_bc[2,2]:.3f}")
            except Exception:
                pass
            arm_point = t_bc + R_bc @ cam_point

            #　MARK: force calibration
            arm_point[0] = arm_point[0] - 24.0
            arm_point[2] = camera_position[2] - depth_mm
            return float(arm_point[0]), float(arm_point[1]), float(arm_point[2])
        
        # Fallback (no calibration): simplified yaw mapping, reflect X already done above
        cam_x, cam_y, cam_z = camera_position
        try:
            yaw_deg = scara_control.get_camera_direction() if scara_control is not None else 0.0
        except Exception:
            yaw_deg = 0.0
        yaw_rad = math.radians(yaw_deg)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        world_dx = (cam_point[0]) * cos_y - (cam_point[1]) * sin_y
        world_dy = (cam_point[0]) * sin_y + (cam_point[1]) * cos_y
        arm_x = cam_x + world_dx - 24.0
        arm_y = cam_y + world_dy
        arm_z = cam_z - cam_point[2] - 48.0
        return (float(arm_x), float(arm_y), float(arm_z))

    def debug_pixel_to_arm_coordinates(self,
                                       j1_deg: float,
                                       j2_deg: float,
                                       j3_mm: float,
                                       j4_deg: float,
                                       camera_position: Tuple[float, float, float],
                                       pixel_x: int,
                                       pixel_y: int,
                                       depth_mm: float,
                                       use_calibration: Optional[bool] = None,
                                       use_undistort: bool = True) -> Tuple[float, float, float]:
        """
        Debug helper: project a pixel at a given depth to arm coordinates, using provided arm state.
        - If calibration is available and enabled, uses hand-eye (preferred) or static extrinsics.
        - Otherwise, falls back to simplified yaw-only mapping (matches _pixel_to_arm_coordinates fallback).
        
        Args:
            j1_deg, j2_deg, j3_mm, j4_deg: Current arm joint values (degrees, degrees, mm, degrees)
            camera_position: (cam_x, cam_y, cam_z) in arm/base frame (mm)
            pixel_x, pixel_y: Pixel coordinates
            depth_mm: Depth in millimeters at that pixel
            use_calibration: Force enable/disable calibration path (default: follow self.use_calibration)
            use_undistort: If True and intrinsics/distortion are available, undistort the pixel
        
        Returns:
            (arm_x, arm_y, arm_z) in millimeters
        """
        try:
            # Decide calibration usage
            use_calib = self.use_calibration if use_calibration is None else use_calibration
            has_intr = (self.calibrator is not None and getattr(self.calibrator, "intrinsics", None) is not None)
            has_handeye = (
                self.calibrator is not None
                and getattr(self.calibrator, "handeye_R_ee2cam", None) is not None
                and getattr(self.calibrator, "handeye_t_ee2cam", None) is not None
            )
            has_extrinsics = (self.calibrator is not None and getattr(self.calibrator, "extrinsics", None) is not None)
            
            # Intrinsics for back-projection
            if has_intr:
                fx = float(self.calibrator.intrinsics.fx)
                fy = float(self.calibrator.intrinsics.fy)
                cx = float(self.calibrator.intrinsics.cx)
                cy = float(self.calibrator.intrinsics.cy)
            else:
                fx = fy = 615.0
                cx, cy = 320.0, 240.0
            
            # Back-project pixel to camera coordinates (reflect X to match runtime convention)
            if (
                use_undistort
                and has_intr
                and getattr(self.calibrator, "camera_matrix", None) is not None
                and getattr(self.calibrator, "dist_coeffs", None) is not None
            ):
                pts = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
                try:
                    und = cv2.undistortPoints(pts, self.calibrator.camera_matrix, self.calibrator.dist_coeffs, P=None)
                    x_norm = float(und[0, 0, 0])
                    y_norm = float(und[0, 0, 1])
                    cam_x = -x_norm * depth_mm
                    cam_y = y_norm * depth_mm
                    cam_z = depth_mm
                except Exception:
                    cam_x = - (pixel_x - cx) * depth_mm / fx
                    cam_y = (pixel_y - cy) * depth_mm / fy
                    cam_z = depth_mm
            else:
                cam_x = - (pixel_x - cx) * depth_mm / fx
                cam_y = (pixel_y - cy) * depth_mm / fy
                cam_z = depth_mm
            camera_point = np.array([cam_x, cam_y, cam_z], dtype=float)
            
            cam_tx, cam_ty, cam_tz = camera_position
            t_bc = np.array([cam_tx, cam_ty, cam_tz], dtype=float)
            
            # If we can build a full R_bc, do so; otherwise use simplified yaw-only mapping
            if use_calib and (has_handeye or has_extrinsics):
                if has_handeye:
                    theta = math.radians(j1_deg + j2_deg + j4_deg)
                    R_be = np.array([
                        [math.cos(theta), -math.sin(theta), 0.0],
                        [math.sin(theta),  math.cos(theta), 0.0],
                        [0.0,              0.0,             1.0]
                    ], dtype=float)
                    R_ee2cam = self.calibrator.handeye_R_ee2cam
                    t_ee2cam = self.calibrator.handeye_t_ee2cam
                    R_bc = R_be @ R_ee2cam
                    # Prefer composed translation if hand-eye is available and arm pose is provided
                    t_bc_full = R_be @ t_ee2cam + np.array([
                        # EE translation t_be from j1,j2,j3
                        self._fk_xy(j1_deg, j2_deg)[0],
                        self._fk_xy(j1_deg, j2_deg)[1],
                        j3_mm
                    ], dtype=float)
                    # Use provided camera_position only if it matches the composed one; otherwise use composed
                    t_bc_use = t_bc_full
                else:
                    # Static extrinsics path: rotation from extrinsics, but translation from provided camera_position
                    R_bc = self.calibrator.extrinsics.rotation_matrix
                    t_bc_use = t_bc

                arm_point = R_bc @ camera_point + t_bc_use
                arm_point[0] = arm_point[0] - 24.0
                arm_point[2] = arm_point[2] - 48.0
                return float(arm_point[0]), float(arm_point[1]), float(arm_point[2])
            else:
                # Simplified yaw-only mapping (camera assumed pointing down)
                yaw_rad = math.radians(j1_deg + j2_deg + j4_deg)
                cos_yaw = math.cos(yaw_rad)
                sin_yaw = math.sin(yaw_rad)
                world_dx = cam_x * cos_yaw - cam_y * sin_yaw
                world_dy = cam_x * sin_yaw + cam_y * cos_yaw
                arm_x = cam_tx + world_dx - 24.0
                arm_y = cam_ty + world_dy
                arm_z = cam_tz - cam_z - 48.0
                return float(arm_x), float(arm_y), float(arm_z)
        except Exception as e:
            print(f"[debug_pixel_to_arm_coordinates] Error: {e}")
            return 0.0, 0.0, 0.0
    
    def _process_aruco_plates(self):
        """Process all detected ArUco markers to define plates"""
        if self.aruco_detector is None:
            return
        
        print("Processing ArUco markers to define plates...")
        
        try:
            # Get all markers from the detector
            all_markers = self.aruco_detector.detected_markers
            
            if not all_markers:
                print("No ArUco markers detected during scanning")
                return
            
            # Define plates from markers
            plates = self.aruco_detector.define_plates_from_markers()
            
            if plates:
                self.detected_plates = plates
                print(f"✓ Defined {len(plates)} plates from ArUco markers")
                
                # Print plate information
                for i, plate in enumerate(plates):
                    print(f"  Plate {i+1}: {plate.dimensions[0]:.0f}x{plate.dimensions[1]:.0f}mm")
                    print(f"    Center: ({plate.center_arm_coords[0]:.1f}, {plate.center_arm_coords[1]:.1f}, {plate.center_arm_coords[2]:.1f})")
                    print(f"    Height: {plate.height_mm:.1f}mm")
                    print(f"    Area: {plate.area_mm2:.0f}mm²")
                    print(f"    Orientation: {plate.orientation_deg:.1f}°")
            else:
                print("⚠ Could not define plates from detected markers")
                
        except Exception as e:
            print(f"Error processing ArUco plates: {e}")
    
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

    def _deduplicate_aruco_markers(self, markers: List) -> List:
        """Deduplicate ArUco markers accumulated over multiple frames.
        Strategy:
        - Group by marker_id
        - Within each group, keep one representative per spatial cluster (in arm coords)
        - Clustering uses a simple greedy threshold on 3D distance
        """
        if not markers:
            return []

        # Group by ID
        by_id: Dict[int, List] = {}
        for m in markers:
            mid = int(getattr(m, 'marker_id', -1))
            by_id.setdefault(mid, []).append(m)

        def distance(a, b) -> float:
            ax, ay, az = a.arm_coords
            bx, by, bz = b.arm_coords
            return math.sqrt((ax-bx)**2 + (ay-by)**2 + (az-bz)**2)

        deduped: List = []
        # Distance threshold in mm to consider same physical marker across frames
        same_marker_distance_mm = 25.0

        for mid, group in by_id.items():
            representatives: List = []
            for m in group:
                placed = False
                for r in representatives:
                    if distance(m, r) < same_marker_distance_mm:
                        # Optionally, keep the one closer to camera (lower depth) or just keep existing
                        placed = True
                        break
                if not placed:
                    representatives.append(m)
            deduped.extend(representatives)

        return deduped
    
    def _convert_numpy(self, obj):
        import numpy as np
        if isinstance(obj, dict):
            return {k: self._convert_numpy(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self._convert_numpy(i) for i in obj]
        elif isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.int32, np.int64)):
            return int(obj)
        elif isinstance(obj, (np.float32, np.float64)):
            return float(obj)
        elif isinstance(obj, tuple):
            return [self._convert_numpy(i) for i in obj]
        elif isinstance(obj, np.bool_):
            return bool(obj)
        else:
            return obj
    
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
                    "total_plates": len(self.detected_plates),
                    "scan_positions": len(self.scan_positions),
                    "model_path": str(self.model_path)
                },
                "detected_objects": [],
                "detected_plates": []
            }
            
            for obj in self.detected_objects:
                obj_data = {
                    "object_id": int(obj.object_id),
                    "class_name": obj.class_name,
                    "confidence": float(obj.confidence),
                    "bbox": [int(obj.bbox[0]), int(obj.bbox[1]), int(obj.bbox[2]), int(obj.bbox[3])],
                    "center_pixel": [int(obj.center_pixel[0]), int(obj.center_pixel[1])],
                    "depth_mm": float(obj.depth_mm),
                    "arm_coordinates": {
                        "x": float(obj.arm_coords[0]),
                        "y": float(obj.arm_coords[1]),
                        "z": float(obj.arm_coords[2])
                    },
                    "camera_position": {
                        "x": float(obj.camera_position[0]),
                        "y": float(obj.camera_position[1]),
                        "z": float(obj.camera_position[2])
                    }
                }
                results["detected_objects"].append(obj_data)
            
            # Add plate data
            for plate in self.detected_plates:
                plate_data = {
                    "plate_id": plate.plate_id,
                    "center_arm_coordinates": {
                        "x": float(plate.center_arm_coords[0]),
                        "y": float(plate.center_arm_coords[1]),
                        "z": float(plate.center_arm_coords[2])
                    },
                    "dimensions": {
                        "width_mm": float(plate.dimensions[0]),
                        "length_mm": float(plate.dimensions[1])
                    },
                    "height_mm": float(plate.height_mm),
                    "area_mm2": float(plate.area_mm2),
                    "orientation_deg": float(plate.orientation_deg),
                    "edge_marker_ids": [int(m.marker_id) for m in plate.edge_markers]
                }
                results["detected_plates"].append(plate_data)
            
            output_file = self.output_dir / filename
            with open(output_file, 'w') as f:
                import json
                json.dump(self._convert_numpy(results), f, indent=2)
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
        
        # Plot detected plates
        if self.detected_plates:
            # Use different colors for each plate
            plate_colors = plt.cm.tab10(np.linspace(0, 1, len(self.detected_plates)))
            
            for i, plate in enumerate(self.detected_plates):
                color = plate_colors[i]
                
                # Plot plate center
                ax.scatter(plate.center_arm_coords[0], plate.center_arm_coords[1], plate.center_arm_coords[2],
                          c=[color], s=150, alpha=0.8, marker='s', label=f'Plate {i+1}')
                
                # Plot plate corners and edges
                corners = plate.corners_arm_coords
                
                # Draw plate edges (connect corners in order)
                for j in range(4):
                    start_corner = corners[j]
                    end_corner = corners[(j + 1) % 4]
                    
                    # Draw edge line
                    ax.plot([start_corner[0], end_corner[0]], 
                           [start_corner[1], end_corner[1]], 
                           [start_corner[2], end_corner[2]], 
                           c=color, linewidth=2, alpha=0.8)
                
                # Add plate annotation
                ax.text(plate.center_arm_coords[0], plate.center_arm_coords[1], plate.center_arm_coords[2] + 15,
                       f'P{i+1}: {plate.dimensions[0]:.0f}x{plate.dimensions[1]:.0f}mm\nH:{plate.height_mm:.0f}mm',
                       fontsize=8, ha='center', va='bottom')
        
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
        
        # Update title to include plate information
        title_parts = [f'3D Object Detection Results\n{len(self.detected_objects)} objects detected']
        if self.detected_plates:
            title_parts.append(f'{len(self.detected_plates)} plates detected')
        ax.set_title('\n'.join(title_parts))
        
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
    
    def visualize_plates_only(self, show_markers: bool = True) -> None:
        """
        Create a dedicated 3D visualization of detected plates
        
        Args:
            show_markers: Whether to show individual ArUco markers
        """
        if not self.detected_plates:
            print("No plates detected to visualize")
            return
        
        print("Creating 3D plate visualization...")
        
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot each plate with detailed information
        for i, plate in enumerate(self.detected_plates):
            # Use distinct color for each plate
            color = plt.cm.tab10(i / len(self.detected_plates))
            
            # Plot plate center
            ax.scatter(plate.center_arm_coords[0], plate.center_arm_coords[1], plate.center_arm_coords[2],
                      c=[color], s=200, alpha=0.9, marker='s', label=f'Plate {i+1} ({plate.plate_id})')
            
            # Plot plate corners
            corners = plate.corners_arm_coords
            corner_coords = np.array(corners)
            ax.scatter(corner_coords[:, 0], corner_coords[:, 1], corner_coords[:, 2],
                      c=[color], s=100, alpha=0.7, marker='o')
            
            # Draw plate edges (thicker lines)
            for j in range(4):
                start_corner = corners[j]
                end_corner = corners[(j + 1) % 4]
                
                ax.plot([start_corner[0], end_corner[0]], 
                       [start_corner[1], end_corner[1]], 
                       [start_corner[2], end_corner[2]], 
                       c=color, linewidth=3, alpha=0.8)
            
            # Add detailed plate annotation
            info_text = f'P{i+1}: {plate.dimensions[0]:.0f}×{plate.dimensions[1]:.0f}mm\n'
            info_text += f'H: {plate.height_mm:.0f}mm\n'
            info_text += f'A: {plate.area_mm2:.0f}mm²\n'
            info_text += f'θ: {plate.orientation_deg:.1f}°'
            
            ax.text(plate.center_arm_coords[0], plate.center_arm_coords[1], plate.center_arm_coords[2] + 20,
                   info_text, fontsize=9, ha='center', va='bottom', 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor='white', alpha=0.8))
            
            # Show individual markers if requested
            if show_markers:
                for j, marker in enumerate(plate.edge_markers):
                    marker_color = plt.cm.Set1(j / 4)  # Different color for each marker
                    ax.scatter(marker.arm_coords[0], marker.arm_coords[1], marker.arm_coords[2],
                              c=[marker_color], s=80, alpha=0.8, marker='^')
                    
                    # Add marker ID
                    ax.text(marker.arm_coords[0], marker.arm_coords[1], marker.arm_coords[2] - 10,
                           f'ArUco:{marker.marker_id}', fontsize=7, ha='center', va='top')
        
        # Add SCARA base reference
        ax.scatter([0], [0], [0], c='red', s=300, marker='x', label='SCARA Base')
        
        # Customize plot
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title(f'3D Plate Detection Results\n{len(self.detected_plates)} plates detected')
        
        # Set aspect ratio
        ax.set_box_aspect([1, 1, 0.5])
        
        plt.tight_layout()
        
        # Save plot
        plot_file = self.output_dir / "3d_plate_results.png"
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        print(f"3D plate plot saved to {plot_file}")
        
        plt.show()
    
    def get_workspace_info(self) -> Dict:
        """
        Get workspace information based on detected plates
        
        Returns:
            Dictionary with workspace information
        """
        if not self.detected_plates:
            return {}
        
        # Find the largest plate (assumed to be the main workspace)
        main_plate = max(self.detected_plates, key=lambda p: p.area_mm2)
        
        workspace_info = {
            "main_plate": {
                "center": main_plate.center_arm_coords,
                "dimensions": main_plate.dimensions,
                "height": main_plate.height_mm,
                "area": main_plate.area_mm2,
                "orientation": main_plate.orientation_deg
            },
            "total_plates": len(self.detected_plates),
            "workspace_bounds": self._calculate_workspace_bounds()
        }
        
        return workspace_info
    
    def _calculate_workspace_bounds(self) -> Dict:
        """Calculate workspace bounds based on all detected plates"""
        if not self.detected_plates:
            return {}
        
        all_x = []
        all_y = []
        all_z = []
        
        for plate in self.detected_plates:
            for corner in plate.corners_arm_coords:
                all_x.append(corner[0])
                all_y.append(corner[1])
                all_z.append(corner[2])
        
        bounds = {
            "x_min": min(all_x),
            "x_max": max(all_x),
            "y_min": min(all_y),
            "y_max": max(all_y),
            "z_min": min(all_z),
            "z_max": max(all_z)
        }
        
        return bounds
    
    def cleanup(self):
        """Clean up resources"""
        if self.camera:
            self.camera.release()
        print("Object detection system cleaned up")


def debug_pixel_to_arm_coordinates(j1_deg: float,
                                   j2_deg: float,
                                   j3_mm: float,
                                   j4_deg: float,
                                   camera_position: Tuple[float, float, float],
                                   pixel_x: int,
                                   pixel_y: int,
                                   depth_mm: float,
                                   use_calibration: Optional[bool] = None,
                                   use_undistort: bool = False) -> Tuple[float, float, float]:
    """
    Debug helper that mirrors the visualizer’s calibrated flow exactly.
    Requires calibration (intrinsics + hand-eye). No SCARA API calls.

    Steps:
      1) Undistort pixel if possible; else pinhole intrinsics
      2) cam_point = [x_norm*Z, y_norm*Z, Z]; then reflect X via diag([-1,1,1])
      3) R_be from yaw = j1+j2+j4 (deg)
      4) R_bc = R_be @ R_ee2cam
      5) t_bc = camera_position (base->cam translation)
      6) arm_point = t_bc + R_bc @ cam_point
    """
    # Build a minimal system holder to reuse loaded calibration
    system = ObjectDetectionSystem(use_calibration=True, save_images=False)
    try:
        from RealSense.camera_calibration import CameraCalibrator
        system.calibrator = CameraCalibrator()
        calib_file = project_root / "RealSense" / "camera_calibration" / "camera_calibration.json"
        if not system.calibrator.load_calibration(calib_file):
            raise RuntimeError("Calibration not available")
    except Exception as e:
        raise RuntimeError(f"Calibration load failed: {e}")

    if use_calibration is False:
        raise RuntimeError("This debug function requires calibrated flow (use_calibration=True)")

    calib = system.calibrator
    if calib is None or getattr(calib, "handeye_R_ee2cam", None) is None or getattr(calib, "handeye_t_ee2cam", None) is None:
        raise RuntimeError("Hand-eye not available in calibration")

    # 1) Intrinsics / normalized pixel
    if getattr(calib, "intrinsics", None) is not None:
        fx = float(calib.intrinsics.fx)
        fy = float(calib.intrinsics.fy)
        cx = float(calib.intrinsics.cx)
        cy = float(calib.intrinsics.cy)
    else:
        fx = fy = 606.0
        cx = 320.0
        cy = 240.0

    if use_undistort and getattr(calib, "camera_matrix", None) is not None and getattr(calib, "dist_coeffs", None) is not None:
        pts = np.array([[[float(pixel_x), float(pixel_y)]]], dtype=np.float32)
        try:
            und = cv2.undistortPoints(pts, calib.camera_matrix, calib.dist_coeffs, P=None)
            x_norm = float(und[0, 0, 0])
            y_norm = float(und[0, 0, 1])
        except Exception:
            x_norm = (float(pixel_x) - cx) / fx
            y_norm = (float(pixel_y) - cy) / fy
    else:
        x_norm = (float(pixel_x) - cx) / fx
        y_norm = (float(pixel_y) - cy) / fy

    # 2) Camera-space point + reflect X
    cam_point = np.array([x_norm * depth_mm, y_norm * depth_mm, depth_mm], dtype=float)
    cam_point = np.diag([-1.0, 1.0, 1.0]) @ cam_point

    # 3) R_be from joints (yaw = j1+j2+j4)
    yaw_deg = float(j1_deg + j2_deg + j4_deg)
    theta = math.radians(yaw_deg)
    R_be = np.array([[math.cos(theta), -math.sin(theta), 0.0],
                     [math.sin(theta),  math.cos(theta), 0.0],
                     [0.0,              0.0,             1.0]], dtype=float)

    # 4) R_bc using corrected hand-eye
    R_ee2cam = calib.handeye_R_ee2cam
    R_bc = R_be @ R_ee2cam

    # 5) t_bc from provided camera_position (base->cam)
    t_bc = np.array([float(camera_position[0]), float(camera_position[1]), float(camera_position[2])], dtype=float)

    # 6) Transform to arm/base frame
    arm_point = t_bc + R_bc @ cam_point
    arm_point[0] = arm_point[0] - 24.0
    arm_point[2] = arm_point[2] - 48.0
    return float(arm_point[0]), float(arm_point[1]), float(arm_point[2])


def main():
    """Main function to run the object detection system"""
    print("=" * 60)
    print("SCARA AGV Object Detection System")
    print("=" * 60)
    
    # Initialize system
    detector = ObjectDetectionSystem(
        model_path="yolo/my_model/my_model.pt",
        use_calibration=True,
        save_images=True  # Enable annotated image saving
    )
    
    try:
        # Initialize all components
        if not detector.initialize():
            print("Failed to initialize system")
            return
        
        # Plan scanning positions
        detector.plan_scanning_positions(
            scan_height=280.0,  # 150mm above table
            grid_spacing=80.0,  # 100mm between scan points
            camera_direction=-90.0,  # Camera pointing down
            x_min=0.0, y_min=-300, y_max=300
        )
        
        # Run the scan
        if not detector.scan_workspace(confidence_threshold=0.7):
            print("Scanning failed")
            return
        
        # Save results
        detector.save_results("detected_objects.json")
        
        # Show workspace information
        workspace_info = detector.get_workspace_info()
        if workspace_info:
            print(f"\nWorkspace Information:")
            print(f"  Main plate: {workspace_info['main_plate']['dimensions'][0]:.0f}x{workspace_info['main_plate']['dimensions'][1]:.0f}mm")
            print(f"  Plate height: {workspace_info['main_plate']['height']:.1f}mm")
            print(f"  Total plates: {workspace_info['total_plates']}")
        
        # Create visualizations
        detector.visualize_3d()
        
        # Also create dedicated plate visualization if plates were detected
        if detector.detected_plates:
            detector.visualize_plates_only()
        
        print(f"\nDetection completed successfully!")
        print(f"Total objects detected: {len(detector.detected_objects)}")
        print(f"Results saved to: {detector.output_dir}")
        
    except KeyboardInterrupt:
        print("\nDetection interrupted by user")
    except Exception as e:
        print(f"Error during detection: {e}")
    finally:
        if scara_control is not None:
            try:
                scara_control.quick(scara_control.ORIGIN_X, scara_control.ORIGIN_Y, scara_control.ORIGIN_Z)
            except Exception as e:
                print(f"⚠ Warning: Could not return to origin: {e}")
        detector.cleanup()


if __name__ == "__main__":
    main()
