#!/usr/bin/env python3
"""
Camera Calibration System for SCARA AGV
Calibrates RealSense camera intrinsics, extrinsics, and arm-camera transformation
"""

import os
import sys
import time
import numpy as np
import cv2
import json
from pathlib import Path
from typing import Tuple, List, Dict, Optional, Union
import pyrealsense2 as rs
from dataclasses import dataclass

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    import Arm_Control.SCARA as scara_control
    from Arm_Control.SCARA_Simulator import SCARA
except ImportError:
    scara_control = None
    from Arm_Control.SCARA_Simulator import SCARA

@dataclass
class CameraIntrinsics:
    """Camera intrinsic parameters"""
    fx: float  # Focal length x
    fy: float  # Focal length y
    cx: float  # Principal point x
    cy: float  # Principal point y
    width: int
    height: int
    distortion_coeffs: np.ndarray  # Distortion coefficients

@dataclass
class CameraExtrinsics:
    """Camera extrinsic parameters (camera to arm transformation)"""
    rotation_matrix: np.ndarray  # 3x3 rotation matrix
    translation_vector: np.ndarray  # 3x1 translation vector

class CameraCalibrator:
    """
    Comprehensive camera calibration system for SCARA AGV
    """
    
    def __init__(self, chessboard_size: Tuple[int, int] = (9, 6), 
                 square_size: float = 20.0, fast_mode: bool = True):
        """
        Initialize camera calibrator
        
        Args:
            chessboard_size: Number of internal corners (width, height)
            square_size: Size of chessboard squares in mm
            fast_mode: Use lower resolution and frame rate for better performance
        """
        self.chessboard_size = chessboard_size
        self.square_size = square_size
        self.fast_mode = fast_mode
        
        # Calibration parameters
        self.intrinsics = None
        self.extrinsics = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Initialize components
        self.camera = None
        self.arm = None
        self.is_real_arm = False
        
        # Calibration data
        self.object_points = []  # 3D points in world coordinates
        self.image_points = []   # 2D points in image coordinates
        self.depth_points = []   # Depth measurements
        
        # Create calibration directory
        self.calibration_dir = Path("camera_calibration")
        self.calibration_dir.mkdir(exist_ok=True)
        
    def _initialize_camera(self):
        """Initialize RealSense camera"""
        try:
            self.camera = rs.pipeline()
            config = rs.config()
            
            if self.fast_mode:
                # Fast mode: lower resolution and frame rate
                width, height, fps = 640, 480, 15
                print("Using fast mode for better performance")
            else:
                # High quality mode
                width, height, fps = 1280, 720, 30
                print("Using high quality mode")
            
            config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            
            # Start streaming
            self.camera.start(config)
            
            # Align depth to color stream
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            
            print(f"RealSense camera initialized successfully ({width}x{height} @ {fps}fps)")
            
        except Exception as e:
            print(f"Error initializing camera: {e}")
            raise
    
    def _initialize_arm(self):
        """Initialize SCARA arm"""
        try:
            if scara_control is not None:
                # Try to use real SCARA (function-based)
                print("Using real SCARA arm control")
                self.arm = None  # We'll call functions directly
                self.is_real_arm = True
            else:
                # Fall back to simulator
                self.arm = SCARA()
                self.is_real_arm = False
                print("SCARA simulator initialized successfully")
        except Exception as e:
            print(f"Warning: Arm initialization failed: {e}")
            print("Running in simulation mode")
            self.arm = SCARA()
            self.is_real_arm = False
    
    def get_frame(self) -> Tuple[bool, np.ndarray, np.ndarray]:
        """Get color and depth frames from camera"""
        try:
            frames = self.camera.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return False, None, None
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            return True, depth_image, color_image
            
        except Exception as e:
            print(f"Error getting frame: {e}")
            return False, None, None
    
    def find_chessboard_corners(self, image: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        Find chessboard corners in image
        
        Args:
            image: Input image
            
        Returns:
            (success, corners) - Whether corners were found and corner coordinates
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Optimize chessboard detection for speed
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        if self.fast_mode:
            # Skip normalization in fast mode for better performance
            flags += cv2.CALIB_CB_FILTER_QUADS
        else:
            flags += cv2.CALIB_CB_NORMALIZE_IMAGE
        
        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, flags)
        
        if ret and not self.fast_mode:
            # Only refine corners in high quality mode (slow operation)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
        return ret, corners
    
    def calibrate_intrinsics(self, num_images: int = 20) -> bool:
        """
        Calibrate camera intrinsics using chessboard pattern
        
        Args:
            num_images: Number of calibration images to capture
            
        Returns:
            True if calibration successful
        """
        print(f"Starting intrinsic calibration with {num_images} images...")
        print("Hold chessboard pattern in front of camera at different angles")
        print("Press 'c' to capture, 'q' to quit")
        
        self._initialize_camera()
        
        # Prepare object points (3D coordinates of chessboard corners)
        objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size  # Scale by square size
        
        captured_count = 0
        last_frame_time = time.time()
        frame_skip = 0
        
        while captured_count < num_images:
            # Limit frame rate to improve responsiveness
            current_time = time.time()
            if current_time - last_frame_time < 0.1:  # 10 FPS max
                frame_skip += 1
                if frame_skip < 3:  # Skip frames but still check for key presses
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        break
                    continue
            
            frame_skip = 0
            last_frame_time = current_time
            
            success, depth_image, color_image = self.get_frame()
            
            if not success:
                print("Failed to get camera frame")
                continue
            
            # Resize for display to improve performance
            display_scale = 0.8
            display_height = int(color_image.shape[0] * display_scale)
            display_width = int(color_image.shape[1] * display_scale)
            
            display_image = cv2.resize(color_image, (display_width, display_height))
            
            # Add text overlays (scaled for display)
            font_scale = 0.6
            cv2.putText(display_image, f"Captured: {captured_count}/{num_images}", 
                       (10, 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), 2)
            cv2.putText(display_image, "Press 'c' to capture, 'q' to quit", 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (255, 255, 255), 2)
            
            # Try to find chessboard (use original resolution)
            ret, corners = self.find_chessboard_corners(color_image)
            
            if ret:
                # Scale corners for display
                display_corners = corners * display_scale
                cv2.drawChessboardCorners(display_image, self.chessboard_size, display_corners, ret)
                cv2.putText(display_image, "Chessboard detected!", 
                           (10, 75), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (0, 255, 0), 2)
            
            cv2.imshow('Camera Calibration', display_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('c') and ret:
                # Capture this image
                self.object_points.append(objp)
                self.image_points.append(corners)
                
                # Get depth at center of chessboard
                center_corner = corners[self.chessboard_size[0] * self.chessboard_size[1] // 2][0]
                center_x, center_y = int(center_corner[0]), int(center_corner[1])
                depth = depth_image[center_y, center_x]
                self.depth_points.append(depth)
                
                captured_count += 1
                print(f"Captured image {captured_count}/{num_images}")
                
                # Save calibration image
                cv2.imwrite(str(self.calibration_dir / f"calib_image_{captured_count}.jpg"), color_image)
                
                time.sleep(0.5)  # Small delay between captures
        
        cv2.destroyAllWindows()
        
        if captured_count < 10:
            print("Not enough images captured for calibration")
            return False
        
        # Perform calibration
        print("Performing intrinsic calibration...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.object_points, self.image_points, 
            color_image.shape[:2][::-1], None, None
        )
        
        if not ret:
            print("Calibration failed")
            return False
        
        # Store calibration results
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        # Create intrinsics object
        self.intrinsics = CameraIntrinsics(
            fx=camera_matrix[0, 0],
            fy=camera_matrix[1, 1],
            cx=camera_matrix[0, 2],
            cy=camera_matrix[1, 2],
            width=color_image.shape[1],
            height=color_image.shape[0],
            distortion_coeffs=dist_coeffs
        )
        
        # Calculate reprojection error
        mean_error = 0
        for i in range(len(self.object_points)):
            imgpoints2, _ = cv2.projectPoints(
                self.object_points[i], rvecs[i], tvecs[i], 
                camera_matrix, dist_coeffs
            )
            error = cv2.norm(self.image_points[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            mean_error += error
        
        print(f"Calibration completed successfully!")
        print(f"Reprojection error: {mean_error/len(self.object_points):.4f} pixels")
        print(f"Camera matrix:\n{camera_matrix}")
        print(f"Distortion coefficients: {dist_coeffs.flatten()}")
        
        return True
    
    def calibrate_extrinsics(self, num_positions: int = 5) -> bool:
        """
        Calibrate camera extrinsics (camera to arm transformation)
        
        Args:
            num_positions: Number of arm positions to use for calibration
            
        Returns:
            True if calibration successful
        """
        if self.intrinsics is None:
            print("Must calibrate intrinsics first")
            return False
        
        print(f"Starting extrinsic calibration with {num_positions} arm positions...")
        print("This will move the arm to different positions to capture the chessboard")
        
        # Initialize camera for extrinsic calibration
        self._initialize_camera()
        self._initialize_arm()
        
        # Define arm positions for calibration
        # These should be positions where the chessboard is visible
        # Positions are for camera placement (not end effector)
        arm_positions = [
            (250, -50, 200),      # Center
            (250, 0, 200),   # Left
            (250, -100, 200),    # Right
            (300, -50, 200),   # Front
            (200, -50, 200),    # Back
        ]
        
        arm_points = []  # Arm coordinates
        camera_points = []  # Camera coordinates
        
        for i, (x, y, z) in enumerate(arm_positions[:num_positions]):
            print(f"Position {i+1}/{num_positions}: ({x}, {y}, {z})")
            
            # Move arm to position
            if self.is_real_arm and scara_control:
                # Use real SCARA functions
                print(f"Moving camera to position: ({x}, {y}, {z})")
                scara_control.quick_camera(x, y, z)
                time.sleep(3.0)  # Wait for movement
            elif self.arm:
                # Use simulator
                self.arm.quick_camera(x, y, z)
                time.sleep(2.0)  # Wait for movement
            else:
                print(f"Simulating movement to position: ({x}, {y}, {z})")
                time.sleep(1.0)  # Simulate movement time
            
            # Show live camera view for positioning
            print(f"Position the chessboard in view and press 'c' to capture, 's' to skip this position")
            chessboard_found = False
            
            while not chessboard_found:
                # Capture image
                success, depth_image, color_image = self.get_frame()
                if not success:
                    print(f"Failed to get camera frame at position {i+1}")
                    time.sleep(0.1)
                    continue
                
                # Create display image
                display_scale = 0.8
                display_height = int(color_image.shape[0] * display_scale)
                display_width = int(color_image.shape[1] * display_scale)
                display_image = cv2.resize(color_image, (display_width, display_height))
                
                # Try to find chessboard
                ret, corners = self.find_chessboard_corners(color_image)
                
                # Add status text
                font_scale = 0.6
                cv2.putText(display_image, f"Extrinsic Calibration - Position {i+1}/{num_positions}", 
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)
                cv2.putText(display_image, f"Camera at: ({x:.0f}, {y:.0f}, {z:.0f})", 
                           (10, 50), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (255, 255, 255), 2)
                
                if ret:
                    # Draw chessboard corners
                    display_corners = corners * display_scale
                    cv2.drawChessboardCorners(display_image, self.chessboard_size, display_corners, ret)
                    cv2.putText(display_image, "Chessboard detected! Press 'c' to capture", 
                               (10, 75), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (0, 255, 0), 2)
                else:
                    cv2.putText(display_image, "Position chessboard in view", 
                               (10, 75), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (0, 0, 255), 2)
                
                cv2.putText(display_image, "Press 'c' to capture, 's' to skip, 'q' to quit", 
                           (10, display_height - 10), cv2.FONT_HERSHEY_SIMPLEX, font_scale * 0.8, (255, 255, 255), 2)
                
                cv2.imshow('Extrinsic Calibration - Camera View', display_image)
                key = cv2.waitKey(30) & 0xFF
                
                if key == ord('q'):
                    cv2.destroyAllWindows()
                    return False
                elif key == ord('s'):
                    print(f"Skipping position {i+1}")
                    chessboard_found = True  # Exit loop but don't use this position
                    continue
                elif key == ord('c'):
                    if ret:
                        chessboard_found = True
                        break
                    else:
                        print("No chessboard detected. Position the chessboard and try again.")
            
            # If we skipped this position, continue to next
            if key == ord('s'):
                continue
            
            # Process the captured chessboard
            if not ret:
                print(f"No chessboard found at position {i+1}")
                continue
            
            # Get depth at center
            center_corner = corners[self.chessboard_size[0] * self.chessboard_size[1] // 2][0]
            center_x, center_y = int(center_corner[0]), int(center_corner[1])
            depth = depth_image[center_y, center_x]
            
            if depth == 0:
                print(f"Invalid depth at position {i+1}")
                continue
            
            # Convert pixel to camera coordinates
            camera_x = (center_x - self.intrinsics.cx) * depth / self.intrinsics.fx
            camera_y = (center_y - self.intrinsics.cy) * depth / self.intrinsics.fy
            camera_z = depth
            
            # Store points
            arm_points.append([x, y, z])
            camera_points.append([camera_x, camera_y, camera_z])
            
            print(f"  Arm: ({x}, {y}, {z}) -> Camera: ({camera_x:.1f}, {camera_y:.1f}, {camera_z:.1f})")
        
        if len(arm_points) < 3:
            print("Not enough valid points for extrinsic calibration")
            return False
        
        # Calculate transformation using SVD
        arm_points = np.array(arm_points, dtype=np.float32)
        camera_points = np.array(camera_points, dtype=np.float32)
        
        # Center the points
        arm_center = np.mean(arm_points, axis=0)
        camera_center = np.mean(camera_points, axis=0)
        
        arm_centered = arm_points - arm_center
        camera_centered = camera_points - camera_center
        
        # Calculate rotation matrix using SVD
        H = camera_centered.T @ arm_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation matrix (det = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Calculate translation
        t = arm_center - R @ camera_center
        
        # Store extrinsics
        self.extrinsics = CameraExtrinsics(
            rotation_matrix=R,
            translation_vector=t
        )
        
        print("Extrinsic calibration completed!")
        print(f"Rotation matrix:\n{R}")
        print(f"Translation vector: {t}")
        
        # Calculate transformation error
        errors = []
        for i in range(len(arm_points)):
            transformed = R @ camera_points[i] + t
            error = np.linalg.norm(arm_points[i] - transformed)
            errors.append(error)
        
        mean_error = np.mean(errors)
        print(f"Mean transformation error: {mean_error:.2f} mm")
        
        # Close the camera view window
        cv2.destroyAllWindows()
        
        return True
    
    def save_calibration(self, filename: str = "camera_calibration.json"):
        """Save calibration parameters to file"""
        if self.intrinsics is None:
            print("No calibration data to save")
            return
        
        calibration_data = {
            "intrinsics": {
                "fx": float(self.intrinsics.fx),
                "fy": float(self.intrinsics.fy),
                "cx": float(self.intrinsics.cx),
                "cy": float(self.intrinsics.cy),
                "width": self.intrinsics.width,
                "height": self.intrinsics.height,
                "distortion_coeffs": self.intrinsics.distortion_coeffs.tolist()
            },
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist()
        }
        
        if self.extrinsics is not None:
            calibration_data["extrinsics"] = {
                "rotation_matrix": self.extrinsics.rotation_matrix.tolist(),
                "translation_vector": self.extrinsics.translation_vector.tolist()
            }
        
        filepath = self.calibration_dir / filename
        with open(filepath, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"Calibration saved to {filepath}")
    
    def load_calibration(self, filename: Union[str, Path] = "camera_calibration.json") -> bool:
        """Load calibration parameters from file
        
        Accepts either a bare filename (looked up in `self.calibration_dir`) or a
        relative/absolute path which will be used as-is.
        """
        input_path = Path(filename)
        # If a directory component is provided or the path is absolute, use it directly
        if input_path.is_absolute() or len(input_path.parts) > 1:
            filepath = input_path
        else:
            filepath = self.calibration_dir / input_path
        
        if not filepath.exists():
            print(f"Calibration file not found: {filepath}")
            return False
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Load intrinsics
            intrinsics_data = data["intrinsics"]
            self.intrinsics = CameraIntrinsics(
                fx=intrinsics_data["fx"],
                fy=intrinsics_data["fy"],
                cx=intrinsics_data["cx"],
                cy=intrinsics_data["cy"],
                width=intrinsics_data["width"],
                height=intrinsics_data["height"],
                distortion_coeffs=np.array(intrinsics_data["distortion_coeffs"])
            )
            
            # Load camera matrix and distortion coefficients
            self.camera_matrix = np.array(data["camera_matrix"])
            self.dist_coeffs = np.array(data["dist_coeffs"])
            
            # Load extrinsics if available
            if "extrinsics" in data:
                extrinsics_data = data["extrinsics"]
                self.extrinsics = CameraExtrinsics(
                    rotation_matrix=np.array(extrinsics_data["rotation_matrix"]),
                    translation_vector=np.array(extrinsics_data["translation_vector"])
                )
            
            print(f"Calibration loaded from {filepath}")
            return True
            
        except Exception as e:
            print(f"Error loading calibration: {e}")
            return False
    
    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        """Undistort image using calibration parameters"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return image
        
        h, w = image.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
        )
        
        # Undistort
        dst = cv2.undistort(image, self.camera_matrix, self.dist_coeffs, None, newcameramtx)
        
        # Crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        
        return dst
    
    def pixel_to_arm_coordinates(self, pixel_x: int, pixel_y: int, depth_mm: float) -> Tuple[float, float, float]:
        """
        Convert pixel coordinates to arm coordinates using calibration
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            depth_mm: Depth in millimeters
            
        Returns:
            (arm_x, arm_y, arm_z) in millimeters
        """
        if self.intrinsics is None:
            print("No calibration data available")
            return 0, 0, 0
        
        # Convert to camera coordinates (undistorted)
        camera_x = (pixel_x - self.intrinsics.cx) * depth_mm / self.intrinsics.fx
        camera_y = (pixel_y - self.intrinsics.cy) * depth_mm / self.intrinsics.fy
        camera_z = depth_mm
        
        # Apply extrinsics transformation if available
        if self.extrinsics is not None:
            camera_point = np.array([camera_x, camera_y, camera_z])
            arm_point = self.extrinsics.rotation_matrix @ camera_point + self.extrinsics.translation_vector
            return arm_point[0], arm_point[1], arm_point[2]
        else:
            # Fall back to simple transformation
            return camera_x, camera_y, camera_z
    
    def cleanup(self):
        """Clean up resources"""
        if self.camera:
            self.camera.stop()
        cv2.destroyAllWindows()


def main():
    """Main calibration function"""
    print("SCARA AGV Camera Calibration System")
    print("=" * 50)
    
    calibrator = CameraCalibrator(chessboard_size=(5, 4), square_size=21.5, fast_mode=True)
    
    try:
        print("Choose calibration mode:")
        print("1. Intrinsic calibration only")
        print("2. Full calibration (intrinsic + extrinsic)")
        print("3. Load existing calibration")
        print("4. Load intrinsics and redo extrinsic calibration")
        
        choice = input("Enter choice (1-4): ").strip()
        
        if choice == "1":
            # Intrinsic calibration only
            if calibrator.calibrate_intrinsics():
                calibrator.save_calibration()
            
        elif choice == "2":
            # Full calibration
            if calibrator.calibrate_intrinsics():
                if calibrator.calibrate_extrinsics():
                    calibrator.save_calibration()
            
        elif choice == "3":
            # Load existing calibration
            if calibrator.load_calibration():
                print("Calibration loaded successfully")
            else:
                print("Failed to load calibration")
        
        elif choice == "4":
            # Load intrinsics and redo extrinsic calibration
            if calibrator.load_calibration():
                print("Existing calibration loaded successfully")
                if calibrator.intrinsics is not None:
                    print("Intrinsic parameters found, proceeding with extrinsic calibration...")
                    if calibrator.calibrate_extrinsics():
                        calibrator.save_calibration()
                        print("Extrinsic calibration completed and saved!")
                    else:
                        print("Extrinsic calibration failed")
                else:
                    print("No intrinsic parameters found in the loaded calibration")
                    print("Please run intrinsic calibration first (option 1 or 2)")
            else:
                print("Failed to load existing calibration")
                print("Please run intrinsic calibration first (option 1 or 2)")
        
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\nCalibration interrupted by user")
    except Exception as e:
        print(f"Calibration error: {e}")
    finally:
        calibrator.cleanup()


if __name__ == "__main__":
    main() 