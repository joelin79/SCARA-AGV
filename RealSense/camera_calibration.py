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
from typing import Tuple, List, Dict, Optional
import pyrealsense2 as rs
from dataclasses import dataclass
import math

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
        # Hand-eye (AX=XB) parameters: ee -> cam
        self.handeye_R_ee2cam = None
        self.handeye_t_ee2cam = None
        
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
        Calibrate camera extrinsics (hand-eye, AX=XB) to obtain T_ee_cam (ee -> camera)
        
        Args:
            num_positions: Number of arm base positions to use for calibration
        
        Returns:
            True if calibration successful
        """
        if self.intrinsics is None or self.camera_matrix is None or self.dist_coeffs is None:
            print("Must calibrate or load intrinsics first")
            return False
        
        print(f"Starting hand-eye (AX=XB) calibration with {num_positions} base positions...")
        print("- Keep chessboard fixed in the workspace (world frame)")
        print("- The procedure will move the arm and vary J4/camera orientation to ensure rotation diversity")
        
        # Initialize camera and arm
        self._initialize_camera()
        self._initialize_arm()
        
        # Define base camera target positions and a set of camera orientations to provide rotation diversity
        arm_positions = [
            (250, -50, 200),
            (250, 0, 200),
            (250, -100, 200),
            (300, -50, 200),
            (200, -50, 200),
        ]
        # Use several camera directions (degrees)
        orientation_set = [-150.0, -120.0, -90.0, -60.0, -30.0, 0.0]
        
        # Precompute chessboard 3D object points (Z=0 plane in target frame)
        board_w, board_h = self.chessboard_size
        objp = np.zeros((board_w * board_h, 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
        objp *= float(self.square_size)
        
        R_gripper2base_list = []
        t_gripper2base_list = []
        R_target2cam_list = []
        t_target2cam_list = []
        
        captures = 0
        min_captures = max(12, num_positions * 2)
        
        for i, (x, y, z) in enumerate(arm_positions[:num_positions]):
            for j, ext_angle in enumerate(orientation_set):
                print(f"Pose {captures+1}: base pos=({x:.1f},{y:.1f},{z:.1f}), cam dir={ext_angle:.1f}°")
                try:
                    if self.is_real_arm and scara_control:
                        scara_control.quick_camera(x, y, z, maintain_extension_direction=True, extension_angle=ext_angle)
                        time.sleep(2.0)
                    elif self.arm:
                        self.arm.quick_camera(x, y, z, maintain_extension_direction=True, extension_angle=ext_angle)
                        time.sleep(1.0)
                    else:
                        time.sleep(0.5)
                except Exception as e:
                    print(f"Movement error: {e}")
                    continue
                
                # Try to capture and find chessboard
                success, depth_image, color_image = self.get_frame()
                if not success:
                    print("  Failed to get camera frame, skipping")
                    continue
                
                ret, corners = self.find_chessboard_corners(color_image)
                if not ret:
                    print("  Chessboard not found, adjust board and retry")
                    continue
                
                # Solve PnP: target (board) to camera
                corners2 = corners.reshape(-1, 2).astype(np.float32)
                objp2 = objp.reshape(-1, 3).astype(np.float32)
                ok, rvec, tvec = cv2.solvePnP(objp2, corners2, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                if not ok:
                    print("  solvePnP failed, skipping")
                    continue
                R_tc, _ = cv2.Rodrigues(rvec)  # target->camera rotation
                t_tc = tvec.reshape(3)
                
                # Build gripper (ee) -> base from SCARA current state
                try:
                    if scara_control is not None:
                        j1 = scara_control.CUR_J1
                        j2 = scara_control.CUR_J2
                        j4 = scara_control.CUR_J4
                        theta = math.radians(j1 + j2 + j4)
                        R_be = np.array([[math.cos(theta), -math.sin(theta), 0.0],
                                         [math.sin(theta),  math.cos(theta), 0.0],
                                         [0.0,              0.0,             1.0]], dtype=float)
                        t_be = np.array([scara_control.CUR_X, scara_control.CUR_Y, scara_control.CUR_Z], dtype=float)
                    elif self.arm is not None:
                        # Simulator: use its API if available, otherwise approximate from direction
                        cx, cy, cz = self.arm.get_camera_position()
                        # ee is 140mm behind camera along -direction
                        yaw_deg = self.arm.current_j4
                        yaw_rad = math.radians(yaw_deg)
                        ee_x = cx - 140.0 * math.cos(yaw_rad)
                        ee_y = cy - 140.0 * math.sin(yaw_rad)
                        ee_z = cz
                        R_be = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0.0],
                                         [math.sin(yaw_rad),  math.cos(yaw_rad), 0.0],
                                         [0.0,                0.0,               1.0]], dtype=float)
                        t_be = np.array([ee_x, ee_y, ee_z], dtype=float)
                    else:
                        print("  No arm interface available, skipping pose")
                        continue
                except Exception as e:
                    print(f"  Pose extraction failed: {e}")
                    continue
                
                # OpenCV calibrateHandEye expects gripper->base and target->camera
                R_gripper2base_list.append(R_be)
                t_gripper2base_list.append(t_be)
                R_target2cam_list.append(R_tc)
                t_target2cam_list.append(t_tc)
                captures += 1
                
                if captures >= min_captures:
                    break
            if captures >= min_captures:
                break
        
        if captures < 6:
            print("Not enough valid poses for hand-eye calibration")
            return False
        
        print(f"Collected {captures} valid poses. Solving hand-eye...")
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base_list, t_gripper2base_list,
                R_target2cam_list, t_target2cam_list,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
        except Exception as e:
            print(f"calibrateHandEye failed: {e}")
            return False
        
        # Convert to ee -> cam
        R_ee2cam = R_cam2gripper.T
        t_ee2cam = -R_cam2gripper.T @ t_cam2gripper.reshape(3)
        
        self.handeye_R_ee2cam = R_ee2cam
        self.handeye_t_ee2cam = t_ee2cam
        
        print("Hand-eye calibration completed!")
        print(f"R_ee2cam:\n{R_ee2cam}")
        print(f"t_ee2cam: {t_ee2cam}")
        
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
        
        # Save hand-eye if available
        if self.handeye_R_ee2cam is not None and self.handeye_t_ee2cam is not None:
            calibration_data["handeye"] = {
                "R_ee2cam": self.handeye_R_ee2cam.tolist(),
                "t_ee2cam": self.handeye_t_ee2cam.tolist()
            }
        
        filepath = self.calibration_dir / filename
        with open(filepath, 'w') as f:
            json.dump(calibration_data, f, indent=2)
        
        print(f"Calibration saved to {filepath}")
    
    def load_calibration(self, filename: str = "camera_calibration.json") -> bool:
        """Load calibration parameters from file"""
        filepath = self.calibration_dir / filename
        
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
            
            # Load old-style extrinsics if available
            if "extrinsics" in data:
                extrinsics_data = data["extrinsics"]
                self.extrinsics = CameraExtrinsics(
                    rotation_matrix=np.array(extrinsics_data["rotation_matrix"]),
                    translation_vector=np.array(extrinsics_data["translation_vector"])
                )
            
            # Load hand-eye calibration if available
            if "handeye" in data:
                he = data["handeye"]
                self.handeye_R_ee2cam = np.array(he["R_ee2cam"]) if "R_ee2cam" in he else None
                self.handeye_t_ee2cam = np.array(he["t_ee2cam"]) if "t_ee2cam" in he else None
            
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
        
        # Convert to camera coordinates (undistorted pinhole model)
        camera_x = (pixel_x - self.intrinsics.cx) * depth_mm / self.intrinsics.fx
        camera_y = (pixel_y - self.intrinsics.cy) * depth_mm / self.intrinsics.fy
        camera_z = depth_mm
        
        camera_point = np.array([camera_x, camera_y, camera_z], dtype=float)
        
        # Preferred: use hand-eye T_ee_cam and current SCARA pose to compute T_base_cam
        if self.handeye_R_ee2cam is not None and self.handeye_t_ee2cam is not None:
            try:
                if scara_control is not None:
                    theta = math.radians(scara_control.CUR_J1 + scara_control.CUR_J2 + scara_control.CUR_J4)
                    R_be = np.array([[math.cos(theta), -math.sin(theta), 0.0],
                                     [math.sin(theta),  math.cos(theta), 0.0],
                                     [0.0,              0.0,             1.0]], dtype=float)
                    t_be = np.array([scara_control.CUR_X, scara_control.CUR_Y, scara_control.CUR_Z], dtype=float)
                elif self.arm is not None:
                    # Approximate from simulator state
                    cx, cy, cz = self.arm.get_camera_position()
                    yaw_rad = math.radians(self.arm.current_j4)
                    ee_x = cx - 140.0 * math.cos(yaw_rad)
                    ee_y = cy - 140.0 * math.sin(yaw_rad)
                    ee_z = cz
                    R_be = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0.0],
                                     [math.sin(yaw_rad),  math.cos(yaw_rad), 0.0],
                                     [0.0,                0.0,               1.0]], dtype=float)
                    t_be = np.array([ee_x, ee_y, ee_z], dtype=float)
                else:
                    # Cannot determine pose
                    raise RuntimeError("No arm interface for pose computation")
                # Compose R_base_cam = R_be * R_ec ; t_base_cam = R_be * t_ec + t_be
                R_bc = R_be @ self.handeye_R_ee2cam
                t_bc = R_be @ self.handeye_t_ee2cam + t_be
                arm_point = R_bc @ camera_point + t_bc
                return float(arm_point[0]), float(arm_point[1]), float(arm_point[2])
            except Exception as e:
                print(f"Hand-eye transform failed, falling back: {e}")
                # Fall through to other methods
        
        # Legacy: if a static extrinsics (camera->arm base) is present, apply it
        if self.extrinsics is not None:
            arm_point = self.extrinsics.rotation_matrix @ camera_point + self.extrinsics.translation_vector
            return float(arm_point[0]), float(arm_point[1]), float(arm_point[2])
        
        # Fallback: return camera coordinates if nothing else is available
        return float(camera_x), float(camera_y), float(camera_z)
    
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