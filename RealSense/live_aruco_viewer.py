#!/usr/bin/env python3
"""
Live ArUco Detection Viewer for SCARA AGV
Shows real-time ArUco marker detection from camera feed
"""

import cv2
import numpy as np
import time
import sys
from pathlib import Path
from typing import Optional, Tuple

# Add project directories to path
current_dir = Path(__file__).parent
project_root = current_dir.parent
sys.path.append(str(project_root))
sys.path.append(str(project_root / "Detection_Models"))
sys.path.append(str(project_root / "RealSense"))

try:
    from aruco_plate_detector import ArUcoPlateDetector
    print("✓ ArUco plate detector imported successfully")
except ImportError as e:
    print(f"✗ Failed to import ArUco plate detector: {e}")
    sys.exit(1)

try:
    from RealSense.realsense_depth import DepthCamera
    print("✓ RealSense camera imported successfully")
except ImportError as e:
    print(f"⚠ RealSense camera not available: {e}")
    DepthCamera = None

try:
    import cv2.aruco
    print("✓ OpenCV ArUco module imported successfully")
except ImportError as e:
    print(f"✗ Failed to import OpenCV ArUco: {e}")
    sys.exit(1)


class LiveArUcoViewer:
    """
    Live video viewer for ArUco marker detection
    """
    
    def __init__(self, camera_source: int = 0, use_realsense: bool = True):
        """
        Initialize the live ArUco viewer
        
        Args:
            camera_source: Camera device index (0 for default webcam)
            use_realsense: Whether to use RealSense camera if available
        """
        self.camera_source = camera_source
        self.use_realsense = use_realsense
        self.cap = None
        self.depth_camera = None
        self.aruco_detector = None
        
        # Display settings
        self.show_fps = True
        self.show_markers = True
        self.show_plates = True
        self.show_depth = False
        self.show_controls = True
        
        # Detection settings
        self.detection_enabled = True
        self.min_confidence = 0.7
        
        # Initialize components
        self._initialize_camera()
        self._initialize_aruco_detector()
        
    def _initialize_camera(self):
        """Initialize camera (RealSense or webcam)"""
        if self.use_realsense and DepthCamera is not None:
            try:
                print("Initializing RealSense camera...")
                self.depth_camera = DepthCamera()
                print("✓ RealSense camera initialized")
            except Exception as e:
                print(f"⚠ RealSense initialization failed: {e}")
                print("Falling back to webcam...")
                self.use_realsense = False
        
        if not self.use_realsense:
            print(f"Initializing webcam (device {self.camera_source})...")
            self.cap = cv2.VideoCapture(self.camera_source)
            if not self.cap.isOpened():
                print(f"✗ Failed to open webcam {self.camera_source}")
                sys.exit(1)
            print("✓ Webcam initialized")
    
    def _initialize_aruco_detector(self):
        """Initialize ArUco detector"""
        try:
            self.aruco_detector = ArUcoPlateDetector(
                aruco_dict_type=cv2.aruco.DICT_4X4_250,
                marker_size_mm=30.0,
                expected_markers=4
            )
            print("✓ ArUco detector initialized")
        except Exception as e:
            print(f"✗ Failed to initialize ArUco detector: {e}")
            sys.exit(1)
    
    def _get_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get frame from camera (color and depth if available)"""
        if self.use_realsense and self.depth_camera is not None:
            try:
                success, depth_frame, color_frame = self.depth_camera.get_frame()
                if success and color_frame is not None and depth_frame is not None:
                    return color_frame, depth_frame
            except Exception as e:
                print(f"⚠ RealSense frame error: {e}")
                return None, None
        elif self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                return frame, None
        
        return None, None
    
    def _detect_aruco_markers(self, color_frame: np.ndarray, depth_frame: Optional[np.ndarray] = None):
        """Detect ArUco markers in the frame"""
        if not self.detection_enabled:
            return []
        
        try:
            # Convert to grayscale for ArUco detection
            gray_frame = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.aruco_detector.detectMarkers(gray_frame)
            
            detected_markers = []
            if ids is not None:
                for i, marker_id in enumerate(ids):
                    marker_corners = corners[i][0]
                    
                    # Calculate center pixel
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    
                    # Get depth if available
                    depth_mm = 0.0
                    if depth_frame is not None:
                        depth_mm = self._get_depth_at_point(depth_frame, center_x, center_y)
                    
                    detected_markers.append({
                        'id': int(marker_id),
                        'corners': marker_corners,
                        'center': (center_x, center_y),
                        'depth': depth_mm
                    })
            
            return detected_markers
            
        except Exception as e:
            print(f"⚠ ArUco detection error: {e}")
            return []
    
    def _get_depth_at_point(self, depth_frame: np.ndarray, x: int, y: int, window_size: int = 5) -> float:
        """Get depth value at a point"""
        h, w = depth_frame.shape
        
        # Clamp coordinates
        x = max(0, min(x, w - 1))
        y = max(0, min(y, h - 1))
        
        # Extract window around point
        half_window = window_size // 2
        x1 = max(0, x - half_window)
        x2 = min(w, x + half_window + 1)
        y1 = max(0, y - half_window)
        y2 = min(h, y + half_window + 1)
        
        depth_window = depth_frame[y1:y2, x1:x2]
        valid_depths = depth_window[depth_window > 0]
        
        if len(valid_depths) > 0:
            return float(np.median(valid_depths))
        else:
            return 0.0
    
    def _draw_markers(self, frame: np.ndarray, markers: list):
        """Draw detected ArUco markers on the frame"""
        for marker in markers:
            # Draw marker corners
            corners = marker['corners'].astype(int)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
            
            # Draw marker ID
            center_x, center_y = marker['center']
            cv2.putText(frame, f"ID:{marker['id']}", 
                       (center_x + 10, center_y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(frame, (center_x, center_y), 4, (255, 0, 255), -1)
            
            # Draw depth if available
            if marker['depth'] > 0:
                depth_text = f"{marker['depth']:.0f}mm"
                cv2.putText(frame, depth_text, 
                           (center_x + 10, center_y + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    
    def _draw_controls(self, frame: np.ndarray):
        """Draw control information on the frame"""
        if not self.show_controls:
            return
        
        # Control instructions
        controls = [
            "Controls:",
            "Q - Quit",
            "D - Toggle detection",
            "M - Toggle markers",
            "P - Toggle plates", 
            "F - Toggle FPS",
            "C - Toggle controls"
        ]
        
        y_offset = 30
        for control in controls:
            cv2.putText(frame, control, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 20
    
    def _draw_fps(self, frame: np.ndarray, fps: float):
        """Draw FPS counter on the frame"""
        if not self.show_fps:
            return
        
        fps_text = f"FPS: {fps:.1f}"
        cv2.putText(frame, fps_text, (frame.shape[1] - 120, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    
    def _draw_stats(self, frame: np.ndarray, markers: list):
        """Draw detection statistics on the frame"""
        stats_text = f"Markers: {len(markers)}"
        cv2.putText(frame, stats_text, (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    def run(self):
        """Main loop for live ArUco detection"""
        print("\n" + "="*60)
        print("Live ArUco Detection Viewer")
        print("="*60)
        print("Press 'Q' to quit, 'H' for help")
        print("="*60)
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Get frame from camera
                color_frame, depth_frame = self._get_frame()
                if color_frame is None:
                    print("⚠ Failed to get frame from camera")
                    time.sleep(0.1)
                    continue
                
                # Detect ArUco markers
                markers = self._detect_aruco_markers(color_frame, depth_frame)
                
                # Create display frame
                display_frame = color_frame.copy()
                
                # Draw detected markers
                if self.show_markers:
                    self._draw_markers(display_frame, markers)
                
                # Calculate and draw FPS
                frame_count += 1
                if frame_count % 30 == 0:  # Update FPS every 30 frames
                    current_time = time.time()
                    fps = frame_count / (current_time - start_time)
                    frame_count = 0
                    start_time = current_time
                else:
                    fps = 0
                
                if self.show_fps and fps > 0:
                    self._draw_fps(display_frame, fps)
                
                # Draw statistics
                self._draw_stats(display_frame, markers)
                
                # Draw controls
                self._draw_controls(display_frame)
                
                # Show frame
                cv2.imshow('Live ArUco Detection', display_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == ord('Q'):
                    break
                elif key == ord('d') or key == ord('D'):
                    self.detection_enabled = not self.detection_enabled
                    print(f"Detection {'enabled' if self.detection_enabled else 'disabled'}")
                elif key == ord('m') or key == ord('M'):
                    self.show_markers = not self.show_markers
                    print(f"Markers {'shown' if self.show_markers else 'hidden'}")
                elif key == ord('p') or key == ord('P'):
                    self.show_plates = not self.show_plates
                    print(f"Plates {'shown' if self.show_plates else 'hidden'}")
                elif key == ord('f') or key == ord('F'):
                    self.show_fps = not self.show_fps
                    print(f"FPS {'shown' if self.show_fps else 'hidden'}")
                elif key == ord('c') or key == ord('C'):
                    self.show_controls = not self.show_controls
                    print(f"Controls {'shown' if self.show_controls else 'hidden'}")
                elif key == ord('h') or key == ord('H'):
                    self._print_help()
                
        except KeyboardInterrupt:
            print("\n⚠ Interrupted by user")
        finally:
            self._cleanup()
    
    def _print_help(self):
        """Print help information"""
        help_text = """
        Live ArUco Detection Viewer - Help
        
        Controls:
        Q - Quit the application
        D - Toggle ArUco detection on/off
        M - Toggle marker visualization on/off
        P - Toggle plate visualization on/off
        F - Toggle FPS counter on/off
        C - Toggle control instructions on/off
        H - Show this help message
        
        Camera:
        - Uses RealSense camera if available, otherwise webcam
        - Press any key to see detected markers in real-time
        
        Detection:
        - ArUco markers are detected using OpenCV 4.7+ compatible API
        - Dictionary: DICT_4X4_250
        - Expected markers per plate: 4
        """
        print(help_text)
    
    def _cleanup(self):
        """Clean up resources"""
        print("\nCleaning up...")
        
        if self.cap is not None:
            self.cap.release()
        
        if self.depth_camera is not None:
            try:
                self.depth_camera.release()
            except:
                pass
        
        cv2.destroyAllWindows()
        print("✓ Cleanup completed")


def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Live ArUco Detection Viewer")
    parser.add_argument("--camera", "-c", type=int, default=0, 
                       help="Camera device index (default: 0)")
    parser.add_argument("--no-realsense", action="store_true",
                       help="Disable RealSense camera and use webcam only")
    
    args = parser.parse_args()
    
    try:
        viewer = LiveArUcoViewer(
            camera_source=args.camera,
            use_realsense=not args.no_realsense
        )
        viewer.run()
    except Exception as e:
        print(f"✗ Error running viewer: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
