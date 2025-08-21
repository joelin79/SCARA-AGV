#!/usr/bin/env python3
"""
ArUco Plate Detection System for SCARA AGV
Detects ArUco markers at plate edges to define plate area and height
"""

import cv2
import numpy as np
import math
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
import json
from pathlib import Path


@dataclass
class ArUcoMarker:
    """Represents a detected ArUco marker"""
    marker_id: int
    corners: np.ndarray  # 4 corners in pixel coordinates
    center_pixel: Tuple[int, int]  # Center pixel coordinates
    depth_mm: float  # Depth at marker center
    arm_coords: Tuple[float, float, float]  # 3D coordinates in arm frame
    confidence: float = 1.0  # Detection confidence


@dataclass
class PlateDefinition:
    """Represents a plate defined by edge ArUco markers"""
    plate_id: str
    edge_markers: List[ArUcoMarker]  # Must have 4 markers for rectangular plate
    corners_arm_coords: List[Tuple[float, float, float]]  # 4 corners in arm coordinates
    center_arm_coords: Tuple[float, float, float]  # Plate center in arm coordinates
    dimensions: Tuple[float, float]  # Width and length in mm
    height_mm: float  # Plate height
    area_mm2: float  # Plate surface area
    orientation_deg: float  # Plate orientation relative to arm X-axis


class ArUcoPlateDetector:
    """
    Detects ArUco markers and defines plate areas based on edge markers
    """
    
    def __init__(self, 
                 aruco_dict_type: int = cv2.aruco.DICT_4X4_250,
                 marker_size_mm: float = 30.0,
                 expected_markers: int = 4):
        """
        Initialize ArUco plate detector
        
        Args:
            aruco_dict_type: OpenCV ArUco dictionary type
            marker_size_mm: Physical size of ArUco markers in mm
            expected_markers: Number of markers expected for a complete plate
        """
        self.aruco_dict_type = aruco_dict_type
        self.marker_size_mm = marker_size_mm
        self.expected_markers = expected_markers
        
        # Initialize ArUco detector (OpenCV 4.7+ compatible)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Detection results
        self.detected_markers: List[ArUcoMarker] = []
        self.detected_plates: List[PlateDefinition] = []
        
        # Plate detection settings
        self.min_marker_confidence = 0.7
        self.max_marker_distance_mm = 500.0  # Max distance between markers for same plate
        self.min_plate_area_mm2 = 10000.0  # Minimum plate area to consider valid
        
    def detect_markers(self, 
                      color_image: np.ndarray, 
                      depth_image: np.ndarray,
                      camera_position: Tuple[float, float, float],
                      pixel_to_arm_converter) -> List[ArUcoMarker]:
        """
        Detect ArUco markers in the image
        
        Args:
            color_image: RGB image from camera
            depth_image: Depth image from camera
            camera_position: Current camera position (x, y, z)
            pixel_to_arm_converter: Function to convert pixel+depth to arm coordinates
            
        Returns:
            List of detected ArUco markers
        """
        # Convert to grayscale for ArUco detection
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers (OpenCV 4.7+ compatible)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray_image)
        
        detected_markers = []
        
        if ids is not None:
            for i, marker_id in enumerate(ids):
                marker_corners = corners[i][0]
                
                # Calculate center pixel
                center_x = int(np.mean(marker_corners[:, 0]))
                center_y = int(np.mean(marker_corners[:, 1]))
                
                # Get depth at marker center
                depth_mm = self._get_depth_at_point(depth_image, center_x, center_y)
                
                if depth_mm > 0:  # Valid depth
                    # Convert to arm coordinates
                    arm_coords = pixel_to_arm_converter(center_x, center_y, depth_mm, camera_position)
                    
                    marker = ArUcoMarker(
                        marker_id=int(marker_id),
                        corners=marker_corners,
                        center_pixel=(center_x, center_y),
                        depth_mm=depth_mm,
                        arm_coords=arm_coords
                    )
                    
                    detected_markers.append(marker)
        
        self.detected_markers = detected_markers
        return detected_markers
    
    def define_plates_from_markers(self) -> List[PlateDefinition]:
        """
        Define plates based on detected edge markers
        
        Returns:
            List of defined plates
        """
        if len(self.detected_markers) < self.expected_markers:
            return []
        
        # Group markers that could form a plate
        potential_plates = self._group_markers_for_plates()
        
        defined_plates = []
        
        for marker_group in potential_plates:
            if len(marker_group) == self.expected_markers:
                plate = self._create_plate_from_markers(marker_group)
                if plate is not None:
                    defined_plates.append(plate)
        
        self.detected_plates = defined_plates
        return defined_plates
    
    def _group_markers_for_plates(self) -> List[List[ArUcoMarker]]:
        """
        Group markers sequentially: 0-3, 4-7, 8-11, etc.
        Only complete groups of 4 markers form plates.
        
        Returns:
            List of marker groups, each potentially forming a plate
        """
        if len(self.detected_markers) < self.expected_markers:
            return []
        
        groups = []
        
        # Sort markers by ID first for consistent grouping
        sorted_markers = sorted(self.detected_markers, key=lambda m: m.marker_id)
        
        # Group every 4 markers together
        for i in range(0, len(sorted_markers), 4):
            group = sorted_markers[i:i+4]
            if len(group) == 4:  # Only add complete groups
                groups.append(group)
        
        return groups
    
    def _create_plate_from_markers(self, markers: List[ArUcoMarker]) -> Optional[PlateDefinition]:
        """
        Create a plate definition from a group of edge markers
        
        Args:
            markers: List of edge markers (should be 4 for rectangular plate)
            
        Returns:
            Plate definition or None if invalid
        """
        if len(markers) != 4:
            return None
        
        try:
            # Sort markers to get corners in order (top-left, top-right, bottom-right, bottom-left)
            sorted_markers = self._sort_markers_for_plate(markers)
            
            # Extract corner coordinates
            corners_arm_coords = [marker.arm_coords for marker in sorted_markers]
            
            # Calculate plate center
            center_x = np.mean([corner[0] for corner in corners_arm_coords])
            center_y = np.mean([corner[1] for corner in corners_arm_coords])
            center_z = np.mean([corner[2] for corner in corners_arm_coords])
            center_arm_coords = (float(center_x), float(center_y), float(center_z))
            
            # Calculate plate dimensions
            width_mm = self._calculate_side_length(corners_arm_coords[0], corners_arm_coords[1])
            length_mm = self._calculate_side_length(corners_arm_coords[1], corners_arm_coords[2])
            dimensions = (width_mm, length_mm)
            
            # Calculate area
            area_mm2 = width_mm * length_mm
            
            # Calculate height (average depth at markers)
            height_mm = np.mean([marker.arm_coords[2] for marker in markers])
            
            # Calculate orientation relative to arm X-axis
            orientation_deg = self._calculate_plate_orientation(corners_arm_coords)
            
            # Create plate ID from marker IDs
            plate_id = f"plate_{'_'.join(str(m.marker_id) for m in sorted_markers)}"
            
            plate = PlateDefinition(
                plate_id=plate_id,
                edge_markers=sorted_markers,
                corners_arm_coords=corners_arm_coords,
                center_arm_coords=center_arm_coords,
                dimensions=dimensions,
                height_mm=float(height_mm),
                area_mm2=float(area_mm2),
                orientation_deg=float(orientation_deg)
            )
            
            return plate
            
        except Exception as e:
            print(f"Error creating plate from markers: {e}")
            return None
    
    def _sort_markers_for_plate(self, markers: List[ArUcoMarker]) -> List[ArUcoMarker]:
        """
        Sort markers to get corners in order for rectangular plate
        
        Args:
            markers: List of 4 edge markers
            
        Returns:
            Sorted markers: [top-left, top-right, bottom-right, bottom-left]
        """
        # Find center of all markers
        center_x = np.mean([m.arm_coords[0] for m in markers])
        center_y = np.mean([m.arm_coords[1] for m in markers])
        
        # Sort by angle from center (counter-clockwise starting from top)
        def angle_from_center(marker):
            dx = marker.arm_coords[0] - center_x
            dy = marker.arm_coords[1] - center_y
            angle = math.atan2(dy, dx)
            # Convert to 0-360 degrees, starting from top (90 degrees)
            angle_deg = math.degrees(angle)
            angle_deg = (90 - angle_deg) % 360
            return angle_deg
        
        sorted_markers = sorted(markers, key=angle_from_center)
        return sorted_markers
    
    def _calculate_side_length(self, corner1: Tuple[float, float, float], 
                              corner2: Tuple[float, float, float]) -> float:
        """Calculate length of side between two corners"""
        dx = corner2[0] - corner1[0]
        dy = corner2[1] - corner1[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def _calculate_plate_orientation(self, corners: List[Tuple[float, float, float]]) -> float:
        """Calculate plate orientation relative to arm X-axis"""
        # Use top edge (first two corners) to determine orientation
        dx = corners[1][0] - corners[0][0]
        dy = corners[1][1] - corners[0][1]
        angle_rad = math.atan2(dy, dx)
        return math.degrees(angle_rad)
    
    def _calculate_3d_distance(self, point1: Tuple[float, float, float], 
                              point2: Tuple[float, float, float]) -> float:
        """Calculate 3D distance between two points"""
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        dz = point2[2] - point1[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
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
    
    def create_annotated_image(self, color_image: np.ndarray, 
                              show_markers: bool = True,
                              show_plates: bool = True) -> np.ndarray:
        """
        Create annotated image showing detected ArUco markers and plates
        
        Args:
            color_image: Original color image
            show_markers: Whether to show individual markers
            show_plates: Whether to show plate boundaries
            
        Returns:
            Annotated image
        """
        annotated_image = color_image.copy()
        
        # Draw detected markers
        if show_markers:
            for marker in self.detected_markers:
                # Draw marker corners
                corners = marker.corners.astype(int)
                cv2.polylines(annotated_image, [corners], True, (0, 255, 0), 2)
                
                # Draw marker ID
                center_x, center_y = marker.center_pixel
                cv2.putText(annotated_image, f"ID:{marker.marker_id}", 
                           (center_x + 10, center_y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(annotated_image, (center_x, center_y), 4, (255, 0, 255), -1)
        
        # Draw plate boundaries
        if show_plates:
            for plate in self.detected_plates:
                # Draw plate corners
                corners_pixels = []
                for corner_arm in plate.corners_arm_coords:
                    # Note: This would need pixel coordinates - simplified for now
                    # In practice, you'd need to project 3D coordinates back to pixels
                    pass
                
                # Draw plate info
                center_x, center_y = plate.edge_markers[0].center_pixel  # Use first marker for text position
                info_text = f"Plate: {plate.dimensions[0]:.0f}x{plate.dimensions[1]:.0f}mm"
                cv2.putText(annotated_image, info_text, 
                           (center_x + 20, center_y + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        return annotated_image
    
    def save_results(self, filename: str = "aruco_plate_results.json") -> bool:
        """
        Save detection results to file
        
        Args:
            filename: Output filename
            
        Returns:
            True if save successful
        """
        try:
            results = {
                "detection_metadata": {
                    "total_markers": len(self.detected_markers),
                    "total_plates": len(self.detected_plates),
                    "aruco_dict_type": self.aruco_dict_type,
                    "marker_size_mm": self.marker_size_mm
                },
                "detected_markers": [],
                "detected_plates": []
            }
            
            # Save marker data
            for marker in self.detected_markers:
                marker_data = {
                    "marker_id": marker.marker_id,
                    "center_pixel": marker.center_pixel,
                    "depth_mm": marker.depth_mm,
                    "arm_coordinates": {
                        "x": marker.arm_coords[0],
                        "y": marker.arm_coords[1],
                        "z": marker.arm_coords[2]
                    }
                }
                results["detected_markers"].append(marker_data)
            
            # Save plate data
            for plate in self.detected_plates:
                plate_data = {
                    "plate_id": plate.plate_id,
                    "center_arm_coordinates": {
                        "x": plate.center_arm_coords[0],
                        "y": plate.center_arm_coords[1],
                        "z": plate.center_arm_coords[2]
                    },
                    "dimensions": {
                        "width_mm": plate.dimensions[0],
                        "length_mm": plate.dimensions[1]
                    },
                    "height_mm": plate.height_mm,
                    "area_mm2": plate.area_mm2,
                    "orientation_deg": plate.orientation_deg,
                    "edge_marker_ids": [m.marker_id for m in plate.edge_markers]
                }
                results["detected_plates"].append(plate_data)
            
            # Save to file
            output_file = Path("detection_output") / filename
            output_file.parent.mkdir(exist_ok=True)
            
            with open(output_file, 'w') as f:
                json.dump(results, f, indent=2)
            
            return True
            
        except Exception as e:
            print(f"Error saving ArUco results: {e}")
            return False
    
    def get_plate_workspace_info(self) -> Dict:
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


def test_aruco_detection():
    """Test function for ArUco detection"""
    print("Testing ArUco Plate Detection System...")
    
    # Create detector
    detector = ArUcoPlateDetector()
    
    # Test with sample data
    print(f"ArUco dictionary type: {detector.aruco_dict_type}")
    print(f"Expected markers per plate: {detector.expected_markers}")
    print(f"Marker size: {detector.marker_size_mm}mm")
    
    print("✓ ArUco detector initialized successfully")


if __name__ == "__main__":
    test_aruco_detection() 