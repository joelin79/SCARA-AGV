#!/usr/bin/env python3
"""
SCARA Arm Simulator for Object Detection Testing
Provides a mock SCARA arm interface for testing without hardware
"""

import time
import math
from typing import Tuple, Optional

class SCARASimulator:
    """
    Simulated SCARA arm for testing object detection system
    """
    
    def __init__(self):
        """Initialize the simulated SCARA arm"""
        # Current position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 300.0
        self.current_j4 = -90.0
        
        # Arm dimensions
        self.LENGTH_J1 = 205  # mm
        self.LENGTH_J2 = 205  # mm
        self.LENGTH_J3 = 200  # mm
        
        # Extension arm dimensions
        self.EXTENSION_CAMERA_LENGTH = 140  # mm from J4 to camera center
        
        # Movement parameters
        self.default_feedrate = 3000
        self.movement_delay = 1.0  # seconds
        
        print("SCARA Arm Simulator initialized")
        print(f"Current position: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})")
    
    def quick_camera(self, x: float, y: float, z: float, 
                    maintain_extension_direction: bool = True, 
                    extension_angle: float = -90.0) -> bool:
        """
        Move arm to position for camera capture
        
        Args:
            x, y, z: Target coordinates (mm)
            maintain_extension_direction: Whether to maintain camera direction
            extension_angle: Camera direction angle (degrees)
            
        Returns:
            True if movement successful
        """
        print(f"Moving camera to: ({x:.1f}, {y:.1f}, {z:.1f}) at angle {extension_angle:.1f}°")
        
        # Simulate movement time
        time.sleep(self.movement_delay)
        
        # Update position
        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_j4 = extension_angle
        
        print(f"Camera positioned at: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})")
        return True
    
    def quick_suction(self, x: float, y: float, z: float,
                     maintain_extension_direction: bool = True,
                     extension_angle: float = -90.0) -> bool:
        """
        Move arm to position for suction cup operation
        
        Args:
            x, y, z: Target coordinates (mm)
            maintain_extension_direction: Whether to maintain suction direction
            extension_angle: Suction cup direction angle (degrees)
            
        Returns:
            True if movement successful
        """
        print(f"Moving suction cup to: ({x:.1f}, {y:.1f}, {z:.1f}) at angle {extension_angle:.1f}°")
        
        # Simulate movement time
        time.sleep(self.movement_delay)
        
        # Update position
        self.current_x = x
        self.current_y = y
        self.current_z = z
        self.current_j4 = extension_angle
        
        print(f"Suction cup positioned at: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})")
        return True
    
    def get_camera_position(self) -> Tuple[float, float, float]:
        """Get current camera position in arm coordinates"""
        # Camera is mounted on extension arm
        camera_x = self.current_x
        camera_y = self.current_y
        camera_z = self.current_z
        
        return camera_x, camera_y, camera_z
    
    def get_suction_cup_position(self) -> Tuple[float, float, float]:
        """Get current suction cup position in arm coordinates"""
        # Suction cup is on opposite side of extension arm
        suction_x = self.current_x
        suction_y = self.current_y
        suction_z = self.current_z
        
        return suction_x, suction_y, suction_z
    
    def home(self) -> bool:
        """Move arm to home position"""
        print("Moving to home position...")
        time.sleep(self.movement_delay)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 300.0
        self.current_j4 = -90.0
        
        print("Arm at home position")
        return True
    
    def get_current_position(self) -> Tuple[float, float, float, float]:
        """Get current arm position and J4 angle"""
        return self.current_x, self.current_y, self.current_z, self.current_j4


# For compatibility with existing code
class SCARA(SCARASimulator):
    """Alias for SCARASimulator to maintain compatibility"""
    pass 