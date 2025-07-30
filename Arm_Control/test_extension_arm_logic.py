#!/usr/bin/env python3
"""
Test script for extension arm logic without serial connection.
This script tests the two-sided extension arm where camera and suction cup point in opposite directions.
"""

import math

# Extension arm dimensions (from J4 center)
EXTENSION_SUCTION_LENGTH = 45   # mm from J4 to suction cup center
EXTENSION_CAMERA_LENGTH = 140   # mm from J4 to camera center

# Arm dimensions
LENGTH_J1 = 205
LENGTH_J2 = 205

# Origin positions
ORIGIN_X = 96.979
ORIGIN_Y = 70.459
ORIGIN_Z = 200
ORIGIN_J1 = 109
ORIGIN_J2 = -146
ORIGIN_J3 = 200

def calculate_j4_for_cartesian_direction(j1: float, j2: float, cartesian_angle: float) -> float:
    """
    Calculate J4 angle needed to point extension arm camera in specified cartesian direction.
    The suction cup will automatically point in the opposite direction.
    
    Args:
        j1, j2: Current joint angles (degrees)
        cartesian_angle: Target camera direction in cartesian coordinates (degrees)
                        0° = +X direction, 90° = +Y direction, -90° = -Y direction, ±180° = -X direction
    
    Returns:
        J4 angle relative to J2 (degrees)
    """
    # Current arm orientation (J1 + J2) in standard coordinates where 0° = +X
    current_arm_orientation = j1 + j2
    
    # User's cartesian system: 0° = +X, 90° = +Y, -90° = -Y, ±180° = -X
    # Standard system: 0° = +X, 90° = +Y, 180° = -X, 270° = -Y
    # For this system, the angles are already aligned with standard coordinates
    target_standard_angle = cartesian_angle
    
    # Calculate required J4 angle
    j4_angle = target_standard_angle - current_arm_orientation
    
    # Normalize angle to [-180, 180]
    while j4_angle > 180:
        j4_angle -= 360
    while j4_angle <= -180:
        j4_angle += 360
    
    return j4_angle

def get_camera_position(cur_x: float, cur_y: float, cur_z: float, j1: float, j2: float, j4: float) -> tuple[float, float, float]:
    """
    Get camera position in world coordinates.
    
    Returns:
        (x, y, z) position of camera center
    """
    # Calculate extension arm absolute angle
    extension_absolute_angle = j1 + j2 + j4
    extension_rad = math.radians(extension_absolute_angle)
    
    # Camera position (points in the direction of the extension arm)
    camera_x = cur_x + EXTENSION_CAMERA_LENGTH * math.cos(extension_rad)
    camera_y = cur_y + EXTENSION_CAMERA_LENGTH * math.sin(extension_rad)
    camera_z = cur_z  # Same height as end effector
    
    return (camera_x, camera_y, camera_z)

def get_suction_cup_position(cur_x: float, cur_y: float, cur_z: float, j1: float, j2: float, j4: float) -> tuple[float, float, float]:
    """
    Get suction cup position in world coordinates.
    
    Returns:
        (x, y, z) position of suction cup center
    """
    # Calculate extension arm absolute angle
    extension_absolute_angle = j1 + j2 + j4
    extension_rad = math.radians(extension_absolute_angle)
    
    # Suction cup is on the opposite side of the camera (180° offset)
    suction_angle_rad = extension_rad + math.pi  # 180° offset
    
    # Suction cup position
    suction_x = cur_x + EXTENSION_SUCTION_LENGTH * math.cos(suction_angle_rad)
    suction_y = cur_y + EXTENSION_SUCTION_LENGTH * math.sin(suction_angle_rad)
    suction_z = cur_z  # Same height as end effector
    
    return (suction_x, suction_y, suction_z)

def get_camera_direction(j1: float, j2: float, j4: float) -> float:
    """
    Get current camera direction in cartesian coordinates.
    
    Returns:
        Camera direction angle in degrees (0° = +X, 90° = +Y, -90° = -Y, ±180° = -X)
    """
    # Calculate extension arm absolute angle
    extension_absolute_angle = j1 + j2 + j4
    
    # Normalize to [-180, 180]
    while extension_absolute_angle > 180:
        extension_absolute_angle -= 360
    while extension_absolute_angle <= -180:
        extension_absolute_angle += 360
    
    return extension_absolute_angle

def get_suction_cup_direction(j1: float, j2: float, j4: float) -> float:
    """
    Get current suction cup direction in cartesian coordinates.
    
    Returns:
        Suction cup direction angle in degrees (0° = +X, 90° = +Y, -90° = -Y, ±180° = -X)
    """
    camera_direction = get_camera_direction(j1, j2, j4)
    suction_direction = camera_direction + 180
    
    # Normalize to [-180, 180]
    if suction_direction > 180:
        suction_direction -= 360
    
    return suction_direction

def test_extension_arm_directions():
    """Test that camera and suction cup point in opposite directions."""
    print("=== Testing Extension Arm Directions ===")
    
    # Set initial position
    cur_x, cur_y, cur_z = ORIGIN_X, ORIGIN_Y, ORIGIN_Z
    j1, j2, j3, j4 = ORIGIN_J1, ORIGIN_J2, ORIGIN_J3, 0
    
    # Test different camera directions
    test_angles = [0, 90, -90, 180, -180, 45, -45]
    
    for camera_angle in test_angles:
        print(f"\n--- Testing camera angle: {camera_angle}° ---")
        
        # Calculate required J4 for this camera direction
        required_j4 = calculate_j4_for_cartesian_direction(j1, j2, camera_angle)
        j4 = required_j4
        
        # Get actual directions
        actual_camera_dir = get_camera_direction(j1, j2, j4)
        actual_suction_dir = get_suction_cup_direction(j1, j2, j4)
        
        # Get positions
        camera_pos = get_camera_position(cur_x, cur_y, cur_z, j1, j2, j4)
        suction_pos = get_suction_cup_position(cur_x, cur_y, cur_z, j1, j2, j4)
        
        print(f"Target camera direction: {camera_angle}°")
        print(f"Actual camera direction: {actual_camera_dir:.1f}°")
        print(f"Actual suction direction: {actual_suction_dir:.1f}°")
        print(f"Direction difference: {abs(actual_camera_dir - camera_angle):.1f}°")
        print(f"Camera-Suction difference: {abs(actual_camera_dir - actual_suction_dir):.1f}° (should be 180°)")
        print(f"Camera position: ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f})")
        print(f"Suction position: ({suction_pos[0]:.1f}, {suction_pos[1]:.1f}, {suction_pos[2]:.1f})")
        
        # Verify that camera and suction cup are 180° apart
        angle_diff = abs(actual_camera_dir - actual_suction_dir)
        if abs(angle_diff - 180) > 1:  # Allow 1° tolerance
            print(f"ERROR: Camera and suction cup are not 180° apart! Difference: {angle_diff:.1f}°")
        else:
            print("✓ Camera and suction cup correctly point in opposite directions")

def test_extension_arm_positions():
    """Test that camera and suction cup positions are correctly calculated."""
    print("\n=== Testing Extension Arm Positions ===")
    
    # Set initial position
    cur_x, cur_y, cur_z = ORIGIN_X, ORIGIN_Y, ORIGIN_Z
    j1, j2, j3, j4 = ORIGIN_J1, ORIGIN_J2, ORIGIN_J3, 0
    
    # Test with camera pointing in -Y direction (default)
    camera_angle = -90
    required_j4 = calculate_j4_for_cartesian_direction(j1, j2, camera_angle)
    j4 = required_j4
    
    print(f"Testing with camera pointing {camera_angle}° (should be -Y direction)")
    
    # Get positions
    camera_pos = get_camera_position(cur_x, cur_y, cur_z, j1, j2, j4)
    suction_pos = get_suction_cup_position(cur_x, cur_y, cur_z, j1, j2, j4)
    
    print(f"End effector position: ({cur_x:.1f}, {cur_y:.1f}, {cur_z:.1f})")
    print(f"Camera position: ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f})")
    print(f"Suction position: ({suction_pos[0]:.1f}, {suction_pos[1]:.1f}, {suction_pos[2]:.1f})")
    
    # Calculate expected positions
    extension_rad = math.radians(j1 + j2 + j4)
    expected_camera_x = cur_x + EXTENSION_CAMERA_LENGTH * math.cos(extension_rad)
    expected_camera_y = cur_y + EXTENSION_CAMERA_LENGTH * math.sin(extension_rad)
    
    suction_rad = extension_rad + math.pi  # 180° offset
    expected_suction_x = cur_x + EXTENSION_SUCTION_LENGTH * math.cos(suction_rad)
    expected_suction_y = cur_y + EXTENSION_SUCTION_LENGTH * math.sin(suction_rad)
    
    print(f"Expected camera position: ({expected_camera_x:.1f}, {expected_camera_y:.1f}, {cur_z:.1f})")
    print(f"Expected suction position: ({expected_suction_x:.1f}, {expected_suction_y:.1f}, {cur_z:.1f})")
    
    # Verify positions
    camera_error = math.sqrt((camera_pos[0] - expected_camera_x)**2 + (camera_pos[1] - expected_camera_y)**2)
    suction_error = math.sqrt((suction_pos[0] - expected_suction_x)**2 + (suction_pos[1] - expected_suction_y)**2)
    
    print(f"Camera position error: {camera_error:.3f}mm")
    print(f"Suction position error: {suction_error:.3f}mm")
    
    if camera_error < 0.1 and suction_error < 0.1:
        print("✓ Position calculations are correct")
    else:
        print("✗ Position calculations have errors")

def test_opposite_directions():
    """Test that when camera points in one direction, suction cup points in opposite direction."""
    print("\n=== Testing Opposite Directions ===")
    
    # Test with camera pointing in -Y direction
    j1, j2 = ORIGIN_J1, ORIGIN_J2
    camera_angle = -90  # -Y direction
    j4 = calculate_j4_for_cartesian_direction(j1, j2, camera_angle)
    
    camera_dir = get_camera_direction(j1, j2, j4)
    suction_dir = get_suction_cup_direction(j1, j2, j4)
    
    print(f"Camera direction: {camera_dir:.1f}° (-Y should be -90°)")
    print(f"Suction direction: {suction_dir:.1f}° (+Y should be +90°)")
    
    # Verify they are opposite
    if abs(camera_dir - (-90)) < 1 and abs(suction_dir - 90) < 1:
        print("✓ Camera points -Y, suction cup points +Y (opposite directions)")
    else:
        print("✗ Directions are not opposite as expected")
    
    # Test with camera pointing in +X direction
    camera_angle = 0  # +X direction
    j4 = calculate_j4_for_cartesian_direction(j1, j2, camera_angle)
    
    camera_dir = get_camera_direction(j1, j2, j4)
    suction_dir = get_suction_cup_direction(j1, j2, j4)
    
    print(f"\nCamera direction: {camera_dir:.1f}° (+X should be 0°)")
    print(f"Suction direction: {suction_dir:.1f}° (-X should be ±180°)")
    
    # Verify they are opposite
    if abs(camera_dir) < 1 and abs(abs(suction_dir) - 180) < 1:
        print("✓ Camera points +X, suction cup points -X (opposite directions)")
    else:
        print("✗ Directions are not opposite as expected")

if __name__ == "__main__":
    print("Extension Arm Logic Test Suite")
    print("=" * 50)
    
    test_extension_arm_directions()
    test_extension_arm_positions()
    test_opposite_directions()
    
    print("\n" + "=" * 50)
    print("Test suite completed!") 