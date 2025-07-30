#!/usr/bin/env python3
"""
Test script for extension arm functionality.
This script tests the two-sided extension arm where camera and suction cup point in opposite directions.
"""

import math
from SCARA import *

def test_extension_arm_directions():
    """Test that camera and suction cup point in opposite directions."""
    print("=== Testing Extension Arm Directions ===")
    
    # Set initial position
    CUR_X, CUR_Y, CUR_Z = ORIGIN_X, ORIGIN_Y, ORIGIN_Z
    CUR_J1, CUR_J2, CUR_J3, CUR_J4 = ORIGIN_J1, ORIGIN_J2, ORIGIN_J3, 0
    
    # Test different camera directions
    test_angles = [0, 90, -90, 180, -180, 45, -45]
    
    for camera_angle in test_angles:
        print(f"\n--- Testing camera angle: {camera_angle}° ---")
        
        # Calculate required J4 for this camera direction
        required_j4 = calculate_j4_for_cartesian_direction(CUR_J1, CUR_J2, camera_angle)
        CUR_J4 = required_j4
        
        # Get actual directions
        actual_camera_dir = get_camera_direction()
        actual_suction_dir = get_suction_cup_direction()
        
        # Get positions
        camera_pos = get_camera_position()
        suction_pos = get_suction_cup_position()
        
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
    CUR_X, CUR_Y, CUR_Z = ORIGIN_X, ORIGIN_Y, ORIGIN_Z
    CUR_J1, CUR_J2, CUR_J3, CUR_J4 = ORIGIN_J1, ORIGIN_J2, ORIGIN_J3, 0
    
    # Test with camera pointing in -Y direction (default)
    camera_angle = -90
    required_j4 = calculate_j4_for_cartesian_direction(CUR_J1, CUR_J2, camera_angle)
    CUR_J4 = required_j4
    
    print(f"Testing with camera pointing {camera_angle}° (should be -Y direction)")
    
    # Get positions
    camera_pos = get_camera_position()
    suction_pos = get_suction_cup_position()
    
    print(f"End effector position: ({CUR_X:.1f}, {CUR_Y:.1f}, {CUR_Z:.1f})")
    print(f"Camera position: ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f})")
    print(f"Suction position: ({suction_pos[0]:.1f}, {suction_pos[1]:.1f}, {suction_pos[2]:.1f})")
    
    # Calculate expected positions
    extension_rad = math.radians(CUR_J1 + CUR_J2 + CUR_J4)
    expected_camera_x = CUR_X + EXTENSION_CAMERA_LENGTH * math.cos(extension_rad)
    expected_camera_y = CUR_Y + EXTENSION_CAMERA_LENGTH * math.sin(extension_rad)
    
    suction_rad = extension_rad + math.pi  # 180° offset
    expected_suction_x = CUR_X + EXTENSION_SUCTION_LENGTH * math.cos(suction_rad)
    expected_suction_y = CUR_Y + EXTENSION_SUCTION_LENGTH * math.sin(suction_rad)
    
    print(f"Expected camera position: ({expected_camera_x:.1f}, {expected_camera_y:.1f}, {CUR_Z:.1f})")
    print(f"Expected suction position: ({expected_suction_x:.1f}, {expected_suction_y:.1f}, {CUR_Z:.1f})")
    
    # Verify positions
    camera_error = math.sqrt((camera_pos[0] - expected_camera_x)**2 + (camera_pos[1] - expected_camera_y)**2)
    suction_error = math.sqrt((suction_pos[0] - expected_suction_x)**2 + (suction_pos[1] - expected_suction_y)**2)
    
    print(f"Camera position error: {camera_error:.3f}mm")
    print(f"Suction position error: {suction_error:.3f}mm")
    
    if camera_error < 0.1 and suction_error < 0.1:
        print("✓ Position calculations are correct")
    else:
        print("✗ Position calculations have errors")

def test_collision_detection():
    """Test collision detection with the two-sided extension arm."""
    print("\n=== Testing Collision Detection ===")
    
    # Test safe position
    j1, j2, j4 = 0, 0, 0  # Straight out
    try:
        check_extension_arm_collision(j1, j2, j4)
        print("✓ Safe position passed collision check")
    except ValueError as e:
        print(f"✗ Safe position failed collision check: {e}")
    
    # Test potentially unsafe position (pointing back toward base)
    j1, j2, j4 = 0, 0, 180  # Pointing back
    try:
        check_extension_arm_collision(j1, j2, j4)
        print("✓ Backward position passed collision check")
    except ValueError as e:
        print(f"✗ Backward position failed collision check: {e}")

if __name__ == "__main__":
    print("Extension Arm Test Suite")
    print("=" * 50)
    
    test_extension_arm_directions()
    test_extension_arm_positions()
    test_collision_detection()
    
    print("\n" + "=" * 50)
    print("Test suite completed!") 