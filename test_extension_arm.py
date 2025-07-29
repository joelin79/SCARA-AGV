#!/usr/bin/env python3
"""
Test script for SCARA extension arm functionality
This script demonstrates the new extension arm features:
- Automatic extension arm orientation control
- Coordinate transformations
- Collision checking
"""

import time
import math
from SCARA import (
    quick, linear, coordinate_mode, calibrate,
    set_extension_direction, calculate_j4_for_cartesian_direction,
    get_suction_cup_position, get_camera_position,
    CUR_X, CUR_Y, CUR_Z, CUR_J1, CUR_J2, CUR_J4,
    EXTENSION_SUCTION_LENGTH, EXTENSION_CAMERA_LENGTH,
    ORIGIN_X, ORIGIN_Y, ORIGIN_Z
)

def test_extension_arm_orientation():
    """Test extension arm orientation control"""
    print("\n" + "="*50)
    print("TESTING EXTENSION ARM ORIENTATION CONTROL")
    print("="*50)
    
    # Test different extension arm directions
    test_angles = [0, 45, 90, 135, 180, -45, -90, -135]
    
    print(f"Current arm position: J1={CUR_J1:.1f}°, J2={CUR_J2:.1f}°")
    
    for angle in test_angles:
        try:
            required_j4 = calculate_j4_for_cartesian_direction(CUR_J1, CUR_J2, angle)
            print(f"Cartesian {angle:4.0f}° → J4 = {required_j4:6.1f}°")
        except Exception as e:
            print(f"Cartesian {angle:4.0f}° → ERROR: {e}")

def test_extension_arm_movement():
    """Test extension arm movement with automatic orientation"""
    print("\n" + "="*50)
    print("TESTING EXTENSION ARM MOVEMENT")
    print("="*50)
    
    # Test positions
    test_positions = [
        (200, 0, ORIGIN_Z),
        (300, 100, ORIGIN_Z),
        (250, -150, ORIGIN_Z),
        (150, 200, ORIGIN_Z)
    ]
    
    print("Moving to test positions with extension arm pointing +Y...")
    
    for i, (x, y, z) in enumerate(test_positions):
        try:
            print(f"\nPosition {i+1}: ({x}, {y}, {z})")
            
            # Move with automatic extension control
            quick(x, y, z, maintain_extension_direction=True, extension_angle=90.0)
            time.sleep(1.0)
            
            # Get positions
            suction_pos = get_suction_cup_position()
            camera_pos = get_camera_position()
            
            print(f"  End effector: ({CUR_X:.1f}, {CUR_Y:.1f}, {CUR_Z:.1f})")
            print(f"  Suction cup:  ({suction_pos[0]:.1f}, {suction_pos[1]:.1f}, {suction_pos[2]:.1f})")
            print(f"  Camera:       ({camera_pos[0]:.1f}, {camera_pos[1]:.1f}, {camera_pos[2]:.1f})")
            print(f"  J4 angle:     {CUR_J4:.1f}°")
            
        except Exception as e:
            print(f"  ERROR: {e}")

def test_different_extension_directions():
    """Test moving with different extension arm directions"""
    print("\n" + "="*50)
    print("TESTING DIFFERENT EXTENSION DIRECTIONS")
    print("="*50)
    
    # Test position
    test_x, test_y, test_z = 250, 0, ORIGIN_Z
    
    # Test different directions
    directions = [
        (90, "+Y direction (default)"),
        (0, "+X direction"), 
        (180, "-X direction"),
        (-90, "-Y direction")
    ]
    
    for angle, description in directions:
        try:
            print(f"\nTesting {description} ({angle}°)")
            quick(test_x, test_y, test_z, 
                  maintain_extension_direction=True, 
                  extension_angle=angle)
            time.sleep(1.0)
            
            suction_pos = get_suction_cup_position()
            print(f"  Suction cup position: ({suction_pos[0]:.1f}, {suction_pos[1]:.1f}, {suction_pos[2]:.1f})")
            print(f"  J4 angle: {CUR_J4:.1f}°")
            
        except Exception as e:
            print(f"  ERROR: {e}")

def demonstrate_pick_and_place_simulation():
    """Simulate a pick and place operation"""
    print("\n" + "="*50)
    print("PICK AND PLACE SIMULATION")
    print("="*50)
    
    # Simulated object positions (where suction cup should go)
    objects = [
        (200, 50, 0, "Object 1"),
        (280, -80, 0, "Object 2"),
        (150, 120, 0, "Object 3")
    ]
    
    # Drop-off position
    dropoff = (350, 0, 0, "Drop-off zone")
    
    print("Simulating pick and place operations...")
    
    for obj_x, obj_y, obj_z, name in objects:
        try:
            print(f"\nPicking up {name} at ({obj_x}, {obj_y}, {obj_z})")
            
            # Calculate end effector position to place suction cup at object
            # Since suction cup is 45mm ahead in +X direction
            end_effector_x = obj_x - EXTENSION_SUCTION_LENGTH
            end_effector_y = obj_y
            end_effector_z = ORIGIN_Z
            
            print(f"  Moving end effector to ({end_effector_x:.1f}, {end_effector_y:.1f}, {end_effector_z:.1f})")
            
            # Move to pick position
            quick(end_effector_x, end_effector_y, end_effector_z)
            time.sleep(1.0)
            
            # Verify suction cup position
            suction_pos = get_suction_cup_position()
            error_x = abs(suction_pos[0] - obj_x)
            error_y = abs(suction_pos[1] - obj_y)
            
            print(f"  Suction cup at ({suction_pos[0]:.1f}, {suction_pos[1]:.1f}, {suction_pos[2]:.1f})")
            print(f"  Position error: ({error_x:.1f}, {error_y:.1f}) mm")
            
            if error_x < 2.0 and error_y < 2.0:
                print(f"  ✓ {name} picked successfully!")
                
                # Move to drop-off
                dropoff_x, dropoff_y, dropoff_z, dropoff_name = dropoff
                end_effector_dropoff_x = dropoff_x - EXTENSION_SUCTION_LENGTH
                end_effector_dropoff_y = dropoff_y
                
                print(f"  Moving to {dropoff_name}")
                quick(end_effector_dropoff_x, end_effector_dropoff_y, ORIGIN_Z)
                time.sleep(1.0)
                print(f"  ✓ {name} delivered to {dropoff_name}")
            else:
                print(f"  ✗ Position error too large for {name}")
                
        except Exception as e:
            print(f"  ERROR with {name}: {e}")

def main():
    """Main test function"""
    print("SCARA Extension Arm Test Suite")
    print("=" * 50)
    
    # Ensure we're in coordinate mode
    coordinate_mode()
    
    try:
        # Start at origin
        print("Moving to origin position...")
        quick(ORIGIN_X, ORIGIN_Y, ORIGIN_Z, maintain_extension_direction=False)
        time.sleep(2.0)
        
        # Test 1: Extension arm orientation calculations
        test_extension_arm_orientation()
        
        # Test 2: Movement with automatic extension control
        test_extension_arm_movement()
        
        # Test 3: Different extension directions
        test_different_extension_directions()
        
        # Test 4: Pick and place simulation
        demonstrate_pick_and_place_simulation()
        
        # Return to origin
        print("\nReturning to origin...")
        quick(ORIGIN_X, ORIGIN_Y, ORIGIN_Z)
        
        print("\n" + "="*50)
        print("EXTENSION ARM TESTS COMPLETED")
        print("="*50)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with error: {e}")

if __name__ == "__main__":
    main() 