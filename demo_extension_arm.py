#!/usr/bin/env python3
"""
Demonstration script for the two-sided extension arm.
This shows how the camera and suction cup point in opposite directions.
"""

import math

# Extension arm dimensions
EXTENSION_SUCTION_LENGTH = 45   # mm from J4 to suction cup center
EXTENSION_CAMERA_LENGTH = 140   # mm from J4 to camera center

def demonstrate_two_sided_extension():
    """Demonstrate the two-sided extension arm concept."""
    print("Two-Sided Extension Arm Demonstration")
    print("=" * 50)
    print()
    print("The extension arm has two sides:")
    print("• Camera side: Points in the direction of the extension arm")
    print("• Suction cup side: Points in the opposite direction (180° offset)")
    print()
    
    # Example positions
    end_effector_x, end_effector_y = 200, 100
    j1, j2 = 30, -20  # Example joint angles
    j4 = 45  # Example extension arm angle
    
    print(f"Example configuration:")
    print(f"• End effector position: ({end_effector_x}, {end_effector_y})")
    print(f"• Joint angles: J1={j1}°, J2={j2}°")
    print(f"• Extension arm angle: J4={j4}°")
    print()
    
    # Calculate extension arm absolute angle
    extension_absolute_angle = j1 + j2 + j4
    extension_rad = math.radians(extension_absolute_angle)
    
    # Camera position (points in extension direction)
    camera_x = end_effector_x + EXTENSION_CAMERA_LENGTH * math.cos(extension_rad)
    camera_y = end_effector_y + EXTENSION_CAMERA_LENGTH * math.sin(extension_rad)
    
    # Suction cup position (opposite side, 180° offset)
    suction_rad = extension_rad + math.pi
    suction_x = end_effector_x + EXTENSION_SUCTION_LENGTH * math.cos(suction_rad)
    suction_y = end_effector_y + EXTENSION_SUCTION_LENGTH * math.sin(suction_rad)
    
    print(f"Extension arm absolute angle: {extension_absolute_angle:.1f}°")
    print(f"Camera direction: {extension_absolute_angle:.1f}°")
    print(f"Suction cup direction: {(extension_absolute_angle + 180) % 360 - 180:.1f}°")
    print()
    
    print(f"Positions:")
    print(f"• Camera: ({camera_x:.1f}, {camera_y:.1f}) - {EXTENSION_CAMERA_LENGTH}mm from end effector")
    print(f"• Suction cup: ({suction_x:.1f}, {suction_y:.1f}) - {EXTENSION_SUCTION_LENGTH}mm from end effector")
    print()
    
    # Calculate the angle between camera and suction cup
    camera_to_suction_angle = math.degrees(math.atan2(suction_y - camera_y, suction_x - camera_x))
    print(f"Angle from camera to suction cup: {camera_to_suction_angle:.1f}°")
    print(f"Expected: 180° (opposite directions)")
    print()

def demonstrate_different_directions():
    """Demonstrate different extension arm directions."""
    print("Different Extension Arm Directions")
    print("=" * 50)
    print()
    
    # Test different camera directions
    directions = [
        (0, "+X direction"),
        (90, "+Y direction"),
        (-90, "-Y direction"),
        (180, "-X direction"),
        (45, "45° direction")
    ]
    
    end_effector_x, end_effector_y = 150, 150
    
    for angle, description in directions:
        print(f"Camera pointing {description} ({angle}°):")
        
        # Calculate positions
        angle_rad = math.radians(angle)
        camera_x = end_effector_x + EXTENSION_CAMERA_LENGTH * math.cos(angle_rad)
        camera_y = end_effector_y + EXTENSION_CAMERA_LENGTH * math.sin(angle_rad)
        
        suction_rad = angle_rad + math.pi
        suction_x = end_effector_x + EXTENSION_SUCTION_LENGTH * math.cos(suction_rad)
        suction_y = end_effector_y + EXTENSION_SUCTION_LENGTH * math.sin(suction_rad)
        
        # Calculate suction cup direction
        suction_angle = (angle + 180) % 360
        if suction_angle > 180:
            suction_angle -= 360
        
        print(f"  • Camera: ({camera_x:.1f}, {camera_y:.1f})")
        print(f"  • Suction cup: ({suction_x:.1f}, {suction_y:.1f}) - pointing {suction_angle}°")
        print()

def demonstrate_practical_usage():
    """Demonstrate practical usage scenarios."""
    print("Practical Usage Scenarios")
    print("=" * 50)
    print()
    
    print("Scenario 1: Camera pointing down (-Y) for object detection")
    print("• Camera points -Y direction to look at objects below")
    print("• Suction cup automatically points +Y direction (upward)")
    print("• This allows the arm to approach objects from above")
    print()
    
    print("Scenario 2: Camera pointing forward (+X) for forward detection")
    print("• Camera points +X direction to look ahead")
    print("• Suction cup automatically points -X direction (backward)")
    print("• This allows the arm to approach objects from the front")
    print()
    
    print("Scenario 3: Camera pointing at 45° for diagonal detection")
    print("• Camera points 45° direction")
    print("• Suction cup automatically points -135° direction")
    print("• This provides a diagonal view while maintaining opposite orientation")
    print()

if __name__ == "__main__":
    demonstrate_two_sided_extension()
    demonstrate_different_directions()
    demonstrate_practical_usage()
    
    print("=" * 50)
    print("Demonstration completed!")
    print()
    print("Key points:")
    print("✓ Camera and suction cup always point in opposite directions")
    print("✓ When you set the camera direction, suction cup automatically points opposite")
    print("✓ This design allows for efficient pick-and-place operations")
    print("✓ The extension arm provides extended reach for both camera and suction cup") 