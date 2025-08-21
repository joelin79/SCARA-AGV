#!/usr/bin/env python3
"""
Object Mover - SCARA AGV Object Management System

This script reads the detected_objects.json file from ObjectDetection.py
and prompts the user to move objects between plates.

ArUco marker assignments:
- Plate 1: ArUco markers 0-3 (edge_marker_ids: [1, 0, 3, 2])
- Plate 2: ArUco markers 4-7 (edge_marker_ids: [5, 4, 7, 6])
- And so on...
"""

import json
import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import math

# Try to import SCARA control
try:
    sys.path.append(str(Path(__file__).parent / "Arm_Control"))
    from SCARA import *
    SCARA_AVAILABLE = True
except ImportError:
    print("⚠ SCARA control not available - running in simulation mode")
    SCARA_AVAILABLE = False

class ObjectMover:
    def __init__(self, json_path: str = "Detection_Models/detection_output/detected_objects.json"):
        self.json_path = Path(json_path)
        self.data = None
        self.plates = {}
        self.objects = []
        self.plate_assignments = {}  # Maps ArUco ranges to plate numbers
        
    def load_data(self) -> bool:
        """Load the detected objects JSON file"""
        try:
            if not self.json_path.exists():
                print(f"❌ JSON file not found: {self.json_path}")
                return False
                
            with open(self.json_path, 'r') as f:
                self.data = json.load(f)
                
            # Parse plates and create ArUco to plate mapping
            self._parse_plates()
            
            # Parse objects
            self.objects = self.data.get("detected_objects", [])
            
            print(f"✅ Loaded {len(self.plates)} plates and {len(self.objects)} objects")
            return True
            
        except Exception as e:
            print(f"❌ Error loading JSON: {e}")
            return False
    
    def _parse_plates(self):
        """Parse plates and create ArUco marker to plate number mapping"""
        detected_plates = self.data.get("detected_plates", [])
        
        for plate in detected_plates:
            plate_id = plate["plate_id"]
            edge_markers = plate["edge_marker_ids"]
            
            # Determine plate number based on ArUco marker range
            min_marker = min(edge_markers)
            max_marker = max(edge_markers)
            
            # Assign plate number (1-based)
            if min_marker <= 3 and max_marker <= 3:
                plate_num = 1
            elif min_marker >= 4 and max_marker <= 7:
                plate_num = 2
            elif min_marker >= 8 and max_marker <= 11:
                plate_num = 3
            else:
                # For other ranges, use a formula
                plate_num = (min_marker // 4) + 1
            
            self.plates[plate_num] = {
                "plate_id": plate_id,
                "center": plate["center_arm_coordinates"],
                "dimensions": plate["dimensions"],
                "height": plate["height_mm"],
                "edge_markers": edge_markers,
                "objects": []
            }
            
            # Store the mapping for reference
            self.plate_assignments[plate_num] = {
                "aruco_range": f"{min_marker}-{max_marker}",
                "markers": edge_markers
            }
    
    def assign_objects_to_plates(self):
        """Assign detected objects to their respective plates based on position"""
        for obj in self.objects:
            obj_pos = obj["arm_coordinates"]
            assigned_plate = self._find_nearest_plate(obj_pos)
            
            if assigned_plate:
                self.plates[assigned_plate]["objects"].append(obj)
                obj["assigned_plate"] = assigned_plate
            else:
                obj["assigned_plate"] = None
                print(f"⚠ Object {obj['object_id']} ({obj['class_name']}) not assigned to any plate")
    
    def _find_nearest_plate(self, obj_pos: Dict) -> Optional[int]:
        """Find the nearest plate to an object position"""
        min_distance = float('inf')
        nearest_plate = None
        
        for plate_num, plate_data in self.plates.items():
            plate_center = plate_data["center"]
            distance = math.sqrt(
                (obj_pos["x"] - plate_center["x"])**2 + 
                (obj_pos["y"] - plate_center["y"])**2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_plate = plate_num
        
        return nearest_plate
    
    def display_workspace_summary(self):
        """Display a summary of the workspace"""
        print("\n" + "="*60)
        print("WORKSPACE SUMMARY")
        print("="*60)
        
        for plate_num in sorted(self.plates.keys()):
            plate = self.plates[plate_num]
            aruco_info = self.plate_assignments[plate_num]
            
            print(f"\n📦 PLATE {plate_num}")
            print(f"   ArUco Markers: {aruco_info['aruco_range']} ({aruco_info['markers']})")
            print(f"   Center: ({plate['center']['x']:.1f}, {plate['center']['y']:.1f}, {plate['center']['z']:.1f})")
            print(f"   Dimensions: {plate['dimensions']['width_mm']:.1f} x {plate['dimensions']['length_mm']:.1f} mm")
            print(f"   Height: {plate['height']:.1f} mm")
            print(f"   Objects: {len(plate['objects'])}")
            
            if plate['objects']:
                for obj in plate['objects']:
                    print(f"     • {obj['class_name']} (ID: {obj['object_id']}) at ({obj['arm_coordinates']['x']:.1f}, {obj['arm_coordinates']['y']:.1f})")
    
    def interactive_move_menu(self):
        """Interactive menu for moving objects between plates"""
        while True:
            print("\n" + "="*60)
            print("OBJECT MOVEMENT MENU")
            print("="*60)
            print("1. Show workspace summary")
            print("2. Move object between plates")
            print("3. Move all objects from one plate to another")
            print("4. Show object details")
            print("5. Exit")
            
            choice = input("\nEnter your choice (1-5): ").strip()
            
            if choice == "1":
                self.display_workspace_summary()
                
            elif choice == "2":
                self.move_single_object()
                
            elif choice == "3":
                self.move_all_objects()
                
            elif choice == "4":
                self.show_object_details()
                
            elif choice == "5":
                print("Exiting...")
                break
                
            else:
                print("Invalid choice. Please enter 1-5.")
    
    def move_single_object(self):
        """Move a single object between plates"""
        if not self.objects:
            print("❌ No objects available to move")
            return
        
        # Show available objects
        print("\nAvailable objects:")
        for i, obj in enumerate(self.objects):
            plate_info = f"Plate {obj.get('assigned_plate', 'Unknown')}" if obj.get('assigned_plate') else "Unassigned"
            print(f"{i+1}. {obj['class_name']} (ID: {obj['object_id']}) - {plate_info}")
        
        try:
            obj_idx = int(input("\nSelect object number: ")) - 1
            if obj_idx < 0 or obj_idx >= len(self.objects):
                print("❌ Invalid object number")
                return
                
            obj = self.objects[obj_idx]
            
            # Show available destination plates
            print(f"\nMoving {obj['class_name']} (ID: {obj['object_id']})")
            print("Available destination plates:")
            
            for plate_num in sorted(self.plates.keys()):
                aruco_info = self.plate_assignments[plate_num]
                print(f"{plate_num}. Plate {plate_num} (ArUco {aruco_info['aruco_range']})")
            
            dest_plate = int(input("Enter destination plate number: "))
            if dest_plate not in self.plates:
                print("❌ Invalid plate number")
                return
            
            # Execute the move
            self._execute_move(obj, dest_plate)
            
        except ValueError:
            print("❌ Invalid input")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    def move_all_objects(self):
        """Move all objects from one plate to another"""
        if not self.plates:
            print("❌ No plates available")
            return
        
        print("\nMove all objects from one plate to another")
        print("Available plates:")
        
        for plate_num in sorted(self.plates.keys()):
            aruco_info = self.plate_assignments[plate_num]
            obj_count = len(self.plates[plate_num]["objects"])
            print(f"{plate_num}. Plate {plate_num} (ArUco {aruco_info['aruco_range']}) - {obj_count} objects")
        
        try:
            source_plate = int(input("\nEnter source plate number: "))
            if source_plate not in self.plates:
                print("❌ Invalid source plate number")
                return
            
            dest_plate = int(input("Enter destination plate number: "))
            if dest_plate not in self.plates:
                print("❌ Invalid destination plate number")
                return
            
            if source_plate == dest_plate:
                print("❌ Source and destination plates must be different")
                return
            
            # Confirm the move
            source_objects = self.plates[source_plate]["objects"]
            if not source_objects:
                print(f"❌ No objects on plate {source_plate}")
                return
            
            print(f"\nThis will move {len(source_objects)} objects from plate {source_plate} to plate {dest_plate}")
            confirm = input("Are you sure? (y/N): ").strip().lower()
            
            if confirm == 'y':
                # Execute the moves
                for obj in source_objects:
                    self._execute_move(obj, dest_plate, update_plate_lists=False)
                
                # Update plate lists
                self._update_plate_assignments()
                print(f"✅ Moved {len(source_objects)} objects from plate {source_plate} to plate {dest_plate}")
            else:
                print("Move cancelled")
                
        except ValueError:
            print("❌ Invalid input")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    def _execute_move(self, obj: Dict, dest_plate: int, update_plate_lists: bool = True):
        """Execute moving an object to a destination plate"""
        source_plate = obj.get("assigned_plate")
        dest_plate_data = self.plates[dest_plate]
        
        print(f"\n🔄 Moving {obj['class_name']} (ID: {obj['object_id']})")
        print(f"   From: {'Unassigned' if source_plate is None else f'Plate {source_plate}'}")
        print(f"   To: Plate {dest_plate} (ArUco {self.plate_assignments[dest_plate]['aruco_range']})")
        
        # Calculate destination position (center of destination plate)
        dest_x = dest_plate_data["center"]["x"]
        dest_y = dest_plate_data["center"]["y"]
        dest_z = dest_plate_data["height"] + 50
        
        if SCARA_AVAILABLE:
            try:
                print(f"   Moving to: ({dest_x:.1f}, {dest_y:.1f}, {dest_z:.1f})")
                
                # Move to object position and pick it up
                obj_pos = obj["arm_coordinates"]
                print("   Picking up object...")
                success = suck_object(obj_pos["x"], obj_pos["y"], obj_pos["z"])
                
                if success:
                    # Move to destination and release
                    print("   Moving to destination...")
                    quick_suction(dest_x, dest_y, LIMIT_J3_MAX + EE2CUP_Z_OFFSET)
                    time.sleep(2)
                    release_object(dest_x, dest_y, dest_z)
                    print("   ✅ Object moved successfully")
                else:
                    print("   ❌ Failed to pick up object")
                    return False
                    
            except Exception as e:
                print(f"   ❌ Error during movement: {e}")
                return False
        else:
            print("   [Simulation] Object would be moved to destination")
        
        # Update object assignment
        obj["assigned_plate"] = dest_plate
        
        if update_plate_lists:
            self._update_plate_assignments()
        
        return True
    
    def _update_plate_assignments(self):
        """Update the object lists for each plate"""
        # Clear all plate object lists
        for plate in self.plates.values():
            plate["objects"] = []
        
        # Reassign objects to plates
        for obj in self.objects:
            if obj.get("assigned_plate"):
                plate_num = obj["assigned_plate"]
                if plate_num in self.plates:
                    self.plates[plate_num]["objects"].append(obj)
    
    def show_object_details(self):
        """Show detailed information about objects"""
        if not self.objects:
            print("❌ No objects available")
            return
        
        print("\nObject Details:")
        for obj in self.objects:
            plate_info = f"Plate {obj.get('assigned_plate', 'Unknown')}" if obj.get('assigned_plate') else "Unassigned"
            print(f"\n• {obj['class_name']} (ID: {obj['object_id']})")
            print(f"  Confidence: {obj['confidence']:.3f}")
            print(f"  Position: ({obj['arm_coordinates']['x']:.1f}, {obj['arm_coordinates']['y']:.1f}, {obj['arm_coordinates']['z']:.1f})")
            print(f"  Assigned to: {plate_info}")
            print(f"  Bounding box: {obj['bbox']}")
            print(f"  Depth: {obj['depth_mm']:.1f} mm")
    
    def save_updated_data(self, filename: str = "updated_objects.json"):
        """Save the updated object assignments to a new JSON file"""
        try:
            # Create a copy of the original data
            updated_data = self.data.copy()
            
            # Update object assignments
            for obj in updated_data["detected_objects"]:
                obj["assigned_plate"] = self.objects[obj["object_id"] - 1].get("assigned_plate")
            
            # Save to file
            output_path = Path(filename)
            with open(output_path, 'w') as f:
                json.dump(updated_data, f, indent=2)
            
            print(f"✅ Updated data saved to: {output_path}")
            return True
            
        except Exception as e:
            print(f"❌ Error saving updated data: {e}")
            return False

def main():
    """Main function"""
    print("="*60)
    print("SCARA AGV OBJECT MOVER")
    print("="*60)
    
    # Initialize object mover
    mover = ObjectMover()
    
    # Load data
    if not mover.load_data():
        print("❌ Failed to load data. Exiting.")
        return
    
    # Assign objects to plates
    mover.assign_objects_to_plates()
    
    # Display initial summary
    mover.display_workspace_summary()
    
    # Start interactive menu
    try:
        mover.interactive_move_menu()
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    # Save updated data
    print("\nSaving updated object assignments...")
    mover.save_updated_data()
    
    print("\nThank you for using Object Mover!")

if __name__ == "__main__":
    main()

