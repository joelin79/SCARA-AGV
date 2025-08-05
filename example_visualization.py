#!/usr/bin/env python3
"""
Example 3D Visualization for SCARA AGV Object Detection
Demonstrates how to create 3D plots of detected objects
"""

import os
import sys
import json
import numpy as np
from pathlib import Path

# Add Detection_Models to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'Detection_Models'))

def create_sample_data():
    """Create sample detection data for demonstration"""
    sample_objects = [
        {
            "class_name": "object",
            "confidence": 0.85,
            "arm_coordinates": [50.0, 30.0, 25.0],
            "pixel_coordinates": [320, 240],
            "depth_mm": 275.0,
            "camera_position": [0.0, 0.0, 300.0],
            "camera_angle": -90.0,
            "timestamp": 1234567890.123
        },
        {
            "class_name": "object",
            "confidence": 0.92,
            "arm_coordinates": [-40.0, 60.0, 20.0],
            "pixel_coordinates": [280, 200],
            "depth_mm": 280.0,
            "camera_position": [0.0, 0.0, 300.0],
            "camera_angle": -90.0,
            "timestamp": 1234567890.124
        },
        {
            "class_name": "object",
            "confidence": 0.78,
            "arm_coordinates": [80.0, -20.0, 30.0],
            "pixel_coordinates": [400, 280],
            "depth_mm": 270.0,
            "camera_position": [0.0, 0.0, 300.0],
            "camera_angle": -90.0,
            "timestamp": 1234567890.125
        },
        {
            "class_name": "object",
            "confidence": 0.95,
            "arm_coordinates": [-60.0, -40.0, 15.0],
            "pixel_coordinates": [240, 320],
            "depth_mm": 285.0,
            "camera_position": [0.0, 0.0, 300.0],
            "camera_angle": -90.0,
            "timestamp": 1234567890.126
        },
        {
            "class_name": "object",
            "confidence": 0.88,
            "arm_coordinates": [20.0, 80.0, 35.0],
            "pixel_coordinates": [360, 160],
            "depth_mm": 265.0,
            "camera_position": [0.0, 0.0, 300.0],
            "camera_angle": -90.0,
            "timestamp": 1234567890.127
        }
    ]
    
    # Create sample results file
    sample_results = {
        "timestamp": 1234567890.123,
        "total_objects": len(sample_objects),
        "objects": sample_objects
    }
    
    # Save to file
    with open("sample_detection_results.json", "w") as f:
        json.dump(sample_results, f, indent=2)
    
    print("Sample detection data created: sample_detection_results.json")
    return sample_results

def demonstrate_visualization():
    """Demonstrate the 3D visualization system"""
    print("3D Object Visualization Demo")
    print("=" * 40)
    
    try:
        # Import visualizer
        from visualization_3d import ObjectVisualizer3D
        
        # Create sample data if no existing data
        if not os.path.exists("sample_detection_results.json"):
            create_sample_data()
        
        # Create visualizer
        visualizer = ObjectVisualizer3D()
        
        # Load objects
        if visualizer.load_objects_from_file("sample_detection_results.json"):
            print(f"Loaded {len(visualizer.objects)} objects for visualization")
            
            # Demonstrate different visualization options
            print("\n1. Creating 3D plot...")
            visualizer.plot_3d_objects(
                show_workspace=True,
                show_camera_positions=True,
                show_confidence=True,
                show_labels=True
            )
            
            print("\n2. Creating top view...")
            visualizer.plot_top_view(show_confidence=True)
            
            print("\n3. Creating statistics...")
            visualizer.plot_statistics()
            
            print("\n4. Saving plots...")
            visualizer.save_visualization("demo_3d_plot.png")
            visualizer.save_visualization("demo_top_view.png")
            visualizer.save_visualization("demo_statistics.png")
            
            print("\nVisualization demo completed!")
            print("Files created:")
            print("  - demo_3d_plot.png")
            print("  - demo_top_view.png")
            print("  - demo_statistics.png")
            
        else:
            print("Failed to load objects for visualization")
            
    except ImportError as e:
        print(f"Visualization not available: {e}")
        print("Install required packages:")
        print("  pip install matplotlib seaborn")
        
    except Exception as e:
        print(f"Error in visualization demo: {e}")

def demonstrate_with_real_data():
    """Demonstrate visualization with real detection data"""
    print("\nReal Data Visualization Demo")
    print("=" * 40)
    
    try:
        # Import visualizer
        from visualization_3d import ObjectVisualizer3D
        
        # Create visualizer
        visualizer = ObjectVisualizer3D()
        
        # Try to load from various possible files
        possible_files = [
            "detection_results.json",
            "test_detection_results.json",
            "example_results.json",
            "sample_detection_results.json"
        ]
        
        loaded = False
        for filename in possible_files:
            if os.path.exists(filename):
                print(f"Found detection results: {filename}")
                if visualizer.load_objects_from_file(filename):
                    loaded = True
                    break
        
        if loaded:
            print(f"Loaded {len(visualizer.objects)} objects")
            
            # Create comprehensive visualization
            print("\nCreating comprehensive visualization...")
            
            # 3D plot
            visualizer.plot_3d_objects(
                show_workspace=True,
                show_camera_positions=True,
                show_confidence=True,
                show_labels=True
            )
            
            # Top view
            visualizer.plot_top_view(show_confidence=True)
            
            # Statistics
            visualizer.plot_statistics()
            
            # Save all plots
            visualizer.save_visualization("real_data_3d.png")
            visualizer.save_visualization("real_data_top.png")
            visualizer.save_visualization("real_data_stats.png")
            
            print("Real data visualization completed!")
            
        else:
            print("No real detection data found")
            print("Run object detection first, or use sample data")
            
    except Exception as e:
        print(f"Error in real data visualization: {e}")

def demonstrate_interactive():
    """Demonstrate interactive 3D visualization"""
    print("\nInteractive 3D Visualization Demo")
    print("=" * 40)
    
    try:
        # Import visualizer
        from visualization_3d import ObjectVisualizer3D
        
        # Create sample data if needed
        if not os.path.exists("sample_detection_results.json"):
            create_sample_data()
        
        # Create visualizer
        visualizer = ObjectVisualizer3D()
        
        # Load objects
        if visualizer.load_objects_from_file("sample_detection_results.json"):
            print("Creating interactive 3D plot...")
            print("(This will open in your web browser)")
            
            visualizer.create_interactive_plot()
            
        else:
            print("Failed to load objects for interactive visualization")
            
    except Exception as e:
        print(f"Error in interactive visualization: {e}")

def main():
    """Main demonstration function"""
    print("SCARA AGV 3D Visualization Examples")
    print("=" * 50)
    
    print("\nChoose demonstration mode:")
    print("1. Sample data visualization")
    print("2. Real data visualization")
    print("3. Interactive visualization")
    print("4. All demonstrations")
    
    try:
        choice = input("\nEnter choice (1-4): ").strip()
        
        if choice == "1":
            demonstrate_visualization()
        elif choice == "2":
            demonstrate_with_real_data()
        elif choice == "3":
            demonstrate_interactive()
        elif choice == "4":
            demonstrate_visualization()
            demonstrate_with_real_data()
            demonstrate_interactive()
        else:
            print("Invalid choice. Running sample visualization...")
            demonstrate_visualization()
            
    except KeyboardInterrupt:
        print("\nDemonstration cancelled by user")
    except Exception as e:
        print(f"Error: {e}")
        print("Running sample visualization as fallback...")
        demonstrate_visualization()

if __name__ == "__main__":
    main() 