#!/usr/bin/env python3
"""
3D Visualization System for SCARA AGV Object Detection
Plots detected objects in 3D space with interactive features
"""

import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches
from matplotlib.colors import ListedColormap
import seaborn as sns
from typing import List, Dict, Optional, Tuple
from pathlib import Path
import argparse

# Add Detection_Models to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'Detection_Models'))

class ObjectVisualizer3D:
    """
    3D visualization system for detected objects
    """
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        """
        Initialize 3D visualizer
        
        Args:
            figsize: Figure size (width, height)
        """
        self.figsize = figsize
        self.objects = []
        self.class_colors = {}
        self.color_palette = sns.color_palette("husl", 10)
        
    def load_objects_from_file(self, filename: str) -> bool:
        """
        Load detected objects from JSON file
        
        Args:
            filename: Path to JSON file with detection results
            
        Returns:
            True if loading successful
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            if 'objects' in data:
                self.objects = data['objects']
                print(f"Loaded {len(self.objects)} objects from {filename}")
                return True
            else:
                print(f"No objects found in {filename}")
                return False
                
        except Exception as e:
            print(f"Error loading objects: {e}")
            return False
    
    def load_objects_from_detector(self, detector) -> bool:
        """
        Load objects directly from detector object
        
        Args:
            detector: SCARAObjectDetection instance
            
        Returns:
            True if loading successful
        """
        if hasattr(detector, 'detected_objects') and detector.detected_objects:
            # Convert DetectedObject instances to dictionaries
            self.objects = []
            for obj in detector.detected_objects:
                obj_dict = {
                    'class_name': obj.class_name,
                    'confidence': obj.confidence,
                    'arm_coordinates': [obj.arm_x, obj.arm_y, obj.arm_z],
                    'pixel_coordinates': [obj.pixel_x, obj.pixel_y],
                    'depth_mm': obj.depth_mm,
                    'camera_position': list(obj.camera_position),
                    'camera_angle': obj.camera_angle,
                    'timestamp': obj.timestamp
                }
                self.objects.append(obj_dict)
            
            print(f"Loaded {len(self.objects)} objects from detector")
            return True
        else:
            print("No objects found in detector")
            return False
    
    def setup_class_colors(self):
        """Setup color mapping for different object classes"""
        unique_classes = list(set(obj['class_name'] for obj in self.objects))
        
        for i, class_name in enumerate(unique_classes):
            self.class_colors[class_name] = self.color_palette[i % len(self.color_palette)]
    
    def plot_3d_objects(self, 
                       show_workspace: bool = True,
                       show_camera_positions: bool = True,
                       show_confidence: bool = True,
                       show_labels: bool = True,
                       workspace_size: Tuple[float, float, float] = (400, 400, 300)) -> None:
        """
        Create 3D plot of detected objects with xy-plane projection
        
        Args:
            show_workspace: Whether to show workspace boundaries
            show_camera_positions: Whether to show camera positions
            show_confidence: Whether to use confidence for point size
            show_labels: Whether to show object labels
            workspace_size: Workspace dimensions (width, height, depth) in mm
        """
        if not self.objects:
            print("No objects to visualize")
            return
        
        # Setup colors
        self.setup_class_colors()
        
        # Create 3D figure
        fig = plt.figure(figsize=self.figsize)
        ax = fig.add_subplot(111, projection='3d')
        
        # Extract coordinates and properties
        x_coords = [obj['arm_coordinates'][0] for obj in self.objects]
        y_coords = [obj['arm_coordinates'][1] for obj in self.objects]
        z_coords = [obj['arm_coordinates'][2] for obj in self.objects]
        classes = [obj['class_name'] for obj in self.objects]
        confidences = [obj['confidence'] for obj in self.objects]
        
        # Plot xy-plane projection (shadow) at z=0
        for i, obj in enumerate(self.objects):
            x, y, z = obj['arm_coordinates']
            class_name = obj['class_name']
            confidence = obj['confidence']
            
            # Color based on class
            color = self.class_colors[class_name]
            
            # Size based on confidence
            if show_confidence:
                size = 50 + confidence * 200  # Scale confidence to size
            else:
                size = 100
            
            # Plot shadow point on xy-plane (z=0)
            ax.scatter(x, y, 0, 
                      c=[color], 
                      s=size * 0.3,  # Smaller shadow points
                      alpha=0.3, 
                      edgecolors='none',
                      marker='o')
            
            # Draw vertical line from shadow to actual point
            ax.plot([x, x], [y, y], [0, z], 
                   color=color, alpha=0.2, linewidth=1)
        
        # Plot actual 3D objects
        for i, obj in enumerate(self.objects):
            x, y, z = obj['arm_coordinates']
            class_name = obj['class_name']
            confidence = obj['confidence']
            
            # Color based on class
            color = self.class_colors[class_name]
            
            # Size based on confidence
            if show_confidence:
                size = 50 + confidence * 200  # Scale confidence to size
            else:
                size = 100
            
            # Plot actual 3D point
            ax.scatter(x, y, z, 
                      c=[color], 
                      s=size, 
                      alpha=0.7, 
                      edgecolors='black', 
                      linewidth=1)
            
            # Add label with ID and Z coordinate
            if show_labels:
                ax.text(x, y, z, f"ID:{i}\nZ:{z:.1f}", 
                       fontsize=8, ha='center', va='bottom')
        
        # Show workspace boundaries
        if show_workspace:
            self._plot_workspace_boundaries(ax, workspace_size)
        
        # Show camera positions
        if show_camera_positions:
            self._plot_camera_positions(ax)
        
        # Setup plot
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_title('3D Object Detection Results with XY-Plane Projection')
        
        # Add legend
        self._add_legend(ax)
        
        # Set equal aspect ratio
        ax.set_box_aspect([1, 1, 1])
        
        # Show plot
        plt.tight_layout()
        plt.show()
    
    def _plot_workspace_boundaries(self, ax, workspace_size: Tuple[float, float, float]):
        """Plot workspace boundaries as wireframe"""
        width, height, depth = workspace_size
        
        # Workspace corners
        corners = np.array([
            [-width/2, -height/2, 0],
            [width/2, -height/2, 0],
            [width/2, height/2, 0],
            [-width/2, height/2, 0],
            [-width/2, -height/2, depth],
            [width/2, -height/2, depth],
            [width/2, height/2, depth],
            [-width/2, height/2, depth]
        ])
        
        # Plot edges
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top
            [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical
        ]
        
        for edge in edges:
            start = corners[edge[0]]
            end = corners[edge[1]]
            ax.plot([start[0], end[0]], [start[1], end[1]], [start[2], end[2]], 
                   'k--', alpha=0.3, linewidth=1)
    
    def _plot_camera_positions(self, ax):
        """Plot camera positions during detection"""
        camera_positions = set()
        
        for obj in self.objects:
            if 'camera_position' in obj:
                pos = tuple(obj['camera_position'])
                camera_positions.add(pos)
        
        for pos in camera_positions:
            x, y, z = pos
            ax.scatter(x, y, z, c='red', s=200, marker='^', 
                      alpha=0.6, label='Camera Position')
            ax.text(x, y, z, 'Camera', fontsize=8, ha='center', va='bottom')
    
    def _add_legend(self, ax):
        """Add legend for object classes"""
        legend_elements = []
        
        for class_name, color in self.class_colors.items():
            legend_elements.append(
                plt.Line2D([0], [0], marker='o', color='w', 
                          markerfacecolor=color, markersize=10, 
                          label=class_name)
            )
        
        ax.legend(handles=legend_elements, loc='upper right')
    
    def plot_top_view(self, show_confidence: bool = True) -> None:
        """
        Create 2D top-down view of objects
        
        Args:
            show_confidence: Whether to use confidence for point size
        """
        if not self.objects:
            print("No objects to visualize")
            return
        
        # Setup colors
        self.setup_class_colors()
        
        # Create figure
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Extract coordinates
        x_coords = [obj['arm_coordinates'][0] for obj in self.objects]
        y_coords = [obj['arm_coordinates'][1] for obj in self.objects]
        classes = [obj['class_name'] for obj in self.objects]
        confidences = [obj['confidence'] for obj in self.objects]
        
        # Plot objects
        for i, obj in enumerate(self.objects):
            x, y = obj['arm_coordinates'][:2]
            class_name = obj['class_name']
            confidence = obj['confidence']
            
            color = self.class_colors[class_name]
            
            if show_confidence:
                size = 50 + confidence * 200
            else:
                size = 100
            
            ax.scatter(x, y, c=[color], s=size, alpha=0.7, 
                      edgecolors='black', linewidth=1)
            
            # Add label with ID and Z coordinate
            z = obj['arm_coordinates'][2]
            ax.annotate(f"ID:{i}\nZ:{z:.1f}", 
                       (x, y), xytext=(5, 5), textcoords='offset points',
                       fontsize=8, ha='left', va='bottom')
        
        # Setup plot
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_title('Top View - Object Detection Results')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # Add legend
        legend_elements = []
        for class_name, color in self.class_colors.items():
            legend_elements.append(
                plt.Line2D([0], [0], marker='o', color='w', 
                          markerfacecolor=color, markersize=10, 
                          label=class_name)
            )
        ax.legend(handles=legend_elements, loc='upper right')
        
        plt.tight_layout()
        plt.show()
    
    def plot_statistics(self) -> None:
        """Plot statistics about detected objects"""
        if not self.objects:
            print("No objects to analyze")
            return
        
        # Extract data
        classes = [obj['class_name'] for obj in self.objects]
        confidences = [obj['confidence'] for obj in self.objects]
        depths = [obj['arm_coordinates'][2] for obj in self.objects]
        
        # Create subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        
        # 1. Class distribution
        unique_classes, class_counts = np.unique(classes, return_counts=True)
        ax1.bar(unique_classes, class_counts, color=self.color_palette[:len(unique_classes)])
        ax1.set_title('Object Class Distribution')
        ax1.set_ylabel('Count')
        ax1.tick_params(axis='x', rotation=45)
        
        # 2. Confidence distribution
        ax2.hist(confidences, bins=20, alpha=0.7, color='skyblue', edgecolor='black')
        ax2.set_title('Confidence Distribution')
        ax2.set_xlabel('Confidence')
        ax2.set_ylabel('Count')
        
        # 3. Depth distribution
        ax3.hist(depths, bins=20, alpha=0.7, color='lightgreen', edgecolor='black')
        ax3.set_title('Depth Distribution')
        ax3.set_xlabel('Depth (mm)')
        ax3.set_ylabel('Count')
        
        # 4. Confidence vs Depth scatter
        ax4.scatter(depths, confidences, alpha=0.6, c='orange')
        ax4.set_title('Confidence vs Depth')
        ax4.set_xlabel('Depth (mm)')
        ax4.set_ylabel('Confidence')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    def save_visualization(self, filename: str, format: str = 'png') -> None:
        """
        Save current visualization to file
        
        Args:
            filename: Output filename
            format: Image format (png, jpg, pdf, etc.)
        """
        plt.savefig(filename, format=format, dpi=300, bbox_inches='tight')
        print(f"Visualization saved to {filename}")
    
    def create_interactive_plot(self) -> None:
        """Create interactive 3D plot using plotly (if available)"""
        try:
            import plotly.graph_objects as go
            import plotly.express as px
            from plotly.subplots import make_subplots
            
            if not self.objects:
                print("No objects to visualize")
                return
            
            # Setup colors
            self.setup_class_colors()
            
            # Extract data
            x_coords = [obj['arm_coordinates'][0] for obj in self.objects]
            y_coords = [obj['arm_coordinates'][1] for obj in self.objects]
            z_coords = [obj['arm_coordinates'][2] for obj in self.objects]
            classes = [obj['class_name'] for obj in self.objects]
            confidences = [obj['confidence'] for obj in self.objects]
            
            # Create 3D scatter plot
            fig = go.Figure()
            
            # Add xy-plane projection (shadows) at z=0
            for class_name in set(classes):
                mask = [c == class_name for c in classes]
                class_x = [x for i, x in enumerate(x_coords) if mask[i]]
                class_y = [y for i, y in enumerate(y_coords) if mask[i]]
                class_z = [z for i, z in enumerate(z_coords) if mask[i]]
                class_conf = [c for i, c in enumerate(confidences) if mask[i]]
                class_indices = [i for i, c in enumerate(classes) if c == class_name]
                
                # Add shadow points on xy-plane
                fig.add_trace(go.Scatter3d(
                    x=class_x,
                    y=class_y,
                    z=[0] * len(class_x),  # All at z=0
                    mode='markers',
                    marker=dict(
                        size=[(20 + conf * 50) * 0.3 for conf in class_conf],
                        color=self.class_colors[class_name],
                        opacity=0.3
                    ),
                    name=f"{class_name} (Shadow)",
                    showlegend=False
                ))
                
                # Add vertical lines from shadow to actual points
                for i, (x, y, z) in enumerate(zip(class_x, class_y, class_z)):
                    fig.add_trace(go.Scatter3d(
                        x=[x, x],
                        y=[y, y],
                        z=[0, z],
                        mode='lines',
                        line=dict(color=self.class_colors[class_name], width=1, opacity=0.2),
                        showlegend=False
                    ))
            
            # Add actual 3D objects by class
            for class_name in set(classes):
                mask = [c == class_name for c in classes]
                class_x = [x for i, x in enumerate(x_coords) if mask[i]]
                class_y = [y for i, y in enumerate(y_coords) if mask[i]]
                class_z = [z for i, z in enumerate(z_coords) if mask[i]]
                class_conf = [c for i, c in enumerate(confidences) if mask[i]]
                class_indices = [i for i, c in enumerate(classes) if c == class_name]
                
                fig.add_trace(go.Scatter3d(
                    x=class_x,
                    y=class_y,
                    z=class_z,
                    mode='markers+text',
                    marker=dict(
                        size=[20 + conf * 50 for conf in class_conf],
                        color=self.class_colors[class_name],
                        opacity=0.7
                    ),
                    text=[f"ID:{idx}<br>Z:{z:.1f}" for idx, z in zip(class_indices, class_z)],
                    name=class_name
                ))
            
            # Update layout
            fig.update_layout(
                title='3D Object Detection Results (Interactive)',
                scene=dict(
                    xaxis_title='X (mm)',
                    yaxis_title='Y (mm)',
                    zaxis_title='Z (mm)',
                    aspectmode='cube'
                ),
                width=800,
                height=600
            )
            
            # Show plot
            fig.show()
            
        except ImportError:
            print("Plotly not available. Install with: pip install plotly")
            print("Falling back to matplotlib visualization...")
            self.plot_3d_objects()


def main():
    """Main function for 3D visualization"""
    parser = argparse.ArgumentParser(description='3D Object Visualization')
    parser.add_argument('--file', type=str, help='JSON file with detection results')
    parser.add_argument('--workspace', action='store_true', help='Show workspace boundaries')
    parser.add_argument('--cameras', action='store_true', help='Show camera positions')
    parser.add_argument('--confidence', action='store_true', help='Use confidence for point size')
    parser.add_argument('--labels', action='store_true', help='Show object labels')
    parser.add_argument('--top-view', action='store_true', help='Show 2D top view')
    parser.add_argument('--stats', action='store_true', help='Show statistics')
    parser.add_argument('--interactive', action='store_true', help='Create interactive plot')
    parser.add_argument('--save', type=str, help='Save visualization to file')
    
    args = parser.parse_args()
    
    # Create visualizer
    visualizer = ObjectVisualizer3D()
    
    # Load objects
    if args.file:
        if not visualizer.load_objects_from_file(args.file):
            return
    else:
        # Try to load from default location
        default_files = [
            'detection_results.json',
            'test_detection_results.json',
            'example_results.json'
        ]
        
        loaded = False
        for filename in default_files:
            if os.path.exists(filename):
                if visualizer.load_objects_from_file(filename):
                    loaded = True
                    break
        
        if not loaded:
            print("No detection results found. Please provide a JSON file with --file")
            return
    
    # Create visualizations
    if args.top_view:
        visualizer.plot_top_view(show_confidence=args.confidence)
    
    if args.stats:
        visualizer.plot_statistics()
    
    if args.interactive:
        visualizer.create_interactive_plot()
    else:
        # Default 3D plot
        visualizer.plot_3d_objects(
            show_workspace=args.workspace,
            show_camera_positions=args.cameras,
            show_confidence=args.confidence,
            show_labels=args.labels
        )
    
    # Save if requested
    if args.save:
        visualizer.save_visualization(args.save)


if __name__ == "__main__":
    main() 