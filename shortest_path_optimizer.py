#!/usr/bin/env python3
"""
Simple Shortest Path Optimizer for SCARA AGV Scanning
Uses nearest neighbor algorithm to minimize total travel distance between scan positions
"""

import math
from typing import List, Tuple
import numpy as np


def optimize_scanning_order(positions: List[Tuple[float, float, float]], 
                           start_position: Tuple[float, float, float] = None) -> List[Tuple[float, float, float]]:
    """
    Optimize scanning order using nearest neighbor algorithm
    
    Args:
        positions: List of (x, y, z) scan positions
        start_position: Starting position (x, y, z). If None, uses first position
        
    Returns:
        List of positions in optimized order
    """
    if not positions:
        return []
    
    if len(positions) == 1:
        return positions
    
    # Convert to numpy array for easier manipulation
    pos_array = np.array(positions)
    
    # Set starting position
    if start_position is None:
        start_pos = pos_array[0]
        remaining_positions = pos_array[1:].copy()
    else:
        start_pos = np.array(start_position)
        # remove the chosen start from remaining if it exists to avoid duplication
        # find exact match index (within tolerance) if present
        remaining_positions = pos_array.copy()
        try:
            diffs = np.linalg.norm(remaining_positions - start_pos, axis=1)
            idx = int(np.argmin(diffs))
            if diffs[idx] < 1e-6:
                remaining_positions = np.delete(remaining_positions, idx, axis=0)
        except Exception:
            pass
    
    # Initialize result with starting position
    optimized_order = [tuple(start_pos)]
    current_pos = start_pos
    
    # Greedy nearest neighbor algorithm
    while len(remaining_positions) > 0:
        # Find nearest remaining position
        distances = np.sqrt(np.sum((remaining_positions - current_pos) ** 2, axis=1))
        nearest_idx = np.argmin(distances)
        
        # Add nearest position to result
        nearest_pos = remaining_positions[nearest_idx]
        optimized_order.append(tuple(nearest_pos))
        
        # Update current position and remove visited position
        current_pos = nearest_pos
        remaining_positions = np.delete(remaining_positions, nearest_idx, axis=0)
    
    return optimized_order


def calculate_total_distance(positions: List[Tuple[float, float, float]]) -> float:
    """
    Calculate total travel distance for a given order of positions
    
    Args:
        positions: List of (x, y, z) positions in order
        
    Returns:
        Total distance in mm
    """
    if len(positions) <= 1:
        return 0.0
    
    total_distance = 0.0
    
    for i in range(len(positions) - 1):
        pos1 = positions[i]
        pos2 = positions[i + 1]
        
        # Calculate Euclidean distance (3D)
        distance = math.sqrt(
            (pos2[0] - pos1[0]) ** 2 +
            (pos2[1] - pos1[1]) ** 2 +
            (pos2[2] - pos1[2]) ** 2
        )
        total_distance += distance
    
    return total_distance


def optimize_with_2opt(positions: List[Tuple[float, float, float]], 
                       max_iterations: int = 100) -> List[Tuple[float, float, float]]:
    """
    Optimize scanning order using 2-opt local search improvement
    
    Args:
        positions: List of (x, y, z) scan positions
        max_iterations: Maximum number of improvement iterations
        
    Returns:
        List of positions in optimized order
    """
    if len(positions) <= 3:
        return positions
    
    # Start with nearest neighbor solution
    current_order = optimize_scanning_order(positions)
    current_distance = calculate_total_distance(current_order)
    
    improved = True
    iteration = 0
    
    while improved and iteration < max_iterations:
        improved = False
        iteration += 1
        
        # Try all possible 2-opt swaps
        for i in range(1, len(current_order) - 2):
            for j in range(i + 1, len(current_order)):
                if j - i == 1:
                    continue  # Skip adjacent positions
                
                # Create new order with 2-opt swap
                new_order = current_order[:i] + current_order[i:j+1][::-1] + current_order[j+1:]
                new_distance = calculate_total_distance(new_order)
                
                # If improvement found, keep it
                if new_distance < current_distance:
                    current_order = new_order
                    current_distance = new_distance
                    improved = True
                    break
            
            if improved:
                break
    
    print(f"2-opt optimization completed in {iteration} iterations")
    print(f"Final distance: {current_distance:.2f}mm")
    
    return current_order


def print_optimization_stats(original_positions: List[Tuple[float, float, float]], 
                           optimized_positions: List[Tuple[float, float, float]]):
    """
    Print statistics comparing original and optimized scanning orders
    
    Args:
        original_positions: Original order of positions
        optimized_positions: Optimized order of positions
    """
    original_distance = calculate_total_distance(original_positions)
    optimized_distance = calculate_total_distance(optimized_positions)
    
    print("\n" + "="*50)
    print("SCANNING OPTIMIZATION RESULTS")
    print("="*50)
    print(f"Number of positions: {len(original_positions)}")
    print(f"Original total distance: {original_distance:.2f}mm")
    print(f"Optimized total distance: {optimized_distance:.2f}mm")
    print(f"Distance reduction: {original_distance - optimized_distance:.2f}mm")
    print(f"Improvement: {((original_distance - optimized_distance) / original_distance * 100):.1f}%")
    print("="*50)


# Example usage and testing
if __name__ == "__main__":
    # Example scan positions (similar to what ObjectDetectionSystem generates)
    example_positions = [
        (50, -200, 150),   # Bottom-left
        (130, -200, 150),  # Bottom-left-center
        (210, -200, 150),  # Bottom-center
        (290, -200, 150),  # Bottom-right-center
        (370, -200, 150),  # Bottom-right
        (450, -200, 150),  # Bottom-far-right
        
        (50, -120, 150),   # Lower-left
        (130, -120, 150),  # Lower-left-center
        (210, -120, 150),  # Lower-center
        (290, -120, 150),  # Lower-right-center
        (370, -120, 150),  # Lower-right
        (450, -120, 150),  # Lower-far-right
        
        (50, -40, 150),    # Lower-mid-left
        (130, -40, 150),   # Lower-mid-left-center
        (210, -40, 150),   # Lower-mid-center
        (290, -40, 150),   # Lower-mid-right-center
        (370, -40, 150),   # Lower-mid-right
        (450, -40, 150),   # Lower-mid-far-right
        
        (50, 40, 150),     # Upper-mid-left
        (130, 40, 150),    # Upper-mid-left-center
        (210, 40, 150),    # Upper-mid-center
        (290, 40, 150),    # Upper-mid-right-center
        (370, 40, 150),    # Upper-mid-right
        (450, 40, 150),    # Upper-mid-far-right
        
        (50, 120, 150),    # Upper-left
        (130, 120, 150),   # Upper-left-center
        (210, 120, 150),   # Upper-center
        (290, 120, 150),   # Upper-right-center
        (370, 120, 150),   # Upper-right
        (450, 120, 150),   # Upper-far-right
        
        (50, 200, 150),    # Top-left
        (130, 200, 150),   # Top-left-center
        (210, 200, 150),   # Top-center
        (290, 200, 150),   # Top-right-center
        (370, 200, 150),   # Top-right
        (450, 200, 150),   # Top-far-right
    ]
    
    print("Original scanning order (grid pattern):")
    for i, pos in enumerate(example_positions):
        print(f"  {i+1:2d}: ({pos[0]:3.0f}, {pos[1]:4.0f}, {pos[2]:3.0f})")
    
    # Test basic nearest neighbor optimization
    print("\n" + "="*60)
    print("NEAREST NEIGHBOR OPTIMIZATION")
    print("="*60)
    
    optimized_basic = optimize_scanning_order(example_positions)
    print_optimization_stats(example_positions, optimized_basic)
    
    # Test 2-opt improvement
    print("\n" + "="*60)
    print("2-OPT IMPROVEMENT")
    print("="*60)
    
    optimized_2opt = optimize_with_2opt(example_positions)
    print_optimization_stats(example_positions, optimized_2opt)
    
    # Show first few optimized positions
    print("\nFirst 10 optimized positions:")
    for i, pos in enumerate(optimized_2opt[:10]):
        print(f"  {i+1:2d}: ({pos[0]:3.0f}, {pos[1]:4.0f}, {pos[2]:3.0f})")
    
    print("...")
    print(f"  {len(optimized_2opt):2d}: ({optimized_2opt[-1][0]:3.0f}, {optimized_2opt[-1][1]:4.0f}, {optimized_2opt[-1][2]:3.0f})") 