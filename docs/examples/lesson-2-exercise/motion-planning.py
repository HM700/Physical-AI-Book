import numpy as np
import random
import math
from typing import List, Tuple, Optional

class Node:
    """Node for RRT algorithm"""
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent: Optional['Node'] = None

class RRT:
    """Rapidly-exploring Random Tree algorithm for path planning"""

    def __init__(self, start: Tuple[float, float], goal: Tuple[float, float],
                 obstacles: List[Tuple[float, float, float]], # x, y, radius
                 bounds: Tuple[float, float, float, float], # x_min, x_max, y_min, y_max
                 step_size: float = 0.1):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.obstacles = obstacles
        self.bounds = bounds
        self.step_size = step_size
        self.nodes = [self.start]

    def distance(self, node1: Node, node2: Node) -> float:
        """Calculate Euclidean distance between two nodes"""
        return math.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def is_in_obstacle(self, x: float, y: float) -> bool:
        """Check if a point is inside any obstacle"""
        for obs_x, obs_y, obs_radius in self.obstacles:
            if math.sqrt((x - obs_x)**2 + (y - obs_y)**2) <= obs_radius:
                return True
        return False

    def is_valid_point(self, x: float, y: float) -> bool:
        """Check if a point is valid (within bounds and not in obstacle)"""
        x_min, x_max, y_min, y_max = self.bounds

        if x < x_min or x > x_max or y < y_min or y > y_max:
            return False

        return not self.is_in_obstacle(x, y)

    def get_random_node(self) -> Node:
        """Generate a random node within bounds"""
        x = random.uniform(self.bounds[0], self.bounds[1])
        y = random.uniform(self.bounds[2], self.bounds[3])
        return Node(x, y)

    def get_nearest_node(self, random_node: Node) -> Node:
        """Find the nearest node in the tree to the random node"""
        nearest_node = self.nodes[0]
        min_dist = self.distance(random_node, nearest_node)

        for node in self.nodes[1:]:
            dist = self.distance(random_node, node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node

        return nearest_node

    def extend_towards(self, from_node: Node, to_node: Node) -> Optional[Node]:
        """Extend from from_node towards to_node by step_size"""
        dist = self.distance(from_node, to_node)

        if dist <= self.step_size:
            new_node = Node(to_node.x, to_node.y)
        else:
            # Calculate direction vector
            dx = to_node.x - from_node.x
            dy = to_node.y - from_node.y
            length = math.sqrt(dx**2 + dy**2)

            # Normalize and scale by step size
            dx = dx * self.step_size / length
            dy = dy * self.step_size / length

            new_node = Node(from_node.x + dx, from_node.y + dy)

        # Check if the new node is valid
        if self.is_valid_point(new_node.x, new_node.y):
            new_node.parent = from_node
            return new_node

        return None

    def is_at_goal(self, node: Node, tolerance: float = 0.1) -> bool:
        """Check if node is close enough to goal"""
        return self.distance(node, self.goal) <= tolerance

    def build_path(self, node: Node) -> List[Tuple[float, float]]:
        """Reconstruct path from goal node to start"""
        path = []
        current = node

        while current is not None:
            path.append((current.x, current.y))
            current = current.parent

        return path[::-1]  # Reverse to get path from start to goal

    def plan(self, max_iterations: int = 1000) -> Optional[List[Tuple[float, float]]]:
        """Plan a path from start to goal"""
        for i in range(max_iterations):
            # Get random node
            random_node = self.get_random_node()

            # Find nearest node in tree
            nearest_node = self.get_nearest_node(random_node)

            # Extend towards random node
            new_node = self.extend_towards(nearest_node, random_node)

            if new_node is not None:
                # Add new node to tree
                self.nodes.append(new_node)

                # Check if we're near the goal
                if self.is_at_goal(new_node):
                    # Try to connect directly to goal
                    goal_node = self.extend_towards(new_node, self.goal)
                    if goal_node is not None:
                        return self.build_path(goal_node)

        # If we couldn't reach the goal, find the closest node and return path to it
        closest_node = min(self.nodes, key=lambda n: self.distance(n, self.goal))
        return self.build_path(closest_node)

class PathSmoother:
    """Simple path smoother using iterative smoothing"""

    def __init__(self, weight_data: float = 0.5, weight_smooth: float = 0.1, tolerance: float = 1e-6):
        self.weight_data = weight_data
        self.weight_smooth = weight_smooth
        self.tolerance = tolerance

    def smooth(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """Smooth a path using iterative optimization"""
        if len(path) < 3:
            return path  # Can't smooth a path with less than 3 points

        # Initialize smoothed path as a copy of the original
        smoothed_path = [list(point) for point in path]

        change = self.tolerance
        while change >= self.tolerance:
            change = 0.0
            for i in range(1, len(path) - 1):  # Don't modify start or end points
                for j in range(2):  # x and y coordinates
                    old_value = smoothed_path[i][j]
                    # Update based on original path and neighbors
                    smoothed_path[i][j] += self.weight_data * (path[i][j] - smoothed_path[i][j])
                    smoothed_path[i][j] += self.weight_smooth * (smoothed_path[i-1][j] + smoothed_path[i+1][j] - 2.0 * smoothed_path[i][j])
                    change += abs(old_value - smoothed_path[i][j])

        return [(x, y) for x, y in smoothed_path]

def visualize_path(start: Tuple[float, float], goal: Tuple[float, float],
                  obstacles: List[Tuple[float, float, float]],
                  path: List[Tuple[float, float]],
                  bounds: Tuple[float, float, float, float]):
    """
    Create a simple text-based visualization of the path
    In a real implementation, this would use matplotlib or another visualization library
    """
    print("Path Planning Visualization")
    print(f"Start: {start}, Goal: {goal}")
    print(f"Bounds: {bounds}")
    print(f"Obstacles: {obstacles}")
    print(f"Path length: {len(path)} points")
    print("First few points:", path[:5])
    print("Last few points:", path[-5:])

# Example usage
if __name__ == "__main__":
    # Define problem parameters
    start = (0.0, 0.0)
    goal = (5.0, 5.0)
    obstacles = [
        (2.0, 2.0, 0.5),  # Obstacle at (2,2) with radius 0.5
        (3.0, 4.0, 0.7),  # Obstacle at (3,4) with radius 0.7
        (4.0, 2.0, 0.6),  # Obstacle at (4,2) with radius 0.6
    ]
    bounds = (0.0, 6.0, 0.0, 6.0)  # x_min, x_max, y_min, y_max

    print("RRT Path Planning Example")
    print(f"Start: {start}")
    print(f"Goal: {goal}")
    print(f"Obstacles: {obstacles}")
    print()

    # Create RRT planner
    rrt = RRT(start, goal, obstacles, bounds, step_size=0.2)

    # Plan path
    path = rrt.plan(max_iterations=2000)

    if path:
        print(f"Path found with {len(path)} waypoints!")

        # Smooth the path
        smoother = PathSmoother(weight_data=0.1, weight_smooth=0.8)
        smoothed_path = smoother.smooth(path)

        print(f"Path smoothed to {len(smoothed_path)} waypoints")

        # Visualize
        visualize_path(start, goal, obstacles, smoothed_path, bounds)

        # Print some statistics
        total_distance = 0
        for i in range(1, len(smoothed_path)):
            dx = smoothed_path[i][0] - smoothed_path[i-1][0]
            dy = smoothed_path[i][1] - smoothed_path[i-1][1]
            total_distance += math.sqrt(dx*dx + dy*dy)

        print(f"Total path distance: {total_distance:.2f}")

    else:
        print("No path found!")