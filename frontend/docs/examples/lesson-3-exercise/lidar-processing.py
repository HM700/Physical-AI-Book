import numpy as np
from typing import List, Tuple, Optional
import math

class PointCloud:
    """Simple point cloud representation"""
    def __init__(self, points: np.ndarray):
        self.points = points  # Shape: (n, 3) where each row is [x, y, z]

    def filter_ground_plane(self, threshold: float = 0.1) -> Tuple['PointCloud', 'PointCloud']:
        """Simple ground plane filtering based on z-coordinate"""
        ground_mask = np.abs(self.points[:, 2]) < threshold
        ground_points = self.points[ground_mask]
        obstacle_points = self.points[~ground_mask]

        return PointCloud(ground_points), PointCloud(obstacle_points)

    def segment_objects(self, distance_threshold: float = 0.5, min_points: int = 5) -> List['PointCloud']:
        """Simple object segmentation using distance-based clustering"""
        if len(self.points) == 0:
            return []

        # Initialize cluster labels
        labels = np.full(len(self.points), -1, dtype=int)
        current_label = 0

        for i in range(len(self.points)):
            if labels[i] != -1:  # Already assigned
                continue

            # Find neighbors within distance threshold
            distances = np.linalg.norm(self.points - self.points[i], axis=1)
            neighbors = np.where(distances < distance_threshold)[0]

            # Assign same label to neighbors that aren't already labeled
            for neighbor_idx in neighbors:
                if labels[neighbor_idx] == -1:
                    labels[neighbor_idx] = current_label

            current_label += 1

        # Create point clouds for each cluster
        clusters = []
        for label in range(current_label):
            cluster_mask = labels == label
            cluster_points = self.points[cluster_mask]
            if len(cluster_points) >= min_points:  # Only include clusters with enough points
                clusters.append(PointCloud(cluster_points))

        return clusters

    def remove_outliers(self, k: int = 10, threshold: float = 1.0) -> 'PointCloud':
        """Remove outliers using k-nearest neighbors approach"""
        if len(self.points) <= k:
            return self

        # Calculate distance to k-th nearest neighbor for each point
        distances = []
        for i, point in enumerate(self.points):
            # Calculate distances to all other points
            all_distances = np.linalg.norm(self.points - point, axis=1)
            # Get k nearest neighbors (excluding the point itself)
            k_distances = np.partition(all_distances, k)[1:k+1]
            # Use median distance to k nearest neighbors as measure
            median_k_dist = np.median(k_distances)
            distances.append(median_k_dist)

        # Convert to numpy array
        distances = np.array(distances)

        # Calculate threshold based on mean and std
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)

        # Keep points within threshold
        keep_mask = distances < (mean_dist + threshold * std_dist)
        filtered_points = self.points[keep_mask]

        return PointCloud(filtered_points)

    def get_bounding_box(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get axis-aligned bounding box (min, max)"""
        if len(self.points) == 0:
            return np.array([0, 0, 0]), np.array([0, 0, 0])

        min_point = np.min(self.points, axis=0)
        max_point = np.max(self.points, axis=0)
        return min_point, max_point

    def get_centroid(self) -> np.ndarray:
        """Get centroid of the point cloud"""
        if len(self.points) == 0:
            return np.array([0, 0, 0])
        return np.mean(self.points, axis=0)

class LIDARProcessor:
    """LIDAR data processing for robot perception"""

    def __init__(self):
        pass

    def simulate_lidar_scan(self, num_points: int = 1000, radius: float = 10.0) -> PointCloud:
        """Simulate a simple LIDAR scan with some objects"""
        # Generate random angles
        angles = np.random.uniform(0, 2 * math.pi, num_points)
        # Generate random distances with some clustering (objects)
        distances = np.random.uniform(1, radius, num_points)

        # Create some clusters to simulate objects
        for _ in range(5):  # Add 5 object clusters
            cluster_center_angle = np.random.uniform(0, 2 * math.pi)
            cluster_center_dist = np.random.uniform(2, radius - 2)
            cluster_size = np.random.randint(10, 50)

            for i in range(cluster_size):
                if len(angles) < num_points:
                    angles = np.append(angles, np.random.normal(cluster_center_angle, 0.1))
                    distances = np.append(distances, np.random.normal(cluster_center_dist, 0.5))

        # Convert to Cartesian coordinates
        x = distances * np.cos(angles)
        y = distances * np.sin(angles)
        z = np.random.normal(0, 0.1, len(x))  # Add some height variation

        points = np.column_stack((x, y, z))
        return PointCloud(points)

    def detect_obstacles(self, point_cloud: PointCloud, min_height: float = 0.1, max_distance: float = 10.0) -> List[PointCloud]:
        """Detect obstacles in the point cloud"""
        # Filter points by distance
        distances = np.linalg.norm(point_cloud.points[:, :2], axis=1)  # Only x,y for distance
        distance_mask = distances <= max_distance
        filtered_points = point_cloud.points[distance_mask]

        # Filter by height (remove ground)
        height_mask = filtered_points[:, 2] >= min_height
        obstacle_points = filtered_points[height_mask]

        # Create new point cloud with obstacles only
        obstacle_cloud = PointCloud(obstacle_points)

        # Segment into individual objects
        obstacles = obstacle_cloud.segment_objects(distance_threshold=0.5, min_points=5)

        return obstacles

    def calculate_free_space(self, point_cloud: PointCloud, robot_radius: float = 0.5) -> np.ndarray:
        """Calculate free space around the robot"""
        # Calculate distances from origin (robot position)
        distances = np.linalg.norm(point_cloud.points[:, :2], axis=1)

        # Points that are too close to be free space
        too_close_mask = distances < robot_radius
        safe_points = point_cloud.points[~too_close_mask]

        return safe_points

    def create_occupancy_grid(self, point_cloud: PointCloud, grid_size: float = 0.5) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
        """Create a 2D occupancy grid from the point cloud"""
        if len(point_cloud.points) == 0:
            return np.zeros((100, 100)), (0, 0, 0, 0)

        # Get bounding box
        min_pt, max_pt = point_cloud.get_bounding_box()

        # Create grid dimensions
        width = int((max_pt[0] - min_pt[0]) / grid_size) + 1
        height = int((max_pt[1] - min_pt[1]) / grid_size) + 1

        # Initialize occupancy grid
        occupancy_grid = np.zeros((height, width), dtype=np.uint8)

        # Fill in occupied cells
        for point in point_cloud.points:
            x_idx = int((point[0] - min_pt[0]) / grid_size)
            y_idx = int((point[1] - min_pt[1]) / grid_size)

            if 0 <= x_idx < width and 0 <= y_idx < height:
                occupancy_grid[y_idx, x_idx] = 1  # Mark as occupied

        grid_bounds = (min_pt[0], max_pt[0], min_pt[1], max_pt[1])
        return occupancy_grid, grid_bounds

class ObjectTracker:
    """Simple object tracking for LIDAR data"""

    def __init__(self, max_distance: float = 1.0):
        self.max_distance = max_distance
        self.tracked_objects = []
        self.object_id_counter = 0

    def update(self, current_objects: List[PointCloud]) -> List[Tuple[int, PointCloud]]:
        """Update object tracking with new detections"""
        if not self.tracked_objects:
            # First frame - assign new IDs
            new_tracks = []
            for obj in current_objects:
                self.tracked_objects.append((self.object_id_counter, obj))
                new_tracks.append((self.object_id_counter, obj))
                self.object_id_counter += 1
            return new_tracks

        # For each new object, find closest existing track
        updated_tracks = []
        used_tracks = set()

        for current_obj in current_objects:
            current_centroid = current_obj.get_centroid()

            best_match = None
            best_distance = float('inf')

            # Find best matching existing track
            for track_id, tracked_obj in self.tracked_objects:
                if track_id in used_tracks:
                    continue

                tracked_centroid = tracked_obj.get_centroid()
                distance = np.linalg.norm(current_centroid[:2] - tracked_centroid[:2])

                if distance < best_distance and distance < self.max_distance:
                    best_distance = distance
                    best_match = track_id

            if best_match is not None:
                # Update existing track
                for i, (track_id, _) in enumerate(self.tracked_objects):
                    if track_id == best_match:
                        self.tracked_objects[i] = (best_match, current_obj)
                        break
                updated_tracks.append((best_match, current_obj))
                used_tracks.add(best_match)
            else:
                # Create new track
                self.tracked_objects.append((self.object_id_counter, current_obj))
                updated_tracks.append((self.object_id_counter, current_obj))
                self.object_id_counter += 1

        # Remove unused tracks
        self.tracked_objects = [track for track in self.tracked_objects if track[0] in [t[0] for t in updated_tracks]]

        return updated_tracks

# Example usage
if __name__ == "__main__":
    print("LIDAR Processing Example")
    print("Simulating LIDAR data...")

    # Initialize processor
    lidar_processor = LIDARProcessor()

    # Simulate a LIDAR scan
    simulated_scan = lidar_processor.simulate_lidar_scan(num_points=500)

    print(f"Simulated scan with {len(simulated_scan.points)} points")

    # Filter ground plane
    ground, obstacles = simulated_scan.filter_ground_plane(threshold=0.2)
    print(f"Ground points: {len(ground.points)}, Obstacle points: {len(obstacles.points)}")

    # Remove outliers from obstacles
    clean_obstacles = obstacles.remove_outliers()
    print(f"After outlier removal: {len(clean_obstacles.points)} obstacle points")

    # Detect individual obstacles
    detected_obstacles = lidar_processor.detect_obstacles(clean_obstacles)
    print(f"Detected {len(detected_obstacles)} obstacles")

    # Create occupancy grid
    occupancy_grid, bounds = lidar_processor.create_occupancy_grid(clean_obstacles)
    print(f"Created occupancy grid of size {occupancy_grid.shape}")

    # Object tracking example
    tracker = ObjectTracker(max_distance=1.0)
    tracked_objects = tracker.update(detected_obstacles[:3])  # Track first 3 obstacles
    print(f"Tracking {len(tracked_objects)} objects with IDs: {[obj_id for obj_id, _ in tracked_objects]}")

    # Print some statistics about detected obstacles
    for i, obstacle in enumerate(detected_obstacles[:3]):  # Just first 3
        centroid = obstacle.get_centroid()
        min_pt, max_pt = obstacle.get_bounding_box()
        print(f"Obstacle {i+1}: Centroid at ({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}), "
              f"Size: ({max_pt[0]-min_pt[0]:.2f}, {max_pt[1]-min_pt[1]:.2f}, {max_pt[2]-min_pt[2]:.2f})")

    print("\nLIDAR processing examples completed!")