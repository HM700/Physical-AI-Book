import numpy as np
from typing import Tuple, List, Optional
import math

class KalmanFilter:
    """Simple Kalman Filter for sensor fusion"""

    def __init__(self, initial_state: np.ndarray, initial_covariance: np.ndarray):
        self.state = initial_state  # [x, y, vx, vy] (position and velocity)
        self.covariance = initial_covariance  # Uncertainty matrix

        # Process noise (how uncertain we are about our model)
        self.process_noise = np.eye(len(initial_state)) * 0.1

        # Measurement noise for different sensors
        self.camera_noise = np.eye(2) * 0.5  # Camera: [x, y]
        self.lidar_noise = np.eye(2) * 0.3   # LIDAR: [x, y]

    def predict(self, dt: float = 0.1):
        """Predict the next state based on motion model"""
        # Motion model: constant velocity
        # x_new = x + vx * dt
        # y_new = y + vy * dt
        # vx_new = vx (assuming constant velocity)
        # vy_new = vy (assuming constant velocity)
        F = np.array([
            [1, 0, dt, 0],   # x_new = x + vx*dt
            [0, 1, 0, dt],   # y_new = y + vy*dt
            [0, 0, 1, 0],    # vx_new = vx
            [0, 0, 0, 1]     # vy_new = vy
        ])

        # Predict state
        self.state = F @ self.state

        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + self.process_noise

    def update(self, measurement: np.ndarray, measurement_noise: np.ndarray, measurement_map: np.ndarray):
        """Update state with a new measurement"""
        # measurement_map maps the full state to the measurement space
        # For position measurements: [1, 0, 0, 0; 0, 1, 0, 0] (extract x, y from state)
        H = measurement_map

        # Calculate Kalman gain
        S = H @ self.covariance @ H.T + measurement_noise
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Calculate innovation (difference between measurement and prediction)
        innovation = measurement - H @ self.state

        # Update state
        self.state = self.state + K @ innovation

        # Update covariance
        I = np.eye(len(self.state))
        self.covariance = (I - K @ H) @ self.covariance

    def get_position(self) -> Tuple[float, float]:
        """Get current position estimate"""
        return float(self.state[0]), float(self.state[1])

    def get_velocity(self) -> Tuple[float, float]:
        """Get current velocity estimate"""
        return float(self.state[2]), float(self.state[3])

class SensorFusionSystem:
    """Fuses data from multiple sensors using Kalman filtering"""

    def __init__(self):
        # Initialize with position (x, y) and velocity (vx, vy)
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        initial_covariance = np.eye(4) * 1.0  # High initial uncertainty

        self.kalman_filter = KalmanFilter(initial_state, initial_covariance)

        # Measurement maps
        # For position measurements: extract x, y from state [x, y, vx, vy]
        self.position_map = np.array([
            [1, 0, 0, 0],  # x measurement
            [0, 1, 0, 0]   # y measurement
        ])

    def process_camera_data(self, camera_pos: Tuple[float, float], dt: float = 0.1):
        """Process camera measurement"""
        # Predict state forward in time
        self.kalman_filter.predict(dt)

        # Create measurement vector
        measurement = np.array(camera_pos)

        # Update with camera measurement
        self.kalman_filter.update(measurement, self.kalman_filter.camera_noise, self.position_map)

        return self.kalman_filter.get_position(), self.kalman_filter.get_velocity()

    def process_lidar_data(self, lidar_pos: Tuple[float, float], dt: float = 0.1):
        """Process LIDAR measurement"""
        # Predict state forward in time
        self.kalman_filter.predict(dt)

        # Create measurement vector
        measurement = np.array(lidar_pos)

        # Update with LIDAR measurement
        self.kalman_filter.update(measurement, self.kalman_filter.lidar_noise, self.position_map)

        return self.kalman_filter.get_position(), self.kalman_filter.get_velocity()

    def process_fused_data(self, camera_pos: Tuple[float, float], lidar_pos: Tuple[float, float], dt: float = 0.1):
        """Fuse camera and LIDAR data using weighted average approach"""
        # First, get individual estimates
        cam_pos, cam_vel = self.process_camera_data(camera_pos, dt/2)  # Process each with half time step
        lidar_pos_est, lidar_vel = self.process_lidar_data(lidar_pos, dt/2)

        # Weighted fusion based on sensor reliability (inverse of noise)
        cam_weight = 1.0 / 0.5  # Inverse of camera noise
        lidar_weight = 1.0 / 0.3  # Inverse of LIDAR noise
        total_weight = cam_weight + lidar_weight

        # Weighted average of positions
        fused_x = (cam_weight * cam_pos[0] + lidar_weight * lidar_pos_est[0]) / total_weight
        fused_y = (cam_weight * cam_pos[1] + lidar_weight * lidar_pos_est[1]) / total_weight

        # Update with fused position
        fused_pos = np.array([fused_x, fused_y])
        self.kalman_filter.predict(dt)  # Predict for full time step
        self.kalman_filter.update(fused_pos, (self.kalman_filter.camera_noise + self.kalman_filter.lidar_noise) / 2, self.position_map)

        return self.kalman_filter.get_position(), self.kalman_filter.get_velocity()

    def get_uncertainty(self) -> float:
        """Get position uncertainty (trace of position covariance)"""
        pos_cov = self.kalman_filter.covariance[:2, :2]  # Position part of covariance
        return float(np.trace(pos_cov))

class ParticleFilter:
    """Simple particle filter for non-linear sensor fusion"""

    def __init__(self, num_particles: int = 1000):
        self.num_particles = num_particles
        self.particles = np.zeros((num_particles, 4))  # [x, y, vx, vy]
        self.weights = np.ones(num_particles) / num_particles

        # Initialize particles with some uncertainty
        self.particles[:, :2] = np.random.normal(0, 1, (num_particles, 2))  # Position
        self.particles[:, 2:] = np.random.normal(0, 0.5, (num_particles, 2))  # Velocity

    def predict(self, dt: float = 0.1):
        """Predict particle motion"""
        # Add some random motion to each particle
        self.particles[:, 0] += self.particles[:, 2] * dt + np.random.normal(0, 0.1, self.num_particles)
        self.particles[:, 1] += self.particles[:, 3] * dt + np.random.normal(0, 0.1, self.num_particles)
        # Velocity changes slightly
        self.particles[:, 2] += np.random.normal(0, 0.05, self.num_particles)
        self.particles[:, 3] += np.random.normal(0, 0.05, self.num_particles)

    def update(self, measurement: Tuple[float, float], sensor_noise: float = 0.5):
        """Update particle weights based on measurement"""
        # Calculate distance from each particle to measurement
        dx = self.particles[:, 0] - measurement[0]
        dy = self.particles[:, 1] - measurement[1]
        distances = np.sqrt(dx**2 + dy**2)

        # Calculate weights based on how likely this measurement is for each particle
        # Use Gaussian probability
        probabilities = np.exp(-0.5 * (distances**2) / (sensor_noise**2))
        self.weights *= probabilities

        # Normalize weights
        self.weights += 1e-300  # Avoid division by zero
        self.weights /= np.sum(self.weights)

    def resample(self):
        """Resample particles based on their weights"""
        # Systematic resampling
        indices = []
        step = 1.0 / self.num_particles
        start = np.random.uniform(0, step)

        i = 0
        cumulative_sum = self.weights[0]
        for j in range(self.num_particles):
            threshold = start + j * step
            while threshold > cumulative_sum:
                i += 1
                cumulative_sum += self.weights[i]
            indices.append(i)

        # Resample particles
        self.particles = self.particles[indices]
        self.weights.fill(1.0 / self.num_particles)

    def estimate(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """Get position and velocity estimate"""
        # Weighted average of particles
        pos_x = np.average(self.particles[:, 0], weights=self.weights)
        pos_y = np.average(self.particles[:, 1], weights=self.weights)
        vel_x = np.average(self.particles[:, 2], weights=self.weights)
        vel_y = np.average(self.particles[:, 3], weights=self.weights)

        return (float(pos_x), float(pos_y)), (float(vel_x), float(vel_y))

class MultiSensorFusion:
    """Advanced sensor fusion combining multiple approaches"""

    def __init__(self):
        self.kalman_fusion = SensorFusionSystem()
        self.particle_filter = ParticleFilter(num_particles=500)

    def process_sensor_data(self, camera_data: Optional[Tuple[float, float]],
                          lidar_data: Optional[Tuple[float, float]],
                          dt: float = 0.1) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
        """Process data from multiple sensors"""
        fused_pos, fused_vel = None, None
        uncertainty = 0

        if camera_data and lidar_data:
            # Both sensors available - use Kalman fusion
            fused_pos, fused_vel = self.kalman_fusion.process_fused_data(camera_data, lidar_data, dt)
            uncertainty = self.kalman_fusion.get_uncertainty()
        elif camera_data:
            # Only camera available
            fused_pos, fused_vel = self.kalman_fusion.process_camera_data(camera_data, dt)
            uncertainty = self.kalman_fusion.get_uncertainty()
        elif lidar_data:
            # Only LIDAR available
            fused_pos, fused_vel = self.kalman_fusion.process_lidar_data(lidar_data, dt)
            uncertainty = self.kalman_fusion.get_uncertainty()

        # Update particle filter as well if we have data
        if camera_data or lidar_data:
            measurement = camera_data if camera_data else lidar_data
            self.particle_filter.predict(dt)
            self.particle_filter.update(measurement)
            self.particle_filter.resample()

        return fused_pos, fused_vel, uncertainty

# Example usage
if __name__ == "__main__":
    print("Sensor Fusion Example")
    print("Initializing sensor fusion system...")

    # Initialize fusion system
    fusion_system = MultiSensorFusion()

    # Simulate sensor readings over time
    true_positions = []
    camera_measurements = []
    lidar_measurements = []
    fused_positions = []

    # Simulate movement in a square pattern
    for t in range(20):
        dt = 0.1
        time = t * dt

        # True position (moving in a square pattern)
        if time < 5:
            true_x, true_y = 2 * time, 0
        elif time < 10:
            true_x, true_y = 10, 2 * (time - 5)
        elif time < 15:
            true_x, true_y = 10 - 2 * (time - 10), 10
        else:
            true_x, true_y = 0, 10 - 2 * (time - 15)

        true_positions.append((true_x, true_y))

        # Simulate sensor measurements with noise
        camera_x = true_x + np.random.normal(0, 0.3)  # Camera is noisier
        camera_y = true_y + np.random.normal(0, 0.3)
        camera_measurements.append((camera_x, camera_y))

        lidar_x = true_x + np.random.normal(0, 0.1)   # LIDAR is more accurate
        lidar_y = true_y + np.random.normal(0, 0.1)
        lidar_measurements.append((lidar_x, lidar_y))

        # Process with sensor fusion
        fused_pos, fused_vel, uncertainty = fusion_system.process_sensor_data(
            camera_measurements[-1], lidar_measurements[-1], dt
        )
        fused_positions.append(fused_pos)

        if t % 5 == 0:  # Print every 5th step
            print(f"Time {time:.1f}s: True=({true_x:.2f}, {true_y:.2f}), "
                  f"Camera=({camera_x:.2f}, {camera_y:.2f}), "
                  f"LIDAR=({lidar_x:.2f}, {lidar_y:.2f}), "
                  f"Fused=({fused_pos[0]:.2f}, {fused_pos[1]:.2f}), "
                  f"Uncertainty={uncertainty:.3f}")

    # Calculate accuracy statistics
    if fused_positions:
        errors = []
        for true, fused in zip(true_positions, fused_positions):
            error = math.sqrt((true[0] - fused[0])**2 + (true[1] - fused[1])**2)
            errors.append(error)

        avg_error = sum(errors) / len(errors)
        max_error = max(errors)

        print(f"\nAccuracy Results:")
        print(f"Average fusion error: {avg_error:.3f}")
        print(f"Maximum fusion error: {max_error:.3f}")
        print(f"Final uncertainty estimate: {uncertainty:.3f}")

    print("\nSensor fusion example completed!")