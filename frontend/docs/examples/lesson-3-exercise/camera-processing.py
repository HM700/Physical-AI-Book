import numpy as np
import cv2
from typing import List, Tuple, Optional
import math

class CameraProcessor:
    """Basic camera processing for robot perception"""

    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height

    def detect_edges(self, image: np.ndarray) -> np.ndarray:
        """Detect edges in an image using Canny edge detection"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return edges

    def detect_circles(self, image: np.ndarray) -> List[Tuple[int, int, int]]:
        """Detect circles in an image using Hough Circle Transform"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to reduce noise
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect circles
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=50,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100
        )

        results = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                results.append((x, y, r))

        return results

    def detect_rectangles(self, image: np.ndarray) -> List[np.ndarray]:
        """Detect rectangles in an image using contour detection"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        rectangles = []
        for contour in contours:
            # Approximate contour to polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if polygon has 4 vertices
            if len(approx) == 4:
                # Check if it's convex
                if cv2.isContourConvex(approx):
                    rectangles.append(approx)

        return rectangles

    def detect_colors(self, image: np.ndarray, color_range: Tuple[Tuple[int, int, int], Tuple[int, int, int]]) -> np.ndarray:
        """Detect specific color range in an image"""
        lower_bound = np.array(color_range[0], dtype="uint8")
        upper_bound = np.array(color_range[1], dtype="uint8")

        # Create mask for the color range
        mask = cv2.inRange(image, lower_bound, upper_bound)

        # Apply mask to image
        result = cv2.bitwise_and(image, image, mask=mask)

        return result, mask

    def find_object_center(self, image: np.ndarray) -> Optional[Tuple[int, int]]:
        """Find the center of the largest object in the image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Find the largest contour
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate moments to find center
        moments = cv2.moments(largest_contour)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
            return (cx, cy)

        return None

    def calculate_depth_from_disparity(self, left_image: np.ndarray, right_image: np.ndarray, baseline: float = 0.1, focal_length: float = 1000) -> np.ndarray:
        """Calculate depth from stereo images (simplified)"""
        # Convert to grayscale
        left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # Compute disparity using StereoBM
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        # Calculate depth: depth = (baseline * focal_length) / disparity
        # Add small value to avoid division by zero
        depth = (baseline * focal_length) / (disparity + 1e-6)

        # Set invalid values to a large depth
        depth[disparity <= 0] = 1000.0

        return depth

class ObjectDetector:
    """Simple object detector for common shapes"""

    def __init__(self):
        self.processor = CameraProcessor()

    def detect_common_objects(self, image: np.ndarray) -> dict:
        """Detect common objects in an image"""
        results = {}

        # Detect circles
        circles = self.processor.detect_circles(image)
        results['circles'] = circles

        # Detect rectangles
        rectangles = self.processor.detect_rectangles(image)
        results['rectangles'] = rectangles

        # Find object center
        center = self.processor.find_object_center(image)
        results['center'] = center

        return results

def create_sample_image() -> np.ndarray:
    """Create a sample image for testing"""
    # Create a blank image
    img = np.zeros((480, 640, 3), dtype=np.uint8)

    # Add a red circle
    cv2.circle(img, (100, 100), 50, (0, 0, 255), -1)

    # Add a blue rectangle
    cv2.rectangle(img, (200, 200), (300, 300), (255, 0, 0), -1)

    # Add a green triangle
    triangle_points = np.array([[400, 200], [450, 300], [350, 300]], np.int32)
    cv2.fillPoly(img, [triangle_points], (0, 255, 0))

    return img

# Example usage
if __name__ == "__main__":
    print("Camera Processing Example")
    print("Creating sample image with shapes...")

    # Create a sample image
    sample_img = create_sample_image()

    # Initialize processor
    processor = CameraProcessor()

    # Detect objects
    detector = ObjectDetector()
    results = detector.detect_common_objects(sample_img)

    print(f"Detected {len(results['circles'])} circles")
    print(f"Detected {len(results['rectangles'])} rectangles")
    print(f"Object center: {results['center']}")

    # Test color detection (red objects)
    red_range = ((0, 0, 100), (100, 100, 255))
    red_mask, _ = processor.detect_colors(sample_img, red_range)
    print(f"Detected red objects in image")

    # Test edge detection
    edges = processor.detect_edges(sample_img)
    print(f"Edge detection completed, image shape: {edges.shape}")

    print("\nCamera processing examples completed!")