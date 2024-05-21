import numpy as np
import cv2
import time


class MapCreator:
    def __init__(
            self, resolution: float, size: int, robot_radius: float,
            timestamp_format: str = '%Y-%m-%d-%H-%M-%S') -> None:
        if size < robot_radius * 10:
            raise ValueError("Map size must be bigger, relative to the robot radius.")
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.size = size
        self.timestamp_format = timestamp_format

    def generate_blank_square_png(self) -> np.ndarray:
        num_pixels = int(self.size / self.resolution)
        image = np.ones((num_pixels, num_pixels, 3), dtype=np.uint8) * 255
        return image

    def add_obstacles(self, image: np.ndarray) -> np.ndarray:
        if self.size <= 5:
            min_obstacle_size = self.robot_radius
            max_obstacle_size = self.robot_radius * 2
            num_obstacles = 10
        elif self.size <= 10:
            min_obstacle_size = self.robot_radius * 2
            max_obstacle_size = self.robot_radius * 5
            num_obstacles = 10
        else:
            min_obstacle_size = self.robot_radius * 5
            max_obstacle_size = self.robot_radius * 10
            num_obstacles = 15

        for _ in range(num_obstacles):
            obstacle_size = np.random.uniform(min_obstacle_size, max_obstacle_size) / self.resolution
            obstacle_x = np.random.randint(0, image.shape[1])
            obstacle_y = np.random.randint(0, image.shape[0])

            # randomize to generate circle or rectangle
            if np.random.randint(0, 2) == 0:
                # Draw a filled rectangle as an obstacle
                cv2.rectangle(
                    image,
                    (obstacle_x - int(obstacle_size), obstacle_y - int(obstacle_size)),
                    (obstacle_x + int(obstacle_size), obstacle_y + int(obstacle_size)),
                    (0, 0, 0),
                    -1
                )
            else:
                # Draw a filled circle as an obstacle
                cv2.circle(image, (obstacle_x, obstacle_y), int(obstacle_size), (0, 0, 0), -1)

        return image

    def create_map(self, obstacles: bool = False, output_filename: str = None) -> None:
        stamp = time.strftime(self.timestamp_format)
        output_filename = output_filename or f"blank_square_{int(self.size)}_{stamp}.png"
        new_map = self.generate_blank_square_png()

        if obstacles is True:
            new_map = self.add_obstacles(new_map)

        cv2.imwrite(output_filename, new_map)

        return output_filename

# Example usage
resolution = 0.05  # 1 m per cell
size = 5  # 1 meter
map_creator = MapCreator(resolution=resolution, size=size, robot_radius=0.2)
for i in range(10):
    map_creator.create_map(obstacles=True, output_filename=f'map_{i}.png')