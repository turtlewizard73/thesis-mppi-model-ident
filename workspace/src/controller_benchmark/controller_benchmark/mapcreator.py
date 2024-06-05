import numpy as np
import cv2
import time
import os
import argparse
import yaml


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
        self.output_dir = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 'maps_generated')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

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

    def create_map(self, obstacles: bool = False, output_filename: str = None) -> str:
        stamp = time.strftime(self.timestamp_format)
        output_filename = output_filename or f'map_{int(self.size)}_{stamp}.png'
        new_map = self.generate_blank_square_png()

        if obstacles is True:
            new_map = self.add_obstacles(new_map)

        cv2.imwrite(os.path.join(self.output_dir, output_filename), new_map)

        map_data = {
            'image': output_filename,
            'resolution': self.resolution,
            'origin': [-self.size/2, -self.size/2, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        yaml_filename = output_filename.replace('.png', '.yaml')
        with open(os.path.join(self.output_dir, yaml_filename), 'w') as file:
            yaml.dump(map_data, file)

        return output_filename


def main():
    parser = argparse.ArgumentParser(description='Map Creator')
    parser.add_argument(
        '-r', '--resolution', type=float, default=0.05, help='Resolution in meters per cell')
    parser.add_argument(
        '-s', '--size', type=int, default=5, help='Size of the map in meters')
    parser.add_argument(
        '-n', '--num_maps', type=int, default=1, help='Number of maps to generate')
    parser.add_argument(
        '-o', '--obstacles', default=False, action='store_true', help='Add obstacles to the map')
    args = parser.parse_args()

    map_creator = MapCreator(resolution=args.resolution, size=args.size, robot_radius=0.2)

    mapname = f'map_{args.size}'
    if args.obstacles is False:
        mapname = f'{mapname}_empty'

    for i in range(args.num_maps):
        map_creator.create_map(obstacles=args.obstacles, output_filename=f'{mapname}_{i}.png')


if __name__ == '__main__':
    main()
