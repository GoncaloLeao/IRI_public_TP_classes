"""
IRI - TP4 - Ex 1
By: Gonçalo Leão
"""
import math

from matplotlib import pyplot as plt

from controller import Robot, Lidar, LidarPoint, Compass, GPS, Keyboard
import numpy as np

from controllers.TP4.occupancy_grid import OccupancyGrid
from controllers.transformations import create_tf_matrix, get_translation
from controllers.utils import cmd_vel, bresenham


class DeterministicOccupancyGrid(OccupancyGrid):
    def __init__(self, origin: (float, float), dimensions: (int, int), resolution: float):
        super().__init__(origin, dimensions, resolution)

        # Initialize the grid
        self.occupancy_grid: np.ndarray  # TODO

    def update_map(self, robot_tf: np.ndarray, lidar_points: [LidarPoint]) -> None:
        # Get the grid coord for the robot pose
        robot_coord: (int, int)  # TODO

        # Get the grid coords for the lidar points
        grid_lidar_coords: [(int, int)] = []
        for point in lidar_points:
            pass  # TODO

        # Set as free the cell of the robot's position
        self.update_cell(...)  # TODO

        # Set as free the cells leading up to the lidar points
        # TODO

        # Set as occupied the cells for the lidar points
        # TODO

    def update_cell(self, coords: (int, int), is_occupied: bool) -> None:
        if self.are_grid_coords_in_bounds(coords):
            # Update the grid cell
            self.occupancy_grid[coords] = ...  # TODO


def main() -> None:
    robot: Robot = Robot()
    timestep: int = 100  # in ms

    kb: Keyboard = Keyboard()
    kb.disable()
    kb.enable(timestep)

    keyboard_linear_vel: float = 0.3
    keyboard_angular_vel: float = 1.5

    map: DeterministicOccupancyGrid = DeterministicOccupancyGrid([0.0, 0.0], [200, 200], 0.01)

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)

    scan_count: int = 0
    while robot.step(timestep) != -1:
        key: int = kb.getKey()
        if key == ord('W'):
            cmd_vel(robot, keyboard_linear_vel, 0)
        elif key == ord('S'):
            cmd_vel(robot, -keyboard_linear_vel, 0)
        elif key == ord('A'):
            cmd_vel(robot, 0, keyboard_angular_vel)
        elif key == ord('D'):
            cmd_vel(robot, 0, -keyboard_angular_vel)
        else:  # Not a movement key
            cmd_vel(robot, 0, 0)
            if key == ord(' '):  # ord('Q'):
                scan_count += 1
                print('scan count: ', scan_count)

                # Read the robot's pose
                gps_readings: [float] = gps.getValues()
                robot_position: (float, float) = (gps_readings[0], gps_readings[1])
                compass_readings: [float] = compass.getValues()
                robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])
                robot_tf: np.ndarray = create_tf_matrix((robot_position[0], robot_position[1], 0.0), robot_orientation)

                # Read the LiDAR and update the map
                map.update_map(robot_tf, lidar.getPointCloud())

                # Show the updated map
                # fig = plt.figure()
                plt.imshow(np.flip(map.occupancy_grid, 0))
                plt.savefig('ex1-scan' + str(scan_count) + '.png')
                # plt.show()


if __name__ == '__main__':
    main()
