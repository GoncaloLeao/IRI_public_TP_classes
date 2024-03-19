"""
IRI - TP3 - Ex 2
By: Gonçalo Leão
"""
import csv
import math

# https://docs.mrpt.org/reference/latest/tutorial-icp-alignment.html

from controller import Robot, Lidar, LidarPoint, Compass, GPS
import numpy as np
from controllers.icp.registration import ICPSVD
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation


def run_icp_with_inital_tf(fixed_cloud: np.ndarray, lidar_cloud: [LidarPoint],
                           initial_tf: np.ndarray,
                           threshold: float, max_iter: int,
                           plot_iterations: bool = False, plot_error: bool = False) -> (np.ndarray, float):
    # Compute the moving point cloud, by applying the initial transformation to each point of the point cloud captured by the LiDAR
    # TODO
    moving_cloud: np.ndarray

    # Call the ICP algorithm
    final_tf, err_list, _ = ICPSVD(fixed_cloud, moving_cloud, threshold, max_iter,
                                   plot_iterations, plot_error, is3D=False, verbose=False)
    return final_tf, err_list[-1]


def grid_search_icp(fixed_cloud: np.ndarray, lidar_cloud: [LidarPoint]) -> np.ndarray:
    n_xy_divisions: int = 8
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0
    n_theta_divisions: int = 8

    # Find the initial transformation using grid search
    min_initial_error: float = 999999
    best_initial_tf: np.ndarray = np.identity(4)
    # TODO

    print("initial translation", get_translation(best_initial_tf))
    print("initial rotation", get_rotation(best_initial_tf))
    print("initial error = ", min_initial_error)

    # Compute the refined transformation using ICP
    refined_tf, refined_error = ICPSVD(...)  # TODO
    print("refined translation", get_translation(refined_tf))
    print("refined rotation", get_rotation(refined_tf))
    print("refined error = ", refined_error)

    # Compute the resulting robot transformation, which corresponds to chaining the initial and refined transformations
    final_tf: np.ndarray  # TODO
    print("final translation = ", get_translation(final_tf))
    print("final rotation = ", get_rotation(final_tf))

    return final_tf


def main() -> None:
    robot: Robot = Robot()

    custom_maps_filepath: str = '../../../worlds/custom_maps/'
    map_name: str = 'obstacles'
    world_points_filename: str = custom_maps_filepath + map_name + '_points.csv'

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)
    robot.step()

    # Read the ground-truth (correct robot pose)
    gps_readings: [float] = gps.getValues()
    actual_position: (float, float) = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    actual_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

    # Read the fixed point cloud
    fixed_points: [(float, float, float)] = []
    with open(world_points_filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            fixed_points.append([float(row[0]), float(row[1]), 0.0])
    fixed_cloud: np.ndarray = np.asarray(fixed_points)

    tf: np.ndarray = grid_search_icp(fixed_cloud, lidar.getPointCloud())
    draw_real_vs_estimated_localization(fixed_cloud,
                                        actual_position, actual_orientation,
                                        [get_translation(tf)], [get_rotation(tf)])

    while robot.step() != -1:
        pass


if __name__ == '__main__':
    main()
