"""
IRI - TP5 - Ex 3
By: Gonçalo Leão
"""
import csv
import math

import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

from controller import Lidar, LidarPoint, Compass, GPS, Supervisor
from controllers.utils import move_robot_to, warp_robot


def compute_attractive_force(robot_position: np.ndarray, final_position: np.ndarray) -> np.ndarray:
    # TODO
    return

def compute_repulsive_force(lidar_cloud: [LidarPoint]) -> np.ndarray:
    d_max: float = 0.2
    window_size: int = 5  # must be odd

    # Find the local minima
    local_minima: [LidarPoint] = []
    # Add copies of the first readings to the end of the list to make comparisons more straightforward
    n_readings: int = len(lidar_cloud)
    for i in range(window_size // 2):
        lidar_cloud.append(lidar_cloud[i])

    for i in range(window_size // 2, n_readings + (window_size // 2)):
        # Determine if the reading is below the max repulsion distance. If not, skip the iteration.
        # TODO
        pass

        # Test if the reading is a local minimum
        # TODO
        pass

    print("n minima = ", len(local_minima))
    print("minima = ", [(point.x, point.y) for point in local_minima])

    # Compute the resulting force
    force: np.ndarray = np.array([0.0, 0.0])
    for point in local_minima:
        # TODO
        pass

    return force


def compute_resulting_force(robot_position: np.ndarray, final_position: np.ndarray, lidar_cloud: [LidarPoint]) -> np.ndarray:
    alpha: float = 1.0
    beta: float = 0.25  # 0.025

    # TODO
    return

def draw_quiver_plots(supervisor: Supervisor, final_position: (float, float), obstacle_cloud: np.ndarray):
    lidar: Lidar = supervisor.getDevice('lidar')
    gps: GPS = supervisor.getDevice('gps')

    n_xy_divisions: int = 42
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0

    min_x: float = max_x / n_xy_divisions / 2.0
    x_increment: float = max_x / n_xy_divisions
    min_y: float = max_y / n_xy_divisions / 2.0
    y_increment: float = max_y / n_xy_divisions

    x_values = np.arange(min_x, max_x, x_increment)
    y_values = np.arange(min_y, max_y, y_increment)

    X, Y = np.meshgrid(x_values, y_values)
    afx = np.zeros_like(X)
    afy = np.zeros_like(Y)
    rfx = np.zeros_like(X)
    rfy = np.zeros_like(Y)
    fx = np.zeros_like(X)
    fy = np.zeros_like(Y)
    for i in range(len(x_values)):
        for j in range(len(y_values)):
            x: float = X[i][j]
            y: float = Y[i][j]
            warp_robot(supervisor, "EPUCK", (x, y))
            supervisor.step()
            gps_readings = gps.getValues()
            robot_position = (gps_readings[0], gps_readings[1])

            attractive_force: np.ndarray = compute_attractive_force(np.array(robot_position), np.array(final_position))
            afx[i][j] = attractive_force[0]
            afy[i][j] = -attractive_force[1]

            repulsive_force: np.ndarray = compute_repulsive_force(lidar.getPointCloud())
            rfx[i][j] = repulsive_force[0]
            rfy[i][j] = -repulsive_force[1]

            resulting_force: np.ndarray = compute_resulting_force(np.array(robot_position), np.array(final_position), lidar.getPointCloud())
            fx[i][j] = resulting_force[0]
            fy[i][j] = -resulting_force[1]

    # Create the PolyCollection for the obstacles, for the plots
    pat: [patches.Rectangle] = []
    for point in obstacle_cloud:
        pat.append(Rectangle((point[0], point[1]), 0.001, 0.001,
                             linewidth=1, edgecolor='black', facecolor='none'))

    # Draw the quiver plots
    fig, ax = plt.subplots()
    ax.quiver(X, Y, afx, afy)
    col: PatchCollection = PatchCollection(pat)
    col.set_edgecolor('black')
    col.set_linewidth(1)
    ax.add_collection(col)
    ax.set_title('Attractive forces')
    plt.show()

    fig, ax = plt.subplots()
    ax.quiver(X, Y, rfx, rfy)
    col: PatchCollection = PatchCollection(pat)
    col.set_edgecolor('black')
    col.set_linewidth(1)
    ax.add_collection(col)
    ax.set_title('Repulsive forces')
    plt.show()

    fig, ax = plt.subplots()
    ax.quiver(X, Y, fx, fy)
    col: PatchCollection = PatchCollection(pat)
    col.set_edgecolor('black')
    col.set_linewidth(1)
    ax.add_collection(col)
    ax.set_title('Resulting force field')
    plt.show()


def main() -> None:
    supervisor: Supervisor = Supervisor()

    custom_maps_filepath: str = '../../../worlds/custom_maps/'
    map_name: str = 'dots'
    obstacle_points_filename: str = custom_maps_filepath + map_name + '_points.csv'
    final_position: (float, float) = (1.8, 1.8)
    max_distance_to_goal: float = 0.01

    timestep: int = int(supervisor.getBasicTimeStep())  # in ms

    lidar: Lidar = supervisor.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = supervisor.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = supervisor.getDevice('gps')
    gps.enable(timestep)
    supervisor.step()

    # Read the robot's initial pose
    gps_readings: [float] = gps.getValues()
    robot_position: (float, float) = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

    # Read the obstacles point cloud
    obstacle_points: [(float, float, float)] = []
    with open(obstacle_points_filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            obstacle_points.append([float(row[0]), float(row[1]), 0.0])
    obstacle_cloud: np.ndarray = np.asarray(obstacle_points)

    # Draw the quiver plot
    draw_quiver_plots(supervisor, final_position, obstacle_cloud)

    # Move the robot back to the initial position
    warp_robot(supervisor, "EPUCK", robot_position)

    while math.hypot(robot_position[0] - final_position[0], robot_position[1] - final_position[1]) > max_distance_to_goal:
        supervisor.step()
        resulting_force: np.ndarray = compute_resulting_force(np.array(robot_position), np.array(final_position), lidar.getPointCloud())
        force_norm = np.linalg.norm(resulting_force)
        if force_norm <= max_distance_to_goal:
            break  # End the run because the resulting force is too low
        step_distance: float = 0.01
        new_position: (float, float) = (robot_position[0] + step_distance*resulting_force[0]/force_norm,
                                        robot_position[1] + step_distance*resulting_force[1]/force_norm)
        move_robot_to(supervisor, robot_position, robot_orientation, new_position, 0.1, math.pi)

        gps_readings = gps.getValues()
        robot_position = (gps_readings[0], gps_readings[1])
        compass_readings = compass.getValues()
        robot_orientation = math.atan2(compass_readings[0], compass_readings[1])


if __name__ == '__main__':
    main()
