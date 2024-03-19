"""
IRI - TP3 - Ex 1
By: Gonçalo Leão
"""
import math

import numpy as np
from matplotlib import pyplot as plt
from numpy import array
from skimage.measure import LineModelND, ransac

from controller import Robot, LidarPoint, Lidar, Compass, GPS
from controllers.localization_utils import draw_real_vs_estimated_localization
from controllers.transformations import create_tf_matrix, get_translation, get_rotation


def find_possible_poses(robot_tf, readings: [LidarPoint], min_x: float, max_x: float, min_y: float, max_y: float) -> ([(float, float)], [float]):
    # Get the transformation for the robot relative to the corner (TCR)
    corner_robot_tf: np.ndarray = find_corner_transformation(readings)

    # Get the list of transformations for each corner relative to the origin (list of TOC)
    corner_tfs: [np.ndarray] = [
        # TODO
    ]

    # Get the list of estimated transformations = list of estimated transformations for the robot relative to the origin (TOR)
    estimated_translations: [(float, float)] = []
    estimated_rotations: [float] = []
    for corner_tf in corner_tfs:
        # TOR = TOC * TCR
        # TODO
        pass

    return estimated_translations, estimated_rotations


def find_corner_transformation(readings: [LidarPoint]) -> np.ndarray:
    # Structure the readings for input to RANSAC
    data = array([[point.x, point.y] for point in readings if math.isfinite(point.x) and math.isfinite(point.y)])

    # Find the first line
    model1, inliers_bools1 = ransac(data, LineModelND, min_samples=2,
                                    residual_threshold=0.005, max_trials=10000)
    # Retrieve the outliers
    outliers1: array  # TODO

    # Find the second line
    assert len(outliers1) >= 2, "Cannot detect the second wall!!"
    model2, inliers_bools2 = ransac(...)  # TODO

    # Retrieve the outliers
    outliers2: array  # TODO
    print("Num outliers: ", len(outliers2))

    # Compute the inliers
    inliers1: array  # TODO
    inliers2: array  # TODO

    # Draw the walls
    draw_walls(data, model1, inliers1, model2, inliers2, outliers2)

    # Find the corner coordinates and angle
    corner_pos: (float, float) = line_line_intersection(model1.params[0], model1.params[1],
                                                        model2.params[0], model2.params[1])
    corner_angle: float = line_line_angle(model1.params[1], model2.params[1])

    return create_tf_matrix([corner_pos[0], corner_pos[1], 0.0], corner_angle)


def draw_walls(data: array, line1: LineModelND, inliers1: array, line2: LineModelND, inliers2: array,
               outliers: array) -> None:
    # Unpack the points into separate lists of x and y coordinates
    data_x, data_y = zip(*data)
    inliers1_x, inliers1_y = zip(*inliers1)
    inliers2_x, inliers2_y = zip(*inliers2)
    outliers_x = []
    outliers_y = []
    if len(outliers) > 0:
        outliers_x, outliers_y = zip(*outliers)

    # Plot the data points with different colors
    plt.scatter(inliers1_x, inliers1_y, color='blue', label='Inliers1', marker='x')
    plt.scatter(inliers2_x, inliers2_y, color='green', label='Inliers2', marker='x')
    plt.scatter(outliers_x, outliers_y, color='red', label='Outliers', marker='x')

    # Add the computed lines to the plot
    line1_x_values, line1_y_values = get_model_line_points(inliers1_x, inliers1_y, line1.params[0], line1.params[1])
    plt.plot(line1_x_values, line1_y_values, color='blue', label='Line1')
    line2_x_values, line2_y_values = get_model_line_points(inliers2_x, inliers2_y, line2.params[0], line2.params[1])
    plt.plot(line2_x_values, line2_y_values, color='green', label='Line2')

    # Add the robot to (0,0)
    arrow_length: float = 0.05
    arrow_start = (-arrow_length / 2.0, 0)
    arrow_direction = (arrow_length, 0)
    plt.arrow(*arrow_start, *arrow_direction,
              color='black', label='Robot', width=arrow_length / 4.0,
              length_includes_head=True, head_length=arrow_length)

    # Add labels and legend
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('LiDAR points and lines relative to the robot')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.gca().set_aspect('equal')
    plt.xlim(min(*data_x, 0) - 0.1, max(*data_x, 0) + 0.1)
    plt.ylim(min(*data_y, 0) - 0.1, max(*data_y, 0) + 0.1)

    # Show the plot
    plt.grid(True)
    plt.show()


def get_model_line_points(points_x_coords: [float], points_y_coords: [float], origin: (float, float),
                          direction: (float, float)) -> (np.ndarray, np.ndarray):
    if direction[0] == 0:  # vertical line
        line_min_y: float = min(points_y_coords)
        line_max_y: float = max(points_y_coords)
        line_y_values: np.ndarray[np.dtype:float] = np.linspace(line_min_y - 0.1, line_max_y + 0.1, 100)

        line_x_values: np.ndarray[np.dtype:float] = np.full(shape=len(line_y_values), fill_value=points_x_coords[0],
                                                            dtype=np.float64)
    else:
        line_min_x: float = min(points_x_coords)
        line_max_x: float = max(points_x_coords)
        line_x_values: np.ndarray[np.dtype:float] = np.linspace(line_min_x - 0.1, line_max_x + 0.1, 100)

        slope: float = direction[1] / direction[0]
        intercept: float = origin[1] - slope * origin[0]
        line_y_values: np.ndarray[np.dtype:float] = slope * line_x_values + intercept
    return line_x_values, line_y_values


def line_line_intersection(origin1: array([float, float]), direction1: array([float, float]),
                           origin2: array([float, float]), direction2: array([float, float])) -> (float, float):
    x1: float = origin1[0]
    y1: float = origin1[1]
    x2: float = origin1[0] + direction1[0]
    y2: float = origin1[1] + direction1[1]
    x3: float = origin2[0]
    y3: float = origin2[1]
    x4: float = origin2[0] + direction2[0]
    y4: float = origin2[1] + direction2[1]

    denom: float = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    x: float = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    y: float = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

    return x, y


def line_line_angle(direction1: array([float, float]), direction2: array([float, float])) -> float:
    # TODO
    return 0.0


def main() -> None:
    robot: Robot = Robot()

    min_x: float = -0.5
    min_y: float = -0.5
    max_x: float = 0.5
    max_y: float = 0.5

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(int(robot.getBasicTimeStep()))
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

    robot_tf: np.ndarray = create_tf_matrix((actual_position[0], actual_position[1], 0.0), actual_orientation)

    # Draw a point cloud for a square map
    num_divisions: int = 100
    fixed_points: [(float, float, float)] = []
    for i in range(num_divisions):
        x: float = min_x + (max_x - min_x) * (i / float(num_divisions - 1))
        fixed_points.append([x, min_y, 0.0])
        fixed_points.append([x, max_y, 0.0])

        y: float = min_y + (max_y - min_y) * (i / float(num_divisions - 1))
        fixed_points.append([min_x, y, 0.0])
        fixed_points.append([max_x, y, 0.0])
    fixed_cloud: np.ndarray = np.asarray(fixed_points)

    estimated_translations, estimated_rotations = find_possible_poses(robot_tf, lidar.getPointCloud(), min_x, max_x, min_y, max_y)
    draw_real_vs_estimated_localization(fixed_cloud,
                                        actual_position, actual_orientation,
                                        estimated_translations, estimated_rotations)

    while robot.step() != -1:
        pass


if __name__ == '__main__':
    main()
