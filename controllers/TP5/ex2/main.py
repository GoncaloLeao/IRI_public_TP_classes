"""
IRI - TP5 - Ex 2
By: Gonçalo Leão
"""
import csv
import math
import random
from typing import Dict

import networkx as nx
import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

from controller import Robot, Compass, GPS
from controllers.TP5.ex1.main import VertexInfo
from controllers.TP5.graph import Graph
from controllers.TP5.metric_graph import MetricGraph
from controllers.TP5.vertex_edge import Vertex
from controllers.utils import is_collision_free_point, is_collision_free_line, move_robot_to


def create_rrt(initial_position: (float, float), final_position: (float, float), obstacle_cloud: np.ndarray) -> (bool, MetricGraph):
    max_iterations: int = 500
    incremental_distance: float = 0.25
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0

    rrt_graph = MetricGraph()

    # Add the initial position vertex
    cur_index: int = 0
    rrt_graph.add_vertex(cur_index, (initial_position[0], initial_position[1]), 'blue')
    cur_index += 1

    # Check if the final vertex is directly accessible.
    # If so, add a vertex for the final vertex, create an edge to it and stop the algorithm.
    last_vertex_dist_to_final: float = math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                  rrt_graph.vertices_info[-1].y - final_position[1])
    if is_collision_free_line(rrt_graph.vertices_info[-1].x, rrt_graph.vertices_info[-1].y,
                              final_position[0], final_position[1],
                              obstacle_cloud):
        # Add the final position vertex
        rrt_graph.add_vertex(cur_index, (final_position[0], final_position[1]), 'blue')
        cur_index += 1

        rrt_graph.add_edge(len(rrt_graph.vertices_info) - 2, len(rrt_graph.vertices_info) - 1, last_vertex_dist_to_final, True)
        return True, rrt_graph

    for k in range(max_iterations):
        while True:
            # Generate a random free position and check that it doesn't collide with the obstacles.
            # If it does, generate a new point.
            # TODO

            # Find the nearest vertex to it in the graph.
            # TODO

            # Define a new position that is in the direction of the random free position
            # TODO

            # Check if the position is accessible from the nearest vertex, if not, skip to the next iteration
            # TODO

            # Add the new vertex to the graph
            # TODO

            # Add an edge between the new vertex and the closest vertex to the graph
            # TODO

            # Check if the last added vertex is directly accessible.
            # If so, add a vertex for the final vertex, create an edge to it and stop the algorithm.
            if is_collision_free_line(rrt_graph.vertices_info[-1].x, rrt_graph.vertices_info[-1].y, final_position[0], final_position[1],
                                      obstacle_cloud):
                # Add the final position vertex
                # TODO
                pass

            break

    return False, rrt_graph


def main() -> None:
    robot: Robot = Robot()

    custom_maps_filepath: str = '../../../worlds/custom_maps/'
    map_name: str = 'mapW'
    obstacle_points_filename: str = custom_maps_filepath + map_name + '_points.csv'
    final_position: (float, float) = (1.8, 1.8)

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    compass: Compass = robot.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = robot.getDevice('gps')
    gps.enable(timestep)
    robot.step()

    # Read the robot's initial position
    gps_readings: [float] = gps.getValues()
    robot_position: (float, float) = (gps_readings[0], gps_readings[1])

    # Read the obstacles point cloud
    obstacle_points: [(float, float, float)] = []
    with open(obstacle_points_filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            obstacle_points.append([float(row[0]), float(row[1]), 0.0])
    obstacle_cloud: np.ndarray = np.asarray(obstacle_points)

    # Check that the final position is in the free space
    if not is_collision_free_point(final_position[0], final_position[1], obstacle_cloud):
        print("Final position ", final_position, " is colliding with an obstacle")
        return

    # Create the graph and find the path
    found_path, rrt_graph = create_rrt(robot_position, final_position, obstacle_cloud)
    if not found_path:
        print("No path found")
        return
    vertex_positions: Dict[int, (float, float)] = nx.get_node_attributes(rrt_graph.visual_graph, 'pos')
    path: [Vertex] = rrt_graph.get_path(0, len(rrt_graph.vertex_set) - 1)

    # Mark with a new color the unvisited vertices and the ones in the path
    new_vertex_colors: dict = {}
    for vertex in path:
        new_vertex_colors[vertex.id] = "green"
    nx.set_node_attributes(rrt_graph.visual_graph, new_vertex_colors, 'color')

    # Create the PolyCollection for the obstacles, for the plot
    pat: [patches.Rectangle] = []
    for point in obstacle_cloud:
        pat.append(Rectangle((point[0], point[1]), 0.001, 0.001,
                             linewidth=1, edgecolor='black', facecolor='none'))

    # Show the graph
    fig, ax = plt.subplots()
    nx.draw_networkx(rrt_graph.visual_graph, vertex_positions, node_size=10,
                     node_color=nx.get_node_attributes(rrt_graph.visual_graph, 'color').values(),
                     with_labels=True)
    col: PatchCollection = PatchCollection(pat)
    col.set_edgecolor('black')
    col.set_linewidth(1)
    ax.add_collection(col)
    plt.show()

    # Move the robot through the path
    for vertex in path:
        # Read the ground-truth (correct robot pose)
        robot.step()
        gps_readings: [float] = gps.getValues()
        robot_position: (float, float) = (gps_readings[0], gps_readings[1])
        compass_readings: [float] = compass.getValues()
        robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

        move_robot_to(robot, robot_position, robot_orientation, vertex_positions[vertex.id], 0.1, math.pi)


if __name__ == '__main__':
    main()
