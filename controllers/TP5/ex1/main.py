"""
IRI - TP5 - Ex 1
By: Gonçalo Leão
"""
import csv
import math
import time
from typing import Union, Dict

from matplotlib import pyplot as plt, patches
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

from controller import Robot, Compass, GPS
import numpy as np

from controllers.TP5.metric_graph import VertexInfo, MetricGraph
from controllers.TP5.vertex_edge import Vertex
from controllers.utils import move_robot_to, is_collision_free_line, is_collision_free_point
import networkx as nx


def find_closest_accessible_vertex(x: float, y: float, vertices_info: [VertexInfo], obstacle_cloud: np.ndarray) -> Union[VertexInfo, None]:
    closest_accessible_vertex: Union[VertexInfo, None] = None
    closest_distance: float = math.inf
    for vertex_info in vertices_info:
        distance: float = math.hypot(x - vertex_info.x, y - vertex_info.y)
        if distance >= closest_distance:
            continue
        if is_collision_free_line(x, y, vertex_info.x, vertex_info.y, obstacle_cloud):
            closest_accessible_vertex = vertex_info
            closest_distance = distance
    return closest_accessible_vertex


def create_grid_graph(initial_pos: (float, float), final_pos: (float, float), obstacle_cloud: np.ndarray) -> MetricGraph:
    grid_graph = MetricGraph()

    # Add the grid vertices
    n_xy_divisions: int = 16
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0

    min_x_offset: float = max_x / n_xy_divisions / 2.0
    x_increment: float = max_x / n_xy_divisions
    min_y_offset: float = max_y / n_xy_divisions / 2.0
    y_increment: float = max_y / n_xy_divisions
    cur_index: int = 0
    for i in range(n_xy_divisions):
        x: float = min_x_offset + i * x_increment
        for j in range(n_xy_divisions):
            y: float = min_y_offset + j * y_increment
            # Add a vertex to point (x,y)
            # TODO
            cur_index += 1

    # Add the initial and final vertices
    additional_points: [(float, float)] = [initial_pos, final_pos]
    for point in additional_points:
        grid_graph.add_vertex(cur_index, point, 'green')
        cur_index += 1

    # Connect the initial point to the closest point in the grid using edges
    closest_vertex_to_initial: Union[VertexInfo, None] = find_closest_accessible_vertex(initial_pos[0], initial_pos[1], grid_graph.vertices_info[:-2], obstacle_cloud)
    if closest_vertex_to_initial is None:
        print("Initial position ", initial_pos, " is not accessible by any point in the grid.")
        return grid_graph
    grid_graph.add_edge(len(grid_graph.vertices_info) - 2, closest_vertex_to_initial.id,
                        math.hypot(initial_pos[0] - closest_vertex_to_initial.x, initial_pos[1] - closest_vertex_to_initial.y))

    # Connect the final point to the closest point in the grid using edges
    # TODO

    # Add the grid edges
    cur_index = 0
    for i in range(n_xy_divisions):
        for j in range(n_xy_divisions):
            # Add an edge with the left neighbor
            if i > 0 and is_collision_free_line(grid_graph.vertices_info[cur_index].x, grid_graph.vertices_info[cur_index].y, grid_graph.vertices_info[cur_index - n_xy_divisions].x, grid_graph.vertices_info[cur_index - n_xy_divisions].y, obstacle_cloud):
                grid_graph.add_edge(cur_index, cur_index - n_xy_divisions, x_increment)
                grid_graph.add_edge(cur_index - n_xy_divisions, cur_index, x_increment)
            # Add an edge with the top neighbor
            if j > 0 and is_collision_free_line(grid_graph.vertices_info[cur_index].x, grid_graph.vertices_info[cur_index].y, grid_graph.vertices_info[cur_index - 1].x, grid_graph.vertices_info[cur_index - 1].y, obstacle_cloud):
                # TODO
                pass
            cur_index += 1

    return grid_graph


def main() -> None:
    robot: Robot = Robot()

    custom_maps_filepath: str = '../../../worlds/custom_maps/'
    map_name: str = 'obstacles'
    obstacle_points_filename: str = custom_maps_filepath + map_name + '_points.csv'
    final_position: (float, float) = (1.76, 1.76)

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

    # Create the graph and find the shortest path
    grid_graph = create_grid_graph(robot_position, final_position, obstacle_cloud)
    vertex_positions: Dict[int, (float, float)] = nx.get_node_attributes(grid_graph.visual_graph, 'pos')
    vertex_distances: Dict[int, float] = {}
    for (id, position) in vertex_positions.items():
        vertex_distances[id] = math.hypot(position[0] - final_position[0],
                                          position[1] - final_position[1])

    def dist_func(v: Vertex):
        return vertex_distances[v.id]
    start = time.time()
    grid_graph.a_star(len(grid_graph.vertex_set) - 2, dist_func, len(grid_graph.vertex_set) - 1)
    end = time.time()
    print("Elapsed time for A* : ", end - start, " seconds")
    path: [Vertex] = grid_graph.get_path(len(grid_graph.vertex_set) - 2, len(grid_graph.vertex_set) - 1)

    # Mark with a new color the unvisited vertices and the ones in the path
    new_vertex_colors: dict = {}
    for vertex in path:
        new_vertex_colors[vertex.id] = "green"
    for vertex in grid_graph.vertex_set:
        if vertex.dist == math.inf:
            new_vertex_colors[vertex.id] = "red"
    nx.set_node_attributes(grid_graph.visual_graph, new_vertex_colors, 'color')

    # Create the PolyCollection for the obstacles, for the plot
    pat: [patches.Rectangle] = []
    for point in obstacle_cloud:
        pat.append(Rectangle((point[0], point[1]), 0.001, 0.001,
                             linewidth=1, edgecolor='black', facecolor='none'))

    # Show the graph
    fig, ax = plt.subplots()
    nx.draw_networkx(grid_graph.visual_graph, vertex_positions, node_size=10,
                     node_color=nx.get_node_attributes(grid_graph.visual_graph, 'color').values(),
                     with_labels=False)
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
