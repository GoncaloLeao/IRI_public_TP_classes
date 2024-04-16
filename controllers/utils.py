"""
Some useful functions, namely for Webots and for working with the epuck robot (https://cyberbotics.com/doc/guide/epuck?version=R2021a).
By: Gonçalo Leão
"""
import math
from typing import List

import numpy as np

from controller import Robot, Motor, Supervisor, Node, Field
from controller.device import Device


# Prints the type of all the devices in a scene with a single robot.
def print_devices() -> None:
    supervisor: Supervisor = Supervisor()
    num_devices: int = supervisor.getNumberOfDevices()
    for i in range(num_devices):
        device: Device = supervisor.getDeviceByIndex(i)
        print(device.getName(), '   - NodeType:',
              list(Node.__dict__.keys())[list(Node.__dict__.values()).index(device.getNodeType())])


# This function uses odometry math to translate the linear and angular velocities
# to the left and right motor speeds.
# Note: the robot may take some time to reach the target speeds, since the motors
# can't instantly start rotating at the target motor speeds.
# Made for the epuck robot.
# https://cyberbotics.com/doc/guide/epuck?version=R2021a
AXLE_LENGTH: float = 0.057  # obtained with manual calibration. It should be 0.052 m according to the documentation.
WHEEL_RADIUS: float = 0.0205
MAX_SPEED: float = 6.28

# tangential/linear speed in m/s.
# tangential speed = angular speed * wheel radius
TANGENTIAL_SPEED: float = MAX_SPEED * WHEEL_RADIUS

# Speed of robot to spinning in place (in cycles per second)
# 1 cycle = 360 degrees.
# Robot rotational speed = tangensial speed / (phi * axle length)
# note: axle length is distance between wheels
# Robot rotational speed = 0.12874 / (phi*0.052) = 0.787744755
ROBOT_ROTATIONAL_SPEED: float = TANGENTIAL_SPEED / (math.pi * AXLE_LENGTH)


def cmd_vel(robot: Robot, linear_vel: float, angular_vel: float) -> None:
    r_omega: float = (linear_vel + angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS
    l_omega: float = (linear_vel - angular_vel * AXLE_LENGTH / 2) / WHEEL_RADIUS

    # Get a handler to the motors and set target position to infinity (speed control)
    left_motor: Motor = robot.getDevice('left wheel motor')
    right_motor: Motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    # Set up the motor speeds.
    left_motor.setVelocity(l_omega)
    right_motor.setVelocity(r_omega)


def move_forward(robot: Robot, distance: float, linear_vel: float) -> None:
    duration: float = distance / abs(linear_vel)
    cmd_vel(robot, linear_vel, 0)
    robot.step(int(1000 * duration))


# Alternative solution
def move_forward2(robot: Robot, distance: float, linear_vel: float) -> None:
    duration: float = distance / abs(linear_vel)
    start_time: float = robot.getTime()
    cmd_vel(robot, linear_vel, 0)
    while robot.getTime() < start_time + duration:
        robot.step()


def rotate(robot: Robot, theta: float, angular_vel: float) -> None:
    angular_vel = abs(angular_vel)
    duration: float = abs(theta) / angular_vel
    cmd_vel(robot, 0, angular_vel if theta > 0 else -angular_vel)
    robot.step(int(1000 * duration))


# Alternative solution
def rotate2(robot: Robot, theta: float, angular_vel: float) -> None:
    angular_vel = abs(angular_vel)
    duration: float = abs(theta) / angular_vel
    start_time: float = robot.getTime()
    cmd_vel(robot, 0, angular_vel if theta > 0 else -angular_vel)
    while robot.getTime() < start_time + duration:
        robot.step()


def move_robot_to(robot: Robot, robot_position: (float, float), robot_orientation: float,
                  destination_position: (float, float),
                  linear_vel: float, angular_vel: float) -> None:
    desired_orientation: float = math.atan2(destination_position[1] - robot_position[1],
                                            destination_position[0] - robot_position[0])
    desired_theta: float = (((desired_orientation - robot_orientation) + math.pi) % (2 * math.pi)) - math.pi
    rotate(robot, desired_theta, angular_vel)

    desired_distance: float = math.hypot(destination_position[0] - robot_position[0],
                                         destination_position[1] - robot_position[1])
    move_forward(robot, desired_distance, linear_vel)


def warp_robot(supervisor: Supervisor, robot_def_name: str, new_position: (float, float)) -> None:
    robot_node = supervisor.getFromDef(robot_def_name)
    trans_field: Field = robot_node.getField("translation")
    translation: List[float] = [new_position[0], new_position[1], 0]
    trans_field.setSFVec3f(translation)
    robot_node.resetPhysics()


def bresenham(initial_point: (int, int), final_point: (int, int)) -> [(int, int)]:
    if abs(final_point[1] - initial_point[1]) < abs(final_point[0] - initial_point[0]):
        if initial_point[0] > final_point[0]:
            return bresenham_low_slope_line(final_point, initial_point)
        else:
            return bresenham_low_slope_line(initial_point, final_point)
    else:
        if initial_point[1] > final_point[1]:
            return bresenham_high_slope_line(final_point, initial_point)
        else:
            return bresenham_high_slope_line(initial_point, final_point)


def bresenham_low_slope_line(initial_point: (int, int), final_point: (int, int)) -> [(int, int)]:
    points: [(int, int)] = []
    dx = final_point[0] - initial_point[0]
    dy = final_point[1] - initial_point[1]
    yi = 1
    if dy < 0:
        yi = -1
        dy = -dy
    D = (2 * dy) - dx
    y = initial_point[1]
    for x in range(initial_point[0], final_point[0] + 1):
        points.append((x, y))
        if D > 0:
            y = y + yi
            D = D + (2 * (dy - dx))
        else:
            D = D + 2 * dy
    return points


def bresenham_high_slope_line(initial_point: (int, int), final_point: (int, int)) -> [(int, int)]:
    points: [(int, int)] = []
    dx = final_point[0] - initial_point[0]
    dy = final_point[1] - initial_point[1]
    xi = 1
    if dx < 0:
        xi = -1
        dx = -dx
    D = (2 * dx) - dy
    x = initial_point[0]
    for y in range(initial_point[1], final_point[1] + 1):
        points.append((x, y))
        if D > 0:
            x = x + xi
            D = D + (2 * (dx - dy))
        else:
            D = D + 2 * dx
    return points


def bresenham_extended(initial_point: (int, int), final_point: (int, int), min_coords: (int, int),
                       max_coords: (int, int)) -> [(int, int)]:
    points: [(int, int)] = []
    x1, y1 = initial_point
    x2, y2 = final_point

    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1

    err = dx - dy

    x = x1
    y = y1
    while True:
        if not (min_coords[0] <= x <= max_coords[0] and min_coords[1] <= y <= max_coords[1]):
            break
        points.append((x, y))

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points


# Based on: https://www.jeffreythompson.org/collision-detection/line-rect.php
# Tests whether the segments (x1,y1)-(x2,y2) and (x3,y3)-(x4,y4) collide.
def collides_segment_segment(x1: float, y1: float, x2: float, y2: float, x3: float, y3: float, x4: float,
                             y4: float) -> bool:
    denom: float = ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1))
    if denom == 0:
        return False

    uA: float = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    uB: float = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

    if 0 <= uA <= 1 and 0 <= uB <= 1:
        return True

    return False


def collides_point_rectangle(x: float, y: float, rx: float, ry: float, w: float, h: float) -> bool:
    return rx <= x <= rx + w and ry <= y <= ry + h


# Tests whether the segment (x1,y1)-(x2,y2) and the axis aligned rectangle with bottom-left corner (rx,ry) and dimensions w x h collide.
def collides_segment_rectangle(x1: float, y1: float, x2: float, y2: float, rx: float, ry: float, w: float,
                               h: float) -> bool:
    # Check if the segment collides with one of the rectangle's sides
    if collides_segment_segment(x1, y1, x2, y2, rx, ry, rx, ry + h):  # left side
        return True
    if collides_segment_segment(x1, y1, x2, y2, rx + w, ry, rx + w, ry + h):  # right side
        return True
    if collides_segment_segment(x1, y1, x2, y2, rx, ry, rx + w, ry):  # top side
        return True
    if collides_segment_segment(x1, y1, x2, y2, rx, ry + h, rx + w, ry + h):  # bottom side
        return True

    # This leaves to cases: the segment is either entirely inside or outside the rectangle.
    # So one can test if either (any) of the segment endpoints is inside the rectangle
    return collides_point_rectangle(x1, y1, rx, ry, w, h)


def is_collision_free_line(x1: float, y1: float, x2: float, y2: float, obstacle_cloud: np.ndarray) -> bool:
    obstacle_size: float = 0.001
    min_obstacle_dist: float = 0.05
    for obstacle in obstacle_cloud:
        if collides_segment_rectangle(x1, y1, x2, y2,
                                      obstacle[0] - min_obstacle_dist, obstacle[1] - min_obstacle_dist,
                                      obstacle_size + 2 * min_obstacle_dist, obstacle_size + 2 * min_obstacle_dist):
            return False
    return True


def is_collision_free_point(x: float, y: float, obstacle_cloud: np.ndarray) -> bool:
    obstacle_size: float = 0.001
    min_obstacle_dist: float = 0.05
    for obstacle in obstacle_cloud:
        if collides_point_rectangle(x, y,
                                    obstacle[0] - min_obstacle_dist, obstacle[1] - min_obstacle_dist,
                                    obstacle_size + 2 * min_obstacle_dist, obstacle_size + 2 * min_obstacle_dist):
            return False
    return True
