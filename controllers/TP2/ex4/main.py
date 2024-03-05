"""
IRI - TP2 - Ex 4
By: Gonçalo Leão
"""
import math

from numpy import random

from controller import Robot, Lidar
from controllers.utils import cmd_vel

def distance_handler(direction: int, dist_values: [float]) -> (float, float):
    maxSpeed: float = 0.1
    distP: float = 10.0
    angleP: float = 7.0
    wallDist: float = 0.1

    # Find the angle of the ray that returned the minimum distance
    # TODO

    # Prepare message for the robot's motors
    linear_vel: float
    angular_vel: float

    # Decide the robot's behavior
    # TODO

    return linear_vel, angular_vel

if __name__ == '__main__':
    # Create the Robot instance.
    robot: Robot = Robot()

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    # Main loop
    while robot.step() != -1:
        linear_vel, angular_vel = distance_handler(1, lidar.getRangeImage())
        cmd_vel(robot, linear_vel, angular_vel)
