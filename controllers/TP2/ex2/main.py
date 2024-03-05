"""
IRI - TP2 - Ex 2
By: Gonçalo Leão
"""

from controller import Robot, DistanceSensor
from controllers.utils import cmd_vel

# Create the Robot instance.
robot: Robot = Robot()

timestep: int = int(robot.getBasicTimeStep())  # in ms

# TODO