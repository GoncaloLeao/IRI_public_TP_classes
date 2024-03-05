"""
IRI - TP2 - Ex 1
By: Gonçalo Leão
"""

from controller import Robot, TouchSensor
from controllers.utils import cmd_vel

# Create the Robot instance.
robot: Robot = Robot()

timestep: int = int(robot.getBasicTimeStep())  # in ms

# TODO