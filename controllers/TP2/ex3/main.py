"""
IRI - TP2 - Ex 3
By: Gonçalo Leão
"""

import math

from controller import Robot, GPS, Compass
from controllers.utils import cmd_vel

# Create the Robot instance.
robot: Robot = Robot()

timestep: int = int(robot.getBasicTimeStep())

# TODO