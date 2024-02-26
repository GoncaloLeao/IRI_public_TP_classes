"""
IRI - TP1 - Ex 10
By: Gonçalo Leão
"""

from controller import Robot, GPS
from controllers.utils import cmd_vel

robot: Robot = Robot()

timestep: int = int(robot.getBasicTimeStep())  # in ms
PRINT_AFTER_N_STEPS: int = 20

gps: GPS = robot.getDevice('gps')
gps.enable(timestep)

radius: float = 0.125
linear_vel: float = 0.02
angular_vel: float = linear_vel / radius
cmd_vel(robot, linear_vel, angular_vel)

print_step: int = 0
while robot.step() != -1:
    print_step += 1
    if print_step % PRINT_AFTER_N_STEPS == 0:
        pass
        # TODO: Print GPS readings
