"""
Some useful functions, namely for Webots and for working with the epuck robot (https://cyberbotics.com/doc/guide/epuck?version=R2021a).
By: Gonçalo Leão
"""
import math

from controller import Robot, Motor, Supervisor, Node
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
    duration: float = theta / angular_vel
    cmd_vel(robot, 0, angular_vel)
    robot.step(int(1000 * duration))


# Alternative solution
def rotate2(robot: Robot, theta: float, angular_vel: float) -> None:
    duration: float = theta / angular_vel
    start_time: float = robot.getTime()
    cmd_vel(robot, 0, angular_vel)
    while robot.getTime() < start_time + duration:
        robot.step()


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
