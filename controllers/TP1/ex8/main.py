"""
IRI - TP1 - Ex 8
By: Gonçalo Leão
"""

from controller import Robot, Lidar, LidarPoint


def main():
    robot: Robot = Robot()

    timestep: int = int(robot.getBasicTimeStep())  # in ms

    lidar: Lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()
    robot.step()
    # TODO: Print the LiDAR readings


if __name__ == '__main__':
    main()
