import math
from enum import Enum

import numpy as np

from controller import Supervisor, TouchSensor, Lidar
from controllers.utils import cmd_vel, full_warp_robot
import gymnasium as gym

from gymnasium.envs.registration import register

register(
    id="WallFollowing-v0",
    entry_point="wall_following_env:WallFollowingEnv",
    max_episode_steps=10000,
)

class WallFollowingEnv(gym.Env):
    def __init__(self):
        self.robot: Supervisor = Supervisor()
        # Enable sensors
        timestep: int = int(self.robot.getBasicTimeStep())  # in ms
        # Distance sensors
        ps_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
        self.ps = [self.robot.getDevice(name) for name in ps_names]
        for sensor in self.ps:
            sensor.enable(timestep)
        # Touch sensor
        self.touch_sensor: TouchSensor = self.robot.getDevice('touch sensor')
        self.touch_sensor.enable(timestep)
        # Lidar
        self.lidar: Lidar = self.robot.getDevice('lidar')
        self.lidar.enable(timestep)
        self.lidar.enablePointCloud()

        # Make the sensors have some readings
        self.robot.step()

        # Store initial robot position
        robot_node = self.robot.getFromDef("EPUCK")
        self.initial_robot_position = robot_node.getField("translation").getSFVec3f()
        self.initial_robot_orientation = robot_node.getField("rotation").getSFRotation()

        self.ideal_distance: float = 0.08
        self.direction: float = 1.0

        self.action_space: gym.spaces = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        self.observation_space: gym.spaces = gym.spaces.Dict(
            {
                "closest_distance": gym.spaces.Box(-1, 1, dtype=np.float32),
                "closest_angle": gym.spaces.Box(-1, 1, dtype=np.float32),
                "front_dist": gym.spaces.Box(-1, 1, dtype=np.float32)
            }
        )
        self.num_epochs = 0

    def get_obs(self):
        dist_values: [float] = self.lidar.getRangeImage()
        # Find the angle of the ray that returned the minimum distance
        size: int = len(dist_values)
        min_index: int = -1
        if self.direction == -1:
            min_index = size - 1
        for i in range(int(size / 2)):
            idx: int = i
            if self.direction == -1:
                idx = size - 1 - i
            if dist_values[idx] < dist_values[min_index] and dist_values[idx] > 0.0:
                min_index = idx
        angle_increment: float = 2 * math.pi / (size - 1)
        angleMin: float = (size // 2 - min_index) * angle_increment
        distMin: float = dist_values[min_index]
        distFront: float = dist_values[size // 2]

        return {
            "closest_distance": distMin,
            "closest_angle": angleMin,
            "front_dist": distFront,
        }

    def normalize_obs(self, obs):
        return {
            "closest_distance": np.array([min(0.25, obs["closest_distance"]) / 0.125 - 1], dtype=np.float32),
            "closest_angle": np.array([obs["closest_angle"] / math.pi], dtype=np.float32),
            "front_dist": np.array([min(0.25, obs["front_dist"]) / 0.125 - 1], dtype=np.float32),
        }

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Warp robot to initial position
        self.robot.step()
        full_warp_robot(self.robot, "EPUCK", self.initial_robot_position, self.initial_robot_orientation)

        obs: dict = self.get_obs()

        self.last_closest_distance = obs["closest_distance"]

        self.num_timesteps = 0
        self.num_epochs += 1

        return self.normalize_obs(obs), {}

    def bumped_into_obstacle(self):
        return self.touch_sensor.getValue() == 1.0 or self.ps[7].getValue() > 80 or self.ps[0].getValue() > 80

    def lost_wall(self, obs):
        return obs["closest_distance"] > 0.35

    def step(self, action):
        # Execute the action
        angular_vel: float = action
        angular_vel *= math.pi/2
        linear_vel: float = 0.1
        cmd_vel(self.robot, linear_vel, angular_vel)
        self.robot.step()

        self.num_timesteps += 1

        # Check if terminal states and compute the reward
        obs: dict = self.get_obs()

        reward: float = 0
        if abs(obs["closest_distance"] - self.ideal_distance) < abs(self.last_closest_distance - self.ideal_distance):
            reward += (obs["closest_distance"] - self.ideal_distance)**2
        else:
            reward -= (obs["closest_distance"] - self.ideal_distance)**2

        terminated: bool = False
        truncated: bool = False
        if self.bumped_into_obstacle():
            terminated = True
            truncated = True
            print("Collision...")
            reward -= 10000 / (1000 + self.num_timesteps)
        elif self.lost_wall(obs):
            terminated = True
            truncated = True
            print("Lost wall...")
            reward -= 10000 / (1000 + self.num_timesteps)

        self.last_closest_distance = obs["closest_distance"]

        return self.normalize_obs(obs), reward, terminated, truncated, {}
