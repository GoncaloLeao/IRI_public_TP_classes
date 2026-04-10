"""
IRI - TP6 - Ex 2
By: Gonçalo Leão
"""
import os

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.base_class import BaseAlgorithm

import controllers.TP6.ex1.wall_following_env


def main() -> None:
    base_dir: str = os.path.join(os.getcwd(), "logs")
    # Change to the .zip file of your own model!
    model: BaseAlgorithm = PPO.load(base_dir + "/best_model_13k_timesteps.zip")
    env: gym.Env = gym.make("WallFollowing-v0")

    obs, _info = # TODO: reset the environment
    while True:
        action, _states = # TODO: predict the next action
        obs, reward, terminated, truncated, _info = # TODO: step the environment
        if truncated:
            # TODO: reset the environment


if __name__ == '__main__':
    main()
