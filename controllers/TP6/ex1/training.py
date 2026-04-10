"""
IRI - TP6 - Ex 2
By: Gonçalo Leão
"""
import os
import time

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.base_class import BaseAlgorithm
from stable_baselines3.common.callbacks import EvalCallback

import controllers.TP6.ex1.wall_following_env

def main() -> None:
    base_dir: str = os.path.join(os.getcwd(), "logs")
    env: gym.Env = gym.make("WallFollowing-v0")
    # In the tensorboard_logs directory, run
    # python -m tensorboard.main --logdir "./"
    model: BaseAlgorithm = PPO("MultiInputPolicy", env, verbose=1,
                               #learning_rate=3e-3,
                               tensorboard_log=base_dir + '/tensorboard_logs',
                               )
    time_str: str = time.strftime("%Y%m%d-%H%M%S")
    eval_callback: EvalCallback = EvalCallback(env,
                                               n_eval_episodes=5,
                                               eval_freq=10000,
                                               best_model_save_path=base_dir + "/best_model",
                                               log_path=base_dir + "/eval_logs_results")
    model.learn(total_timesteps=10000000 * 50000, log_interval=10, tb_log_name=time_str, callback=eval_callback)


if __name__ == '__main__':
    main()
