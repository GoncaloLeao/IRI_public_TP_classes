"""
Some useful functions for visualizing the solutions of robot 2D localization problems.
By: Gonçalo Leão
"""
import math

import numpy as np
from matplotlib import pyplot as plt


def draw_real_vs_estimated_localization(wall_cloud: np.ndarray,
                                        actual_translation: (float, float), actual_rotation: float,
                                        estimated_translations: [(float, float)], estimated_rotations: [float]) -> None:
    fig = plt.figure()

    ax = fig.add_subplot(111)
    ax.scatter(wall_cloud[0:, 0], wall_cloud[0:, 1], c='black', marker='.')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')

    arrow_length: float = 0.08
    plt.arrow(actual_translation[0] - arrow_length * math.cos(actual_rotation) / 2.0,
              actual_translation[1] - arrow_length * math.sin(actual_rotation) / 2.0,
              arrow_length * math.cos(actual_rotation), arrow_length * math.sin(actual_rotation),
              color='r', width=arrow_length / 4.0,
              length_includes_head=True, head_length=arrow_length)
    for (estimated_translation, estimated_rotation) in zip(estimated_translations, estimated_rotations):
        plt.arrow(estimated_translation[0] - arrow_length * math.cos(estimated_rotation) / 2.0,
                  estimated_translation[1] - arrow_length * math.sin(estimated_rotation) / 2.0,
                  arrow_length * math.cos(estimated_rotation), arrow_length * math.sin(estimated_rotation),
                  color='b', width=arrow_length / 4.0,
                  length_includes_head=True, head_length=arrow_length)
    plt.show()
    return
