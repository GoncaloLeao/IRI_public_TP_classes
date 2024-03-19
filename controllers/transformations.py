"""
Some useful functions for working with 3D transformations.
By: Gonçalo Leão
"""
import math

import numpy as np


def get_translation(tf_mat: np.ndarray) -> (float, float):
    return tf_mat[0][3], tf_mat[1][3]


def get_rotation(tf_mat: np.ndarray) -> float:
    return math.atan2(tf_mat[1][0], tf_mat[0][0])


def create_tf_matrix(translation: (float, float, float), rotation_z: float) -> np.ndarray:
    cos: float = math.cos(rotation_z)
    sin: float = math.sin(rotation_z)
    return np.array([[cos, -sin, 0, translation[0]],
                     [sin, cos, 0, translation[1]],
                     [0, 0, 1, translation[2]],
                     [0, 0, 0, 1]])
