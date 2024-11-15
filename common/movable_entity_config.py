from common.vehicle import Vehicle
import typing as t
from common.vehicle import Vehicle
from tools.PathTools import shortestAngleDiff
import numpy as np
import math

class MovableEntityConfig:
    GOAL_EPSILON: float = 2
    Ks: float=0.1
    Kv: float=2
    L: float=1.0
    steering_max_angle: float = np.pi/2
    dt: float = 0.003
    throttle_min: int = 400
    throttle_max: int = 1000
