import numpy as np
from common.movable_entity_config import MovableEntityConfig

class ParticleConfig():
    VELOCITY_MEASURE_ERROR_SIGMA: float = (MovableEntityConfig.throttle_max-MovableEntityConfig.throttle_min) * 10/100
    ROTATION_MEASURE_ERROR_SIGMA: float = (MovableEntityConfig.steering_max_angle) * 10/100
    RANGE_MEASURE_ERROR_SIGMA: float = 2
    NEW_GEN_X_SIGMA: float = 1.0 #0.2
    NEW_GEN_Y_SIGMA: float = 1.0 #0.2
    NEW_GEN_THETA_SIGMA: float = np.pi/8

    def __init__(self) -> None:
        pass