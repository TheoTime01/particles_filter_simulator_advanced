import shortuuid
import typing as t
from common.movable_entity import MovableEntity
import numpy as np
import random
from common.movable_entity_config import MovableEntityConfig
class Particle(MovableEntity):

    weight: float = 0
    id: str

    def __init__(self,config, 
                 x: int = 0, 
                 y: int = 0, 
                 theta: float = 0, 
                 Ks: float = MovableEntityConfig.Ks, 
                 Kv: float = MovableEntityConfig.Kv, 
                 L: float = MovableEntityConfig.L, 
                 steering_max_angle: float = MovableEntityConfig.steering_max_angle, 
                 dt: float = MovableEntityConfig.dt) -> None:
        super().__init__(x, y, theta, Ks, Kv, L, steering_max_angle, dt)
        self.config = config
        self.id = shortuuid.uuid()
    
    def __str__(self) -> str:
        return "P[%s]: x:%f, y:%f, theta:%f, weight:%f"%(self.id,self.x,self.y,self.theta,self.weight)
    
    def model_with_error(self, throttle, guidon):
        """ Apply given motion command plus error

        Parameters
		----------
        - throttle: flaot: Velovity
        - guidon: int :guidon ange
        
        """

        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################
        
        # current command + error
        # Modify the following two commandes
        throttle_with_error_velocity = throttle
        guidon_with_error_theta = guidon

        # keep the command below for applying the new commmand to the current particle
        self.model(throttle_with_error_velocity,guidon_with_error_theta)

    def generate_new_coord_theta(self,x:int,y:int,theta:float):
        """ Generate new pose (x,y,theta) of the current particle around (exploration) the given pose

        Parameters
		----------
        - x: int: x base
        - y: int : y base
        - theta: float :theta base
        """

        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################
        
        # self.x , self.y ,self.theta  are set with new values

    def generate_rand_coord(self, h: int ,w:int, obs_m:np.typing.ArrayLike,OBSTACLE_VALUE =1,h_min:int=0, w_min:int=0, theta_max=np.pi*2):
        x = random.randint(w_min, w-1);
        y = random.randint(h_min, h-1);
        theta = random.random()*theta_max
        #CAUTION Invert x and y ... WHY ?
        if obs_m[y,x] != OBSTACLE_VALUE:

                self.x = x
                self.y = y
                self.theta = theta
                self.weight = 0
