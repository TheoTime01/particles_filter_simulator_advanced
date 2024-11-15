from common.vehicle import Vehicle
import typing as t
from common.vehicle import Vehicle
from common.movable_entity_config import MovableEntityConfig
from tools.PathTools import shortestAngleDiff
import numpy as np
import math

class MovableEntity(Vehicle):
    GOAL_EPSILON: float = 2
    x:int = 0
    y:int = 0
    theta: float = 0
    ranges: t.List[float] = []
    theta_ranges : t.List[float] = [] 
    

    def __init__(self, x:int = 0, 
                 y:int = 0, 
                 theta:float =0,
                 Ks: float=MovableEntityConfig.Ks, 
                 Kv: float=MovableEntityConfig.Kv, 
                 L: float=MovableEntityConfig.L, 
                 steering_max_angle: float = MovableEntityConfig.steering_max_angle, 
                 dt: float = MovableEntityConfig.dt,
                 throttle_min: int = MovableEntityConfig.throttle_min,
                 throttle_max: int = MovableEntityConfig.throttle_max) -> None:
        self.x =x
        self.y =y
        self.theta =theta

        self.Ks = Ks        # Steering proportionnal coefficient
        self.Kv = Kv        # Velocity proportionnal coefficient
        self.L = L          # Distance between front and back wheel
        self.steering_max_angle = steering_max_angle  # How much you can turn the front wheel
        self.dt = dt                # Time of one code cycle
        self.throttle_min = throttle_min     # Min saturation throttle based of euclidan distance
        self.throttle_max = throttle_max    # Max saturation throttle based of euclidan distance



    def model(self, throttle, guidon):
        """  Bicycle model
                           ___________________
                          |                   |
                          |                   |---> x           
            throttle  --->|                   |    
                          |      Bicycle      |
                          |                   |---> y
                          |       Model       |                        
            guidon    --->|                   |   
                          |                   |---> theta      
                          |___________________|          
        """        
        v = min(throttle, self.throttle_max)
        g = min(guidon, self.steering_max_angle) 
        dt = self.dt

        x_p = v * math.cos(self.theta)
        y_p = v * math.sin(self.theta)
        theta_p = (v / self.L) * math.tan( g )

        self.x = self.x + x_p*dt
        self.y = self.y + y_p*dt
        self.theta = self.theta + theta_p*dt

        return self.x, self.y, self.theta



    def toPoint(self, x, y, eps=GOAL_EPSILON):
        """ Move to a point (x, y)

            Parameters:
            x (int): x coordinate to reach in the map
            y (int): y coordinate to reach in the map
            eps (float): epsilon below which we consider the goal is reached

            Returns:
            float: Returning throttle value (corresponding to euclidian distance vehicle-->goal) 

        """        
        Ks = self.Ks
        Kv = self.Kv          
        eucli_throttle = eps
        while eucli_throttle >= eps:

            eucli_throttle = math.sqrt( (x-self.x)**2 + (y-self.y)**2 )
            steering = math.atan2(  (y-self.y)  , (x-self.x) )

            v = max(min(eucli_throttle * Kv, self.throttle_max), self.throttle_min)
            s = shortestAngleDiff(steering, self.theta) * Ks
        
            self.model(v, s)

            yield eucli_throttle, v , s



    def turn(self, phi, eps_angle=0.1):

        Ks = self.Ks
        Kv = self.Kv
        s = 1

        while math.fabs(s) > eps_angle:

            v = 10 # Trick... Because a bicyle/car CAN'T turn without linear velocity
            s = shortestAngleDiff(phi, self.theta) 

            self.model(v, math.copysign(self.steering_max_angle, s))

            yield s            


    
    def toPose(self, x, y, theta):

        pass


