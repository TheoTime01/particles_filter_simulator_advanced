
import random
import numpy as np
import numpy.typing
import typing as t
from common.particle import Particle
from common.robot import Robot
from common.particle_config import ParticleConfig

from tools.range_tools import make_scan_multi
import math
import copy

class ParticlesFilter:
    FIXED_Y_NO_VALUE :int =-1
    LIDAR_RANGE = 100
    MAX_SCAN_ANGLE = np.pi
    NB_OF_RAYS = 10
    OBSTACLE_VALUE = 1
    NB_PARTICLES: int = 10
    height :int
    width  :int
    obs_matrix :numpy.typing.ArrayLike = []
    particles_list: t.List[Particle]
    p_weight_max: float =0
    weight_list: t.List[float]
    particle_config: ParticleConfig
    fixed_y:int = FIXED_Y_NO_VALUE
    

    def __init__(self, h: int ,w:int , obs_m:numpy.typing.ArrayLike,fixed_y:float=FIXED_Y_NO_VALUE) -> None:
        self.height = h
        self.width = w
        self.obs_matrix =obs_m
        self.fixed_y = fixed_y
        self.particle_config = ParticleConfig()
        if self.fixed_y == self.FIXED_Y_NO_VALUE:
            self.particles_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, 0, self.height)
        else:
            self.particles_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, self.fixed_y, self.fixed_y+1,max_theta=0)
            self.particle_config.NEW_GEN_Y_SIGMA = 0
            self.particle_config.NEW_GEN_THETA_SIGMA = 0
            self.particle_config.ROTATION_MEASURE_ERROR_SIGMA = 0
        #self.particles_list =self.getRandParticleAround(self.NB_PARTICLES,10,25,0)
        #self.particles_list = self.getFixedParticles()

    def getFixedParticles(self):
        particles_list = []
        nbr = 5

        particle1 = Particle(self.particle_config,x=10,y=20,theta=0)
        particle1.weight =1 / float(nbr)
        particles_list.append(particle1)


        particle2 = Particle(x=20,y=40,theta=np.pi/4)
        particle2.weight =1 / float(nbr)
        particles_list.append(particle2)


        particle3 = Particle(x=30,y=50,theta=3*np.pi/4)
        particle3.weight =1 / float(nbr)
        particles_list.append(particle3)

        particle4 = Particle(x=40,y=60,theta=5*np.pi/4)
        particle4.weight =1 / float(nbr)
        particles_list.append(particle4)


        particle5 = Particle(x=50,y=70,theta=-np.pi/4)
        particle5.weight =1 / float(nbr)
        particles_list.append(particle5)

        return particles_list
    
    def getRandParticleAround(self,nbr: int,x :int, y:int,theta:float, SIGMA_XY:float =2,SIGMA_THETA:float = np.pi/8):
        """ Generate random particles the given coordinate

        Parameters
		----------
        - nb: int: number of particles
        - x: int : x reference
        - y: int : y reference
        - theta: float : theta reference
        - SIGMA_XY: int : x and y dispersion 
        - SIGMA_THETA: int : theta  dispersion

        Returns:
        - particles_list: T.List[Particle]: list of nex generated particles
        """
        
        particles_list = []
        
        
        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################

        # Use code below for simple mode, fix the y and your theta value of your particles
        # if self.fixed_y != self.FIXED_Y_NO_VALUE:
        #    y = self.fixed_y
        #    theta = np.random.normal(theta, 0, 1)[0]


        # keep this for display
        w_list=np.zeros((nbr))
        w_list=w_list+1 / float(nbr)
        self.weight_list = np.asarray(w_list)
        return particles_list


    def getRandParticle(self,nbr: int, min_x:int , max_x:int , min_y: int , max_y: int, max_theta:float= np.pi*2):
        """ Generate random particles

        Parameters
		----------
        - nb: int: number of particles
        - min_x: int :min x value
        - min_y: int :min y value
        - max_x: int :max x value
        - max_y: int :max y value
        - max_theta: float: max theta value

        Returns:
        - particles_list: T.List[Particle]: list of new generated particles
        """
        particles_list = []

        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################
        
        for i in range(nbr):
            particles_list.append(Particle(random.uniform(min_x,max_x),random.uniform(min_y,max_y),random.uniform(0,max_theta)))
            


        # keep this for display
        w_list=np.zeros((nbr))
        w_list=w_list+1 / float(nbr)
        self.weight_list = np.asarray(w_list)
        return particles_list
    
    def resetParticule(self, around_robot:bool =False, coord_theta:t.Tuple[int,int,float]=(0,0,0)):
        """ Reset Particles (used mainly on UI Interaction)

        Parameters
		----------
        - around_robot: bool: are particles centered or not on given coord_theta
        - coord_theta:t.Tuple[int,int,float]: if around_robot==True particles coord are centered around coord_theta
        """
        if around_robot:
            self.particles_list = self.getRandParticleAround(self.NB_PARTICLES,coord_theta[0],coord_theta[1],coord_theta[2])
        else:
            if self.fixed_y == self.FIXED_Y_NO_VALUE:
                self.particles_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, 0, self.height)
            else:
                self.particles_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, self.fixed_y, self.fixed_y+1,max_theta=0)



    def motion_prediction(self, u:t.Tuple[float,float]):
        """ Get the robot command u and apply it on all particles

        Parameters
		----------
        - u: t.Tuple[float,float]: motion command send to the robot (throttle,guidon)
        """
        for p in self.particles_list:
            p.model_with_error(u[0],u[1])
            #if particles outside map generate random
            if not self.check_coord(p):
                if self.fixed_y == self.FIXED_Y_NO_VALUE:
                    p.generate_rand_coord(self.height,self.width,self.obs_matrix)
                else:
                    p.generate_rand_coord(self.fixed_y+1,self.width,self.obs_matrix, h_min=self.fixed_y,theta_max=0)

    def weighting_particles_list(self,robot:Robot):
        """ Evaluate each particle in self.particles_list according their ranges perception
        p.ranges hold the distance to obstacles. Each range represents a distance between the current particle p to an obstacle given an angle
        p.ranges and robot.ranges are order in the same way. Meaning that p.ranges[0] and robot.ranges[0] represent the perception from the same angle

        Parameters
		----------
        - robot: Robot: Robot , ranges is used to evaluate each particle (robot.ranges)

        Results
		----------
        - Each particle weight (p.weight) belonging to self.particles_list is set
        """


        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################
        
        # keep this for display
        # p_weight_max is the maximum weight computed for all particles
        # weight_list holds weight of particle following the same order than self.particles_list
        p_weight_max =0
        weight_list=[]

        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################
        self.p_weight_max = p_weight_max
        self.weight_list = weight_list

        return weight_list, p_weight_max
    

    def resample_particles(self):

        """ Create a new generation of particles based on the weight and pose of the ancestors.

        Results
		----------
        - self.particles_list receives the new generation of particles
        """

        new_particles_list =[]


        #########################
        #      WORK  TODO       #
        #########################
        #                       #
        #                       #
        #                       #
        #                       #
        #########################

        # Tips do not forget to use self.weight_list for new particles generation creation
        #       complete Particle, generate_new_coord_theta function and use it here like p.generate_new_coord_theta(p.x,p.y,p.theta) 
        
        if len(new_particles_list) >0:
            self.particles_list = new_particles_list
            


            

    def _get_ranges_of_entities(self, robot:Robot):
        """ Compute ranges of each particles plus robot

        Results
		----------
        - each particle get its ranges list set (p.ranges)
        - robot get its ranges set (r.ranges)
        """
        coord_theta_list=[]
        for i in range(len(self.particles_list)):
            p=self.particles_list[i]
            #CAUTION need to inverse x and y
            coord_theta_list.append((p.x,p.y,p.theta))
        coord_theta_list.append((robot.x,robot.y,robot.theta))
        q_list, ranges = make_scan_multi(self.obs_matrix,
                                         coord_theta_list,
                                         1,
                                         self.NB_OF_RAYS,
                                         self.MAX_SCAN_ANGLE,
                                         self.LIDAR_RANGE,
                                         display=False,
                                         save_trace=False)

        for i in range(len(q_list)-1):
            p = self.particles_list[i]
            p.theta_ranges = p.theta + np.linspace(-self.MAX_SCAN_ANGLE, self.MAX_SCAN_ANGLE, self.NB_OF_RAYS)
            p.ranges = ranges[ q_list[i]: q_list[i] + self.NB_OF_RAYS]
        # robot is the last elt of the array
        robot.theta_ranges = robot.theta + np.linspace(-self.MAX_SCAN_ANGLE, self.MAX_SCAN_ANGLE, self.NB_OF_RAYS)
        robot.ranges = ranges[ q_list[len(q_list)-1]: q_list[len(q_list)-1] + self.NB_OF_RAYS]


    def check_coord(self,p:Particle):
        if p.x >=0 and p.x < self.width and p.y >=0 and p.y < self.height:
            return True