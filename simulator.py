from common.particles_filter import  ParticlesFilter
from common.robot import Robot
from common.particle_config import ParticleConfig
from display.ui_mng import UIManager
from display.ui_status import ENUM_STATUS_UI
from tools import env_builder
import numpy as np
import numpy.typing
import time
import sys
import typing as t

class Simulator():
    p_filter:ParticlesFilter
    ui_mng: UIManager
    obs_map: numpy.typing.ArrayLike = []
    is_end: bool =False
    robot: Robot
    ROBOT_INIT_POSE: t.Tuple[int,int,float] = (10,25,0)
    one_time : bool = True
    display_p_ranges:bool =False
    simple_mode: bool =False

    def __init__(self, h:int,w:int,r_x:int,r_y:int,r_theta:float, simple_mode:bool =False) -> None:
        map=input("Choissisez le numéro de la carte à charger (1 à 5):")
        self.obs_map = env_builder.load_black_and_white_env('./maps/map-room'+map+'.png')
        
        
        self.simple_mode=simple_mode
        self.robot = Robot(r_x,r_y)
        if simple_mode:
            fixed_y =r_y
        else:
            fixed_y= ParticlesFilter.FIXED_Y_NO_VALUE

        self.p_filter = ParticlesFilter(self.obs_map.shape[0],self.obs_map.shape[1],self.obs_map,fixed_y=fixed_y)
        self.ui_mng = UIManager(self.obs_map,self.ui_callback,self.ui_display_callback,self.p_filter.particle_config, fixed_y=fixed_y)

        start = time.time()
        self.robot.theta =r_theta
        self.ui_mng.display_robot(self.robot)
        self.ui_mng.display_particles_arrow(self.p_filter.particles_list, color='g')
        end = time.time()
        print("Particles display compute time:"+str(end-start))
        self.is_end =False
        self.ROBOT_INIT_POSE = [r_x,r_y,r_theta]
        
        
    def ui_callback(self, nb_particles: int , nb_rays:int , lidar_range:int,p_config:ParticleConfig, reset:bool =False, around_robot:bool=False):
        self.p_filter.NB_OF_RAYS = nb_rays
        self.p_filter.NB_PARTICLES = nb_particles
        self.p_filter.LIDAR_RANGE = lidar_range
        self.p_filter.particle_config = p_config
        if reset:
            if around_robot:
                if self.simple_mode:
                    self.p_filter.resetParticule(around_robot=around_robot, coord_theta=(self.robot.x,self.robot.y,self.robot.theta))
                else:
                    self.p_filter.resetParticule(around_robot=around_robot, coord_theta=(self.robot.x,self.robot.y,self.robot.theta))
            else:
                self.p_filter.resetParticule()
                self.robot = Robot(self.ROBOT_INIT_POSE[0],self.ROBOT_INIT_POSE[1])
                self.robot.theta = self.ROBOT_INIT_POSE[2]
            self.one_time = True
        #FIXME after he command bellow no more actions possible
        #self.ui_mng.refresh(reset_artist=True)

    def ui_display_callback(self, display_p_ranges:bool = False):
        self.display_p_ranges = display_p_ranges
        
        
    
    def loop(self):
        self.one_time = True
        while not self.is_end:
            
            # Listen Click on map for new goal
            if self.ui_mng.ENUM_STATUS_UI == ENUM_STATUS_UI.NEW_GOAL:
                self.one_time = True
                # In case of goal launch motion
                for e,v,s in self.robot.toPoint(self.ui_mng.current_goal[0], self.ui_mng.current_goal[1]):
                
                    start =time.time()
                    #Resample
                    self.p_filter.resample_particles()

                    #motion
                    self.p_filter.motion_prediction((v,s))


                    #Get ranges
                    self.p_filter._get_ranges_of_entities(self.robot)


                    #Weight
                    self.p_filter.weighting_particles_list(self.robot)                   
                    
                    ##print("V : %f, S: %f, theta:%f"%(v,s,self.robot.theta))
                    # Display markers
                    self.ui_mng.display_particles_arrow(self.p_filter.particles_list,max_weight=self.p_filter.p_weight_max)
                    if  self.display_p_ranges:
                        self.ui_mng.display_particles_ranges(self.p_filter.particles_list)
                    self.ui_mng.display_robot(self.robot)
                    self.ui_mng.display_robot_ranges(self.robot)
                    self.ui_mng.refresh()
                    end =time.time()
                    #print("compute new pose, compute range, and display:"+str(end-start))
                    end2 =time.time()
                    #print("refresh display:"+str(end2-end))
                    time.sleep(0.01)
                    endT =time.time()
                    #print("Total move:"+str(endT-start))

                    if e < Robot.GOAL_EPSILON:
                        self.ui_mng. ENUM_STATUS_UI= ENUM_STATUS_UI.NO_GOAL

                    if self.ui_mng. ENUM_STATUS_UI == ENUM_STATUS_UI.NO_GOAL:
                        break
            else:
                if self.one_time:
                    self.p_filter._get_ranges_of_entities(self.robot)
                    if  self.display_p_ranges:
                        self.ui_mng.display_particles_ranges(self.p_filter.particles_list)
                    self.ui_mng.display_robot_ranges(self.robot)
                    self.ui_mng.display_particles_arrow(self.p_filter.particles_list,max_weight=self.p_filter.p_weight_max)
                    self.ui_mng.display_robot(self.robot)
                    self.one_time = False
                
                self.ui_mng.refresh()

if __name__ == "__main__":
    simple_mode =False
    args = sys.argv[1:]
    if len(args)>0:
        if args[0]== "--simple":
            simple_mode = True

    #s=Simulator(100,200,r_x=10,r_y=25,r_theta=0)
    s=Simulator(100,200,r_x=10,r_y=50,r_theta=0,simple_mode=simple_mode)
    s.loop()
    print()
