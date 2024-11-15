import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.typing
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from matplotlib.artist import Artist
from matplotlib.patches import FancyArrowPatch
from matplotlib.collections import LineCollection
import matplotlib.lines as mlines
import matplotlib.colors as mcolors
import matplotlib as mpl
#import mpl_interactions.ipyplot as iplt
from matplotlib.widgets import Button, Slider,CheckButtons



from matplotlib.backend_bases import MouseButton

from common.particle import Particle
from common.robot import Robot
from common.particle_config import ParticleConfig
from common.movable_entity_config import MovableEntityConfig

import numpy as np
import numpy.typing

import typing as t
import math

from display.ui_status import ENUM_STATUS_UI
from display.ui_artist_labels import UIArtistLabels

class UIManager:
    ENUM_STATUS_UI: str = ENUM_STATUS_UI.NO_GOAL
    ARROW_LENGHT:float = 5.0
    obs_matrix :numpy.typing.ArrayLike = []
    fig: Figure
    ax: Axes
    axslider: Axes
    slider_nd_particles: Slider
    particles_list:t.List[Particle]
    current_goal: t.List= []
    artists_map: t.Dict[UIArtistLabels,t.List[Artist]]={}
    check_p_ranges_toggle: bool = False
    fixed_y:int
    
    def __init__(self, obs_matrix, ui_callback,ui_display_callback, config:ParticleConfig ,fixed_y:int =-1) -> None:
        self.obs_matrix = obs_matrix
        self.config = config
        #self.fig, (self.ax, self.ax_ui) =  plt.subplots(2)
        
        self.fig = plt.figure(0)
        self.fig.canvas.manager.set_window_title('Particles Filter Simulator')
        self.fig_ui = plt.figure(1)
        self.fig_ui.canvas.manager.set_window_title('Particles Filter Configurator')
        self.fig.clear()
        self.fig_ui.clear()
        self.ax = self.fig.add_subplot()
        #self.ax_ui = self.fig_ui.add_subplot()
        self.ax.imshow(self.obs_matrix)
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        #plt.ion()
        self.cmap=mpl.colormaps["magma"]
#       plt.pause(0.5)
        #plt.show(block=False)
        self.add_controlers(self.fig_ui)
        self.refresh()
        self.fig_ui.show()
        self.ui_callback =ui_callback
        self.ui_display_callback=ui_display_callback
        self.fixed_y=fixed_y

    
    ####################################
    #   INTERACTIVE UI COMPONENT       #
    ####################################

    def add_controlers(self, fig):
        
        self.ax_button =  fig.add_subplot([0.55, 0.05, 0.13, 0.05])
        self.button_reset = Button(self.ax_button, 'Reset')
        self.button_reset.on_clicked(self.reset_simulation_callback)

        self.ax_button_ra =  fig.add_subplot([0.68, 0.05, 0.13, 0.05])
        self.button_reset_a = Button(self.ax_button_ra, 'Reset Ar.')
        self.button_reset_a.on_clicked(self.reset_around_simulation_callback)
        
        self.ax_button_a =  fig.add_subplot([0.81, 0.05, 0.13, 0.05])
        self.button_apply = Button(self.ax_button_a, 'Apply')
        self.button_apply.on_clicked(self.apply_modif_callback)
        
        self.ax_nbr_particles =  fig.add_subplot([0.55, 0.13, 0.35, 0.03])
        self.slider_nb_particles = Slider(self.ax_nbr_particles, 'Nb of Particles', 1, 500, valinit=10,valstep=1)

        self.ax_nbr_of_rays =  fig.add_subplot([0.55, 0.18, 0.35, 0.03])
        self.slider_nb_of_rays = Slider(self.ax_nbr_of_rays, 'Nb of Ray', 3, 300, valinit=10,valstep=1)

        self.ax_lidar_range =  fig.add_subplot([0.55, 0.23, 0.35, 0.03])
        self.slider_lidar_range = Slider(self.ax_lidar_range, 'Lidar Range', 2, 500, valinit=100,valstep=1)

        self.ax_v_m_error_sigma =  fig.add_subplot([0.55, 0.33, 0.35, 0.03])
        self.slider_v_m_error_sigma = Slider(self.ax_v_m_error_sigma, 'VELOCITY_MEASURE_ERROR_SIGMA', 0, 500, valinit=self.config.VELOCITY_MEASURE_ERROR_SIGMA,valstep=0.1)

        self.ax_rot_error_sigma =  fig.add_subplot([0.55, 0.38, 0.35, 0.03])
        self.slider_rot_error_sigma = Slider(self.ax_rot_error_sigma, 'ROTATION_MEASURE_ERROR_SIGMA', 0, np.pi *2 , valinit=self.config.ROTATION_MEASURE_ERROR_SIGMA,valstep=0.05)

        self.ax_range_error_sigma =  fig.add_subplot([0.55, 0.43, 0.35, 0.03])
        self.slider_range_error_sigma = Slider(self.ax_range_error_sigma, 'RANGE_MEASURE_ERROR_SIGMA', 0, 100 , valinit=self.config.RANGE_MEASURE_ERROR_SIGMA,valstep=1)

        self.ax_new_gen_x_sigma =  fig.add_subplot([0.55, 0.53, 0.35, 0.03])
        self.slider_new_gen_x_sigma = Slider(self.ax_new_gen_x_sigma, 'NEW_GEN_X_SIGMA', 0, 100 , valinit=self.config.NEW_GEN_X_SIGMA,valstep=1)

        self.ax_new_gen_y_sigma =  fig.add_subplot([0.55, 0.58, 0.35, 0.03])
        self.slider_new_gen_y_sigma = Slider(self.ax_new_gen_y_sigma, 'NEW_GEN_Y_SIGMA', 0, 100 , valinit=self.config.NEW_GEN_Y_SIGMA,valstep=1)

        self.ax_new_gen_theta_sigma =  fig.add_subplot([0.55, 0.63, 0.35, 0.03])
        self.slider_new_gen_theta_sigma = Slider(self.ax_new_gen_theta_sigma, 'NEW_GEN_THETA_SIGMA',0, np.pi *2  , valinit=self.config.NEW_GEN_THETA_SIGMA,valstep=0.01)

        self.ax_check_ranges_bt =  fig.add_subplot([0.55, 0.70, 0.35, 0.03])
        self.check_ranges_bt = CheckButtons(
                                    ax=self.ax_check_ranges_bt,
                                    labels=["Display Particles Ranges"],
                                    )
        
        self.check_ranges_bt.on_clicked(self.display_p_ranges_callback)


    def _get_particle_config_values_from_ui(self) -> ParticleConfig:
        particleConfig = ParticleConfig()
        particleConfig.NEW_GEN_THETA_SIGMA = self.slider_new_gen_theta_sigma.val
        particleConfig.NEW_GEN_Y_SIGMA = self.slider_new_gen_y_sigma.val
        particleConfig.NEW_GEN_X_SIGMA = self.slider_new_gen_x_sigma.val
        particleConfig.RANGE_MEASURE_ERROR_SIGMA = self.slider_range_error_sigma.val
        particleConfig.ROTATION_MEASURE_ERROR_SIGMA = self.slider_rot_error_sigma.val
        particleConfig.VELOCITY_MEASURE_ERROR_SIGMA = self.slider_v_m_error_sigma.val
        return particleConfig
    
    def display_p_ranges_callback(self,val):
        if self.check_p_ranges_toggle:
            self.check_p_ranges_toggle = False
        else:
            self.check_p_ranges_toggle = True
        print(f'display p ranges : {self.check_p_ranges_toggle}')
        self.ui_display_callback(display_p_ranges =self.check_p_ranges_toggle)

    def reset_simulation_callback(self,val):
        print(f"Reset ask")
        
        p_config =self._get_particle_config_values_from_ui()

        self.ui_callback(self.slider_nb_particles.val,self.slider_nb_of_rays.val,self.slider_lidar_range.val,p_config, reset=True)

    def reset_around_simulation_callback(self,val):
        p_config =self._get_particle_config_values_from_ui()
        self.ui_callback(self.slider_nb_particles.val,self.slider_nb_of_rays.val,self.slider_lidar_range.val,p_config, reset=True, around_robot=True)
    
    def apply_modif_callback(self,val):
        p_config =self._get_particle_config_values_from_ui()
        print(f"Reset ask")
        self.ui_callback(self.slider_nb_particles.val,self.slider_nb_of_rays.val, self.slider_lidar_range.val,p_config)


    ####################################
    #       DISPLAY FUNCTIONS          #
    ####################################

    def display_particles_arrow(self, particles_list:t.List[Particle], color:str= 'w',max_weight=0):
        arrows = []
        
        p_arrow_artist = []

        if UIArtistLabels.P_ARROW_LABEL in self.artists_map:
            p_arrow_artist = self.artists_map[UIArtistLabels.P_ARROW_LABEL]
            # check if nb of particles change
            if len(p_arrow_artist) != len(particles_list):
                self._reset_set_of_artists(p_arrow_artist)
                p_arrow_artist = []
        else:
            self.artists_map[UIArtistLabels.P_ARROW_LABEL]=[]
            
        for index, p in enumerate(particles_list):
            o_coord = (p.x, p.y)

            x2 = p.x + self.ARROW_LENGHT * math.cos(p.theta ) 
            y2 = p.y + self.ARROW_LENGHT * math.sin(p.theta)

            t_coord = (x2,y2)
            #arrows.append((o_coord,t_coord))
            if max_weight != 0:
                c_coeff =p.weight/max_weight
                #if nb of particles changes
                if c_coeff > 1:
                    c_coeff = 0
            else:
                c_coeff =p.weight

            p_color = self.cmap.colors[int(np.rint(len(self.cmap.colors)*(c_coeff)))-1]
            if len(p_arrow_artist) <= index:

                arrow = FancyArrowPatch(o_coord, 
                                t_coord,
                                arrowstyle='->', 
                                color= p_color,
                                mutation_scale=15, 
                                lw=1.5)
                self.ax.add_artist(arrow)
                self.artists_map[UIArtistLabels.P_ARROW_LABEL].append(arrow)
            else:
                p_arrow_artist[index].set_positions(o_coord,t_coord)
                p_arrow_artist[index].set_color(p_color)
        
    def display_particles_ranges(self, particles_list:t.List[Particle]):

        p_ranges_artist = None

        if UIArtistLabels.P_RANGES_LABEL in self.artists_map:
            p_ranges_artist = self.artists_map[UIArtistLabels.P_RANGES_LABEL]
            # check if nb of particles change
            if len(p_ranges_artist) != len(particles_list):
                self._reset_set_of_artists(p_ranges_artist)
                p_ranges_artist = None
        else:
            self.artists_map[UIArtistLabels.P_RANGES_LABEL]=[]


        ray_list=[]
        for p in particles_list:
            x1 = p.x
            y1 = p.y
            for j in range(len(p.ranges)):
                x2 = int(np.rint(p.x + p.ranges[j] * math.cos(p.theta_ranges[j])))
                y2 = int(np.rint(p.y + p.ranges[j] * math.sin(p.theta_ranges[j])))
                ray_list.append([(x1,y1), (x2,y2)])

        if p_ranges_artist == None:
            lc_p = LineCollection(ray_list, linewidths=0.5, colors='grey',alpha=0.5, zorder=0)
            self.ax.add_artist(lc_p)
            self.artists_map[UIArtistLabels.P_RANGES_LABEL] = []    
            self.artists_map[UIArtistLabels.P_RANGES_LABEL].append(lc_p)
        else:
            p_ranges_artist[0].set_segments(ray_list)

    def display_robot(self, robot:Robot, color:str= 'r'):

        p_arrow_artist = None
        if UIArtistLabels.R_ARROW_LABEL in self.artists_map:
            p_arrow_artist = self.artists_map[UIArtistLabels.R_ARROW_LABEL]

        o_coord = (robot.x, robot.y)
        x2 = robot.x + self.ARROW_LENGHT * math.cos(robot.theta)
        y2 = robot.y + self.ARROW_LENGHT * math.sin(robot.theta)
        t_coord = (x2,y2)
        
        if p_arrow_artist == None:
            arrow_r = FancyArrowPatch(o_coord, 
                        t_coord,
                        arrowstyle='->', 
                        color= color, 
                        mutation_scale=15, 
                        lw=1.5)
            self.ax.add_artist(arrow_r)
            self.artists_map[UIArtistLabels.R_ARROW_LABEL] = [] 
            self.artists_map[UIArtistLabels.R_ARROW_LABEL].append(arrow_r)
        else:
            p_arrow_artist[0].set_positions(o_coord,t_coord)
            p_arrow_artist[0].set_color(color)


    def display_robot_ranges(self, robot:Robot):

        r_ranges_artist = None
        if UIArtistLabels.R_RANGES_LABEL in self.artists_map:
            r_ranges_artist = self.artists_map[UIArtistLabels.R_RANGES_LABEL]

        ray_list=[]
        for j in range(len(robot.ranges)):
            x2 = int(np.rint(robot.x + robot.ranges[j] * math.cos(robot.theta_ranges[j])))
            y2 = int(np.rint(robot.y + robot.ranges[j] * math.sin(robot.theta_ranges[j])))
            ray_list.append([(robot.x,robot.y), (x2,y2)])
        if r_ranges_artist == None:
            lc_r = LineCollection(ray_list, linewidths=0.5, colors='red',alpha=0.8, zorder=0)
            self.ax.add_artist(lc_r)
            self.artists_map[UIArtistLabels.R_RANGES_LABEL] = []
            self.artists_map[UIArtistLabels.R_RANGES_LABEL].append(lc_r)
        else:
            r_ranges_artist[0].set_segments(ray_list)

    def refresh(self, time=0.1, reset_artist=False):
        plt.pause(time)
        self.fig.canvas.draw_idle()
        if reset_artist:
            self.reset_artists()

    def reset_artists(self):
        # Clear all artists added to the current plot
        for key,item in self.artists_map.items():
            self._reset_set_of_artists(item)
        
        self.artists_map.clear()
        self.ax.relim(visible_only=False)

    def _reset_set_of_artists(self, artist_list):
        for artist in artist_list:
                try:
                    artist.remove() # there should be a better way to do this. For example,
                    # initially use add_artist and draw_artist later on
                except Exception as e:
                    artist.set_visible(False)
                    pass

    def on_click(self,event):
        #Check that event in on the graph and not on interative button and sliders
        if event.inaxes == self.ax :
            if event.button is MouseButton.LEFT:
                print(f'data coords {event.ydata} {event.xdata},',
                    f'pixel coords {event.x} {event.y}')
                #if len(self.current_goal)  == 0:
                # fix the height or not
                if self.fixed_y ==-1:
                    self.current_goal=(int(np.rint(event.xdata)),int(np.rint(event.ydata)))
                else:
                    self.current_goal=(int(np.rint(event.xdata)),self.fixed_y)

                self.ENUM_STATUS_UI = ENUM_STATUS_UI.NEW_GOAL
            if event.button is MouseButton.RIGHT:
                self.ENUM_STATUS_UI = ENUM_STATUS_UI.NO_GOAL
       
    def cstm_autumn_r(self, x):
        return plt.cm.autumn_r((np.clip(x,2,10)-2)/8.)
    