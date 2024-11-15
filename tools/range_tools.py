__author__ = "J.Saraydaryan"

import range_libc
import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.collections import LineCollection

def  make_scan_multi(obs_map,
                    coord_theta_list,
                    resolution,
                    nb_of_rays, 
                    max_scan_angle,
                    lidar_range,
					map_origin=(0,0), 
                    display=False,
					save_trace=False,
					trace_save_filename="/tmp/trace_save.png"):
		""" Make a Scan on a for a set of agents.
		
		Parameters
		----------
		- obs_map (numpy bool) :  discret map with obstacles
		- coord_theta_list : list of tuple representing agenfs (px,py,theta)
		- resolution : resolution of the given map
		- nb_of_rays: number of rays to measure
		- max_scan_angle : define the FoV of the agent, ray and orientation are computed as follow np.linspace(-MAX_SCAN_ANGLE, MAX_SCAN_ANGLE, n_ranges)
		- lidar_range: maximum distance of the sensor
		- map_origin: tuple (x,y) representing the origin of the map
		- display: True enable Matplotlib display for debug 
		- save_trace: enable to save the range_lib trace as image
		- trace_save_filename : file name to save the trace

		Returns
		----------
		- q_list , ranges  
			- q_list list of start indices in ranges matrix for each agent,
			- ranges: list of ranges measure len(ranges)= nbr_of_agent * nb_of_rays

		"""
		MAX_SCAN_ANGLE = max_scan_angle
    	# create a range_lib map through numpy array
		mm = range_libc.PyOMap(np.array(obs_map, dtype=bool))
		# init the obstacle detection mode here only PyBresenhamsLine is used, see https://github.com/kctess5/range_libc/tree/deploy/docs for other methods

		bl = range_libc.PyBresenhamsLine(mm , lidar_range/resolution)
		#bl = range_libc.PyRayMarching(mm , lidar_range/resolution)
		#bl = range_libc.PyRayMarchingGPU(mm , lidar_range/resolution)

    	# Setup range to measure
		queries = np.zeros((nb_of_rays * len(coord_theta_list),3),dtype=np.float32)
		ranges = np.zeros(nb_of_rays * len(coord_theta_list),dtype=np.float32)
		
		index_start = 0
		q_list = []
		for (x, y, theta) in coord_theta_list:
			q_list.append(index_start)
			x_o = x + map_origin[0]
			y_o = y + map_origin[1]
    		#update coordinate according map resolution
			x_matrix= int(np.rint(x_o / float(resolution)))
			y_matrix= int(np.rint(y_o / float(resolution)))

			# In our case all ranges are measured from the same point origin
			queries[index_start:index_start + nb_of_rays ,0] = x_matrix
			queries[index_start:index_start + nb_of_rays ,1] = y_matrix
			# Define the set on angles on which range is computed
			queries[index_start:index_start + nb_of_rays,2] = theta + np.linspace(-MAX_SCAN_ANGLE, MAX_SCAN_ANGLE, nb_of_rays)
			index_start = index_start + nb_of_rays 
	
		# Use the rangelib to measure distance on each ray, measure are held in ranges array
		bl.calc_range_many(queries,ranges)
		if display:
			display_trace_mlines(obs_map, queries, ranges, display_block=False, graph_title="Ray tracing", fig_num=1)

		if save_trace:
			bl.saveTrace(trace_save_filename.encode('utf-8'))

		return q_list , ranges


def display_trace(map, queries, ranges, display_block=False,graph_title="my title", fig_num=1):
	
	fig = plt.figure(fig_num)
	fig.clear()
	fig.suptitle(graph_title, fontsize=16)
	ax = fig.add_subplot()
	#ax = fig.add_axes([map.shape[0], map.shape[1], 1, 1])
	#fig, ax = plt.subplots()
	# For each ray compute a segment to display
	X=[]
	Y=[]
	for i in range (len(queries)):
		x1 = queries[i][0]
		y1 = queries[i][1]
		theta_r = queries[i][2]
		x2 = x1 + ranges[i] * math.cos(theta_r)
		y2 = y1 + ranges[i] * math.sin(theta_r)
		X.append(x1)
		X.append(x2)
		Y.append(y1)
		Y.append(y2)
		# draw segment
	ax.plot(X,Y, marker = '.', linewidth=0.5,markersize=2, color='white')
		
	ax.imshow(map)
	
	plt.show(block=display_block)
	plt.pause(0.1)
	
def display_trace_mlines(map, queries, ranges, display_block=False,graph_title="my title", fig_num=1):
    fig = plt.figure(fig_num)
    fig.clear()
    fig.suptitle(graph_title, fontsize=16)
    ax = fig.add_subplot()
    X_ranges = []
    Y_ranges = []
    ray_list=[]
    for i in range (len(queries)):
        x1 = queries[i][0]
        y1 = queries[i][1]
        theta_r = queries[i][2]
        x2 = x1 + ranges[i] * math.cos(theta_r)
        y2 = y1 + ranges[i] * math.sin(theta_r)
        ray_list.append([(x1,y1), (x2,y2)])
            
    lc = LineCollection(ray_list, linewidths=0.5, colors='grey',alpha=0.8, zorder=0)
	
    ax.imshow(map)
    ax.add_artist(lc)
	
    plt.show(block=display_block)
    plt.pause(0.1)