#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle
import numpy as np
import math
from .waymo_training_plot_tools import *

def read_and_parse_network_json_data(file_path,
                                     scenario_index,
                                     node_json_file_name,
                                     link_json_file_name,
                                     turn_movement_file_name,
                                     lane_json_file_name,
                                     dpi,
                                     size_pixels,
                                     size_inch,
                                     node_shape,
                                     node_color,
                                     node_size,
                                     link_shape,
                                     link_color,
                                     link_width):
    
    node_json_file = file_path + node_json_file_name
    link_json_file = file_path + link_json_file_name
    turn_json_file = file_path + turn_movement_file_name
    lane_json_file = file_path + lane_json_file_name    
    
    node_x_list = []
    node_y_list = []
    
    link_start_x_list = []
    link_start_y_list = []
    
    link_end_x_list = []
    link_end_y_list = []
    
    link_x = None
    link_y = None
    
    link_list = []
    
    lane_list = []
    
    number_of_center_lane = 0
    
    # get node
    with open(node_json_file, "r") as node_file:
        node_data = json.load(node_file)
        for node_id, node in node_data.items():
            node_x = node["x"]
            node_y = node["y"]
            node_x_list.append(node_x)
            node_y_list.append(node_y)     

    # get link
    with open(link_json_file, "r") as link_file:
        link_data = json.load(link_file)
        for link_id, link in link_data.items():
            unode_id = str(link["unode_id"])
            dnode_id = str(link["dnode_id"])
            #print("unode id is: ", unode_id)
            #print("dnode id is: ", dnode_id)
            unode = node_data[unode_id]
            dnode = node_data[dnode_id]
            unode_x = unode["x"]
            unode_y = unode["y"]
            dnode_x = dnode["x"] 
            dnode_y = dnode["y"]        
            link_start_x_list.append(unode_x)
            link_start_y_list.append(unode_y)  
            link_end_x_list.append(dnode_x)
            link_end_y_list.append(dnode_y) 
            link_x = [unode_x, dnode_x]
            link_y = [unode_y, dnode_y]
            link_list.append([link_x,link_y])
            

    # get lane 
    with open(lane_json_file, "r") as lane_file:
        lane_data = json.load(lane_file)
        for lane_id, lane in lane_data.items():
            number_of_center_lane += 1
            shape_points = lane["shape_points"]
            lane_x_list = []
            lane_y_list = []
            lane_x_y_list = []
            for point in shape_points:
                lane_x_list.append(point[0])
                lane_y_list.append(point[1])
            lane_x_y_list.append(lane_x_list)
            lane_x_y_list.append(lane_y_list)
            lane_list.append(lane_x_y_list)



    # plot          
    fig, ax = create_figure_and_axes(dpi, size_pixels, size_inch)   

    # plot lane           
    for lane_x_y_list in lane_list:  
        random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
        ax.scatter(lane_x_y_list[0], lane_x_y_list[1], c = random_color[0], marker = '.', linewidths = 0.4) 
        
    # plot link
    for link in link_list:
        ax.plot(link[0], link[1], color = link_color,  linestyle = link_shape, linewidth = link_width) 
        
    # plot node
    ax.scatter(node_x_list, node_y_list, color = node_color, marker = node_shape, linewidths = node_size)    
    
    plot_title = "network_" + str(scenario_index) + ": from json file"
    ax.set_title(plot_title, fontsize = 20)    

    ax.set_xlabel("x(m)", fontsize = 20)
    ax.set_ylabel("y(m)", fontsize = 20)
    ax.tick_params(axis = 'x', labelsize = 20)
    ax.tick_params(axis = 'y', labelsize = 20)
    ax.grid('off')
    plt.tight_layout()
    
    file_name = file_path + "network_" + str(scenario_index)
    fig.savefig(file_name, dpi = dpi)
    fig.clear()
    plt.clf()
    plt.close("all")
