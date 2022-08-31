#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import uuid
import random
import glob
from matplotlib import cm
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle
import numpy as np
import math
from .waymo_training_math_tools import *
from .find_near_vehicles_in_lane import *
from .find_near_vehicles_in_neighbor_lane import *
from copy import copy
from matplotlib.patches import Polygon
#from .find_near_vehicles_in_right_lane import *

def create_figure_and_axes(dpi, size_pixels, size_inch):

    fig, ax = plt.subplots(1, 1, num=uuid.uuid4())
    # Sets output image to pixel resolution
    #size_inches = size_pixels / dpi
    fig.set_size_inches([size_inch, size_inch])
    fig.set_dpi(dpi)
    fig.set_facecolor('white')
    ax.set_facecolor('darkgrey')
    ax.xaxis.label.set_color('black')
    ax.tick_params(axis='x', colors='black')
    ax.yaxis.label.set_color('black')
    ax.tick_params(axis='y', colors='black')
    fig.set_tight_layout(True)
    ax.grid(False)
    return fig, ax
    

def _generate_fig_canvas_image(fig):
  # Returns a [H, W, 3] uint8 np.array image from fig.canvas.tostring_rgb()
  # Just enough margin in the figure to display xticks and yticks
  fig.subplots_adjust(left=0.08, bottom=0.08, right=0.98, top=0.98, wspace=0.0, hspace=0.0)
  fig.canvas.draw()
  data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
  fig.clear()
  return data.reshape(fig.canvas.get_width_height()[::-1] + (3,))


def _generate_best_fit_view_point(scenario):    
    
    #print("max x in scenario: ", np.max(scenario.all_agents_x_list))
    #print("max y in scenario: ", np.max(scenario.all_agents_y_list))
    #print("min x in scenario: ", np.min(scenario.all_agents_x_list))
    #print("min y in scenario: ", np.min(scenario.all_agents_y_list))    
    #print("range x in scenario: ", np.ptp(scenario.all_agents_x_list))
    #print("range y in scenario: ", np.ptp(scenario.all_agents_y_list))
    
    #print(" ---------- size of center lane array is: ", len(scenario.center_lane_x_list))
    
    #center_x = (np.max(scenario.center_lane_x_list) + np.min(scenario.center_lane_x_list)) / 2
    #center_y = (np.max(scenario.center_lane_y_list) + np.min(scenario.center_lane_y_list)) / 2       
    #range_x = np.ptp(scenario.center_lane_x_list) + 30
    #range_y = np.ptp(scenario.center_lane_y_list) + 30      
    
    center_x = (np.max(scenario.all_agents_x_list) + np.min(scenario.all_agents_x_list)) / 2
    center_y = (np.max(scenario.all_agents_y_list) + np.min(scenario.all_agents_y_list)) / 2       
    range_x = np.ptp(scenario.all_agents_x_list) + 100
    range_y = np.ptp(scenario.all_agents_y_list) + 100
    width = max(range_x, range_y)    
    return center_x, center_y, width  

def generate_single_step_plot(scenario, 
                              time_index, 
                              dpi, 
                              size_pixels, 
                              size_inch, 
                              plot_path, 
                              plot_topology, 
                              agent_shape,
                              agent_round_shape_width,
                              map_line_width,
                              plot_straight_lane,
                              plot_preceeding_following_in_lane,
                              plot_preceeding_following_in_lane_color,
                              plot_preceeding_following_in_lane_style,
                              plot_preceeding_following_in_left_lane,
                              plot_preceeding_following_in_left_lane_color,
                              plot_preceeding_following_in_left_lane_style,
                              plot_preceeding_following_in_right_lane,
                              plot_preceeding_following_in_right_lane_color,
                              plot_preceeding_following_in_right_lane_style,
                              plot_preceeding_following_line_width,
                              show_agnets_id,
                              white_line_color,
                              yellow_line_color,
                              center_line_color,
                              edge_line_color,
                              crossing_line_color,
                              stop_sign_color,
                              sdc_color,
                              vehicle_color,
                              bicycle_color,
                              pedestrian_color,
                              show_agents_coordinates,
                              agents_id_color,
                              agents_id_x_offset,
                              agents_id_y_offset,
                              agents_coordinates_x_offset,
                              agents_coordinates_y_offset,
                              plot_node_link,
                              plot_node_shape,
                              plot_node_color,
                              plot_node_size,
                              plot_link_shape,
                              plot_link_color,
                              plot_link_width,
                              plot_lane_boundary,
                              plot_left_right_turn,
                              plot_agents_trajectories,
                              plot_multiple_figures,
                              plot_lane_and_boundaries,
                              plot_lane_polygon,
                              plot_signal_light):
    
    white_yellow_line_plot = True
    topology_plot = False
    left_right_boundary_plot = False 
    left_right_turn_plot = False
    straight_lane_plot = False
    agents_trajectories_plot = False
    multiple_figures_plot = False   
    lane_and_boundaries_plot = False
    lane_polygon_plot = False
    node_link_plot = False
    
    if plot_topology == "True":
        white_yellow_line_plot = False
        topology_plot = True
        agents_trajectories_plot = False
        
        
        
    #else:
    #    white_yellow_line_plot = True
    #    topology_plot = False  
        
    if plot_straight_lane == "True":
        white_yellow_line_plot = False
        straight_lane_plot = True
        agents_trajectories_plot = False
    #else:
    #    straight_lane_plot = False
    #    white_yellow_line_plot = True

    if plot_lane_boundary == "True":
        left_right_boundary_plot = True
        plot_topology = False
        white_yellow_line_plot = False
        topology_plot = False
        plot_straight_lane = False
        agents_trajectories_plot = False
    #else:
    #    white_yellow_line_plot = True

    if plot_left_right_turn == "True":
        left_right_turn_plot = True
        left_right_boundary_plot = False
        plot_lane_boundary = False
        plot_topology = False
        white_yellow_line_plot = False
        topology_plot = False
        straight_lane_plot = False
        agents_trajectories_plot = False
    #else:
    #    white_yellow_line_plot =True
    
    if plot_agents_trajectories == "True":
        agents_trajectories_plot = True
        plot_left_right_turn = False
        left_right_boundary_plot = False
        plot_lane_boundary = False
        plot_topology = False
        white_yellow_line_plot = False
        topology_plot = False
        straight_lane_plot = False
    #else:
    #    agents_trajectories_plot = False
    #    white_yellow_line_plot = True

    '''
    if plot_multiple_figures == "True":
        multiple_figures_plot = True
        agents_trajectories_plot = False
        plot_left_right_turn = False
        plot_lane_boundary = False
        plot_topology = False
        normal_plot = False
        topology_plot = False
        straight_lane_plot = False
    else:
        agents_trajectories_plot = False
        white_yellow_line_plot = True
        multiple_figures_plot = False
    '''
        
    if plot_lane_and_boundaries == "True":
        lane_and_boundaries_plot = True
        agents_trajectories_plot = False
        left_right_boundary_plot = False
        plot_left_right_turn = False
        plot_lane_boundary = False
        plot_topology = False
        white_yellow_line_plot = False
        topology_plot = False
        straight_lane_plot = False        
    
    if plot_lane_polygon == "True":
        lane_polygon_plot = True
        lane_and_boundaries_plot = False
        agents_trajectories_plot = False
        left_right_boundary_plot = False
        plot_left_right_turn = False
        plot_lane_boundary = False
        plot_topology = False
        white_yellow_line_plot = False
        topology_plot = False
        straight_lane_plot = False          
    

    if plot_node_link == "True":
        lane_polygon_plot = False
        lane_and_boundaries_plot = False
        agents_trajectories_plot = False
        left_right_boundary_plot = False
        plot_left_right_turn = False
        plot_lane_boundary = False
        plot_topology = False
        white_yellow_line_plot = True
        topology_plot = False
        straight_lane_plot = False      
        node_link_plot = True

    #print("plot straight line: ", plot_straight_lane)
    #print("norm plot: ", white_yellow_line_plot)
    #print("topology plot: ", topology_plot)
    
    fig, ax = create_figure_and_axes(dpi, size_pixels, size_inch)
    
    center_x, center_y, width = _generate_best_fit_view_point(scenario)
    
    # ========== part1: plot map points ==========       
    # plot map points - white
    # normal plot
    if white_yellow_line_plot:
        print("      ====== norm plot enabled !!!")
        ax.scatter(scenario.broken_single_white_x_list, scenario.broken_single_white_y_list, color = white_line_color, marker = '.', linewidths = map_line_width)
        ax.scatter(scenario.solid_single_white_x_list,  scenario.solid_single_white_y_list,  color = white_line_color, marker = '.', linewidths = map_line_width)
        ax.scatter(scenario.solid_double_white_x_list,  scenario.solid_double_white_y_list,  color = white_line_color, marker = '.', linewidths = map_line_width)
        
        # plot map points - yellow 
        ax.scatter(scenario.broken_single_yellow_x_list,  scenario.broken_single_yellow_y_list, color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.broken_double_yellow_x_list,  scenario.broken_double_yellow_y_list, color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.solid_single_yellow_x_list,   scenario.solid_single_yellow_y_list,  color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.solid_double_yellow_x_list,   scenario.solid_double_yellow_y_list,  color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.passing_double_yellow_x_list, scenario.passing_double_yellow_y_list,color = yellow_line_color, marker = '.', linewidths = map_line_width)  
    
        node_number = 0
        if node_link_plot:
            if len(scenario.node_up) > 1:
                node_number += 1
                ax.scatter(scenario.node_up[0],  scenario.node_up[1], color = yellow_line_color, marker = '.', s = 180) 
            if len(scenario.node_down) > 1:
                node_number += 1
                ax.scatter(scenario.node_down[0],  scenario.node_down[1], color = yellow_line_color, marker = '.', s = 180) 
            if len(scenario.node_left) > 1:
                node_number += 1
                ax.scatter(scenario.node_left[0],  scenario.node_left[1], color = yellow_line_color, marker = '.', s = 180) 
            if len(scenario.node_right) > 1:
                node_number += 1
                ax.scatter(scenario.node_right[0],  scenario.node_right[1], color = yellow_line_color, marker = '.', s = 180) 
                
            ax.scatter(scenario.node_x,  scenario.node_y, color = yellow_line_color, marker = '.', s = 180) 
            
            print("   ---  node number is: ", node_number)
            
            for link in scenario.link_list:
                ax.plot(link.link_x,  link.link_y, color = 'red', marker = '.', linewidth = 4) 
            print("   ---  link number is: ", len(scenario.link_list))
            #print("   --- length of second point x :", len(scenario.sceond_cluster_points_x_list))
            #print("   --- length of second point y :", len(scenario.sceond_cluster_points_y_list))             
            
            ax.scatter(scenario.first_cluster_points_x_list,  scenario.first_cluster_points_y_list, color = 'red', marker = '.', s = 180) 
            ax.scatter(scenario.second_cluster_points_x_list, scenario.second_cluster_points_y_list, color = 'blue', marker = '.', s = 180) 
            ax.scatter(scenario.third_cluster_points_x_list,  scenario.third_cluster_points_y_list, color = 'green', marker = '.', s = 180) 
            
            
        # plot map points - lane - black 
        #print("center lane point number:", len(scenario.lane_black_list[0]))
        #if len(scenario.lane_black_list) > 0:
        #    ax.scatter(scenario.lane_black_list[0], scenario.lane_black_list[1], color = center_line_color, marker = '.', linewidths = 0.2)          

    if left_right_boundary_plot:
        print("      ====== plot lane boundary begin !")
        for lane in scenario.center_lane_list:
            lane_id = lane.lane_id
            #print(" --- p --- lane id is :", lane_id)
            left_boundary_x_list = lane.left_boundary_point_x_list
            left_boundary_y_list = lane.left_boundary_point_y_list
            right_boundary_x_list = lane.right_boundary_point_x_list
            right_boundary_y_list = lane.right_boundary_point_y_list  
            #print(" --- lane boundary left points number: ", len(left_boundary_x_list))
            #print(left_boundary_x_list)
            #print(" --- lane boundary right points number: ", len(right_boundary_x_list))
            #ax.scatter(lane.point_x_list, lane.point_y_list, c = 'white', marker = '.', linewidths = 0.4) 
            ax.scatter(left_boundary_x_list, left_boundary_y_list, c = 'red', marker = '.', linewidths = 0.4) 
            ax.scatter(right_boundary_x_list, right_boundary_y_list, c = 'blue', marker = '.', linewidths = 0.4) 

    if left_right_turn_plot:
        for center_lane in scenario.center_lane_list:
            center_lane_id = center_lane.lane_id
            if center_lane.is_main_lane:
                if center_lane.is_right_turn:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, c = 'red', marker = '.', linewidths = 0.4) 
                if center_lane.is_left_turn:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, c = 'blue', marker = '.', linewidths = 0.4) 
                    

    if topology_plot:
        
        print("      ====== topology plot enabled !!!!!")
     
        # print center id too
        center_lane_index = 0
                     
        for center_lane in scenario.center_lane_list:  
            random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
            ax.scatter(center_lane.point_x_list, center_lane.point_y_list, c = random_color[0], marker = '.', linewidths = 0.4) 
            # plot first point in lane
            ax.scatter(center_lane.point_x_list[0], center_lane.point_y_list[0], c = random_color[0], marker = 'o', linewidths = 0.4) 
            # plot last point in lane
            ax.scatter(center_lane.point_x_list[-1], center_lane.point_y_list[-1], c = random_color[0], marker = '^', linewidths = 0.4) 
            center_lane_point_number = len(center_lane.point_x_list)
            if center_lane_point_number != 0:
                middle_index = int(0.5 * center_lane_point_number)
                middle_point_x = center_lane.point_x_list[middle_index]
                middle_point_y = center_lane.point_y_list[middle_index]
                ax.text(middle_point_x, middle_point_y, str(int(center_lane.lane_id)), c = random_color[0], fontsize = 12)
                
            #ax.scatter(center_lane.point_x_list, center_lane.point_y_list, color = center_line_color, marker = '.', linewidths = 0.1) 
            center_lane_index += 1
            #if center_lane_index == 1:
            #    break        
        
        
        # print boundary lines
        if False:
            for road_line in scenario.road_line_list:
                road_lane_id = road_line.line_id
                #random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                #ax.scatter(road_line.point_x_list, road_line.point_y_list, c = random_color[0], marker = '.', linewidths = 0.4) 
                ax.scatter(road_line.point_x_list, road_line.point_y_list, c = 'black', marker = '.', linewidths = 0.4) 
                road_lane_point_number = len(road_line.point_x_list)
                if road_lane_point_number != 0:
                    middle_index = int(0.5 * road_lane_point_number)
                    middle_point_x = road_line.point_x_list[middle_index]
                    middle_point_y = road_line.point_y_list[middle_index]
                    #ax.text(middle_point_x, middle_point_y, str(int(road_line.lane_id)), c = random_color[0], fontsize = 12)
                    ax.text(middle_point_x, middle_point_y, str(int(road_line.line_id)), c = 'black', fontsize = 12)

        # plot conflict points
        if False:
            ax.scatter(scenario.node_x, scenario.node_y, c = 'yellow', marker = 'o', linewidths = 4) 
            for center_lane in scenario.center_lane_list:
                try:
                    conflict_point_dict = center_lane.dict["crossing"]
                    for other_lane_id, conflict_dict in conflict_point_dict.items():
                        conflict_location = conflict_dict["location"]                                  
                        ax.scatter(conflict_location[0], conflict_location[1], c = 'red', marker = '*', linewidths = 2) 
                except:
                    print("")
                    
    if lane_and_boundaries_plot:  
    #if lane_polygon_plot: 
        plot_index = 0  
        
        print("      ====== lane and boundary plot enabled !!!!!")
        
        # print center lanes                 
        for center_lane in scenario.center_lane_list:         
            center_lane_id = center_lane.lane_id
            
            if center_lane_id == 153 or center_lane_id == 159:
                print(" found 159 and 153 !!! ")      
            
            if center_lane_id != 188 and center_lane_id != 273 and center_lane_id != 276:
                continue
                        
            random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
            ax.scatter(center_lane.point_x_list, center_lane.point_y_list, c = random_color[0], marker = '.', linewidths = 0.4) 
            center_lane_point_number = len(center_lane.point_x_list)
            if center_lane_point_number != 0:
                middle_index = int(0.5 * center_lane_point_number)
                middle_point_x = center_lane.point_x_list[middle_index]
                middle_point_y = center_lane.point_y_list[middle_index]
                ax.text(middle_point_x, middle_point_y, str(int(center_lane.lane_id)), c = random_color[0], fontsize = 12)
    
                for left_segment in center_lane.left_boundary_segment_list:
                    segment_x_list = []
                    segment_y_list = []
                    left_lane_start_index = left_segment.lane_start_index
                    left_lane_end_index = left_segment.lane_end_index
                    
                    left_boundary_line_id = left_segment.boundary_feature_id
                    
                    #boundary_line = self.road_line_dict[boundary_line_id]
                    boundary_line = scenario.map_feature_dict[left_boundary_line_id]                
                    boundary_line_x_list = boundary_line.point_x_list
                    boundary_line_y_list = boundary_line.point_y_list
                    ax.scatter(boundary_line_x_list, boundary_line_y_list, c = 'black', marker = '.', linewidths = 0.4) 
                    segment_lane_point_number = len(boundary_line_x_list)
                    if segment_lane_point_number != 0:
                        middle_index = int(0.5 * segment_lane_point_number)
                        middle_point_x = boundary_line_x_list[middle_index]
                        middle_point_y = boundary_line_y_list[middle_index]
                        ax.text(middle_point_x, middle_point_y, 'left' +  str(int(left_boundary_line_id)), c = 'black', fontsize = 12)

                    if left_lane_start_index == left_lane_end_index:
                        continue
                    
                    for i in range(left_lane_start_index, left_lane_end_index):
                        point_x = center_lane.point_x_list[i]
                        point_y = center_lane.point_y_list[i]
                        segment_x_list.append(point_x)
                        segment_y_list.append(point_y)
                    ax.scatter(segment_x_list, segment_y_list, c = random_color[0], marker = '.', linewidths = 0.4) 
                    segment_lane_point_number = len(segment_x_list)
                    if segment_lane_point_number != 0:
                        middle_index = int(0.5 * segment_lane_point_number)
                        middle_point_x = segment_x_list[middle_index]
                        middle_point_y = segment_y_list[middle_index]
                        ax.text(middle_point_x, middle_point_y, 'segemnt', c = random_color[0], fontsize = 12)
                                    
                for right_segment in center_lane.right_boundary_segment_list:
                    segment_x_list = []
                    segment_y_list = []
                    right_lane_start_index = right_segment.lane_start_index
                    right_lane_end_index = right_segment.lane_end_index
                    
                    right_boundary_line_id = right_segment.boundary_feature_id
                    
                    #boundary_line = self.road_line_dict[boundary_line_id]
                    boundary_line = scenario.map_feature_dict[right_boundary_line_id]                
                    boundary_line_x_list = boundary_line.point_x_list
                    boundary_line_y_list = boundary_line.point_y_list
                    ax.scatter(boundary_line_x_list, boundary_line_y_list, c = 'black', marker = '.', linewidths = 0.4) 
                    segment_lane_point_number = len(boundary_line_x_list)
                    if segment_lane_point_number != 0:
                        middle_index = int(0.5 * segment_lane_point_number)
                        middle_point_x = boundary_line_x_list[middle_index]
                        middle_point_y = boundary_line_y_list[middle_index]
                        ax.text(middle_point_x, middle_point_y, 'right' +  str(int(right_boundary_line_id)), c = 'black', fontsize = 12)

                    if right_lane_start_index == right_lane_end_index:
                        continue
                    
                    for i in range(right_lane_start_index, right_lane_end_index):
                        point_x = center_lane.point_x_list[i]
                        point_y = center_lane.point_y_list[i]
                        segment_x_list.append(point_x)
                        segment_y_list.append(point_y)
                    ax.scatter(segment_x_list, segment_y_list, c = random_color[0], marker = '.', linewidths = 0.4) 
                    segment_lane_point_number = len(segment_x_list)
                    if segment_lane_point_number != 0:
                        middle_index = int(0.5 * segment_lane_point_number)
                        middle_point_x = segment_x_list[middle_index]
                        middle_point_y = segment_y_list[middle_index]
                        ax.text(middle_point_x, middle_point_y, 'segemnt', c = random_color[0], fontsize = 12)                    


    if straight_lane_plot:     
        print("      ====== straight lane plot enabled !!!!!")
        center_lane_index = 0      
        for center_lane in scenario.center_lane_list:  
            if center_lane.is_straight:
                if center_lane.lane_angle >= -20 and center_lane.lane_angle <= 20:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, color = 'red', marker = '.', linewidths = 0.4) 
                if center_lane.lane_angle >= 70 and center_lane.lane_angle <= 110:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, color = 'blue', marker = '.', linewidths = 0.4)                     
                if center_lane.lane_angle >= 160 and center_lane.lane_angle <= 180:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, color = 'green', marker = '.', linewidths = 0.4)                     
                if center_lane.lane_angle >= -180 and center_lane.lane_angle <= -160:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, color = 'green', marker = '.', linewidths = 0.4) 
                if center_lane.lane_angle >= -110 and center_lane.lane_angle <= -70:
                    ax.scatter(center_lane.point_x_list, center_lane.point_y_list, color = 'yellow', marker = '.', linewidths = 0.4) 
    
    if lane_polygon_plot:  
    #if False:     
        print("      ====== lane polygon plot enabled !!!!!")   
        plot_record = 0
        for center_lane in scenario.center_lane_list:
            center_lane_id = center_lane.lane_id
            if center_lane.polygon != None:
                new_polygon = copy(center_lane.polygon)
                ax.add_patch(new_polygon) 
                center_lane_point_number = len(center_lane.point_x_list)
                if center_lane_point_number != 0:
                    middle_index = int(0.5 * center_lane_point_number)
                    middle_point_x = center_lane.point_x_list[middle_index]
                    middle_point_y = center_lane.point_y_list[middle_index]
                    ax.text(middle_point_x, middle_point_y, str(int(center_lane.lane_id)), c = 'black', fontsize = 12)                      
                plot_record += 1
                
        if plot_signal_light == "True":
            print("      ====== signal light plot enabled !!!!!")
            signal_background_color = 'white'
            
            for signal_id, signal_light_dict in scenario.signal_light_dict.items():
                signal_light_lane_id = signal_light_dict["lane_id"]
                signal_location = signal_light_dict["location"]
                signal_color_dict = signal_light_dict["color"]
                signal_color = signal_color_dict[time_index]
                
                #print(" --- id, lane, color, location is : ", signal_id, signal_light_lane_id, signal_color, signal_location)
                singal_shape = "circle"
                signal_show = "steady"
                if signal_color != "None":
                    ax.plot(signal_location[0], signal_location[1], marker = 'o', color = signal_background_color, markersize = 20)
                    ax.plot(signal_location[0], signal_location[1], marker = 'o', color = signal_color, markersize = 14)                
            
            scenario_signal_light_state = scenario.signal_light_state_dict[time_index]
            for lane_id in scenario_signal_light_state.lane_id_list:
                lane_signal_state = scenario_signal_light_state.lane_signal_state_dict[lane_id]
                signal_light_x = lane_signal_state["x"]
                signal_light_y = lane_signal_state["y"]
                signal_light_z = lane_signal_state["z"]
                signal_state = lane_signal_state["signal_state"]
                
                signal_color = None
                singal_shape = None
                signal_show = None                
                
                if signal_state == 0:
                    continue
                if signal_state == 1:
                    signal_color = "red"
                    singal_shape = "arrow"
                    signal_show = "steady"
                if signal_state == 2:
                    signal_color = "yellow"
                    singal_shape = "arrow"
                    signal_show = "steady"
                if signal_state == 3:
                    signal_color = "green"
                    singal_shape = "arrow"
                    signal_show = "steady"

                if signal_state == 4:
                    signal_color = "red"  
                    singal_shape = "circle"
                    signal_show = "steady"
                if signal_state == 5:
                    signal_color = "yellow"
                    singal_shape = "circle"
                    signal_show = "steady"
                if signal_state == 6:
                    signal_color = "green"
                    singal_shape = "circle"
                    signal_show = "steady"

                if signal_state == 7:
                    signal_color = "red"
                    singal_shape = "circle"
                    signal_show = "flashing"
                if signal_state == 8:
                    signal_color = "yellow"
                    singal_shape = "circle"        
                    signal_show = "flashing"
                    
                if singal_shape == "arrow" and signal_show == "steady":
                    #print("      ====== signal light arrow !!!!!")
                    ax.arrow(signal_light_x, signal_light_y, 0, 0, color = signal_background_color, width = 1)
                    ax.arrow(signal_light_x, signal_light_y, 0, 0, color = signal_color, width = 0.7)
                    
                    
                if singal_shape == "circle" and signal_show == "steady":
                    #print("      ====== signal light circle !!!!!")
                    #print("      ====== signal light location: ", signal_light_x, signal_light_y)
                    ax.plot(signal_light_x, signal_light_y, marker = 'o', color = signal_background_color, markersize = 20)
                    ax.plot(signal_light_x, signal_light_y, marker = 'o', color = signal_color, markersize = 14)
                    

                if signal_show == "flashing":
                    #print("      ====== signal light flashing !!!!!")
                    ax.plot(signal_light_x, signal_light_y, marker = 'H', color = signal_background_color, markersize = 20)
                    ax.plot(signal_light_x, signal_light_y, marker = 'H', color = signal_color, markersize = 14)
                    
                    
    # plot stop sign - red
    if len(scenario.stop_sign_list) > 0:
        ax.scatter(scenario.stop_sign_list[0], scenario.stop_sign_list[1], color = stop_sign_color, marker = '^', linewidths = 2)
    
    # plot cross walk - darkorange   
    if len(scenario.cross_walk_list) > 0:
        for crosswalk in scenario.cross_walk_list:
            plt.plot(crosswalk.point_x_list, crosswalk.point_y_list, color = crossing_line_color, linestyle = '--', linewidth = 2)  
            plt.plot([crosswalk.point_x_list[0], crosswalk.point_x_list[-1]], [crosswalk.point_y_list[0],crosswalk.point_y_list[-1]], color = 'darkorange', linestyle = '--', linewidth = 2)  
        
    # plot road edge - brown
    if len(scenario.road_edge_list) > 0:
        ax.scatter(scenario.road_edge_x_list, scenario.road_edge_y_list, color = edge_line_color, marker = '.', linewidths = 0.3)      

    if agents_trajectories_plot:
        print("      ====== plot trajectories enabled !!!")
        ax.scatter(scenario.broken_single_white_x_list, scenario.broken_single_white_y_list, color = white_line_color, marker = '.', linewidths = map_line_width)
        ax.scatter(scenario.solid_single_white_x_list,  scenario.solid_single_white_y_list,  color = white_line_color, marker = '.', linewidths = map_line_width)
        ax.scatter(scenario.solid_double_white_x_list,  scenario.solid_double_white_y_list,  color = white_line_color, marker = '.', linewidths = map_line_width)
        
        # plot map points - yellow 
        ax.scatter(scenario.broken_single_yellow_x_list,  scenario.broken_single_yellow_y_list, color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.broken_double_yellow_x_list,  scenario.broken_double_yellow_y_list, color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.solid_single_yellow_x_list,   scenario.solid_single_yellow_y_list,  color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.solid_double_yellow_x_list,   scenario.solid_double_yellow_y_list,  color = yellow_line_color, marker = '.', linewidths = map_line_width)  
        ax.scatter(scenario.passing_double_yellow_x_list, scenario.passing_double_yellow_y_list,color = yellow_line_color, marker = '.', linewidths = map_line_width)          

        for agent in scenario.agents_list: 
            random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
            agent_x_list = []
            agent_y_list = []        
            if agent.states_array.size == 0:
                continue    
            states = agent.states_array
            first_state = states[0]
            state_number = len(states)
            agent_type = first_state[2]
            agent_id = int(first_state[1])
            
            state_index = 0
            while state_index < state_number:
                state = agent.states_array[state_index, :]             
                if state[12] == '-1':
                    state_index += 1
                    continue            
                agent_x = state[3]
                agent_y = state[4]
                agent_yaw    = radian_to_degree(state[9])
                agent_length = state[6]
                agent_width  = state[7]
                agent_height = state[8]
                agent_velocity  = math.sqrt(state[10]**2 + state[11]**2)                        
                agent_x_list.append(agent_x)
                agent_y_list.append(agent_y)
                state_index += 1
                
            # draw surrounding vehicles
            if agent_type == 1:           
                ax.scatter(agent_x_list, agent_y_list, color = random_color[0], marker='.', linewidths = 0.4)   
            # draw surrounding pedestrians                         
            if agent_type == 2:           
                ax.scatter(agent_x_list, agent_y_list, color = random_color[0], marker='.', linewidths = 0.4) 
            # draw cyclist
            if agent_type == 3:           
                ax.scatter(agent_x_list, agent_y_list, color = random_color[0], marker='.', linewidths = 0.4)   
            # draw sdc
            if agent_type == -1:           
                ax.scatter(agent_x_list, agent_y_list, color = random_color[0], marker='.', linewidths = 0.4)   
            
            trajectory_point_number = len(agent_x_list)
            if trajectory_point_number != 0:
                middle_index = int(0.5 * trajectory_point_number)
                middle_point_x = agent_x_list[middle_index]
                middle_point_y = agent_y_list[middle_index]
                ax.text(middle_point_x, middle_point_y, str(agent_id), c = random_color[0], fontsize = 12)
                if agent_type == -1:
                    ax.text(middle_point_x, middle_point_y, str(agent_id) + ": sdc", c = random_color[0], fontsize = 12)
        
    if agents_trajectories_plot == False:         
        # ========== part2: plot agents states ========== 
        
        print("      ====== plot agents for one moment")
        
        for agent in scenario.agents_list:
            
            if agent.states_array.size == 0:
                continue    
            
            state = agent.states_array[time_index, :]
                
            if state[12] == '-1':
                continue
            
            agent_x = state[3]
            agent_y = state[4]
            agent_yaw    = radian_to_degree(state[9])
            agent_yaw_for_rectangle = radian_to_360_degree(state[9])
            agent_length = state[6]
            agent_width  = state[7]
            agent_height = state[8]
            agent_velocity  = math.sqrt(state[10]**2 + state[11]**2)
            agent_type = state[2]
            agent_id = int(state[1])
                       
            # draw surrounding vehicles
            if agent_type == 1:           
                if agent_shape == "rectangle":

                    rectangle_list = generate_rectangle_point(agent_x, agent_y, agent_width, agent_length, agent_yaw_for_rectangle)
                    rec_polygon = Polygon(rectangle_list, color = 'blue', alpha = 1)
                    ax.add_patch(rec_polygon)
                                    
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)                             
                
                if agent_shape == "circle":
                    ax.scatter(agent_x, agent_y, marker='o', linewidths = agent_round_shape_width, color='blue',)
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)            
    
                if show_agnets_id == "True":
                    ax.text(agent_x + agents_id_x_offset, agent_y + agents_id_y_offset, "{}".format(agent_id), color = 'cyan', fontsize = 12)     
                    
                if show_agents_coordinates == "True":
                    ax.text(agent_x + agents_coordinates_x_offset, agent_y + agents_coordinates_y_offset, "({:.2f},{:.2f})".format(agent_x,agent_y), color = 'blue', fontsize = 12)  
    
            # draw surrounding pedestrians        
            if agent_type == 2:            
                if agent_shape == "rectangle":
                    rectangle_list = generate_rectangle_point(agent_x, agent_y, agent_width, agent_length, agent_yaw_for_rectangle)
                    rec_polygon = Polygon(rectangle_list, color = 'green', alpha = 1)
                    ax.add_patch(rec_polygon)
                                    
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)                             
                    
                if agent_shape == "circle":
                    ax.scatter(agent_x, agent_y, marker='o', linewidths = agent_round_shape_width, color='green',)
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)  
    
                if show_agnets_id == "True":
                    ax.text(agent_x + agents_id_x_offset, agent_y + agents_id_y_offset, "{}".format(agent_id), color = 'green', fontsize = 12)              
    
                if show_agents_coordinates == "True":
                    ax.text(agent_x + agents_coordinates_x_offset, agent_y + agents_coordinates_y_offset, "({:.2f},{:.2f})".format(agent_x,agent_y), color = 'green', fontsize = 12) 
    
            # draw surrounding cyclists        
            if agent_type == 3: 
                if agent_shape == "rectangle":
                    rectangle_list = generate_rectangle_point(agent_x, agent_y, agent_width, agent_length, agent_yaw_for_rectangle)
                    rec_polygon = Polygon(rectangle_list, color = 'yellow', alpha = 1)
                    ax.add_patch(rec_polygon)
                                    
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)                             
      
                if agent_shape == "circle":
                    ax.scatter(agent_x, agent_y, marker='o', linewidths = agent_round_shape_width, color='yellow',)
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)             
    
                if show_agnets_id == "True":
                    ax.text(agent_x + agents_id_x_offset, agent_y + agents_id_y_offset, "{}".format(agent_id), color = 'yellow', fontsize = 12)  
    
                if show_agents_coordinates == "True":
                    ax.text(agent_x + agents_coordinates_x_offset, agent_y + agents_coordinates_y_offset, "({:.2f},{:.2f})".format(agent_x,agent_y), color = 'yellow', fontsize = 12) 
    
            # draw self-driving car itself        
            if agent_type == -1: 
                #print("this is a sdc !")
                if agent_shape == "rectangle":
                    rectangle_list = generate_rectangle_point(agent_x, agent_y, agent_width, agent_length, agent_yaw_for_rectangle)
                    rec_polygon = Polygon(rectangle_list, color = 'red', alpha = 1)
                    ax.add_patch(rec_polygon)
                                    
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)                             
   
    
                if agent_shape == "circle":
                    ax.scatter(agent_x, agent_y, marker='o', linewidths = agent_round_shape_width, color='red',)
                    x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, agent_x, 
                                                               agent_y, agent_length, agent_width)
                    ax.arrow(agent_x, agent_y, delta_x, delta_y, width = 0.1)
                    ax.text(agent_x + delta_x, agent_y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)   
    
                if show_agnets_id == "True":
                    ax.text(agent_x + agents_id_x_offset, agent_y + agents_id_y_offset, "{}".format(agent_id), color = 'red', fontsize = 12)  
    
                if show_agents_coordinates == "True":
                    ax.text(agent_x + agents_coordinates_x_offset, agent_y + agents_coordinates_y_offset, "({:.2f},{:.2f})".format(agent_x,agent_y), color = 'red', fontsize = 12)             
    
                
                if plot_preceeding_following_in_lane == "True":
                    #print("-------- need to plot agents in the same lane --------")
                    #print(" ") 
                    #print(" ")                   
                    
                    '''
                    preceeding_vehicle_id, \
                    preceeding_vehicle_location, \
                    preceeding_to_ego_distance, \
                    following_vehicle_id, \
                    following_vehicle_location, \
                    following_to_ego_distance = find_preceeding_following_in_lane(scenario, 
                                                            time_index,
                                                            agent_id,
                                                            agent_x, 
                                                            agent_y, 
                                                            agent_length, 
                                                            agent_width, 
                                                            agent_yaw)
                    '''
                    
                    preceeding_vehicle_id = agent.preceeding_vehicle_id_list[time_index]
                    preceeding_to_ego_distance = agent.preceeding_vehicle_distance_list[time_index]
                    preceeding_vehicle_distance_to_ego_lane_end = agent.preceeding_vehicle_distance_to_ego_lane_end_list[time_index]
                    preceeding_vehicle_location = agent.preceeding_vehicle_location_list[time_index]
        
                    following_vehicle_id = agent.following_vehicle_id_list[time_index]
                    following_to_ego_distance = agent.following_vehicle_distance_list[time_index]
                    following_vehicle_distance_to_ego_lane_end = agent.following_vehicle_distance_to_ego_lane_end_list[time_index]
                    following_vehicle_location = agent.following_vehicle_location_list[time_index]
                  
                    plot_color = plot_preceeding_following_in_lane_color
                    plot_style = None
                    if plot_preceeding_following_in_lane_style == "solid":
                        plot_style = '-'
                    if plot_preceeding_following_in_lane_style == "dashed":
                        plot_style = '--'                   
                    if plot_preceeding_following_in_lane_style == "doted":
                        plot_style = '.'
                        
                    print(" --- preceeding vehicle id is: ", preceeding_vehicle_id)
                    print(" --- following vehicle id is: ", following_vehicle_id)
                    print(" --- preceeding vehicle location is: ", preceeding_vehicle_location)
                    print(" --- following vehicle location is: ", following_vehicle_location)
                    
                    if preceeding_vehicle_id != "None" and following_vehicle_id != "None":
                        preceeding_vehicle_x = [agent_x, preceeding_vehicle_location[0]]
                        preceeding_vehicle_y = [agent_y, preceeding_vehicle_location[1]]
                        following_vehicle_x = [agent_x, following_vehicle_location[0]]
                        following_vehicle_y = [agent_y, following_vehicle_location[1]]
                                         
                        ax.plot(preceeding_vehicle_x,  preceeding_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width) 
                        distance_text_x = 0.5 * (agent_x + preceeding_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + preceeding_vehicle_location[1])
                        ax.text(distance_text_x, distance_text_y, "{:.2f}".format(preceeding_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                        
                        ax.plot(following_vehicle_x,  following_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width) 
                        distance_text_x = 0.5 * (agent_x + following_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + following_vehicle_location[1])
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(following_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                    
                    if preceeding_vehicle_id != "None" and following_vehicle_id == "None":
                        preceeding_vehicle_x = [agent_x, preceeding_vehicle_location[0]]
                        preceeding_vehicle_y = [agent_y, preceeding_vehicle_location[1]]
                        distance_text_x = 0.5 * (agent_x + preceeding_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + preceeding_vehicle_location[1])                  
                        ax.plot(preceeding_vehicle_x,  preceeding_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width)                     
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(preceeding_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                    
                    if preceeding_vehicle_id == "None" and following_vehicle_id != "None":
                        following_vehicle_x = [agent_x, following_vehicle_location[0]]
                        following_vehicle_y = [agent_y, following_vehicle_location[1]]
                        distance_text_x = 0.5 * (agent_x + following_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + following_vehicle_location[1])    
                        
                        ax.plot(following_vehicle_x,  following_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width)
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(following_to_ego_distance) + "m", color = plot_color, fontsize = 12)                     
                
                    if preceeding_vehicle_id == "None" and following_vehicle_id == "None":
                        print(" ")
    
                #print(" --- left lane plot :", plot_preceeding_following_in_left_lane)
                if plot_preceeding_following_in_left_lane == "True":
                    
                    #print("-------- need to plot agents in left lane --------")
                    #print(" ") 
                    #print(" ") 
                    
                    '''
                    preceeding_vehicle_id, \
                    preceeding_vehicle_location, \
                    preceeding_to_ego_distance, \
                    following_vehicle_id, \
                    following_vehicle_location, \
                    following_to_ego_distance = find_preceeding_following_in_left_lane(scenario, 
                                                            time_index,
                                                            agent_id,
                                                            agent_x, 
                                                            agent_y, 
                                                            agent_length, 
                                                            agent_width, 
                                                            agent_yaw)
                    '''
                    
                    preceeding_vehicle_id = agent.left_preceeding_vehicle_id_list[time_index]
                    preceeding_to_ego_distance = agent.left_preceeding_vehicle_distance_list[time_index]
                    preceeding_vehicle_distance_to_ego_lane_end = agent.left_preceeding_vehicle_distance_to_ego_lane_end_list[time_index]
                    preceeding_vehicle_location = agent.left_preceeding_vehicle_location_list[time_index]
        
                    following_vehicle_id = agent.left_following_vehicle_id_list[time_index]
                    following_to_ego_distance = agent.left_following_vehicle_distance_list[time_index]
                    following_vehicle_distance_to_ego_lane_end = agent.left_following_vehicle_distance_to_ego_lane_end_list[time_index]
                    following_vehicle_location = agent.left_following_vehicle_location_list[time_index]
                    
                    plot_color = plot_preceeding_following_in_left_lane_color
                    plot_style = None
                    if plot_preceeding_following_in_lane_style == "solid":
                        plot_style = "-"
                    if plot_preceeding_following_in_lane_style == "dashed":
                        plot_style = "--"                    
                    if plot_preceeding_following_in_lane_style == "doted":
                        plot_style = "."   
                        
                    #print("left preceeding vehicle id is: ", preceeding_vehicle_id)
                    #print("left following vehicle id is: ", following_vehicle_id)
                    #print("left preceeding vehicle location is: ", preceeding_vehicle_location)
                    #print("left following vehicle location is: ", following_vehicle_location)
                    
                    if preceeding_vehicle_id != "None" and following_vehicle_id != "None":
                        preceeding_vehicle_x = [agent_x, preceeding_vehicle_location[0]]
                        preceeding_vehicle_y = [agent_y, preceeding_vehicle_location[1]]
                        following_vehicle_x = [agent_x, following_vehicle_location[0]]
                        following_vehicle_y = [agent_y, following_vehicle_location[1]]
                        
                        ax.plot(preceeding_vehicle_x,  preceeding_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width) 
                        distance_text_x = 0.5 * (agent_x + preceeding_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + preceeding_vehicle_location[1])  
                        ax.text(distance_text_x, distance_text_y, "{:.2f}".format(preceeding_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                        
                        ax.plot(following_vehicle_x,  following_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width) 
                        distance_text_x = 0.5 * (agent_x + following_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + following_vehicle_location[1])
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(following_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                    
                    if preceeding_vehicle_id != "None" and following_vehicle_id == "None":
                        preceeding_vehicle_x = [agent_x, preceeding_vehicle_location[0]]
                        preceeding_vehicle_y = [agent_y, preceeding_vehicle_location[1]]
                        distance_text_x = 0.5 * (agent_x + preceeding_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + preceeding_vehicle_location[1])                     
                        ax.plot(preceeding_vehicle_x,  preceeding_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width)                     
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(preceeding_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                    
                    if preceeding_vehicle_id == "None" and following_vehicle_id != "None":
                        following_vehicle_x = [agent_x, following_vehicle_location[0]]
                        following_vehicle_y = [agent_y, following_vehicle_location[1]]
                        distance_text_x = 0.5 * (agent_x + following_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + following_vehicle_location[1])                    
                        ax.plot(following_vehicle_x,  following_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width)
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(following_to_ego_distance) + "m", color = plot_color, fontsize = 12)                     
                
                    if preceeding_vehicle_id == "None" and following_vehicle_id == "None":
                        print("")
                
                #print(" --- right lane plot :", plot_preceeding_following_in_right_lane)
                if plot_preceeding_following_in_right_lane == "True":
                    
                    #print("-------- need to plot agents in right lane --------")
                    #print(" ") 
                    #print(" ")  
                    
                    '''
                    preceeding_vehicle_id, \
                    preceeding_vehicle_location, \
                    preceeding_to_ego_distance, \
                    following_vehicle_id, \
                    following_vehicle_location, \
                    following_to_ego_distance = find_preceeding_following_in_right_lane(scenario, 
                                                            time_index,
                                                            agent_id,
                                                            agent_x, 
                                                            agent_y, 
                                                            agent_length, 
                                                            agent_width, 
                                                            agent_yaw)
                    '''
                    
                    preceeding_vehicle_id = agent.right_preceeding_vehicle_id_list[time_index]
                    preceeding_to_ego_distance = agent.right_preceeding_vehicle_distance_list[time_index]
                    preceeding_vehicle_distance_to_ego_lane_end = agent.right_preceeding_vehicle_distance_to_ego_lane_end_list[time_index]
                    preceeding_vehicle_location = agent.right_preceeding_vehicle_location_list[time_index]
        
                    following_vehicle_id = agent.right_following_vehicle_id_list[time_index]
                    following_to_ego_distance = agent.right_following_vehicle_distance_list[time_index]
                    following_vehicle_distance_to_ego_lane_end = agent.right_following_vehicle_distance_to_ego_lane_end_list[time_index]
                    following_vehicle_location = agent.right_following_vehicle_location_list[time_index]
                    
                    plot_color = plot_preceeding_following_in_right_lane_color
                    plot_style = None
                    if plot_preceeding_following_in_lane_style == "solid":
                        plot_style = "-"
                    if plot_preceeding_following_in_lane_style == "dashed":
                        plot_style = "--"                    
                    if plot_preceeding_following_in_lane_style == "doted":
                        plot_style = "."   
                        
                    #print("right preceeding vehicle id is: ", preceeding_vehicle_id)
                    #print("right following vehicle id is: ", following_vehicle_id)
                    #print("right preceeding vehicle location is: ", preceeding_vehicle_location)
                    #print("right following vehicle location is: ", following_vehicle_location)
                    
                    if preceeding_vehicle_id != "None" and following_vehicle_id != "None":
                        preceeding_vehicle_x = [agent_x, preceeding_vehicle_location[0]]
                        preceeding_vehicle_y = [agent_y, preceeding_vehicle_location[1]]
                        following_vehicle_x = [agent_x, following_vehicle_location[0]]
                        following_vehicle_y = [agent_y, following_vehicle_location[1]]
                        
                        ax.plot(preceeding_vehicle_x,  preceeding_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width) 
                        distance_text_x = 0.5 * (agent_x + preceeding_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + preceeding_vehicle_location[1])  
                        ax.text(distance_text_x, distance_text_y, "{:.2f}".format(preceeding_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                        
                        ax.plot(following_vehicle_x,  following_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width) 
                        distance_text_x = 0.5 * (agent_x + following_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + following_vehicle_location[1])
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(following_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                    
                    if preceeding_vehicle_id != "None" and following_vehicle_id == "None":
                        preceeding_vehicle_x = [agent_x, preceeding_vehicle_location[0]]
                        preceeding_vehicle_y = [agent_y, preceeding_vehicle_location[1]]
                        distance_text_x = 0.5 * (agent_x + preceeding_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + preceeding_vehicle_location[1])                     
                        ax.plot(preceeding_vehicle_x,  preceeding_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width)                     
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(preceeding_to_ego_distance) + "m", color = plot_color, fontsize = 12)
                    
                    if preceeding_vehicle_id == "None" and following_vehicle_id != "None":
                        following_vehicle_x = [agent_x, following_vehicle_location[0]]
                        following_vehicle_y = [agent_y, following_vehicle_location[1]]
                        distance_text_x = 0.5 * (agent_x + following_vehicle_location[0])
                        distance_text_y = 0.5 * (agent_y + following_vehicle_location[1])                    
                        ax.plot(following_vehicle_x,  following_vehicle_y, color = plot_color,  linestyle = plot_style, linewidth = plot_preceeding_following_line_width)
                        ax.text(distance_text_x, distance_text_y,  "{:.2f}".format(following_to_ego_distance) + "m", color = plot_color, fontsize = 12)                     
                
                    if preceeding_vehicle_id == "None" and following_vehicle_id == "None":
                        print("")
            
    # Title.
    title = "agents states"
    ax.set_title(title, fontsize = 20)
    ax.set_xlabel("x(m)", fontsize = 20)
    ax.set_ylabel("y(m)", fontsize = 20)
    ax.tick_params(axis = 'x', labelsize = 20)
    ax.tick_params(axis = 'y', labelsize = 20)
    
    # Set axes.
    size = max(40, width * 1.0)
    ax.axis([-size / 2 + center_x, size / 2 + center_x, -size / 2 + center_y, size / 2 + center_y])
    ax.set_aspect('equal')
    
    image = _generate_fig_canvas_image(fig)
    fig.clear()
    plt.clf() 
    plt.close("all") 
    return image        


    
def generate_all_agents_plot_in_one_scenario(scenario, dpi, size_pixels, size_inch, plot_path):
    
    fig, ax = create_figure_and_axes(dpi, size_pixels, size_inch)
    
    center_x, center_y, width = _generate_best_fit_view_point(scenario)
    
    # ========== part1: plot map points ==========       
    # plot map points - white
    ax.scatter(scenario.broken_single_white_list[0], scenario.broken_single_white_list[1], color = 'w', marker = '.', linewidths = 0.3)
    ax.scatter(scenario.solid_single_white_list[0],  scenario.solid_single_white_list[1],  color = 'w', marker = '.', linewidths = 0.3)
    ax.scatter(scenario.solid_double_white_list[0],  scenario.solid_double_white_list[1],  color = 'w', marker = '.', linewidths = 0.3)
    
    # plot map points - yellow    
    ax.scatter(scenario.broken_single_yellow_list[0],  scenario.broken_single_yellow_list[1], color = 'yellow', marker = '.', linewidths = 0.3)  
    ax.scatter(scenario.broken_double_yellow_list[0],  scenario.broken_double_yellow_list[1], color = 'yellow', marker = '.', linewidths = 0.3)  
    ax.scatter(scenario.solid_single_yellow_list[0],   scenario.solid_single_yellow_list[1],  color = 'yellow', marker = '.', linewidths = 0.3)  
    ax.scatter(scenario.solid_double_yellow_list[0],   scenario.solid_double_yellow_list[1],  color = 'yellow', marker = '.', linewidths = 0.3)  
    ax.scatter(scenario.passing_double_yellow_list[0], scenario.passing_double_yellow_list[1],color = 'yellow', marker = '.', linewidths = 0.3)  

    # plot map points - lane - black 
    #print("center lane point number:", len(scenario.lane_black_list[0]))
    ax.scatter(scenario.lane_black_list[0], scenario.lane_black_list[1], color = 'red', marker = '.', linewidths = 0.3)      

    # plot stop sign - red
    ax.scatter(scenario.stop_sign_list[0], scenario.stop_sign_list[1], color = 'red', marker = '^', linewidths = 10)
    
    # plot cross walk - darkyellow    
    ax.scatter(scenario.cross_walk_list[0], scenario.cross_walk_list[1], color = 'darkorange', marker = '.', linewidths = 3)  
    
    # plot road edge - brown
    ax.scatter(scenario.road_edge_list[0], scenario.road_edge_list[1], color = 'saddlebrown', marker = '.', linewidths = 0.3)      
      
    # ========== part2: plot agents states ==========        
    for agent in scenario.agents_list:
        states = agent.states_array
        states_mask = agent.states_mask
        masked_x = states[:, 3][states_mask]
        masked_y = states[:, 4][states_mask]
                
        for state in states:
            if state.valid == '-1':
                continue
            agent_x = state.x
            agent_y = state.y
            agent_yaw    = radian_to_degree(state.heading)
            agent_length = state.length
            agent_width  = state.width
            agent_height = state.height
            agent_velocity  = math.sqrt(state.velocity_x**2 + state.velocity_y**2)
            agent_type = state.agent_type
                       
            # draw surrounding vehicles
            if agent_type == 1:
              x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(agent_x, 
                                          agent_y, agent_length, agent_width, agent_yaw)
              ax.add_patch(Rectangle((x_bottom_left, y_bottom_left), agent_length, agent_width, 
                          angle = agent_yaw,
                          edgecolor = 'blue',
                          facecolor = 'blue',
                          fill=True,
                          lw=1))
              x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, x_bottom_left, 
                                                         y_bottom_left, agent_length, agent_width)
              ax.arrow(x, y, delta_x, delta_y, width = 0.1)
              ax.text(x + delta_x, y + delta_y, "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)
              
            # draw surrounding pedestrians        
            if agent_type == 2:
              x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(agent_x, 
                                              agent_y, agent_length, agent_width, agent_yaw)
              ax.add_patch(Rectangle((x_bottom_left, y_bottom_left), agent_length, agent_width, 
                          angle = agent_yaw,
                          edgecolor = 'green',
                          facecolor = 'green',
                          fill=True,
                          lw=1))
              x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, x_bottom_left, y_bottom_left, 
                                                         agent_length, agent_width)
              ax.arrow(x, y, delta_x, delta_y, width = 0.1)     
              ax.text(x + delta_x, y + delta_y,  "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)
              
            # draw surrounding cyclists        
            if agent_type == 3: 
              x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(agent_x, 
                                              agent_y, agent_length, agent_width, agent_yaw)
              ax.add_patch(Rectangle((x_bottom_left, y_bottom_left), agent_length, agent_width, 
                          angle = agent_yaw,
                          edgecolor = 'yellow',
                          facecolor = 'yellow',
                          fill=True,
                          lw=1))     
              x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, x_bottom_left, y_bottom_left, 
                                                         agent_length, agent_width)
              ax.arrow(x, y, delta_x, delta_y, width = 0.1)     
              ax.text(x + delta_x, y + delta_y,  "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)
              
            # draw self-driving car itself        
            if agent_type == -1: 
              #print("there is a sdc !")
              x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(agent_x, 
                                              agent_y, agent_length, agent_width, agent_yaw)
              ax.add_patch(Rectangle((x_bottom_left, y_bottom_left), agent_length, agent_width, 
                          angle = agent_yaw,
                          edgecolor = 'red',
                          facecolor = 'red',
                          fill=True,
                          lw=1))   
              x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, x_bottom_left, y_bottom_left, 
                                                         agent_length, agent_width)
              ax.arrow(x, y, delta_x, delta_y, width = 0.1)    
              ax.text(x + delta_x, y + delta_y,  "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)
            index += 1
        
    ax.set_xlabel("x(m)", fontsize = 20)
    ax.set_ylabel("y(m)", fontsize = 20)
    ax.tick_params(axis = 'x', labelsize = 20)
    ax.tick_params(axis = 'y', labelsize = 20)
    
    # Set axes.
    size = max(40, width * 1.0)
    ax.axis([-size / 2 + center_x, size / 2 + center_x, -size / 2 + center_y, size / 2 + center_y])
    ax.set_aspect('equal')
    
    image = _generate_fig_canvas_image(fig) 
    plt.clf()  
    fig.clear()
    plt.close("all")
    return image        


def create_save_single_plot(single_image, plot_path, scenario_id, plot_number, dpi):

    fig, ax = plt.subplots()
    
    #size_inches = 1000 / dpi
    size_inches = 20
    fig.set_size_inches([size_inches, size_inches])

    red_patch = mpatches.Patch(color='red', label='autonomous vehicle')
    blue_patch = mpatches.Patch(color='blue', label='surrounding vehicles')
    green_patch = mpatches.Patch(color='green', label='pedestrians')
    yellow_patch = mpatches.Patch(color='yellow', label='cyclists')  

    # Title.
    #plot_title = scenario_id + ": agent states at timestep:" + '0'
    plot_title = scenario_id + ": agent states at timestep:" + '{:03}'.format(plot_number)
    ax.set_title(plot_title, fontsize = 20)    
    ax.legend(handles=[red_patch, blue_patch, green_patch, yellow_patch], fontsize = 14)
    ax.imshow(single_image)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid('off')
    plt.tight_layout()
    if plot_number == None:
        file_name = plot_path + scenario_id
        fig.savefig(file_name, dpi = dpi)
        fig.clear()
        plt.clf()
        plt.close("all")
    else:    
        #file_name = plot_path + scenario_id + "_" + '{:03}'.format(plot_number)
        file_name = plot_path + '{:03}'.format(plot_number)
        fig.savefig(file_name, dpi = dpi)
        fig.clear()
        plt.clf()
        plt.close("all")
    #plt.show()


 
