#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import uuid
import glob
from matplotlib import cm
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle
import numpy as np
import math
from .waymo_training_math_tools import *
from .waymo_training_find_vehicles_in_left_right_lane import *

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
    
    center_x = (np.max(scenario.all_agents_x_list) + np.min(scenario.all_agents_x_list)) / 2
    center_y = (np.max(scenario.all_agents_y_list) + np.min(scenario.all_agents_y_list)) / 2       
    range_x = np.ptp(scenario.all_agents_x_list) + 30
    range_y = np.ptp(scenario.all_agents_y_list) + 30   
    width = max(range_x, range_y)    
    return center_x, center_y, width  


def generate_single_step_plot(scenario, time_index, dpi, size_pixels, size_inch, plot_path):
    
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
    if len(scenario.lane_black_list) > 0:
        ax.scatter(scenario.lane_black_list[0], scenario.lane_black_list[1], color = 'black', marker = '.', linewidths = 0.2)          

    # plot stop sign - red
    if len(scenario.stop_sign_list) > 0:
        ax.scatter(scenario.stop_sign_list[0], scenario.stop_sign_list[1], color = 'red', marker = '^', linewidths = 2)
    
    # plot cross walk - darkorange   
    if len(scenario.cross_walk_list) > 0:
        for crosswalk in scenario.cross_walk_list:
            plt.plot(crosswalk.point_x_list, crosswalk.point_y_list, color = 'darkorange', linestyle = '--', linewidth = 2)  
            plt.plot([crosswalk.point_x_list[0], crosswalk.point_x_list[-1]], [crosswalk.point_y_list[0],crosswalk.point_y_list[-1]], color = 'darkorange', linestyle = '--', linewidth = 2)  
        
    # plot road edge - brown
    if len(scenario.road_edge_list) > 0:
        ax.scatter(scenario.road_edge_list[0], scenario.road_edge_list[1], color = 'brown', marker = '.', linewidths = 0.3)      
      
    # ========== part2: plot agents states ========== 
    
    for agent in scenario.agents_list:
        
        if agent.states_array.size == 0:
            continue    
        
        state = agent.states_array[time_index, :]
            
        if state[12] == '-1':
            continue
        
        agent_x = state[3]
        agent_y = state[4]
        agent_yaw    = radian_to_degree(state[9])
        agent_length = state[6]
        agent_width  = state[7]
        agent_height = state[8]
        agent_velocity  = math.sqrt(state[10]**2 + state[11]**2)
        agent_type = state[2]
        agent_id = state[1]
                   
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
                        edgecolor = 'orange',
                        facecolor = 'orange',
                        fill=True,
                        lw=1))     
            x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, x_bottom_left, y_bottom_left, 
                                                       agent_length, agent_width)
            ax.arrow(x, y, delta_x, delta_y, width = 0.1)     
            ax.text(x + delta_x, y + delta_y,  "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)
          
        # draw self-driving car itself        
        if agent_type == -1: 
            #print("this is a sdc !")
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
            
            '''
            vehicle1, vehicle2 = find_preceeding_following_in_lane(scenario, 
                                                    time_index,
                                                    agent_id,
                                                    agent_x, 
                                                    agent_y, 
                                                    agent_length, 
                                                    agent_width, 
                                                    agent_yaw)
    
            vehicle1_plot_x = [agent_x, vehicle1[0]]
            vehicle1_plot_y = [agent_y, vehicle1[1]]
            vehicle2_plot_x = [agent_x, vehicle2[0]]
            vehicle2_plot_y = [agent_y, vehicle2[1]] 
            ax.plot(vehicle1_plot_x,  vehicle1_plot_y, color = 'red',  linestyle = '--', linewidth = 2) 
            ax.plot(vehicle2_plot_x,  vehicle2_plot_y, color = 'red',  linestyle = '--', linewidth = 2) 
            '''
            
            '''
            left_vehicle1, left_vehicle2, right_vehicle1, right_vehicle2 = \
            find_closet_vehicles_in_left_right_lane(scenario, 
                                                    time_index,
                                                    agent_id,
                                                    agent_x, 
                                                    agent_y, 
                                                    agent_length, 
                                                    agent_width, 
                                                    agent_yaw)
            
            left_vehicle1_plot_x = [agent_x, left_vehicle1[0]]
            left_vehicle1_plot_y = [agent_y, left_vehicle1[1]]
            left_vehicle2_plot_x = [agent_x, left_vehicle2[0]]
            left_vehicle2_plot_y = [agent_y, left_vehicle2[1]] 
            
            right_vehicle1_plot_x = [agent_x, right_vehicle1[0]]
            right_vehicle1_plot_y = [agent_y, right_vehicle1[1]]
            right_vehicle2_plot_x = [agent_x, right_vehicle2[0]]
            right_vehicle2_plot_y = [agent_y, right_vehicle2[1]]             
            
            ax.plot(left_vehicle1_plot_x,  left_vehicle1_plot_y, color = 'red',  linestyle = '--', linewidth = 2) 
            ax.plot(left_vehicle2_plot_x,  left_vehicle2_plot_y, color = 'red',  linestyle = '--', linewidth = 2) 
            ax.plot(right_vehicle1_plot_x, right_vehicle1_plot_y, color = 'red', linestyle = '--', linewidth = 2) 
            ax.plot(right_vehicle2_plot_x, right_vehicle2_plot_y, color = 'red', linestyle = '--', linewidth = 2) 
            '''
            
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
    print("center lane point number:", len(scenario.lane_black_list[0]))
    ax.scatter(scenario.lane_black_list[0], scenario.lane_black_list[1], color = 'red', marker = '.', linewidths = 0.3)      

    # plot stop sign - red
    ax.scatter(scenario.stop_sign_list[0], scenario.stop_sign_list[1], color = 'red', marker = '^', linewidths = 10)
    
    # plot cross walk - darkorange    
    ax.scatter(scenario.cross_walk_list[0], scenario.cross_walk_list[1], color = 'darkorange', marker = '.', linewidths = 3)  
    
    # plot road edge - brown
    ax.scatter(scenario.road_edge_list[0], scenario.road_edge_list[1], color = 'brown', marker = '.', linewidths = 0.3)      
      
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
                          edgecolor = 'orange',
                          facecolor = 'orange',
                          fill=True,
                          lw=1))     
              x,y,delta_x,delta_y = calculate_arrow_base(agent_yaw, x_bottom_left, y_bottom_left, 
                                                         agent_length, agent_width)
              ax.arrow(x, y, delta_x, delta_y, width = 0.1)     
              ax.text(x + delta_x, y + delta_y,  "{:.2f}".format(agent_velocity) + "m/s", fontsize = 12)
              
            # draw self-driving car itself        
            if agent_type == -1: 
              print("there is a sdc !")
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
    orange_patch = mpatches.Patch(color='orange', label='cyclists')  

    # Title.
    plot_title = scenario_id + ": agent states at timestep:" + '{:03}'.format(plot_number)
    ax.set_title(plot_title, fontsize = 20)    
    ax.legend(handles=[red_patch, blue_patch, green_patch, orange_patch], fontsize = 14)
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
        file_name = plot_path + scenario_id + "_" + '{:03}'.format(plot_number)
        fig.savefig(file_name, dpi = dpi)   
        fig.clear()
        plt.clf()  
        plt.close("all")
    #plt.show()


 
