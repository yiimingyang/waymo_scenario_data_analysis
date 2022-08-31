#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import uuid
from matplotlib import cm
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle
import numpy as np
import math

def degree_to_rdian(degree):
    return degree * math.pi* 0.005556


def radian_to_degree(radian):
    return radian * 180 / math.pi


def get_bottem_left_point_of_rectangle_origin(center_x, center_y, height, width, angle):
    
    angle = angle * math.pi / 180

    #top right:
    top_right_x = center_x + ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_right_y = center_y + ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
       
    #top left:
    top_left_x = center_x - ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_left_y = center_y - ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
        
    #bottom left:
    bot_left_x = center_x - ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_left_y = center_y - ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))
         
    #bottom right:
    bot_right_x = center_x + ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_right_y = center_y + ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))

    #bottom center:
    bot_center_x = 0.5 * (bot_right_x + bot_left_x)
    bot_center_y = 0.5 * (bot_right_y + bot_left_y)

    return bot_left_x, bot_left_y


def get_bottem_left_point_of_rectangle(center_x, center_y, height, width, angle):
            
    angle = angle * math.pi / 180

    #top right:
    top_right_x = center_x + ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_right_y = center_y + ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
       
    #top left:
    top_left_x = center_x - ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_left_y = center_y - ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
        
    #bottom left:
    bot_left_x = center_x - ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_left_y = center_y - ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))
         
    #bottom right:
    bot_right_x = center_x + ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_right_y = center_y + ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))

    #bottom center:
    bot_center_x = 0.5 * (bot_right_x + bot_left_x)
    bot_center_y = 0.5 * (bot_right_y + bot_left_y)
    
    angle = radian_to_degree(angle)
    
    if math.sin(angle) >= 0 and math.cos(angle) >= 0:
        return bot_left_x, bot_left_y
    if math.sin(angle) >= 0 and math.cos(angle) <= 0:
        return bot_left_x, bot_left_y   
    if math.sin(angle) <= 0 and math.cos(angle) >= 0:
        return bot_right_x, bot_right_y
    if math.sin(angle) <= 0 and math.cos(angle) <= 0:
        return bot_right_x, bot_right_y    
       
    
def calculate_arrow_base(yaw_theta, rectangle_x, rectangle_y, rectangle_length, rectangle_width):
    theta = yaw_theta + 90
    if theta >= 360:
        theta = theta - 360
    if theta <= -360:
        theta = theta + 360
    if theta >= 180:
        theta = theta - 360
    if theta <= -180:
        theta = theta + 360        

    theta = theta * math.pi / 180
    yaw_theta = yaw_theta * math.pi / 180
    x = rectangle_x + 0.5 * rectangle_width * math.cos(theta)
    y = rectangle_y + 0.5 * rectangle_width * math.sin(theta)
    dx = (rectangle_length + 1) * math.cos(yaw_theta)
    dy = (rectangle_length + 1) * math.sin(yaw_theta)  
    return x, y,dx,dy
    
    
def create_figure_and_axes(dpi, size_pixels, size_inch):
    """Initializes a unique figure and axes for plotting."""
    fig, ax = plt.subplots(1, 1, num=uuid.uuid4())
    
    # Sets output image to pixel resolution.
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


def fig_canvas_image(fig):
  """Returns a [H, W, 3] uint8 np.array image from fig.canvas.tostring_rgb()."""
  # Just enough margin in the figure to display xticks and yticks.
  fig.subplots_adjust(left=0.08, bottom=0.08, right=0.98, top=0.98, wspace=0.0, hspace=0.0)
  fig.canvas.draw()
  data = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
  return data.reshape(fig.canvas.get_width_height()[::-1] + (3,))


def get_colormap(num_agents):
  """Compute a color map array of shape [num_agents, 4]."""
  colors = cm.get_cmap('jet', num_agents)
  colors = colors(range(num_agents))
  np.random.shuffle(colors)
  return colors


def get_viewport(all_states, all_states_mask):
  """Gets the region containing the data.

  Args:
    all_states: states of agents as an array of shape [num_agents, num_steps,
      2].
    all_states_mask: binary mask of shape [num_agents, num_steps] for
      `all_states`.

  Returns:
    center_y: float. y coordinate for center of data.
    center_x: float. x coordinate for center of data.
    width: float. Width of data.
  """
  valid_states = all_states[all_states_mask]
  all_y = valid_states[..., 1]
  all_x = valid_states[..., 0]

  center_y = (np.max(all_y) + np.min(all_y)) / 2
  center_x = (np.max(all_x) + np.min(all_x)) / 2

  range_y = np.ptp(all_y) + 30
  range_x = np.ptp(all_x) + 30

  width = max(range_y, range_x)

  return center_y, center_x, width


def create_fig_for_single_step(states,
                       mask,
                       agents_type,
                       agents_motion_states,
                       roadgraph_info,
                       title,
                       center_y,
                       center_x,
                       width,
                       dpi,
                       size_pixels,
                       size_inch):

    # Create figure and axes.
    fig, ax = create_figure_and_axes(dpi, size_pixels, size_inch)
    
    # Plot roadgraph.
    roadgraph_points = roadgraph_info[:, :2].T
    roadgraph_type = roadgraph_info[:, 3].T
    
    # lane-center-freeway, lane-center-surfacestreet, lanecenter-bikelane, roadline-brokensinglewhite, 
    center_line_list = []
    
    # roadline-solidsinglewhite 
    solid_single_white_line_list = []
    
    # roadline-soliddoublewhite
    solid_double_white_line_list = []
    
    # roadline-soliddoubleyellow
    solid_double_yellow_line_list = []
    
    # roadline_broken_double_yellow
    roadline_broken_double_yellow_line_list = []
    
    # roadline_solid_single_yellow
    roadline_solid_single_yellow_line_list = []
    
    # stop_sign
    roadline_stop_sign_list = []
    
    # crossing_walk
    crossing_walk_list = [] 
    
    # center line list
    # LaneCenter-Freeway = 1, 
    # LaneCenter-SurfaceStreet = 2, 
    # LaneCenter-BikeLane = 3, 
    # RoadLine-BrokenSingleWhite = 6, 
    # RoadLine-SolidSingleWhite = 7, 
    # RoadLine-SolidDoubleWhite = 8, 
    # RoadLine-BrokenSingleYellow = 9, 
    # RoadLine-BrokenDoubleYellow = 10, 
    # Roadline-SolidSingleYellow = 11, 
    # Roadline-SolidDoubleYellow=12, 
    # RoadLine-PassingDoubleYellow = 13, 
    # RoadEdgeBoundary = 15, 
    # RoadEdgeMedian = 16, 
    # StopSign = 17, 
    # Crosswalk = 18, 
    # SpeedBump = 19, 
    
    road_line_type_center_list = [1, 2]
    road_line_type_white_list = [6, 7, 8]
    road_line_type_yellow_list = [9, 10, 11, 12, 13]
    road_line_type_boundary_list = [15]

    center_line_x_list = []
    center_line_y_list = []
    white_line_x_list = []
    white_line_y_list = []
    yellow_line_x_list = []
    yellow_line_y_list = []
    stop_sign_x_list = []
    stop_sign_y_list = []
    cross_walk_x_list = []
    cross_walk_y_list = []
    SpeedBump_x_list = []
    SpeedBump_y_list = []
    boundary_x_list = []
    boundary_y_list = []    
    other_line_x_list = []
    other_line_y_list = []
    
    #center_line_type_list = [1, 2, 6, 9]
    #center_line_type_list = [1, 2, 6, 9]
    #center_line_x_list = []
    #center_line_y_list = []
    
    #boundary_line_list
    #boundary_line_type_list = [8, 11, 12]
    #boundary_line_list = []
    #boundary_line_x_list = []
    #boundary_line_y_list = []
    

    # extract road_graph type    
    index = 0
    while index < roadgraph_type.shape[0]:
        point_type = roadgraph_type[index]
        if point_type in road_line_type_white_list:
            white_line_x_list.append(roadgraph_points[0, index])
            white_line_y_list.append(roadgraph_points[1, index])
        elif point_type in road_line_type_yellow_list:
            yellow_line_x_list.append(roadgraph_points[0, index])
            yellow_line_y_list.append(roadgraph_points[1, index])
        elif point_type == 15:
            boundary_x_list.append(roadgraph_points[0, index])
            boundary_y_list.append(roadgraph_points[1, index])               
        elif point_type == 17:
            stop_sign_x_list.append(roadgraph_points[0, index])
            stop_sign_y_list.append(roadgraph_points[1, index])
        elif point_type == 18:
            cross_walk_x_list.append(roadgraph_points[0, index])
            cross_walk_y_list.append(roadgraph_points[1, index])  
        elif point_type in road_line_type_center_list:
            center_line_x_list.append(roadgraph_points[0, index])
            center_line_y_list.append(roadgraph_points[1, index])              
        else:
            #print("road type:",point_type)
            other_line_x_list.append(roadgraph_points[0, index])
            other_line_y_list.append(roadgraph_points[1, index])
            
        index += 1
    
    #ax.plot(roadgraph_points[0, :], roadgraph_points[1, :], 'k.', alpha=1, ms=2)
    
    ax.scatter(white_line_x_list, white_line_y_list, color = 'w', marker = '.', linewidths = 0.3)
    ax.scatter(yellow_line_x_list, yellow_line_y_list, color = 'yellow', marker = '.', linewidths = 0.3)  
    ax.scatter(stop_sign_x_list, stop_sign_y_list, color = 'red', marker = '^', linewidths = 2)
    ax.scatter(cross_walk_x_list, cross_walk_y_list, color = 'darkorange', marker = '.', linewidths = 2)     
    ax.scatter(boundary_x_list, boundary_y_list, color = 'brown', marker = '.', linewidths = 0.3)      
    #ax.scatter(other_line_x_list, other_line_y_list, color = 'black', marker = '.', linewidths = 0.3)  
    
    ax.scatter(center_line_x_list, center_line_y_list, color = 'white', marker = '.', linewidths = 0.3, alpha = 0.5)  
    
    '''
    ax.plot(white_line_x_list, white_line_y_list, 'w.', alpha=1, ms=2)
    ax.plot(yellow_line_x_list, yellow_line_y_list, 'y.', alpha=1, ms=2)  
    ax.plot(stop_sign_x_list, stop_sign_y_list, 'red', marker = '^', alpha=1, ms=6)
    ax.plot(cross_walk_x_list, cross_walk_y_list, 'darkorange.', alpha=1, ms=2)     
    ax.plot(boundary_x_list, boundary_y_list, 'brown.', alpha=1, ms=2)      
    ax.plot(other_line_x_list, other_line_y_list, 'k.', alpha=1, ms=2)  
    '''
    
    masked_x = states[:, 0][mask]
    masked_y = states[:, 1][mask]
       
    index = 0
    while index < masked_x.shape[0]:
      agent_yaw = radian_to_degree(agents_motion_states[index, 0])
      agent_length = agents_motion_states[index, 1]
      agent_width  = agents_motion_states[index, 2]
      agent_height = agents_motion_states[index, 3]
      agent_velocity  = math.sqrt(agents_motion_states[index, 4]**2 + agents_motion_states[index, 4]**2)

      # draw surrounding agent
      if agents_type[index, 0] == 1 and agents_type[index, 1] == 0:
        #x_bottom_left, y_bottom_left = masked_x[index], masked_y[index]

        x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(masked_x[index], 
                                    masked_y[index], agent_length, agent_width, agent_yaw)
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
      if agents_type[index, 0] == 2 and agents_type[index, 1] == 0:
        #x_bottom_left, y_bottom_left = masked_x[index], masked_y[index]
        x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(masked_x[index], 
                                        masked_y[index], agent_length, agent_width, agent_yaw)
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
      if agents_type[index, 0] == 3 and agents_type[index, 1] == 0: 
        #x_bottom_left, y_bottom_left = masked_x[index], masked_y[index]
        x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(masked_x[index], 
                                        masked_y[index], agent_length, agent_width, agent_yaw)
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
      if agents_type[index, 1] == 1: 
        print("there is a sdc !")
        #x_bottom_left, y_bottom_left = masked_x[index], masked_y[index]
        x_bottom_left, y_bottom_left = get_bottem_left_point_of_rectangle(masked_x[index], 
                                        masked_y[index], agent_length, agent_width, agent_yaw)
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
         
    # Title.
    ax.set_title(title, fontsize = 20)
    ax.set_xlabel("x(m)", fontsize = 20)
    ax.set_ylabel("y(m)", fontsize = 20)  
    ax.tick_params(axis = 'x', labelsize = 20)
    ax.tick_params(axis = 'y', labelsize = 20)
    
    # Set axes.  Should be at least 10m on a side and cover 160% of agents.
    size = max(40, width * 1.0)
    ax.axis([-size / 2 + center_x, size / 2 + center_x, -size / 2 + center_y, size / 2 + center_y])
    ax.set_aspect('equal')
    
    image = fig_canvas_image(fig)
    plt.close(fig)
    return image



def create_save_single_plot(single_image, plot_path, plot_title, plot_number):
    
    #plt.ioff()
    fig, ax = plt.subplots()
    dpi = 300
    #size_inches = 1000 / dpi
    size_inches = 20
    fig.set_size_inches([size_inches, size_inches])
    #plt.ion()
    plt.tight_layout()

    red_patch = mpatches.Patch(color='red', label='autonomous vehicle')
    blue_patch = mpatches.Patch(color='blue', label='surrounding vehicles')
    green_patch = mpatches.Patch(color='green', label='pedestrians')
    orange_patch = mpatches.Patch(color='orange', label='cyclists')  
    
    ax.legend(handles=[red_patch, blue_patch, green_patch, orange_patch], fontsize = 10)
    ax.imshow(single_image)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.grid('off')
    if plot_number == None:    
        plot_name = plot_path + plot_title
        fig.savefig(plot_name)
        plt.close(fig)
    else:    
        plot_name = plot_path + plot_title + str(plot_number)
        fig.savefig(plot_name)   
        plt.close(fig)
    #plt.show()



def create_animation(images):
    """ Creates a Matplotlib animation of the given images.
    
    Args:
      images: A list of numpy arrays representing the images.
    
    Returns:
      A matplotlib.animation.Animation.
    
    Usage:
      anim = create_animation(images)
      anim.save('/tmp/animation.avi')
      HTML(anim.to_html5_video())
    """
    
    plt.ioff()
    fig, ax = plt.subplots()
    dpi = 300
    #size_inches = 1000 / dpi
    size_inches = 20
    fig.set_size_inches([size_inches, size_inches])
    plt.ion()
    
    def animate_func(i):
      ax.imshow(images[i])
      ax.set_xticks([])
      ax.set_yticks([])
      ax.grid('off')
    
    anim = animation.FuncAnimation(
        fig, animate_func, frames=len(images) // 2, interval=100)
    plt.close(fig)
    return anim


