# -*- coding: utf-8 -*-


import csv
import os
import glob
import math
from fnmatch import fnmatch
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# line width and marker size related
line_width = 1
start_marker_size = 16
available_marker_size = 10
marker_size = 6
label_size = 18
font_size = 18
label_line_width = 2
map_location_line_width = 1
plot_line_width = 0.5


# show single map or total map
show_single_plot = False
show_total_plot = True
show_3d_plot = True
show_grid = False


# 3D plot setting
time_line_color = 'grey'
speed_line_color = 'blue'
trajectory_line_color = 'brown'

time_line_label = 'Time (s)'
speed_line_label = 'Speed (m/s)'
trajectory_line_label = 'X-Y plate trajectory'

time_line_style = '--'
speed_line_style = '-'
trajectory_line_style = '-'

plot_3D_line_width = 1
plot_3D_surface_color = "lightskyblue"
plot_3D_surface_alpha = 0.3

plot_3D_start_point_color = 'green'
plot_3D_start_point_marker = 'o'
plot_3D_start_point_size = 10
plot_3D_start_point_label = 'Start point'

plot_3D_end_point_color = 'red'
plot_3D_end_point_marker = '^'
plot_3D_end_point_size = 10
plot_3D_end_point_label = 'End point'

plot_3D_title = 'Space - Time/Speed'
plot_3D_title_font_size = 14
plot_3D_legend_size = 10
plot_3D_bbox_to_anchor_size = (1.05, 0.65)
plot_3D_font_size = 10

plot_3d_x_label = 'x (m)'
plot_3d_y_label = 'y (m)'

# line/marker order/overlap choice
vertical_line_z_order = 0
horizontal_line_z_order = 0
plot_line_z_order = 10

velocity_unit_use_mph_flag = True

show_entire_trajectory = True
entire_trajectory_color = 'black'
entire_trajectory_label = 'Velocity'

map_location_lines_alpha_value = 1

map_start_color = 'cyan'
#map_start_color = (0, 238, 238, 0.25)
map_start_marker = '*'

stop_bar_color = 'red'
stop_bar_marker = 's'

crossing_center_color = 'purple'
#crossing_center_color = (191, 62, 255, 0.25)
crossing_center_marker = '^'

system_available_color = 'blue'
system_available_marker = 'd'

approach_inform_color = 'orange'
approach_inform_marker = 'o'

approach_warning_color = 'red'
approach_warning_marker = 'o'

distance_inform_linewidth = 2
distance_inform_linestyle = '--'

distance_warning_linewidth = 2
distance_warning_linestyle = '--'

distance_available_linewidth = 2
distance_available_linestyle = '--'

map_start_linestyle = '-'
stop_bar_linestyle = '-'
crossing_center_linestyle = '-'

figure_time_velocity_x_label = 'Time (sec)'

figure_time_velocity_y_label = None
figure_distance_velocity_y_label = None
if velocity_unit_use_mph_flag:
    figure_time_velocity_y_label = 'Velocity (mph)'
    figure_distance_velocity_y_label = 'Velocity (mph)'
else:
    figure_time_velocity_y_label = 'Velocity (kph)'
    figure_distance_velocity_y_label = 'Velocity (kph)'

figure_time_velocity_title = "Time - Velocity"
figure_time_velocity_x_label = 'time (sec)'
figure_time_velocity_y_label = 'speed (m/s)'

figure_distance_velocity_title = "Distance - Velocity"
figure_distance_velocity_x_label = 'distance (m)'
figure_distance_velocity_y_label = 'speed (m/s)'

figure_time_distance_title = "Time - Distance"
figure_time_distance_x_label = 'time (sec)'
figure_time_distance_y_label = 'distance (m)'

figure_xy_trajectory_title = "Trajectory - X and Y"
figure_xy_trajectory_x_label = 'x (sec)'
figure_xy_trajectory_y_label = 'y (m)'

figure_trajectory_x_label = 'x (m)'
figure_trajectory_y_label = 'y (m)'

y_axis_velocity_limitation_min = 0
y_axis_velocity_limitation_max = 20

y_axis_distance_limitation_min = 0
y_axis_distance_limitation_max = 60

figure_size = (15,8)


'''
=================== python color name list ===================
'''


color_list = ['orange', 'blue', 'red', 'green', 'purple', 'cyan', 'steal', 'magenta', 'yellow', 'black', 'white']


'''
=================== coordinate convert defination part ===================
'''
import math as math
import numpy as mathlib
use_numpy = True


K0 = 0.9996

E = 0.00669438
E2 = E * E
E3 = E2 * E
E_P2 = E / (1.0 - E)

SQRT_E = mathlib.sqrt(1 - E)
_E = (1 - SQRT_E) / (1 + SQRT_E)
_E2 = _E * _E
_E3 = _E2 * _E
_E4 = _E3 * _E
_E5 = _E4 * _E

M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
M3 = (15 * E2 / 256 + 45 * E3 / 1024)
M4 = (35 * E3 / 3072)

P2 = (3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5)
P3 = (21. / 16 * _E2 - 55. / 32 * _E4)
P4 = (151. / 96 * _E3 - 417. / 128 * _E5)
P5 = (1097. / 512 * _E4)

R = 6378137

ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"

def negative(x):
    if use_numpy:
        return mathlib.max(x) < 0
    return x < 0


def mixed_signs(x):
    return use_numpy and mathlib.min(x) < 0 <= mathlib.max(x)


'''
=================== data alignment parameters ===========
'''
crossing_center_min_distance_general = 0
stop_bar_min_distance_general = -15.47
map_start_point_min_distance_general = -372

map_start_point_min_distance_13 = -372
map_start_point_min_speed_13 = 64 / 1.609

map_start_point_min_distance_2 = -372
map_start_point_min_speed_2 = 54 / 1.609 
            

'''
=================== coordinate convert function defination part ===================
'''
def lat_lon_to_utm(latitude, longitude, central_longitude):
    
        
    central_lon_rad = mathlib.radians(central_longitude)

    lat_rad = mathlib.radians(latitude)
    lat_sin = mathlib.sin(lat_rad)
    lat_cos = mathlib.cos(lat_rad)

    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2

    lon_rad = mathlib.radians(longitude)

    n = R / mathlib.sqrt(1 - E * lat_sin ** 2)
    c = E_P2 * lat_cos ** 2

    a = lat_cos * (lon_rad - central_lon_rad)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a

    m = R * (M1 * lat_rad - M2 * mathlib.sin(2 * lat_rad) + M3 * mathlib.sin(4 * lat_rad) - M4 * mathlib.sin(6 * lat_rad))
    easting = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) + a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000
    northing = K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) + a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))

    if mixed_signs(latitude):
        raise ValueError("latitudes must all have the same sign")
    elif negative(latitude):
        northing += 10000000

    return easting, northing


def get_average(value_list):
    return sum(value_list)/len(value_list)

def get_seconds(time_str):
    h, m, s = time_str.split(':')
    return int(h) * 3600 + int(m) * 60 + int(s)



'''
=================== x-y coordinates in-range or not ===================
'''
def get_distance(x, y, x0, y0):
    distance = math.sqrt((x - x0)**2 + (y - y0)**2)
    return distance


'''
=================== find point in trajectory nearest to crossing1/2 ===================
'''
def get_nearest_point(x_list, y_list, x0, y0):
    min_index = 0
    index = 0
    target_x = 0
    target_y = 0
    min_distance = 200
    number_of_points = len(x_list)
    while index < number_of_points:
        x = x_list[index]
        y = y_list[index]
        distance = math.sqrt((x - x0)**2 + (y - y0)**2)
        if distance < min_distance:
            min_distance = distance
            min_index = index
            target_x = x
            target_y = y
        index += 1
    return min_distance, target_x, target_y



csv_pattern_1 = "dataset"
csv_pattern_2 = "scenario"
csv_pattern_3 = "vehicle"
csv_pattern_4 = ".csv"
csv_pattern_custome = "1546"

trajectory_file_list = []
current_folder_path = os. getcwd()
files = os.listdir(current_folder_path)
   
# find csv data files
for file in files:
    if csv_pattern_4 in file:
        trajectory_file_list.append(file)


total_time_list = []
total_time_list_for_3D_plot_list = []
total_distance_list = []
total_speed_list = []
total_location_x_list = []
total_location_y_list = []
total_vehicle_id_list = []

file_index = 0
for file in trajectory_file_list:
    file_str = file.split('.')[0]
    vehicle_id = file_str.split('_')[3] 
    turn_type  = file_str.split('_')[4] + "_" + file_str.split('_')[5]
    plot_label = turn_type + ": vehicle" + vehicle_id
    total_vehicle_id_list.append(plot_label)
    print(file)

    time_list = []
    time_list_for_3D_plot_list = []
    acceleration_list = []
    speed_list = []
    location_x_list = []
    location_y_list = []
    distance_list = []

    with open(file) as trajectory_file:   
        trajectory_data = csv.reader(trajectory_file)  
        next(trajectory_data)
        trajectory_frames = [row for row in trajectory_data] 
       
    line_index = 0
    for trajectory_frame in trajectory_frames:
        line_index += 1
        time_str = trajectory_frame[0]
        location_x_str = trajectory_frame[3]
        location_y_str = trajectory_frame[4]        
        speed_str = trajectory_frame[6]
        odometer_str = trajectory_frame[10]
                   
        time_seconds = float(time_str)
        speed_mps = float(speed_str)
        distance_m = float(odometer_str)
        location_x_m = float(location_x_str)
        location_y_m = float(location_y_str)


        if location_x_m != 0 and location_y_m != 0:
            time_list_for_3D_plot_list.append(time_seconds)
            location_x_list.append(location_x_m)
            location_y_list.append(location_y_m)
            time_list.append(time_seconds)
            speed_list.append(speed_mps)           
            distance_list.append(distance_m)
        
    total_time_list.append(time_list)
    total_time_list_for_3D_plot_list.append(time_list_for_3D_plot_list)
    total_distance_list.append(distance_list)
    total_speed_list.append(speed_list)
    total_location_x_list.append(location_x_list)
    total_location_y_list.append(location_y_list)
    
    if show_single_plot:
        
        '''
        ==================== figure1 : time - speed  ====================
        '''
        fig, ax1 = plt.subplots()  
    
        ax1.plot(time_list, speed_list, color = entire_trajectory_color,  linewidth = line_width, label = entire_trajectory_label) 
        leg = ax1.legend(loc = 'lower right', fontsize = font_size)
        ax1.tick_params(axis='x', labelsize = label_size)
        ax1.tick_params(axis='y', labelsize = label_size)
        
        if show_grid:
            ax1.grid()
        ax1.set_xlabel(figure_time_velocity_x_label, fontsize = font_size)
        ax1.set_ylabel(figure_time_velocity_y_label, fontsize = font_size)
        
        for line in leg.get_lines():
            line.set_linewidth(label_line_width)
        title_name = figure_time_velocity_title
        plt.title(title_name, fontsize = font_size)           
        plt.rcParams["figure.figsize"] = figure_size
        plt.tight_layout()
        plt.tick_params(axis='x', labelsize = label_size)
        plt.tick_params(axis='y', labelsize = label_size)
        plt.ylim(y_axis_velocity_limitation_min, y_axis_velocity_limitation_max)  
    
    
    
        '''
        ==================== figure2 : distance - velocity ====================
        '''
        fig, ax1 = plt.subplots()  
    
        ax1.plot(distance_list, speed_list, color = entire_trajectory_color,  linewidth = line_width, label = entire_trajectory_label) 
        leg = ax1.legend(loc = 'lower right', fontsize = font_size)
        ax1.tick_params(axis='x', labelsize = label_size)
        ax1.tick_params(axis='y', labelsize = label_size)
        
        if show_grid:
            ax1.grid()
        ax1.set_xlabel(figure_distance_velocity_x_label, fontsize = font_size)
        ax1.set_ylabel(figure_distance_velocity_y_label, fontsize = font_size)
        
        for line in leg.get_lines():
            line.set_linewidth(label_line_width)
        title_name = figure_distance_velocity_title
        plt.title(title_name, fontsize = font_size)           
        plt.rcParams["figure.figsize"] = figure_size
        plt.tight_layout()
        plt.tick_params(axis='x', labelsize = label_size)
        plt.tick_params(axis='y', labelsize = label_size)
        plt.ylim(y_axis_velocity_limitation_min, y_axis_velocity_limitation_min)  
        
    
        '''
        ==================== figure3 : time - distance ====================
        '''
    
        fig, ax1 = plt.subplots()  
        
        ax1.plot(time_list, distance_list, color = entire_trajectory_color,  linewidth = line_width, label = entire_trajectory_label) 
        leg = ax1.legend(loc = 'lower right', fontsize = font_size)
        ax1.tick_params(axis='x', labelsize = label_size)
        ax1.tick_params(axis='y', labelsize = label_size)
        
        if show_grid:
            ax1.grid()
        ax1.set_xlabel(figure_time_distance_x_label, fontsize = font_size)
        ax1.set_ylabel(figure_time_distance_y_label, fontsize = font_size)
        
        for line in leg.get_lines():
            line.set_linewidth(label_line_width)
        title_name = figure_time_distance_title
        plt.title(title_name, fontsize = font_size)           
        plt.rcParams["figure.figsize"] = figure_size
        plt.tight_layout()
        plt.tick_params(axis='x', labelsize = label_size)
        plt.tick_params(axis='y', labelsize = label_size)
        plt.ylim(y_axis_distance_limitation_min, y_axis_distance_limitation_max)  

        '''
        ==================== figure4 : x - y trajectory ====================
        '''
    
        fig, ax1 = plt.subplots()  
        
        ax1.plot(location_x_list, location_y_list, color = entire_trajectory_color,  linewidth = line_width, label = entire_trajectory_label) 
        leg = ax1.legend(loc = 'lower right', fontsize = font_size)
        ax1.tick_params(axis='x', labelsize = label_size)
        ax1.tick_params(axis='y', labelsize = label_size)
        
        if show_grid:
            ax1.grid()
        ax1.set_xlabel(figure_xy_trajectory_x_label, fontsize = font_size)
        ax1.set_ylabel(figure_xy_trajectory_y_label, fontsize = font_size)
        
        for line in leg.get_lines():
            line.set_linewidth(label_line_width)
        title_name = figure_xy_trajectory_title
        plt.title(title_name, fontsize = font_size)           
        plt.rcParams["figure.figsize"] = figure_size
        plt.tight_layout()
        plt.tick_params(axis='x', labelsize = label_size)
        plt.tick_params(axis='y', labelsize = label_size)
        #plt.ylim(y_axis_limitation_min, y_axis_limitation_max)  
        
    if show_3d_plot:
        fig = plt.figure()
        ax1 = plt.axes(projection='3d')
        ax1.plot3D(location_x_list, location_y_list, time_list_for_3D_plot_list, color = time_line_color, linestyle = time_line_style, label = time_line_label)
        ax1.plot3D(location_x_list, location_y_list, speed_list, color = speed_line_color, linestyle = speed_line_style, label = speed_line_label)
        ax1.plot(location_x_list, location_y_list, color = trajectory_line_color, linestyle = trajectory_line_style, label = trajectory_line_label)
              
        location_x_start = location_x_list[0]
        location_x_end = location_x_list[-1]
        location_y_start = location_y_list[0]
        location_y_end = location_y_list[-1]
        
        location_x_min = min(location_x_list)
        location_x_max = max(location_x_list)
        location_y_min = min(location_y_list)
        location_y_max = max(location_y_list)   
        
        x = np.arange(location_x_min, location_x_max, 0.0001)
        y = np.arange(location_y_min, location_y_max, 0.0001)
        #X, Y = np.meshgrid(x, y)
        #Z = np.zeros(shape=(X.shape[0], X.shape[1]))
        
        #ax1.plot_surface(X, Y, Z, color = plot_3D_surface_color, alpha = plot_3D_surface_alpha)
        ax1.set_xlabel(plot_3d_x_label)
        ax1.set_ylabel(plot_3d_y_label)
        #ax1.set_zlabel('time (0.1s)');
                       
        ax1.plot(location_x_start, location_y_start, plot_3D_start_point_marker, color = plot_3D_start_point_color, label = plot_3D_start_point_label, markersize = plot_3D_start_point_size)
        ax1.plot(location_x_end, location_y_end, plot_3D_end_point_marker, color = plot_3D_end_point_color, label = plot_3D_end_point_label, markersize = plot_3D_end_point_size)

        title_name = plot_3D_title
        plt.title(title_name, fontsize = plot_3D_title_font_size)   
        ax1.legend(bbox_to_anchor = plot_3D_bbox_to_anchor_size, fontsize = plot_3D_font_size)
        plt.tight_layout()
        plt.show()
        

if show_total_plot:
    # total time - velocity plot
    number_of_plot = len(total_time_list)
    plot_index = 0
    fig, ax1 = plt.subplots()  
    while plot_index < number_of_plot:
        time_list = total_time_list[plot_index]
        speed_list = total_speed_list[plot_index]   
        plot_label = total_vehicle_id_list[plot_index]
        plot_color = color_list[plot_index]
        ax1.plot(time_list, speed_list, color = plot_color,  linewidth = line_width, label = plot_label) 
        plot_index += 1
        
    ax1.tick_params(axis='x', labelsize = label_size)
    ax1.tick_params(axis='y', labelsize = label_size)    
    if show_grid:
        ax1.grid()
    ax1.set_xlabel(figure_time_velocity_x_label, fontsize = font_size)
    ax1.set_ylabel(figure_time_velocity_y_label, fontsize = font_size)
    leg = ax1.legend(loc = 'lower right', fontsize = font_size)
    for line in leg.get_lines():
        line.set_linewidth(label_line_width)
    title_name = figure_time_velocity_title
    plt.title(title_name, fontsize = font_size)           
    plt.rcParams["figure.figsize"] = figure_size
    plt.tight_layout()
    plt.tick_params(axis='x', labelsize = label_size)
    plt.tick_params(axis='y', labelsize = label_size)
    plt.ylim(y_axis_velocity_limitation_min, y_axis_velocity_limitation_max)  
    
    
    # total distance - velocity plot
    number_of_plot = len(total_time_list)
    plot_index = 0
    fig, ax1 = plt.subplots()  
    while plot_index < number_of_plot:
        distance_list = total_distance_list[plot_index]
        speed_list = total_speed_list[plot_index]   
        plot_label = total_vehicle_id_list[plot_index]
        plot_color = color_list[plot_index]
        ax1.plot(distance_list, speed_list, color = plot_color,  linewidth = line_width, label = plot_label) 
        plot_index += 1
        
    ax1.tick_params(axis='x', labelsize = label_size)
    ax1.tick_params(axis='y', labelsize = label_size)    
    if show_grid:
        ax1.grid()
    ax1.set_xlabel(figure_distance_velocity_x_label, fontsize = font_size)
    ax1.set_ylabel(figure_distance_velocity_y_label, fontsize = font_size)
    leg = ax1.legend(loc = 'lower right', fontsize = font_size)
    for line in leg.get_lines():
        line.set_linewidth(label_line_width)
    title_name = figure_distance_velocity_title
    plt.title(title_name, fontsize = font_size)           
    plt.rcParams["figure.figsize"] = figure_size
    plt.tight_layout()
    plt.tick_params(axis='x', labelsize = label_size)
    plt.tick_params(axis='y', labelsize = label_size)
    plt.ylim(y_axis_velocity_limitation_min, y_axis_velocity_limitation_max)      
    
    # total time - distance plot 
    number_of_plot = len(total_time_list)
    plot_index = 0
    fig, ax1 = plt.subplots()  
    while plot_index < number_of_plot:
        time_list = total_time_list[plot_index]
        distance_list = total_distance_list[plot_index]
        plot_label = total_vehicle_id_list[plot_index]
        plot_color = color_list[plot_index]
        ax1.plot(time_list, distance_list, color = plot_color,  linewidth = line_width, label = plot_label)  
        plot_index += 1
        
    ax1.tick_params(axis='x', labelsize = label_size)
    ax1.tick_params(axis='y', labelsize = label_size)    
    if show_grid:
        ax1.grid()
    ax1.set_xlabel(figure_time_distance_x_label, fontsize = font_size)
    ax1.set_ylabel(figure_time_distance_y_label, fontsize = font_size)
    leg = ax1.legend(loc = 'lower right', fontsize = font_size)
    for line in leg.get_lines():
        line.set_linewidth(label_line_width)
    title_name = figure_time_distance_title
    plt.title(title_name, fontsize = font_size)           
    plt.rcParams["figure.figsize"] = figure_size
    plt.tight_layout()
    plt.tick_params(axis='x', labelsize = label_size)
    plt.tick_params(axis='y', labelsize = label_size)
    plt.ylim(y_axis_distance_limitation_min, y_axis_distance_limitation_max)
    
    # total trajectory - x and y plot 
    number_of_plot = len(total_time_list)
    plot_index = 0
    fig, ax1 = plt.subplots()  
    while plot_index < number_of_plot:
        location_x_list = total_location_x_list[plot_index]
        location_y_list = total_location_y_list[plot_index]
        plot_label = total_vehicle_id_list[plot_index]
        plot_color = color_list[plot_index]
        ax1.plot(location_x_list, location_y_list, color = plot_color,  linewidth = line_width, label = plot_label)  
        plot_index += 1
        
    ax1.tick_params(axis='x', labelsize = label_size)
    ax1.tick_params(axis='y', labelsize = label_size)    
    if show_grid:
        ax1.grid()
    ax1.set_xlabel(figure_trajectory_x_label, fontsize = font_size)
    ax1.set_ylabel(figure_trajectory_y_label, fontsize = font_size)
    leg = ax1.legend(loc = 'lower right', fontsize = font_size)
    for line in leg.get_lines():
        line.set_linewidth(label_line_width)
    title_name = figure_xy_trajectory_title
    plt.title(title_name, fontsize = font_size)           
    plt.rcParams["figure.figsize"] = figure_size
    plt.tight_layout()
    plt.tick_params(axis='x', labelsize = label_size)
    plt.tick_params(axis='y', labelsize = label_size)
    #plt.ylim(y_axis_distance_limitation_min, y_axis_distance_limitation_max)
    plt.show()