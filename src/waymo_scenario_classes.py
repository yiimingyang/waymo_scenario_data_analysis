#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

class Scenarios:
    def __init__(self): 
        self.scenarios_file_name = None
        self.scenarios_list = []

class Agent_states:
    def __init__(self): 
        self.agent_timestep = None
        self.agent_id = None
        self.agent_type =None
        self.center_x = None
        self.center_y = None
        self.center_z = None
        self.length = None
        self.width = None
        self.height = None
        self.heading = None
        self.velocity_x = None
        self.velocity_y = None
        self.valid = None

 

class Map_feature:
    def __init__(self): 
        self.feature_type = None
        self.speed_limit = None
        self.lane_type = None
        
    
    
class Motion_training_scenario:
    def __init__(self): 
        self.scenario_id = None
        self.current_time_index = None
        self.sdc_track_index = None
        self.number_of_map_features = None
        
        self.timestep_seconds_list = None
        self.timestep_list = []
        
        self.all_agents_x_list = []
        self.all_agents_y_list = []    
        
        self.agents_list = []
        self.map_features_list = []
        
        # white line lists
        self.broken_single_white_list = []
        self.solid_single_white_list  = []
        self.solid_double_white_list  = []
    
        # yellow line lists    
        self.broken_single_yellow_list = []
        self.broken_double_yellow_list = []
        self.solid_single_yellow_list  = []
        self.solid_double_yellow_list  = []  
        self.passing_double_yellow_list = [] 

        # center lane list  
        self.center_lane_list = [] 
        
        # center lane dict   
        self.center_lane_dict = {}         
        
        # lane lists black   
        self.lane_black_list = []        
    
        # stop sign list
        self.stop_sign_list = []
    
        # cross walk list     
        self.cross_walk_list = []
        
        # road edge list
        self.road_edge_list = []
        
        
    

class Motion_training_agent:
    def __init__(self):
        self.agent_type = None  # unset = 0, vehicle = 1, pedestrian = 2, cyclist = 3, other = 4
        self.agent_id = None
        self.states_array = None       
        self.states_mask_list = [] 
        self.states_list = []
        
        #states_array = np.empty((0,13), dtype = float)
   
    
   
    
# map features related classes

class Center_lane:
    lane_id = None
    lane_type = None    # undefined, freeway, surface_street, bike_line
    speed_limit = None   
    entry_lines_list = []  # A list of IDs for lanes that this lane may be entered from    
    exit_lanes_list  = [] # A list of IDs for lanes that this lane may exit to   
    left_boundary_segment_list = []   
    right_boundary_segment_list = []   
    left_neighbors_segment_list = []  
    right_neighbors_segment_list = []     
    points_list = []
    point_x_list = []
    point_y_list = []
    
    left_neighbors_id_list = []
    right_neighbors_id_list = []
    entry_lane_id_list = []
    exit_lane_id_list = []    


class neighbor_lane:
    lane_id = None
    start_index = None    # undefined, freeway, surface_street, bike_line
    end_index = None   
    neighbor_start_index = None
    neighbor_end_index = None

    

'''
TYPE_UNKNOWN = 0;
TYPE_BROKEN_SINGLE_WHITE = 1;
TYPE_SOLID_SINGLE_WHITE = 2;
TYPE_SOLID_DOUBLE_WHITE = 3;
TYPE_BROKEN_SINGLE_YELLOW = 4;
TYPE_BROKEN_DOUBLE_YELLOW = 5;
TYPE_SOLID_SINGLE_YELLOW = 6;
TYPE_SOLID_DOUBLE_YELLOW = 7;
TYPE_PASSING_DOUBLE_YELLOW = 8;
'''

class Road_line:
    
    line_id = None
    line_type = None    # undefined, freeway, surface_street, bike_line  
    points_list = []
    
    # white line x lists
    broken_single_white_x_list = []
    solid_single_white_x_list  = []
    solid_double_white_x_list  = []

    # white line y lists
    broken_single_white_y_list = []
    solid_single_white_y_list  = []
    solid_double_white_y_list  = []

    # yellow line x lists    
    broken_single_yellow_x_list = []
    broken_double_yellow_x_list = []
    solid_single_yellow_x_list  = []
    solid_double_yellow_x_list  = []  
    passing_double_yellow_x_list = []     

    # yellow line y lists    
    broken_single_yellow_y_list = []
    broken_double_yellow_y_list = []
    solid_single_yellow_y_list  = []
    solid_double_yellow_y_list  = []  
    passing_double_yellow_y_list = []    
 
    # unknown x and y list
    unknown_x_list = []
    unknown_y_list = []  
    
    point_x_list = []
    point_y_list = []
 

class Road_edge:
    
    line_id = None
    line_type = None    # undefined, freeway, surface_street, bike_line  
    points_list = []
    point_x_list = []
    point_y_list = []    
  
    
class Stop_sign:
    
    conrtolled_lane_id = None
    stop_sign_id = None    # undefined, freeway, surface_street, bike_line 
    stop_sign_type = None
    position_x = None
    position_y = None
    position_z = None
    point_x_list = []
    point_y_list = []    
  
    
class Cross_walk:
    def __init__(self):
        self.crosswalk_list = [] 
        self.point_x_list = []
        self.point_y_list = []         
    
class Speed_bump:

    points_list = []    
    point_x_list = []
    point_y_list = []    
    