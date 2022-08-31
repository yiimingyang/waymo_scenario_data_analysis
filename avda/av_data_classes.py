#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
from sklearn.cluster import AgglomerativeClustering
from sklearn.cluster import KMeans
from .waymo_training_math_tools import *
#from shapely.geometry import Point, Polygon
from matplotlib.patches import Polygon
import random
from enum import Enum

class MAP_feature(Enum):
    TYPE_UNDEFINED = 0
    TYPE_FREEWAY = 1
    TYPE_SURFACE_STREET = 2
    TYPE_BIKE_LANE = 3

class Agent_type(Enum):
    TYPE_UNSET = 0
    TYPE_VEHICLE = 1
    TYPE_PEDESTRIAN = 2
    TYPE_CYCLIST = 3
    TYPE_OTHER = 4

class Road_line_type(Enum):
    TYPE_UNKNOWN = 0
    TYPE_BROKEN_SINGLE_WHITE = 1
    TYPE_SOLID_SINGLE_WHITE = 2
    TYPE_SOLID_DOUBLE_WHITE = 3
    TYPE_BROKEN_SINGLE_YELLOW = 4
    TYPE_BROKEN_DOUBLE_YELLOW = 5
    TYPE_SOLID_SINGLE_YELLOW = 6
    TYPE_SOLID_DOUBLE_YELLOW = 7
    TYPE_PASSING_DOUBLE_YELLOW = 8 

class Signal_state(Enum):
    LANE_STATE_UNKNOWN = 0
    LANE_STATE_ARROW_STOP = 1
    LANE_STATE_ARROW_CAUTION = 2
    LANE_STATE_ARROW_GO = 3
    
    LANE_STATE_STOP = 4
    LANE_STATE_CAUTION = 5
    LANE_STATE_GO = 6

    LANE_STATE_FLASHING_STOP = 7
    LANE_STATE_FLASHING_CAUTION = 8

class Scenario_parser:
    def __init__(self): 
        self.waymo_data_file_location = None
        self.waymo_dataset_list = []
        self.scenarios_to_plot_list = []
        self.image_save_path = None
        self.animation_save_path = None
        self.generate_individual_images = None
        self.size_pixels = None
        self.dpi = None
        self.size_inch = None
        self.start_dataset_index = None
        self.scenario_in_dataset_index = None
        self.generate_animation = None
        self.animation_format = None
        self.gif_save_all = None
        self.gif_duration = None
        self.gif_loop = None
        self.plot_topology = None
        self.plot_single_image = None
        self.plot_straight_lane = None
        self.plot_preceeding_and_following_agents_in_lane = None
        self.plot_preceeding_and_following_agents_in_left_lane = None
        self.plot_preceeding_and_following_agents_in_right_lane = None
        self.plot_preceeding_and_following_agents_line_width = None
        self.show_agents_id = None
        self.plot_image_number = None
        self.generate_video = None
        self.white_line_color = None
        self.yellow_line_color = None
        self.center_line_color = None
        self.edge_line_color = None
        self.crossing_line_color = None
        self.stop_sign_color = None
        self.sdc_color = None
        self.vehicle_color = None
        self.bicycle_color = None
        self.pedestrian_color = None
        self.show_agents_coordinates = None
        self.agents_id_color = None
        self.agents_id_x_offset = None        
        self.agents_id_y_offset = None
        self.agents_coordinates_x_offset = None
        self.agents_coordinates_y_offset = None
        self.start_frame_number = None
        self.generate_mp4 = None
        self.plot_node_link = None
        self.plot_node_shape = None
        self.plot_node_color = None
        self.plot_node_size = None
        self.plot_link_shape = None
        self.plot_link_color = None
        self.plot_link_width = None
        self.plot_lane_boundary = None
        self.plot_left_right_turn = None
        self.output_map_network_csv = None
        self.output_map_network_json = None
        self.read_network_data_from_json = None
        self.plot_agents_trajectories = None
        self.do_map_matching = None
        self.output_map_matching_csv = None
        self.output_map_matching_right_turn = None
        self.output_map_matching_left_turn = None
        self.output_map_matching_u_turn = None
        self.output_map_matching_straight = None
        self.plot_different_types_of_figure = None
        self.plot_lane_and_boundaries = None
        self.plot_lane_polygon = None
        self.lane_polygon_width = None
        self.right_turn_lane_polygon_width = None
        self.right_turn_lane_polygon_offset = None
        self.filtered_left_turn_output_path = None
        self.filtered_right_turn_output_path = None
        self.filtered_u_turn_output_path = None
        self.filtered_straight_output_path = None
        self.plot_signal_light = None
        

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
        self.lane_id = None
        self.link_id = None

 

class Map_feature:
    def __init__(self): 
        self.feature_type = None
        self.speed_limit = None
        self.lane_type = None
        
    
    
class Waymo_motion_training_scenario:
    def __init__(self):
        self.dataset_index = None
        self.scenario_index = None
        self.scenario_id = None
        self.current_time_index = None
        self.sdc_track_index = None
        self.number_of_map_features = None
        
        self.timestep_seconds_list = None
        self.timestep_list = []
        
        self.all_agents_x_list = []
        self.all_agents_y_list = []   
        self.all_agents_z_list = []  
        
        self.agents_list = []
        self.agents_dict = {}
        self.map_features_list = []
        self.signal_light_state_list = []
        self.signal_light_state_dict = {}
        
        self.lane_angle_dict = {}
        
        # topology network
        self.topology_network = None
        self.reference_center_line_list = []
        self.node_list = []
        self.node_dict = {}
        self.link_list = []
        self.link_dict = {}
        self.turn_movement_list = []
        self.turn_movement_dict = {}
        self.lane_dict = {}
        self.connection_point_dict = {}
        self.crossing_point_dict = {}
        self.node_x = None
        self.node_y = None
        self.node_z = 0
        self.virtual_node_up_x = None
        self.virtual_node_up_y = None  
        self.virtual_node_up_z = None  
        self.virtual_node_down_x = None
        self.virtual_node_down_y = None 
        self.virtual_node_down_z = None 
        self.virtual_node_left_x = None
        self.virtual_node_left_y = None  
        self.virtual_node_left_z = None 
        self.virtual_node_right_x = None
        self.virtual_node_right_y = None  
        self.virtual_node_right_z = None 
        self.node_left = []
        self.node_right = []
        self.node_up = []
        self.node_down = []
        self.cluster_centers = None
        self.up_link_equation = []
        self.down_link_equation = []
        self.left_link_equation = []
        self.right_link_equation = []
        
        # white line lists
        self.broken_single_white_list = []
        self.solid_single_white_list  = []
        self.solid_double_white_list  = []
    
        self.broken_single_white_x_list = []
        self.solid_single_white_x_list  = []
        self.solid_double_white_x_list  = []    

        self.broken_single_white_y_list = []
        self.solid_single_white_y_list  = []
        self.solid_double_white_y_list  = []    
    
        # yellow line lists    
        self.broken_single_yellow_list = []
        self.broken_double_yellow_list = []
        self.solid_single_yellow_list  = []
        self.solid_double_yellow_list  = []  
        self.passing_double_yellow_list = [] 

        self.broken_single_yellow_x_list = []
        self.broken_double_yellow_x_list = []
        self.solid_single_yellow_x_list  = []
        self.solid_double_yellow_x_list  = []  
        self.passing_double_yellow_x_list = [] 

        self.broken_single_yellow_y_list = []
        self.broken_double_yellow_y_list = []
        self.solid_single_yellow_y_list  = []
        self.solid_double_yellow_y_list  = []  
        self.passing_double_yellow_y_list = [] 

        self.road_edge_x_list = []
        self.road_edge_y_list = []


        # center lane list  
        self.center_lane_list = [] 
        
        # center lane x list 
        self.center_lane_x_list = [] 
        
        # center lane y list 
        self.center_lane_y_list = [] 
        
        # center lane left boundary list
        self.center_lane_left_boundary_segment_list = []

        # center lane right boundary list
        self.center_lane_right_boundary_segment_list = []
        
        # left boundary segment x,y list
        self.left_boundary_point_x_list = []   
        self.left_boundary_point_y_list = []  
        
        # right boundary segment x,y list
        self.right_boundary_point_x_list = []  
        self.right_boundary_point_y_list = []  
        
        # feature dict
        self.map_feature_dict = {} 
        
        # center lane dict   
        self.center_lane_dict = {}         
        
        # lane lists black   
        self.lane_black_list = []        
    
        # stop sign list
        self.stop_sign_list = []
    
        # cross walk list     
        self.cross_walk_list = []
        
        # road line list
        self.road_line_list = []

        # road line dict
        self.road_line_dict = {}
        
        # road edge list
        self.road_edge_list = []
        
        # region width
        self.scenario_width = None
        
        # scenario center x
        self.scenario_center_x = None
        
        # scenario center y
        self.scenario_center_y = None  
        
        # scenario min x
        self.scenario_min_x = None

        # scenario max x
        self.scenario_max_x = None
        
        # scenario min y
        self.scenario_min_y = None

        # scenario max y
        self.scenario_max_y = None     
        
        # four direction lane list
        self.lane_up_inbound_list   = []
        self.lane_up_outbound_list   = []
        
        self.lane_down_inbound_list = []
        self.lane_down_outbound_list = []
        
        self.lane_left_inbound_list = []
        self.lane_left_outbound_list = []
        
        self.lane_right_inbound_list = []   
        self.lane_right_outbound_list = []
        
        # four direction lane id list
        self.lane_up_inbound_id_list   = []
        self.lane_up_outbound_id_list   = []
        
        self.lane_down_inbound_id_list = []
        self.lane_down_outbound_id_list = []
        
        self.lane_left_inbound_id_list = []
        self.lane_left_outbound_id_list = []
        
        self.lane_right_inbound_id_list = []   
        self.lane_right_outbound_id_list = []


        # four direction turn lane id list
        self.lane_up_inbound_turn_id_list   = []
        self.lane_up_outbound_turn_id_list   = []
        
        self.lane_down_inbound_turn_id_list = []
        self.lane_down_outbound_turn_id_list = []
        
        self.lane_left_inbound_turn_id_list = []
        self.lane_left_outbound_turn_id_list = []
        
        self.lane_right_inbound_turn_id_list = []   
        self.lane_right_outbound_turn_id_list = []
        

        # four direction signal light state list
        self.left_inbound_lane_straight_signal_state_list = []
        self.left_inbound_lane_left_turn_signal_state_list = []
        self.left_inbound_lane_right_turn_signal_state_list = []
        
        self.right_inbound_lane_straight_signal_state_list = []
        self.right_inbound_lane_left_turn_signal_state_list = []
        self.right_inbound_lane_right_turn_signal_state_list = []
      
        self.up_inbound_lane_straight_signal_state_list = []
        self.up_inbound_lane_left_turn_signal_state_list = []
        self.up_inbound_lane_right_turn_signal_state_list = []
    
        self.down_inbound_lane_straight_signal_state_list = []
        self.down_inbound_lane_left_turn_signal_state_list = []
        self.down_inbound_lane_right_turn_signal_state_list = []        

        # four direction turn lane list
        self.left_through_turn_lane_id_list = []
        self.left_left_turn_lane_id_list = []
        self.left_right_turn_lane_id_list = []
        
        self.right_through_turn_lane_id_list = []
        self.right_left_turn_lane_id_list = []
        self.right_right_turn_lane_id_list = []
      
        self.up_through_turn_lane_id_list = []
        self.up_left_turn_lane_id_list = []
        self.up_right_turn_lane_id_list = []
    
        self.down_through_turn_lane_id_list = []
        self.down_left_turn_lane_id_list = []
        self.down_right_turn_lane_id_list = []
    
        # signal light information dict
        self.signal_light_dict = {}
        self.signal_light_list = []
        
        self.lane_singal_light_state_dict = {}
        self.lane_id_singal_light_list = []
        self.lane_id_singal_light_state_list = []
        
        
        self.first_cluster_points_list = []
        self.second_cluster_points_list = []
        self.third_cluster_points_list = []

        self.first_cluster_points_x_list = []
        self.second_cluster_points_x_list = []
        self.third_cluster_points_x_list = []

        self.first_cluster_points_y_list = []
        self.second_cluster_points_y_list = []
        self.third_cluster_points_y_list = []        
    
    def calculate_scenario_region(self):    
              
        #center_x = (np.max(self.all_agents_x_list) + np.min(self.all_agents_x_list)) / 2
        #center_y = (np.max(self.all_agents_y_list) + np.min(self.all_agents_y_list)) / 2       
        #range_x = np.ptp(self.all_agents_x_list) + 30
        #range_y = np.ptp(self.all_agents_y_list) + 30  
        
        center_x = (np.max(self.center_lane_x_list) + np.min(self.center_lane_x_list)) / 2
        center_y = (np.max(self.center_lane_y_list) + np.min(self.center_lane_y_list)) / 2       
        range_x = np.ptp(self.center_lane_x_list) + 30
        range_y = np.ptp(self.center_lane_y_list) + 30          
        
        width = max(range_x, range_y)    
        self.scenario_center_x =  center_x 
        self.scenario_center_y = center_y
        self.scenario_width = width
        self.scenario_min_x = center_x - 0.5 * width
        self.scenario_max_x = center_x + 0.5 * width
        self.scenario_min_y = center_y - 0.5 * width
        self.scenario_max_y = center_y + 0.5 * width
    
    def get_center_lane_x_y_list(self):
        self.center_lane = self.center_lane_list[0]
        self.center_lane_x_list = self.center_lane.point_x_list
        self.center_lane_y_list = self.center_lane.point_y_list
        self.center_lane_left_boundary_segment_list = self.center_lane.left_boundary_segment_list
        self.center_lane_right_boundary_segment_list = self.center_lane.right_boundary_segment_list
    
    def calculate_left_right_lane_boundary_points_v1(self):
        
        for center_lane in self.center_lane_list:
            center_lane_id = center_lane.lane_id
            #print(" --- p --- lane id is :", center_lane_id)
            for left_segment in center_lane.left_boundary_segment_list:
                left_lane_start_index = left_segment.lane_start_index
                left_lane_end_index = left_segment.lane_end_index
                boundary_line_id = left_segment.boundary_feature_id
                #boundary_line = self.road_line_dict[boundary_line_id]
                boundary_line = self.map_feature_dict[boundary_line_id]                
                boundary_line_x_list = boundary_line.point_x_list
                boundary_line_y_list = boundary_line.point_y_list
                
                #print("---lane id :", center_lane_id, "left id : ", boundary_line_id, ' s: ', left_lane_start_index, ' e: ', left_lane_end_index)
                
                #if center_lane_id == 160:
                #    #print(" =========== center lane id is : ", center_lane_id)
                #    #print(" =========== left boundary id is: ", boundary_line_id)
                #    #print(" =========== left boundary start index is: ", left_lane_start_index) 

                '''
                if left_lane_end_index > len(boundary_line_x_list):
                    left_lane_end_index = left_lane_end_index - left_lane_start_index
                    left_lane_start_index = 0
                    left_lane_end_index = len(boundary_line_x_list) - 1           
                '''
                
                if left_lane_start_index == left_lane_end_index:
                    continue
                for i in range(left_lane_start_index, left_lane_end_index):
                    point_x = boundary_line_x_list[i]
                    point_y = boundary_line_y_list[i]
                    center_lane.left_boundary_point_x_list.append(point_x)
                    center_lane.left_boundary_point_y_list.append(point_y)                          

            for right_segment in center_lane.right_boundary_segment_list:
                right_lane_start_index = right_segment.lane_start_index
                right_lane_end_index = right_segment.lane_end_index
                boundary_line_id = right_segment.boundary_feature_id
                #boundary_line = self.road_line_dict[boundary_line_id]
                boundary_line = self.map_feature_dict[boundary_line_id]
                
                boundary_line_x_list = boundary_line.point_x_list
                boundary_line_y_list = boundary_line.point_y_list   
                
                number_of_points_in_boundary = len(boundary_line_x_list)
                
                #print("---lane id :", center_lane_id, "right id:", boundary_line_id, '#:', number_of_points_in_boundary, 's:', left_lane_start_index, 'e:', left_lane_end_index)
                
                '''
                if right_lane_end_index > len(boundary_line_x_list):
                    right_lane_end_index = right_lane_end_index - right_lane_start_index
                    right_lane_start_index = 0
                    right_lane_end_index = len(boundary_line_x_list) - 1
                '''
                               
                if right_lane_start_index == right_lane_end_index:
                    continue
                
                for i in range(right_lane_start_index, right_lane_end_index):
                    point_x = boundary_line_x_list[i]
                    point_y = boundary_line_y_list[i]
                    center_lane.right_boundary_point_x_list.append(point_x)
                    center_lane.right_boundary_point_y_list.append(point_y)    


    def calculate_left_right_lane_boundary_points_v2(self):  
        for center_lane in self.center_lane_list:
            center_lane_id = center_lane.lane_id
            #print(" --- p --- lane id is :", center_lane_id)
            for left_segment in center_lane.left_boundary_segment_list:
                left_lane_start_index = left_segment.lane_start_index
                left_lane_end_index = left_segment.lane_end_index
                boundary_line_id = left_segment.boundary_feature_id
                #boundary_line = self.road_line_dict[boundary_line_id]
                boundary_line = self.map_feature_dict[boundary_line_id]                
                boundary_line_x_list = boundary_line.point_x_list
                boundary_line_y_list = boundary_line.point_y_list
                number_of_points_in_boundary = len(boundary_line_x_list)
                
                #print("---lane id :", center_lane_id, "left id : ", boundary_line_id, ' s: ', left_lane_start_index, ' e: ', left_lane_end_index)                        
                
                if left_lane_start_index == left_lane_end_index:
                    continue
                
                i = 0
                while i < number_of_points_in_boundary:
                    point_x = boundary_line_x_list[i]
                    point_y = boundary_line_y_list[i]
                    center_lane.left_boundary_point_x_list.append(point_x)
                    center_lane.left_boundary_point_y_list.append(point_y)
                    point = (point_x, point_y)
                    center_lane.left_boundary_point_list.append(point)
                    i += 1

            for right_segment in center_lane.right_boundary_segment_list:
                right_lane_start_index = right_segment.lane_start_index
                right_lane_end_index = right_segment.lane_end_index
                boundary_line_id = right_segment.boundary_feature_id
                #boundary_line = self.road_line_dict[boundary_line_id]
                boundary_line = self.map_feature_dict[boundary_line_id]
                
                boundary_line_x_list = boundary_line.point_x_list
                boundary_line_y_list = boundary_line.point_y_list                   
                number_of_points_in_boundary = len(boundary_line_x_list)
                
                #print("---lane id :", center_lane_id, "right id:", boundary_line_id, '#:', number_of_points_in_boundary, 's:', left_lane_start_index, 'e:', left_lane_end_index)               
                
                if right_lane_start_index == right_lane_end_index:
                    continue
                
                i = 0
                while i < number_of_points_in_boundary:
                    point_x = boundary_line_x_list[i]
                    point_y = boundary_line_y_list[i]
                    center_lane.right_boundary_point_x_list.append(point_x)
                    center_lane.right_boundary_point_y_list.append(point_y)
                    point = (point_x, point_y)
                    center_lane.right_boundary_point_list.append(point)
                    i += 1
    
    def calculate_left_right_lane_boundary_points(self):
        for left_segment in self.center_lane_left_boundary_segment_list:
            left_segment_x_list = []
            left_segment_y_list = []
            left_lane_start_index = left_segment.lane_start_index
            left_lane_end_index = left_segment.lane_end_index
            if left_lane_start_index == left_lane_end_index:
                continue
            for i in range(left_lane_start_index, left_lane_end_index):
                point_x = self.center_lane_x_list[i]
                point_y = self.center_lane_y_list[i]
                left_segment_x_list.append(point_x)
                left_segment_y_list.append(point_y)
            self.left_boundary_point_x_list.append(left_segment_x_list)
            self.left_boundary_point_y_list.append(left_segment_y_list)

        for right_segment in self.center_lane_right_boundary_segment_list:
            right_segment_x_list = []
            right_segment_y_list = []
            right_lane_start_index = right_segment.lane_start_index
            right_lane_end_index = right_segment.lane_end_index
            if right_lane_start_index == right_lane_end_index:
                continue
            for i in range(right_lane_start_index, right_lane_end_index):
                point_x = self.center_lane_x_list[i]
                point_y = self.center_lane_y_list[i]
                right_segment_x_list.append(point_x)
                right_segment_y_list.append(point_y)
            self.right_boundary_point_x_list.append(right_segment_x_list)
            self.right_boundary_point_y_list.append(right_segment_y_list)  


    def calculate_lane_polygon(self):
        for center_lane in self.center_lane_list:
            center_lane_id = center_lane.lane_id
            if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
                if len(center_lane.left_boundary_point_x_list) != 0 and len(center_lane.right_boundary_point_x_list) != 0:
                    random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                    polygon_point_list = center_lane.left_boundary_point_list + center_lane.right_boundary_point_list
                    polygon = Polygon(polygon_point_list, color = random_color[0], alpha = 0.5)
                    center_lane.polygon = polygon

                
    def calculate_nodes_and_links_v1(self):
        retry_times = 0
        while True:
            #retry_times += 1
            try: 
                reference_lane_list = []
                intersection_point_x_list = []
                intersection_point_y_list = []
                for center_lane in self.center_lane_list:
                    center_lane_id = center_lane.lane_id
                    #print(" ============ center lane : ", center_lane_id)
                    center_lane_type = center_lane.lane_type
                    center_lane_slope = center_lane.slope
                    if center_lane.is_straight:
                        #if center_lane_id == 155:                   
                        #    #print(" ============ center type : ", center_lane_type)
                        if center_lane_type == MAP_feature.TYPE_FREEWAY or center_lane_type == MAP_feature.TYPE_SURFACE_STREET:   
                            #print(" ============ center lane up node: ", center_lane_id)
                            reference_lane_list.append(center_lane)
                            self.reference_center_line_list.append(center_lane)
        
                
                used_center_lane_id_list = []
                for center_lane in reference_lane_list:
                    center_lane_id = center_lane.lane_id
                    lane_equation = center_lane.equation_list
                    for other_center_lane in reference_lane_list:
                        other_center_lane_id = other_center_lane.lane_id
                        if other_center_lane_id == center_lane_id:
                            continue
                        if other_center_lane_id in used_center_lane_id_list:
                            continue
                        other_center_lane_slope = other_center_lane.slope
                        angle_between_two_lanes = math.degrees(math.atan((other_center_lane_slope - center_lane_slope)/(1+(other_center_lane_slope * center_lane_slope))))
                        angle_between_two_lanes = abs(angle_between_two_lanes)
                        
                        if angle_between_two_lanes < 50 or angle_between_two_lanes > 130:
                            continue
                        
                        #print("center x is : ", self.scenario_center_x)
                        #print("center y is : ", self.scenario_center_y)
                        #print("scenario width is : ", self.scenario_width)
                        
                        # calculate intersection point
                        other_center_lane_equation = other_center_lane.equation_list
                        L2 = other_center_lane_equation
                        L1 = lane_equation
                        D  = L1[0] * L2[1] - L1[1] * L2[0]
                        Dx = - L1[2] * L2[1] + L1[1] * L2[2]
                        Dy = - L1[0] * L2[2] + L1[2] * L2[0]                
                        if D != 0:
                            x = Dx / D
                            y = Dy / D
                            if x > self.scenario_center_x - 0.5 * self.scenario_width and x < self.scenario_center_x + 0.5 * self.scenario_width:
                                if y > self.scenario_center_y - 0.5 * self.scenario_width and y < self.scenario_center_y + 0.5 * self.scenario_width:
                                    intersection_point_x_list.append(x)
                                    intersection_point_y_list.append(y)
                    used_center_lane_id_list.append(other_center_lane_id)
                
                point_index = 0
                point_list  = []
                
                first_intersection_point_x_list  = []
                first_intersection_point_y_list  = []
                second_intersection_point_x_list = []
                second_intersection_point_y_list = []   
                third_intersection_point_x_list = []
                third_intersection_point_y_list = []      
                
                number_of_points = len(intersection_point_x_list)
                while point_index < number_of_points:
                    x = intersection_point_x_list[point_index]
                    y = intersection_point_y_list[point_index]
                    point = [x, y]
                    point_list.append(point)
                    point_index += 1
                
                points_array = np.array([[point[0], point[1]] for point in point_list])
        
                model = KMeans(n_clusters = 3)
                # Fit model to training data
                model.fit(points_array)
        
                # Get predicted cluster centers
                model.cluster_centers_ # shape: (n_cluster, n_features) 
                
                self.cluster_centers = model.cluster_centers_
                #print(" --- labels: ", model.labels_)
                
                label_index = -1
                for label in model.labels_:
                    label_index += 1
                    x = intersection_point_x_list[label_index]
                    y = intersection_point_y_list[label_index]
                    if label == 0:
                        first_intersection_point_x_list.append(x)
                        first_intersection_point_y_list.append(y)
                    if label == 1:
                        second_intersection_point_x_list.append(x)
                        second_intersection_point_y_list.append(y)
                    if label == 2:
                        third_intersection_point_x_list.append(x)
                        third_intersection_point_y_list.append(y)       
                
                #print("   --- length of second point x :", len(second_intersection_point_x_list))
                #print("   --- length of second point y :", len(second_intersection_point_y_list))   
                
                self.first_cluster_points_x_list = first_intersection_point_x_list
                self.first_cluster_points_y_list = first_intersection_point_y_list

                self.second_cluster_points_x_list = second_intersection_point_x_list
                self.second_cluster_points_y_list = second_intersection_point_y_list
                
                self.third_cluster_points_x_list = third_intersection_point_x_list
                self.third_cluster_points_y_list = third_intersection_point_y_list
                        
                if len(first_intersection_point_x_list) > len(second_intersection_point_x_list):
                    intersection_point_x_list = first_intersection_point_x_list
                    intersection_point_y_list = first_intersection_point_y_list
                    self.node_x = self.cluster_centers[0, 0]
                    self.node_y = self.cluster_centers[0, 1]
                else:
                    intersection_point_x_list = second_intersection_point_x_list
                    intersection_point_y_list = second_intersection_point_y_list   
                    self.node_x = self.cluster_centers[1, 0]
                    self.node_y = self.cluster_centers[1, 1]
        
                # initialize topology network of the scenario
                center_node = Waymo_scenario_topology_node()
                center_node.node_id = 0
                center_node.coordinate = [self.node_x, self.node_y]
                center_node.dict["node_id"] = 0
                center_node.dict["x"] = self.node_x
                center_node.dict["y"] = self.node_y
                center_node.dict["z"] = self.node_z
                center_node.dict["type"] = "Intersection"
        
                lane_up_list   = []
                lane_down_list = []
                lane_left_list = []
                lane_right_list = []
        
                lane_up_inbound_list   = []
                lane_down_inbound_list = []
                lane_left_inbound_list = []
                lane_right_inbound_list = []
                lane_up_outbound_list   = []
                lane_down_outbound_list = []
                lane_left_outbound_list = []
                lane_right_outbound_list = []
        
                for center_lane in reference_lane_list:
                    center_lane_id = center_lane.lane_id
                    #print(" ============ center lane up ID: ", center_lane_id)
                    lane_equation = center_lane.equation_list
                    
                    center_lane_first_x = center_lane.point_x_list[0]
                    center_lane_first_y = center_lane.point_y_list[0]
                    center_lane_last_x  = center_lane.point_x_list[-1]
                    center_lane_last_y  = center_lane.point_y_list[-1]
                    
                    if center_lane_first_y < self.node_y and center_lane_last_y < self.node_y:
                        # inbound lane
                        #print(" ============ center lane up node: ", center_lane_id)
                        if center_lane_first_y < center_lane_last_y:
                            if abs(center_lane_first_y - center_lane_last_y) > abs(center_lane_first_x - center_lane_last_x):                
                                center_lane.is_straight = True
                                lane_down_inbound_list.append(center_lane)
                                
                        # outbound lane
                        if center_lane_first_y > center_lane_last_y:
                            if abs(center_lane_first_y - center_lane_last_y) > abs(center_lane_first_x - center_lane_last_x):                   
                                center_lane.is_straight = True
                                lane_down_outbound_list.append(center_lane)
                                
                        # lanes for calculate link
                        if center_lane_first_y < center_lane_last_y or center_lane_first_y > center_lane_last_y:
                            if abs(center_lane_first_y - center_lane_last_y) > abs(center_lane_first_x - center_lane_last_x):
                                if calculate_distance_between_two_points(center_lane_first_x, center_lane_first_y, self.node_x, self.node_y) < 40 or\
                                    calculate_distance_between_two_points(center_lane_last_x, center_lane_last_y, self.node_x, self.node_y) < 40:
                                    if center_lane.lane_length > 15:   
                                        center_lane.is_straight = True
                                        lane_down_list.append(center_lane)
                                        #print(" --- lane down list id is : ", center_lane.lane_id)
                                        #print(" --- lane down list length is : ", center_lane.lane_length)            
                    
                    if center_lane_first_y > self.node_y and center_lane_last_y > self.node_y:
                        # outbound lane
                        #print(" ============ center lane up node: ", center_lane_id)
                        if center_lane_first_y < center_lane_last_y:
                            if abs(center_lane_first_y - center_lane_last_y) > abs(center_lane_first_x - center_lane_last_x):                
                                lane_up_outbound_list.append(center_lane)
                                center_lane.is_straight = True
                                
                        # inbound lane
                        if center_lane_first_y > center_lane_last_y:
                            #print(" ============ center lane up node: ", center_lane_id)
                            if abs(center_lane_first_y - center_lane_last_y) > abs(center_lane_first_x - center_lane_last_x):                   
                                lane_up_inbound_list.append(center_lane)
                                center_lane.is_straight = True
                        
                        if center_lane_first_y > center_lane_last_y or center_lane_first_y < center_lane_last_y:
                            if abs(center_lane_first_y - center_lane_last_y) > abs(center_lane_first_x - center_lane_last_x):                    
                                if calculate_distance_between_two_points(center_lane_first_x, center_lane_first_y, self.node_x, self.node_y) < 40 or\
                                    calculate_distance_between_two_points(center_lane_last_x, center_lane_last_y, self.node_x, self.node_y) < 40:
                                    if center_lane.lane_length > 15:
                                        center_lane.is_straight = True
                                        lane_up_list.append(center_lane)   
                                        #print(" --- lane up list id is : ", center_lane.lane_id)
        
                    if center_lane_first_x < self.node_x and center_lane_last_x < self.node_x:
                        # inbound lane
                        if center_lane_first_x < center_lane_last_x:
                            if abs(center_lane_first_y - center_lane_last_y) < abs(center_lane_first_x - center_lane_last_x):                 
                                lane_left_inbound_list.append(center_lane)
                                center_lane.is_straight = True
                                
                        # outbound lane
                        if center_lane_first_x > center_lane_last_x:
                            if abs(center_lane_first_y - center_lane_last_y) < abs(center_lane_first_x - center_lane_last_x):                    
                                lane_left_outbound_list.append(center_lane)
                                center_lane.is_straight = True
        
                        if center_lane_first_x < center_lane_last_x or center_lane_first_x > center_lane_last_x:
                            if abs(center_lane_first_y - center_lane_last_y) < abs(center_lane_first_x - center_lane_last_x):                    
                                if calculate_distance_between_two_points(center_lane_first_x, center_lane_first_y, self.node_x, self.node_y) < 40 or\
                                    calculate_distance_between_two_points(center_lane_last_x, center_lane_last_y, self.node_x, self.node_y) < 40:
                                    if center_lane.lane_length > 15:  
                                        lane_left_list.append(center_lane)
                                        center_lane.is_straight = True
                                        #print(" --- lane left list id is : ", center_lane.lane_id)
        
                    if center_lane_first_x > self.node_x and center_lane_last_x > self.node_x:
                        # outbound lane
                        if center_lane_first_x < center_lane_last_x:
                            if abs(center_lane_first_y - center_lane_last_y) < abs(center_lane_first_x - center_lane_last_x):                 
                                lane_right_outbound_list.append(center_lane)
                                center_lane.is_straight = True
                                
                        # outbound lane
                        if center_lane_first_x > center_lane_last_x:
                            if abs(center_lane_first_y - center_lane_last_y) < abs(center_lane_first_x - center_lane_last_x):                    
                                lane_right_inbound_list.append(center_lane)
                                center_lane.is_straight = True
                        
                        if center_lane_first_x > center_lane_last_x or center_lane_first_x < center_lane_last_x:
                            if abs(center_lane_first_y - center_lane_last_y) < abs(center_lane_first_x - center_lane_last_x):                    
                                if calculate_distance_between_two_points(center_lane_first_x, center_lane_first_y, self.node_x, self.node_y) < 40 or\
                                    calculate_distance_between_two_points(center_lane_last_x, center_lane_last_y, self.node_x, self.node_y) < 40:                  
                                    if center_lane.lane_length > 15: 
                                        lane_right_list.append(center_lane)
                                        center_lane.is_straight = True
                                        #print(" --- lane right list id is : ", center_lane.lane_id)
        
                self.lane_up_inbound_list   = lane_up_inbound_list
                self.lane_up_outbound_list  = lane_up_outbound_list
                
                self.lane_down_inbound_list = lane_down_inbound_list
                self.lane_down_outbound_list = lane_down_outbound_list
                
                self.lane_left_inbound_list = lane_left_inbound_list
                self.lane_left_outbound_list = lane_left_outbound_list
                
                self.lane_right_inbound_list = lane_right_inbound_list  
                self.lane_right_outbound_list = lane_right_outbound_list
        
                for lane in lane_up_inbound_list:
                    lane_id = lane.lane_id
                    self.lane_up_inbound_id_list.append(lane_id)
                for lane in lane_down_inbound_list:
                    lane_id = lane.lane_id
                    self.lane_down_inbound_id_list.append(lane_id)                    
                for lane in lane_left_inbound_list:
                    lane_id = lane.lane_id
                    self.lane_left_inbound_id_list.append(lane_id)
                for lane in lane_right_inbound_list:
                    lane_id = lane.lane_id
                    self.lane_right_inbound_id_list.append(lane_id)
                    
                self.virtual_node_left_x  = self.scenario_min_x
                self.virtual_node_right_x = self.scenario_max_x
                self.virtual_node_up_y    = self.scenario_max_y
                self.virtual_node_down_y  = self.scenario_min_y  
            
                #  ==================== up link ====================== 
                positive_a_list = []
                positive_b_list = []
                negitive_a_list = []
                negitive_b_list = []        
                for center_lane in lane_up_list:
                    lane_equation = center_lane.equation_list
                    a = lane_equation[0]
                    b = lane_equation[1]
                    if a >= 0:                        
                        positive_a_list.append(a)
                        positive_b_list.append(b)
                    else:
                        negitive_a_list.append(a)
                        negitive_b_list.append(b)                
                a = None
                b = None
                c = None
                if len(positive_a_list) >= len(negitive_a_list):
                    a = sum(positive_a_list)/len(positive_a_list)
                    b = sum(positive_b_list)/len(positive_b_list)
                    c = -1 * a * self.node_x - 1 * b * self.node_y      
                else:
                    a = sum(negitive_a_list)/len(negitive_a_list)
                    b = sum(negitive_b_list)/len(negitive_b_list)
                    c = -1 * a * self.node_x - 1 * b * self.node_y       
                self.up_link_equation = [a,b,c]     
                x_up = (-b / a) * self.virtual_node_up_y + (-c / a)
                self.virtual_node_up_x = x_up
                self.virtual_node_up_z = 0
                
                # construct node and link in up direction
                virtual_node_up = Waymo_scenario_topology_node()
                virtual_node_up.node_id = 4    
                virtual_node_up.coordinate = [self.virtual_node_up_x, self.virtual_node_up_y]
                virtual_node_up.dict["node_id"] = 4
                virtual_node_up.dict["x"] = self.virtual_node_up_x
                virtual_node_up.dict["y"] = self.virtual_node_up_y
                virtual_node_up.dict["z"] = self.virtual_node_up_z
                virtual_node_up.dict["type"] = "Intersection"
                  
                link_up_inbound = Waymo_scenario_topology_link()
                link_up_inbound.link_id = 6
                link_up_inbound.link_from_node_id = virtual_node_up.node_id
                link_up_inbound.link_to_node_id = center_node.node_id
                link_up_inbound.link_from_node = virtual_node_up
                link_up_inbound.link_to_node = center_node
                link_up_inbound.lane_list = lane_up_inbound_list  
                link_up_inbound.link_x = [self.virtual_node_up_x, self.node_x]
                link_up_inbound.link_y = [self.virtual_node_up_y, self.node_y]
                shape_point_start = (self.virtual_node_up_x, self.virtual_node_up_y, self.virtual_node_up_z)
                shape_point_end = (self.node_x, self.node_y, self.node_z)
                
                link_up_inbound.dict["link_id"] = 6
                link_up_inbound.dict["unode_id"] = virtual_node_up.node_id
                link_up_inbound.dict["dnode_id"] = center_node.node_id
                link_up_inbound.dict["number_lane"] = len(lane_up_inbound_list)
                link_up_inbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
                 
        
                link_up_outbound = Waymo_scenario_topology_link()
                link_up_outbound.link_id = 7
                link_up_outbound.link_from_node_id = center_node.node_id
                link_up_outbound.link_to_node_id = virtual_node_up.node_id
                link_up_outbound.link_from_node = center_node
                link_up_outbound.link_to_node = virtual_node_up
                link_up_outbound.lane_list = lane_up_outbound_list
                link_up_outbound.link_x = [self.node_x, self.virtual_node_up_x]
                link_up_outbound.link_y = [self.node_y, self.virtual_node_up_y]
                shape_point_start = (self.node_x, self.node_y, self.node_z)
                shape_point_end = (self.virtual_node_up_x, self.virtual_node_up_y, self.virtual_node_up_z)
                
                link_up_outbound.dict["link_id"] = 7
                link_up_outbound.dict["unode_id"] = center_node.node_id
                link_up_outbound.dict["dnode_id"] = virtual_node_up.node_id
                link_up_outbound.dict["number_lane"] = len(lane_up_outbound_list)
                link_up_outbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
                
                virtual_node_up.inbound_link_list.append(link_up_outbound)
                virtual_node_up.outbound_link_list.append(link_up_inbound)
                
                center_node.inbound_link_list.append(link_up_inbound)
                center_node.outbound_link_list.append(link_up_outbound)      
                
                for center_lane in link_up_inbound.lane_list:
                    center_lane.link_id = 6
        
                for center_lane in link_up_outbound.lane_list:
                    center_lane.link_id = 7           
                   
                self.node_list.append(virtual_node_up)     
                self.link_list.append(link_up_inbound)
                self.link_list.append(link_up_outbound)
                self.node_dict[virtual_node_up.node_id] = virtual_node_up.dict
                self.link_dict[link_up_inbound.link_id] = link_up_inbound.dict
                self.link_dict[link_up_outbound.link_id] = link_up_outbound.dict     
                
                # ==================== down link =====================
                positive_a_list = []
                positive_b_list = []
                negitive_a_list = []
                negitive_b_list = []        
                for center_lane in lane_down_list:
                    lane_equation = center_lane.equation_list
                    a = lane_equation[0]
                    b = lane_equation[1]
                    if a >= 0:                        
                        positive_a_list.append(a)
                        positive_b_list.append(b)
                    else:
                        negitive_a_list.append(a)
                        negitive_b_list.append(b)                
                a = None
                b = None
                c = None
                
                if len(positive_a_list) == len(negitive_a_list) == 0:
                    a = 1
                    b = 0
                    c = -1 * a * self.node_x - 1 * b * self.node_y
                else:
                    if len(positive_a_list) >= len(negitive_a_list):
                        a = sum(positive_a_list)/len(positive_a_list)
                        b = sum(positive_b_list)/len(positive_b_list)
                        c = -1 * a * self.node_x - 1 * b * self.node_y      
                    else:
                        a = sum(negitive_a_list)/len(negitive_a_list)
                        b = sum(negitive_b_list)/len(negitive_b_list)
                        c = -1 * a * self.node_x - 1 * b * self.node_y               
                    
                self.down_link_equation = [a,b,c]        
                x_down = (-b / a) * self.virtual_node_down_y + (-c / a)
                self.virtual_node_down_x = x_down
                self.virtual_node_down_z = 0
                
                # construct node and link in down direction
                virtual_node_down = Waymo_scenario_topology_node()
                virtual_node_down.node_id = 2     
                virtual_node_down.coordinate = [self.virtual_node_down_x, self.virtual_node_down_y]
                virtual_node_down.dict["node_id"] = 2
                virtual_node_down.dict["x"] = self.virtual_node_down_x
                virtual_node_down.dict["y"] = self.virtual_node_down_y
                virtual_node_down.dict["z"] = self.virtual_node_down_z
                virtual_node_down.dict["type"] = "Intersection"
                  
                link_down_inbound = Waymo_scenario_topology_link()
                link_down_inbound.link_id = 2
                link_down_inbound.link_from_node_id = virtual_node_down.node_id
                link_down_inbound.link_to_node_id = center_node.node_id
                link_down_inbound.link_from_node = virtual_node_down
                link_down_inbound.link_to_node = center_node
                link_down_inbound.lane_list = lane_down_inbound_list  
                link_down_inbound.link_x = [self.virtual_node_down_x, self.node_x]
                link_down_inbound.link_y = [self.virtual_node_down_y, self.node_y]
                shape_point_start = (self.virtual_node_down_x, self.virtual_node_down_y, self.virtual_node_down_z)
                shape_point_end = (self.node_x, self.node_y, self.node_z)
                
                link_down_inbound.dict["link_id"] = 2
                link_down_inbound.dict["unode_id"] = virtual_node_down.node_id
                link_down_inbound.dict["dnode_id"] = center_node.node_id
                link_down_inbound.dict["number_lane"] = len(lane_down_inbound_list)
                link_down_inbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
        
        
                link_down_outbound = Waymo_scenario_topology_link()
                link_down_outbound.link_id = 3
                link_down_outbound.link_from_node_id = center_node.node_id
                link_down_outbound.link_to_node_id = virtual_node_down.node_id
                link_down_outbound.link_from_node = center_node
                link_down_outbound.link_to_node = virtual_node_down
                link_down_outbound.lane_list = lane_down_outbound_list
                link_down_outbound.link_x = [self.node_x, self.virtual_node_down_x]
                link_down_outbound.link_y = [self.node_x, self.virtual_node_down_x]
                shape_point_start = (self.node_x, self.node_y, self.node_y)
                shape_point_end = (self.virtual_node_down_x, self.virtual_node_down_y, self.virtual_node_down_z)
                
                link_down_outbound.dict["link_id"] = 3
                link_down_outbound.dict["unode_id"] = center_node.node_id
                link_down_outbound.dict["dnode_id"] = virtual_node_down.node_id
                link_down_outbound.dict["number_lane"] = len(lane_down_outbound_list)
                link_down_outbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
                
                virtual_node_down.inbound_link_list.append(link_down_outbound)
                virtual_node_down.outbound_link_list.append(link_down_inbound)
                
                center_node.inbound_link_list.append(link_down_inbound)
                center_node.outbound_link_list.append(link_down_outbound)        
        
                for center_lane in link_down_inbound.lane_list:
                    center_lane.link_id = 2
        
                for center_lane in link_down_outbound.lane_list:
                    center_lane.link_id = 3          
        
                self.node_list.append(virtual_node_down)
                self.link_list.append(link_down_inbound)
                self.link_list.append(link_down_outbound)
                self.node_dict[virtual_node_down.node_id] = virtual_node_down.dict
                self.link_dict[link_down_inbound.link_id] = link_down_inbound.dict
                self.link_dict[link_down_outbound.link_id] = link_down_outbound.dict  
                
                # ==================== left link ====================
                positive_a_list = []
                positive_b_list = []
                negitive_a_list = []
                negitive_b_list = []        
                for center_lane in lane_left_list:
                    lane_equation = center_lane.equation_list
                    a = lane_equation[0]
                    b = lane_equation[1]
                    if a >= 0:                        
                        positive_a_list.append(a)
                        positive_b_list.append(b)
                    else:
                        negitive_a_list.append(a)
                        negitive_b_list.append(b)                
                a = None
                b = None
                c = None
                if len(positive_a_list) >= len(negitive_a_list):
                    a = sum(positive_a_list)/len(positive_a_list)
                    b = sum(positive_b_list)/len(positive_b_list)
                    c = -1 * a * self.node_x - 1 * b * self.node_y      
                else:
                    a = sum(negitive_a_list)/len(negitive_a_list)
                    b = sum(negitive_b_list)/len(negitive_b_list)
                    c = -1 * a * self.node_x - 1 * b * self.node_y          
                self.left_link_equation = [a,b,c]
                y_left = (-a / b) * self.virtual_node_left_x + (-c / b)
                self.virtual_node_left_y = y_left
                self.virtual_node_left_z = 0
        
                # construct node and link in left direction
                virtual_node_left = Waymo_scenario_topology_node()
                virtual_node_left.node_id = 1
                virtual_node_left.coordinate = [self.virtual_node_left_x, self.virtual_node_left_y]
                virtual_node_left.dict["node_id"] = 1
                virtual_node_left.dict["x"] = self.virtual_node_left_x
                virtual_node_left.dict["y"] = self.virtual_node_left_y
                virtual_node_left.dict["z"] = self.virtual_node_left_z
                virtual_node_left.dict["type"] = "Intersection"
                         
                link_left_inbound = Waymo_scenario_topology_link()
                link_left_inbound.link_id = 0
                link_left_inbound.link_from_node_id = virtual_node_left.node_id
                link_left_inbound.link_to_node_id = center_node.node_id
                link_left_inbound.link_from_node = virtual_node_left
                link_left_inbound.link_to_node = center_node
                link_left_inbound.lane_list = lane_left_inbound_list  
                link_left_inbound.link_x = [self.virtual_node_left_x, self.node_x]
                link_left_inbound.link_y = [self.virtual_node_left_y, self.node_y]
                shape_point_start = (self.virtual_node_left_x, self.virtual_node_left_y, self.virtual_node_left_z)
                shape_point_end = (self.node_x, self.node_y, self.node_z)
                
                link_left_inbound.dict["link_id"] = 0
                link_left_inbound.dict["unode_id"] = virtual_node_left.node_id
                link_left_inbound.dict["dnode_id"] = center_node.node_id
                link_left_inbound.dict["number_lane"] = len(lane_left_inbound_list)
                link_left_inbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
        
        
                link_left_outbound = Waymo_scenario_topology_link()
                link_left_outbound.link_id = 1
                link_left_outbound.link_from_node_id = center_node.node_id
                link_left_outbound.link_to_node_id = virtual_node_left.node_id
                link_left_outbound.link_from_node = center_node
                link_left_outbound.link_to_node = virtual_node_left
                link_left_outbound.lane_list = lane_left_outbound_list
                link_left_outbound.link_x = [self.node_x, self.virtual_node_left_x]
                link_left_outbound.link_y = [self.node_x, self.virtual_node_left_x]
                shape_point_start = (self.node_x, self.node_y, self.node_z)
                shape_point_end = (self.virtual_node_left_x, self.virtual_node_left_y, self.virtual_node_left_z)
                
                link_left_outbound.dict["link_id"] = 1
                link_left_outbound.dict["unode_id"] = center_node.node_id
                link_left_outbound.dict["dnode_id"] = virtual_node_left.node_id
                link_left_outbound.dict["number_lane"] = len(lane_left_outbound_list)
                link_left_outbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
        
                
                virtual_node_left.inbound_link_list.append(link_left_outbound)
                virtual_node_left.outbound_link_list.append(link_left_inbound)
                
                center_node.inbound_link_list.append(link_left_inbound)
                center_node.outbound_link_list.append(link_left_outbound)  
        
                for center_lane in link_left_inbound.lane_list:
                    center_lane.link_id = 0
        
                for center_lane in link_left_outbound.lane_list:
                    center_lane.link_id = 1  
        
                self.node_list.append(virtual_node_left)
                self.link_list.append(link_left_inbound)
                self.link_list.append(link_left_outbound)
                self.node_dict[virtual_node_left.node_id] = virtual_node_left.dict
                self.link_dict[link_left_inbound.link_id] = link_left_inbound.dict
                self.link_dict[link_left_outbound.link_id] = link_left_outbound.dict
        
                # ==================== right link ======================
                positive_a_list = []
                positive_b_list = []
                negitive_a_list = []
                negitive_b_list = []        
                for center_lane in lane_right_list:
                    lane_equation = center_lane.equation_list
                    a = lane_equation[0]
                    b = lane_equation[1]
                    if a >= 0:                        
                        positive_a_list.append(a)
                        positive_b_list.append(b)
                    else:
                        negitive_a_list.append(a)
                        negitive_b_list.append(b)                
                a = None
                b = None
                c = None
                if len(positive_a_list) >= len(negitive_a_list):
                    a = sum(positive_a_list)/len(positive_a_list)
                    b = sum(positive_b_list)/len(positive_b_list)
                    c = -1 * a * self.node_x - 1 * b * self.node_y      
                else:
                    a = sum(negitive_a_list)/len(negitive_a_list)
                    b = sum(negitive_b_list)/len(negitive_b_list)
                    c = -1 * a * self.node_x - 1 * b * self.node_y          
                self.right_link_equation = [a,b,c]
                y_right = (-a / b) * self.virtual_node_right_x + (-c / b)   
                self.virtual_node_right_y = y_right      
                self.virtual_node_right_z = 0   
        
                # construct node and link in right direction
                virtual_node_right = Waymo_scenario_topology_node()
                virtual_node_right.node_id = 3      
                virtual_node_right.coordinate = [self.virtual_node_right_x, self.virtual_node_right_y]
                virtual_node_right.dict["node_id"] = 3
                virtual_node_right.dict["x"] = self.virtual_node_right_x
                virtual_node_right.dict["y"] = self.virtual_node_right_y
                virtual_node_right.dict["z"] = self.virtual_node_right_z
                virtual_node_right.dict["type"] = "Intersection"
                  
                link_right_inbound = Waymo_scenario_topology_link()
                link_right_inbound.link_id = 4
                link_right_inbound.link_from_node_id = virtual_node_right.node_id
                link_right_inbound.link_to_node_id = center_node.node_id
                link_right_inbound.link_from_node = virtual_node_right
                link_right_inbound.link_to_node = center_node
                link_right_inbound.lane_list = lane_right_inbound_list  
                link_right_inbound.link_x = [self.virtual_node_right_x, self.node_x]
                link_right_inbound.link_y = [self.virtual_node_right_y, self.node_y]
                shape_point_start = (self.virtual_node_right_x, self.virtual_node_right_y, self.virtual_node_right_z)
                shape_point_end = (self.node_x, self.node_y, self.node_z)
                
                link_right_inbound.dict["link_id"] = 4
                link_right_inbound.dict["unode_id"] = virtual_node_right.node_id
                link_right_inbound.dict["dnode_id"] = center_node.node_id
                link_right_inbound.dict["number_lane"] = len(lane_right_inbound_list)
                link_right_inbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
                
                link_right_outbound = Waymo_scenario_topology_link()
                link_right_outbound.link_id = 5
                link_right_outbound.link_from_node_id = center_node.node_id
                link_right_outbound.link_to_node_id = virtual_node_right.node_id
                link_right_outbound.link_from_node = center_node
                link_right_outbound.link_to_node = virtual_node_right
                link_right_outbound.lane_list = lane_right_outbound_list
                link_right_outbound.link_x = [self.node_x, self.virtual_node_right_x]
                link_right_outbound.link_y = [self.node_x, self.virtual_node_right_x]
                shape_point_start = (self.node_x, self.node_y, self.node_z)
                shape_point_end = (self.virtual_node_right_x, self.virtual_node_right_y, self.virtual_node_right_z)
                
                link_right_outbound.dict["link_id"] = 5
                link_right_outbound.dict["unode_id"] = center_node.node_id
                link_right_outbound.dict["dnode_id"] = virtual_node_right.node_id
                link_right_outbound.dict["number_lane"] = len(lane_right_outbound_list)
                link_right_outbound.dict["shape_point_curve"] = [shape_point_start, shape_point_end]
                
                virtual_node_right.inbound_link_list.append(link_right_outbound)
                virtual_node_right.outbound_link_list.append(link_right_inbound)
                
                center_node.inbound_link_list.append(link_right_inbound)
                center_node.outbound_link_list.append(link_right_outbound)  
        
                for center_lane in link_right_inbound.lane_list:
                    center_lane.link_id = 4
        
                for center_lane in link_right_outbound.lane_list:
                    center_lane.link_id = 5 
        
                self.node_list.append(virtual_node_right)
                self.link_list.append(link_right_inbound)
                self.link_list.append(link_right_outbound)
                self.node_dict[virtual_node_right.node_id] = virtual_node_right.dict
                self.link_dict[link_right_inbound.link_id] = link_right_inbound.dict
                self.link_dict[link_right_outbound.link_id] = link_right_outbound.dict
                
                self.node_list.append(center_node)
                self.node_dict[center_node.node_id] = center_node.dict  
        
                # ======================= calculate turn movements : left =========================== 
                
                # 1. from left to down : 0 - 3
                turn_from_left_to_down = Waymo_scenario_topology_turn_movement()
                turn_from_left_to_down.turn_id = 0
                turn_from_left_to_down.turn_type = "right"
                turn_from_left_to_down.from_link_id = 0
                turn_from_left_to_down.to_link_id = 3
                turn_from_left_to_down.from_link = link_left_inbound
                turn_from_left_to_down.to_link = link_down_outbound
                
                turn_from_left_to_down.dict["turn_movement_id"] = 0
                turn_from_left_to_down.dict["type"] = "RIGHT_TURN"
                turn_from_left_to_down.dict["inbound_link_id"]  = 0
                turn_from_left_to_down.dict["outbound_link_id"] = 3
                turn_from_left_to_down.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
                
                for center_lane in link_left_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_right_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                       
                                exist_lane.turn_movement_id = 0
                                exist_lane.node_id = -1
                                turn_from_left_to_down.lane_list.append(exist_lane)  
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]
                                #print(" --- left to down lanes: ", exit_lane_id)
                                
                turn_from_left_to_down.dict["number_lane"] = len(turn_from_left_to_down.lane_list)
                turn_from_left_to_down.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]
                
                self.turn_movement_list.append(turn_from_left_to_down)
                if turn_from_left_to_down != None:         
                    self.turn_movement_dict[turn_from_left_to_down.turn_id] = turn_from_left_to_down.dict
                            
                # 2. from left to up : 0 - 7
                turn_from_left_to_up = Waymo_scenario_topology_turn_movement()
                turn_from_left_to_up.turn_id = 1
                turn_from_left_to_up.turn_type = "left"
                turn_from_left_to_up.from_link_id = 0
                turn_from_left_to_up.to_link_id = 7
                turn_from_left_to_up.from_link = link_left_inbound
                turn_from_left_to_up.to_link = link_up_outbound
                
                turn_from_left_to_up.dict["turn_movement_id"] = 1
                turn_from_left_to_up.dict["type"] = "LEFT_TURN"
                turn_from_left_to_up.dict["inbound_link_id"]  = 0
                turn_from_left_to_up.dict["outbound_link_id"] = 7
                turn_from_left_to_up.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
                
                for center_lane in link_left_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_left_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                       
                                exist_lane.turn_movement_id = 1
                                exist_lane.node_id = -1
                                turn_from_left_to_up.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                        
                                #print(" --- left to up lanes: ", exit_lane_id)
                turn_from_left_to_up.dict["number_lane"] = len(turn_from_left_to_up.lane_list)
                turn_from_left_to_up.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]   
                self.turn_movement_list.append(turn_from_left_to_up)
                if turn_from_left_to_up != None:         
                    self.turn_movement_dict[turn_from_left_to_up.turn_id] = turn_from_left_to_up.dict                
        
                # 3. from left to right : 0 - 5
                turn_from_left_to_right = Waymo_scenario_topology_turn_movement()
                turn_from_left_to_right.turn_id = 2
                turn_from_left_to_right.turn_type = "straight"
                turn_from_left_to_right.from_link_id = 0
                turn_from_left_to_right.to_link_id = 5
                turn_from_left_to_right.from_link = link_left_inbound
                turn_from_left_to_right.to_link = link_right_outbound
        
                turn_from_left_to_right.dict["turn_movement_id"] = 2
                turn_from_left_to_right.dict["type"] = "THROUGH"
                turn_from_left_to_right.dict["inbound_link_id"]  = 0
                turn_from_left_to_right.dict["outbound_link_id"] = 5
                turn_from_left_to_right.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_left_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_straight:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point) and is_in_range_for_straight(start_point, end_point, center_point):                    
                                exist_lane.turn_movement_id = 2
                                exist_lane.node_id = -1
                                turn_from_left_to_right.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                         
                                #print(" --- left to right lanes: ", exit_lane_id)
                turn_from_left_to_right.dict["number_lane"] = len(turn_from_left_to_right.lane_list)
                turn_from_left_to_right.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_left_to_right)  
                if turn_from_left_to_right != None:        
                    self.turn_movement_dict[turn_from_left_to_right.turn_id] = turn_from_left_to_right.dict
        
                # check if there is u-turn
                turn_from_left_to_left = None
                for center_lane in link_left_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_u_turn:                   
                            turn_from_left_to_left = Waymo_scenario_topology_turn_movement()
                            turn_from_left_to_left.turn_id = 10
                            turn_from_left_to_left.turn_type = "u_turn"
                            turn_from_left_to_left.from_link_id = 0
                            turn_from_left_to_left.to_link_id = 1
                            turn_from_left_to_left.from_link = link_left_inbound
                            turn_from_left_to_left.to_link = link_left_outbound
        
                            turn_from_left_to_left.dict["turn_movement_id"] = 10
                            turn_from_left_to_left.dict["type"] = "U_TURN"
                            turn_from_left_to_left.dict["inbound_link_id"]  = 0
                            turn_from_left_to_left.dict["outbound_link_id"] = 1
                            turn_from_left_to_left.dict["node_id"] = 0
                            
                            turn_lane_start_point = None
                            turn_lane_middle_point = None
                            turn_lane_end_point = None
                            
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):     
                                exist_lane.turn_movement_id = 10
                                exist_lane.node_id = -1                        
                                turn_from_left_to_left.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- left to left lanes: ", exit_lane_id)        
        
                            turn_from_left_to_left.dict["number_lane"] = len(turn_from_left_to_left.lane_list)
                            turn_from_left_to_left.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
        
                self.turn_movement_list.append(turn_from_left_to_left)
                if turn_from_left_to_left != None:
                    self.turn_movement_dict[turn_from_left_to_left.turn_id] = turn_from_left_to_left.dict
        
                # ======================= calculate turn movements : down =========================== 
                            
                # 4. from down to left: 2 - 1
                turn_from_down_to_left = Waymo_scenario_topology_turn_movement()
                turn_from_down_to_left.turn_id = 3
                turn_from_down_to_left.turn_type = "left"
                turn_from_down_to_left.from_link_id = 2
                turn_from_down_to_left.to_link_id = 1
                turn_from_down_to_left.from_link = link_down_inbound
                turn_from_down_to_left.to_link = link_left_outbound
        
                turn_from_down_to_left.dict["turn_movement_id"] = 3
                turn_from_down_to_left.dict["type"] = "LEFT_TURN"
                turn_from_down_to_left.dict["inbound_link_id"]  = 2
                turn_from_down_to_left.dict["outbound_link_id"] = 1
                turn_from_down_to_left.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_down_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_left_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):  
                                exist_lane.turn_movement_id = 3
                                exist_lane.node_id = -1                       
                                turn_from_down_to_left.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- down to left lanes: ", exit_lane_id)
                turn_from_down_to_left.dict["number_lane"] = len(turn_from_down_to_left.lane_list)
                turn_from_down_to_left.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_down_to_left) 
                if turn_from_down_to_left != None:         
                    self.turn_movement_dict[turn_from_down_to_left.turn_id] = turn_from_down_to_left.dict         
                            
                # 5. from down to right : 2 - 5
                turn_from_down_to_right = Waymo_scenario_topology_turn_movement()
                turn_from_down_to_right.turn_id = 4
                turn_from_down_to_right.turn_type = "right"
                turn_from_down_to_right.from_link_id = 2
                turn_from_down_to_right.to_link_id = 5
                turn_from_down_to_right.from_link = link_down_inbound
                turn_from_down_to_right.to_link = link_right_outbound
        
                turn_from_down_to_right.dict["turn_movement_id"] = 4
                turn_from_down_to_right.dict["type"] = "RIGHT_TURN"
                turn_from_down_to_right.dict["inbound_link_id"]  = 2
                turn_from_down_to_right.dict["outbound_link_id"] = 5
                turn_from_down_to_right.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_down_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_right_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point): 
                                exist_lane.turn_movement_id = 4
                                exist_lane.node_id = -1                         
                                turn_from_down_to_right.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- down to right lanes: ", exit_lane_id)
                turn_from_down_to_right.dict["number_lane"] = len(turn_from_down_to_right.lane_list)
                turn_from_down_to_right.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_down_to_right) 
                if turn_from_down_to_left != None:                      
                    self.turn_movement_dict[turn_from_down_to_left.turn_id] = turn_from_down_to_right.dict 
        
                # 6. from down to up : 2 - 7
                turn_from_down_to_up = Waymo_scenario_topology_turn_movement()
                turn_from_down_to_up.turn_id = 5
                turn_from_down_to_up.turn_type = "straight"
                turn_from_down_to_up.from_link_id = 2
                turn_from_down_to_up.to_link_id = 7
                turn_from_down_to_up.from_link = link_down_inbound
                turn_from_down_to_up.to_link = link_up_outbound
        
                turn_from_down_to_up.dict["turn_movement_id"] = 5
                turn_from_down_to_up.dict["type"] = "THROUGH"
                turn_from_down_to_up.dict["inbound_link_id"]  = 2
                turn_from_down_to_up.dict["outbound_link_id"] = 7
                turn_from_down_to_up.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_down_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_straight:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point) and is_in_range_for_straight(start_point, end_point, center_point):                  
                                exist_lane.turn_movement_id = 5
                                exist_lane.node_id = -1                         
                                turn_from_down_to_up.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- down to up lanes: ", exit_lane_id)
                turn_from_down_to_up.dict["number_lane"] = len(turn_from_down_to_up.lane_list)
                turn_from_down_to_up.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_down_to_up)
                if turn_from_down_to_up != None:        
                    self.turn_movement_dict[turn_from_down_to_up.turn_id] = turn_from_down_to_up.dict
        
                # check if there is u-turn
                turn_from_down_to_down = None
                for center_lane in link_down_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_u_turn:
                            
                            turn_from_down_to_down = Waymo_scenario_topology_turn_movement()
                            turn_from_down_to_down.turn_id = 11
                            turn_from_down_to_down.turn_type = "U_TURN"
                            turn_from_down_to_down.from_link_id = 2
                            turn_from_down_to_down.to_link_id = 3
                            turn_from_down_to_down.from_link = link_down_inbound
                            turn_from_down_to_down.to_link = link_down_outbound
        
                            turn_from_down_to_down.dict["turn_movement_id"] = 11
                            turn_from_down_to_down.dict["type"] = "u_turn"
                            turn_from_down_to_down.dict["inbound_link_id"]  = 2
                            turn_from_down_to_down.dict["outbound_link_id"] = 3
                            turn_from_down_to_down.dict["node_id"] = 0
                            
                            turn_lane_start_point = None
                            turn_lane_middle_point = None
                            turn_lane_end_point = None
                            
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                    
                                exist_lane.turn_movement_id = 11
                                exist_lane.node_id = -1 
                                turn_from_down_to_down.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- down to down lanes: ", exit_lane_id)        
        
                            turn_from_down_to_down.dict["number_lane"] = len(turn_from_down_to_down.lane_list)
                            turn_from_down_to_down.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                
                self.turn_movement_list.append(turn_from_down_to_down)
                if turn_from_down_to_down != None:         
                    self.turn_movement_dict[turn_from_down_to_down.turn_id] = turn_from_down_to_down.dict 
        
                # ======================= calculate turn movements : right =========================== 
                
                # 7. from right to down : 4 - 3
                turn_from_right_to_down = Waymo_scenario_topology_turn_movement()
                turn_from_right_to_down.turn_id = 6
                turn_from_right_to_down.turn_type = "left"
                turn_from_right_to_down.from_link_id = 4
                turn_from_right_to_down.to_link_id = 3
                turn_from_right_to_down.from_link = link_right_inbound
                turn_from_right_to_down.to_link = link_down_outbound
        
                turn_from_right_to_down.dict["turn_movement_id"] = 6
                turn_from_right_to_down.dict["type"] = "LEFT_TURN"
                turn_from_right_to_down.dict["inbound_link_id"]  = 4
                turn_from_right_to_down.dict["outbound_link_id"] = 3
                turn_from_right_to_down.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_right_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_left_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                       
                                exist_lane.turn_movement_id = 6
                                exist_lane.node_id = -1 
                                turn_from_right_to_down.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- right to down lanes: ", exit_lane_id)
                turn_from_right_to_down.dict["number_lane"] = len(turn_from_right_to_down.lane_list)
                turn_from_right_to_down.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_right_to_down)
                if turn_from_right_to_down != None: 
                    self.turn_movement_dict[turn_from_right_to_down.turn_id] = turn_from_right_to_down.dict 
        
                # 8. from right to up : 4 - 7
                turn_from_right_to_up = Waymo_scenario_topology_turn_movement()
                turn_from_right_to_up.turn_id = 7
                turn_from_right_to_up.turn_type = "right"
                turn_from_right_to_up.from_link_id = 4
                turn_from_right_to_up.to_link_id = 7
                turn_from_right_to_up.from_link = link_right_inbound
                turn_from_right_to_up.to_link = link_up_outbound
        
                turn_from_right_to_up.dict["turn_movement_id"] = 7
                turn_from_right_to_up.dict["type"] = "RIGHT_TURN"
                turn_from_right_to_up.dict["inbound_link_id"]  = 4
                turn_from_right_to_up.dict["outbound_link_id"] = 7
                turn_from_right_to_up.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_right_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_right_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                       
                                exist_lane.turn_movement_id = 7
                                exist_lane.node_id = -1 
                                turn_from_right_to_up.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                             
                                #print(" --- right to up lanes: ", exit_lane_id)
                turn_from_right_to_up.dict["number_lane"] = len(turn_from_right_to_up.lane_list)
                turn_from_right_to_up.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_right_to_up)
                self.turn_movement_dict[turn_from_right_to_up.turn_id] = turn_from_right_to_up.dict       
        
                # 9. from right to left : 4 - 1
                turn_from_right_to_left = Waymo_scenario_topology_turn_movement()
                turn_from_right_to_left.turn_id = 8
                turn_from_right_to_left.turn_type = "straight"
                turn_from_right_to_left.from_link_id = 4
                turn_from_right_to_left.to_link_id = 1
                turn_from_right_to_left.from_link = link_right_inbound
                turn_from_right_to_left.to_link = link_left_outbound
        
                turn_from_right_to_left.dict["turn_movement_id"] = 8
                turn_from_right_to_left.dict["type"] = "THROUGH"
                turn_from_right_to_left.dict["inbound_link_id"]  = 4
                turn_from_right_to_left.dict["outbound_link_id"] = 1
                turn_from_right_to_left.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_right_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_straight:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point) and is_in_range_for_straight(start_point, end_point, center_point):               
                                exist_lane.turn_movement_id = 8
                                exist_lane.node_id = -1 
                                turn_from_right_to_left.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                          
                                #print(" --- right to left lanes: ", exit_lane_id)
                turn_from_right_to_left.dict["number_lane"] = len(turn_from_right_to_left.lane_list)
                turn_from_right_to_left.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_right_to_left)
                self.turn_movement_dict[turn_from_right_to_left.turn_id] = turn_from_right_to_left.dict   
        
                # check if there is u-turn
                turn_from_right_to_right = None
                for center_lane in link_right_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_u_turn:
                            
                            turn_from_right_to_right = Waymo_scenario_topology_turn_movement()
                            turn_from_right_to_right.turn_id = 12
                            turn_from_right_to_right.turn_type = "u_turn"
                            turn_from_right_to_right.from_link_id = 4
                            turn_from_right_to_right.to_link_id = 5
                            turn_from_right_to_right.from_link = link_right_inbound
                            turn_from_right_to_right.to_link = link_right_outbound
        
                            turn_from_right_to_right.dict["turn_movement_id"] = 12
                            turn_from_right_to_right.dict["type"] = "U_TURN"
                            turn_from_right_to_right.dict["inbound_link_id"]  = 4
                            turn_from_right_to_right.dict["outbound_link_id"] = 5
                            turn_from_right_to_right.dict["node_id"] = 0
                            
                            turn_lane_start_point = None
                            turn_lane_middle_point = None
                            turn_lane_end_point = None
                            
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                    
                                exist_lane.turn_movement_id = 12
                                exist_lane.node_id = -1 
                                turn_from_right_to_right.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                           
                                #print(" --- right to right lanes: ", exit_lane_id)        
                            turn_from_right_to_right.dict["number_lane"] = len(turn_from_right_to_right.lane_list)
                            turn_from_right_to_right.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
        
                self.turn_movement_list.append(turn_from_right_to_right)
                if turn_from_right_to_right != None:
                    self.turn_movement_dict[turn_from_right_to_right.turn_id] = turn_from_right_to_right.dict           
        
                # ======================= calculate turn movements : up =========================== 
                            
                # 10. from up to left : 6 - 5
                turn_from_up_to_left = Waymo_scenario_topology_turn_movement()
                turn_from_up_to_left.turn_id = 9
                turn_from_up_to_left.turn_type = "left"
                turn_from_up_to_left.from_link_id = 6
                turn_from_up_to_left.to_link_id = 5
                turn_from_up_to_left.from_link = link_up_inbound
                turn_from_up_to_left.to_link = link_right_outbound
        
                turn_from_up_to_left.dict["turn_movement_id"] = 9
                turn_from_up_to_left.dict["type"] = "LEFT_TURN"
                turn_from_up_to_left.dict["inbound_link_id"]  = 6
                turn_from_up_to_left.dict["outbound_link_id"] = 5
                turn_from_up_to_left.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None
        
                for center_lane in link_up_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_left_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                       
                                exist_lane.turn_movement_id = 9
                                exist_lane.node_id = -1 
                                turn_from_up_to_left.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                           
                                #print(" --- up to left lanes: ", exit_lane_id)
                turn_from_up_to_left.dict["number_lane"] = len(turn_from_up_to_left.lane_list)
                turn_from_up_to_left.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_up_to_left)
                self.turn_movement_dict[turn_from_up_to_left.turn_id] = turn_from_up_to_left.dict      
        
                # 11. from up to right : 6 - 1
                turn_from_up_to_right = Waymo_scenario_topology_turn_movement()
                turn_from_up_to_right.turn_id = 10
                turn_from_up_to_right.turn_type = "right"
                turn_from_up_to_right.from_link_id = 6
                turn_from_up_to_right.to_link_id = 1
                turn_from_up_to_right.from_link = link_up_inbound
                turn_from_up_to_right.to_link = link_left_outbound
        
                turn_from_up_to_right.dict["turn_movement_id"] = 10
                turn_from_up_to_right.dict["type"] = "RIGHT_TURN"
                turn_from_up_to_right.dict["inbound_link_id"]  = 6
                turn_from_up_to_right.dict["outbound_link_id"] = 1
                turn_from_up_to_right.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None        
                
                for center_lane in link_up_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_right_turn:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                       
                                exist_lane.turn_movement_id = 10
                                exist_lane.node_id = -1 
                                turn_from_up_to_right.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]  
                                #print(" --- up to right lanes: ", exit_lane_id)
                turn_from_up_to_right.dict["number_lane"] = len(turn_from_up_to_right.lane_list)
                turn_from_up_to_right.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
                self.turn_movement_list.append(turn_from_up_to_right)
                self.turn_movement_dict[turn_from_up_to_right.turn_id] = turn_from_up_to_right.dict
        
                # 12. from up to down : 6 - 3
                turn_from_up_to_down = Waymo_scenario_topology_turn_movement()
                turn_from_up_to_down.turn_id = 11
                turn_from_up_to_down.turn_type = "straight"
                turn_from_up_to_down.from_link_id = 6
                turn_from_up_to_down.to_link_id = 3
                turn_from_up_to_down.from_link = link_up_inbound
                turn_from_up_to_down.to_link = link_down_outbound
        
                turn_from_up_to_down.dict["turn_movement_id"] = 11
                turn_from_up_to_down.dict["type"] = "THROUGH"
                turn_from_up_to_down.dict["inbound_link_id"]  = 6
                turn_from_up_to_down.dict["outbound_link_id"] = 3
                turn_from_up_to_down.dict["node_id"] = 0
                
                turn_lane_start_point = None
                turn_lane_middle_point = None
                turn_lane_end_point = None      
        
                for center_lane in link_up_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    #print(" --- center lane id is : ", center_lane_id)
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        #print(" ---- up to down lanes: ", exit_lane_id)
                        if exist_lane.is_straight:
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)                
                            if is_in_range(start_point, end_point, center_point) and is_in_range_for_straight(start_point, end_point, center_point):
                                exist_lane.turn_movement_id = 11
                                exist_lane.node_id = -1 
                                point_number = len(exist_lane.points_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1] 
                                turn_from_up_to_down.lane_list.append(exist_lane)                       
                turn_from_up_to_down.dict["number_lane"] = len(turn_from_up_to_down.lane_list)
                turn_from_up_to_down.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
        
                self.turn_movement_list.append(turn_from_up_to_down) 
                self.turn_movement_dict[turn_from_up_to_down.turn_id] = turn_from_up_to_down.dict        
        
                # check if there is u-turn
                turn_from_up_to_up = None
                for center_lane in link_up_inbound.lane_list:
                    center_lane_id = center_lane.lane_id
                    for exit_lane_id in center_lane.exit_lane_id_list:
                        exist_lane = self.center_lane_dict[exit_lane_id]
                        if exist_lane.is_u_turn:
                            
                            turn_from_up_to_up = Waymo_scenario_topology_turn_movement()
                            turn_from_up_to_up.turn_id = 13
                            turn_from_up_to_up.turn_type = "u_turn"
                            turn_from_up_to_up.from_link_id = 6
                            turn_from_up_to_up.to_link_id = 7
                            turn_from_up_to_up.from_link = link_right_inbound
                            turn_from_up_to_up.to_link = link_right_outbound
        
                            turn_from_up_to_up.dict["turn_movement_id"] = 13
                            turn_from_up_to_up.dict["type"] = "U_TURN"
                            turn_from_up_to_up.dict["inbound_link_id"]  = 6
                            turn_from_up_to_up.dict["outbound_link_id"] = 7
                            turn_from_up_to_up.dict["node_id"] = 0
                            
                            turn_lane_start_point = None
                            turn_lane_middle_point = None
                            turn_lane_end_point = None  
                            
                            start_point = exist_lane.points_list[0]
                            end_point = exist_lane.points_list[-1]
                            center_point = (self.node_x, self.node_y)
                            if is_in_range(start_point, end_point, center_point):                    
                                exist_lane.turn_movement_id = 13
                                exist_lane.node_id = -1 
                                turn_from_up_to_up.lane_list.append(exist_lane)
                                point_number = len(exist_lane.points_xyz_list)
                                turn_lane_start_point = exist_lane.points_xyz_list[0]
                                turn_lane_middle_point = exist_lane.points_xyz_list[int(0.5 * point_number)]
                                turn_lane_end_point = exist_lane.points_xyz_list[-1]                        
                                #print(" --- up to up lanes: ", exit_lane_id)                      
                            turn_from_up_to_up.dict["number_lane"] = len(turn_from_up_to_up.lane_list)
                            turn_from_up_to_up.dict["shape_point"] = [turn_lane_start_point, turn_lane_middle_point, turn_lane_end_point]           
        
                self.turn_movement_list.append(turn_from_up_to_up)  
                if turn_from_up_to_up != None:        
                    self.turn_movement_dict[turn_from_up_to_up.turn_id] = turn_from_up_to_up.dict         
        
                # ================== add virtual to object ==================
                self.node_left.append(self.virtual_node_left_x)
                self.node_left.append(self.virtual_node_left_y)
        
                self.node_right.append(self.virtual_node_right_x)
                self.node_right.append(self.virtual_node_right_y)        
                
                self.node_up.append(self.virtual_node_up_x)
                self.node_up.append(self.virtual_node_up_y)
        
                self.node_down.append(self.virtual_node_down_x)
                self.node_down.append(self.virtual_node_down_y)     
                
                self.node_list.append(self.node_left)
                self.node_list.append(self.node_right)
                self.node_list.append(self.node_up)
                self.node_list.append(self.node_down)       
                
                for center_lane in self.center_lane_list:
                    center_lane_id = center_lane.lane_id
                    center_lane_link_id = center_lane.link_id
                    center_lane_turn_id = center_lane.turn_movement_id
                    center_lane_node_id = center_lane.node_id
                    center_lane_width = center_lane.lane_width
                    center_lane.dict["lane_width"] = center_lane_width
                    center_lane.dict["lane_id"] = center_lane_id
                    center_lane.dict["link_id"] = center_lane_link_id           
                    center_lane.dict["turn_movement_id"] = center_lane_turn_id            
                    center_lane.dict["node_id"] = center_lane_node_id
                    center_lane.dict["left_lane_set"] = center_lane.left_neighbors_id_list
                    center_lane.dict["right_lane_set"] = center_lane.right_neighbors_id_list
                    center_lane.dict["inbound_lane_set"] = center_lane.entry_lines_list
                    center_lane.dict["outbound_lane_set"] = center_lane.exit_lanes_list
                    #center_lane.dict["shape_points"] = center_lane.points_list
                    center_lane.dict["shape_points"] = center_lane.points_xyz_list
                    center_lane.dict["start_lane_connection_point_id"] = center_lane.lane_connection_point_id_a
                    center_lane.dict["end_lane_connection_point_id"] = center_lane.lane_connection_point_id_b
                    #center_lane.dict["lane_direction"] = center_lane.lane_direction
                    
                    center_lane.dict["curvature"] = center_lane.lane_curvature_list
                    center_lane.dict["average_curvature"] = center_lane.lane_average_curvature
                    center_lane.dict["grade"] = center_lane.lane_grade_list
                    try:
                        lane_average_grade = calculate_list_average(center_lane.lane_grade_list)
                        center_lane.dict["average_grade"] = lane_average_grade
                    except:
                        center_lane.dict["average_grade"] = "None"
                    
                    self.lane_dict[center_lane_id] = center_lane.dict
            except:            
                print("   ====== link node error, will retry ", retry_times)
                retry_times += 1
                if retry_times > 2:
                    print(self.scenario_index)
                    break
            else:
                break
                             
            
    def calculate_nodes_and_links(self):
        reference_lane_list = []
        intersection_point_x_list = []
        intersection_point_y_list = []
        for center_lane in self.center_lane_list:
            center_lane_id = center_lane.lane_id
            center_lane_type = center_lane.lane_type
            center_lane_slope = center_lane.slope
            if center_lane.is_straight:
                if center_lane_type == MAP_feature.TYPE_FREEWAY or center_lane_type == MAP_feature.TYPE_SURFACE_STREET:
                    reference_lane_list.append(center_lane)
                    self.reference_center_line_list.append(center_lane)
        
        used_center_lane_id_list = []
        for center_lane in reference_lane_list:
            center_lane_id = center_lane.lane_id
            lane_equation = center_lane.equation_list
            for other_center_lane in reference_lane_list:
                other_center_lane_id = other_center_lane.lane_id
                if other_center_lane_id == center_lane_id:
                    continue
                if other_center_lane_id in used_center_lane_id_list:
                    continue
                other_center_lane_slope = other_center_lane.slope
                angle_between_two_lanes = math.degrees(math.atan((other_center_lane_slope - center_lane_slope)/(1+(other_center_lane_slope * center_lane_slope))))
                angle_between_two_lanes = abs(angle_between_two_lanes)
                
                if angle_between_two_lanes < 50 or angle_between_two_lanes > 130:
                    continue
                
                #print("center x is : ", self.scenario_center_x)
                #print("center y is : ", self.scenario_center_y)
                #print("scenario width is : ", self.scenario_width)
                
                # calculate intersection point
                other_center_lane_equation = other_center_lane.equation_list
                L2 = other_center_lane_equation
                L1 = lane_equation
                #D  = L1[0] * L2[1] - L1[1] * L2[0]
                #Dx = L1[2] * L2[1] - L1[1] * L2[2]
                #Dy = L1[0] * L2[2] - L1[2] * L2[0]
                D  = L1[0] * L2[1] - L1[1] * L2[0]
                Dx = - L1[2] * L2[1] + L1[1] * L2[2]
                Dy = - L1[0] * L2[2] + L1[2] * L2[0]                
                if D != 0:
                    x = Dx / D
                    y = Dy / D
                    #print("x is : ", x)
                    #print("y is : ", y)
                    if x > self.scenario_center_x - 0.5 * self.scenario_width and x < self.scenario_center_x + 0.5 * self.scenario_width:
                        if y > self.scenario_center_y - 0.5 * self.scenario_width and y < self.scenario_center_y + 0.5 * self.scenario_width:
                            intersection_point_x_list.append(x)
                            intersection_point_y_list.append(y)
            used_center_lane_id_list.append(other_center_lane_id)
        
        point_index = 0
        point_list  = []
        first_intersection_point_x_list  = []
        first_intersection_point_y_list  = []
        second_intersection_point_x_list = []
        second_intersection_point_y_list = []        
        number_of_points = len(intersection_point_x_list)
        while point_index < number_of_points:
            x = intersection_point_x_list[point_index]
            y = intersection_point_y_list[point_index]
            point = [x, y]
            point_list.append(point)
            point_index += 1
            
        points_array = np.array([[point[0], point[1]] for point in point_list])  

        model = KMeans(n_clusters = 3)
        # Fit model to training data
        model.fit(points_array)

        # Get predicted cluster centers
        model.cluster_centers_ # shape: (n_cluster, n_features)             
        
        self.cluster_centers = model.cluster_centers_
        #print(" --- labels: ", model.labels_)
        
        label_index = 0
        for label in model.labels_:
            x = intersection_point_x_list[label_index]
            y = intersection_point_y_list[label_index]
            if label == 0:
                first_intersection_point_x_list.append(x)
                first_intersection_point_y_list.append(y)
            if label == 1:
                second_intersection_point_x_list.append(x)
                second_intersection_point_y_list.append(y)
                
        if len(first_intersection_point_x_list) > len(second_intersection_point_x_list):
            intersection_point_x_list = first_intersection_point_x_list
            intersection_point_y_list = first_intersection_point_y_list
            self.node_x = self.cluster_centers[0, 0]
            self.node_y = self.cluster_centers[0, 1]
        else:
            intersection_point_x_list = second_intersection_point_x_list
            intersection_point_y_list = second_intersection_point_y_list            
            self.node_x = self.cluster_centers[1, 0]
            self.node_y = self.cluster_centers[1, 1]
            
        #self.node_x = sum(intersection_point_x_list) / len(intersection_point_x_list)
        #self.node_y = sum(intersection_point_y_list) / len(intersection_point_y_list)
        
        link_reference_lane_list = []
        lane_end_points_near_crossing_list = []
        lane_id_distance_to_node_dict = {}
        for center_lane in reference_lane_list:
            center_lane_id = center_lane.lane_id
            lane_equation = center_lane.equation_list
            
            center_lane_first_x = center_lane.point_x_list[0]
            center_lane_first_y = center_lane.point_y_list[0]
            center_lane_last_x  = center_lane.point_x_list[-1]
            center_lane_last_y  = center_lane.point_y_list[-1]
            
            distance_to_node = calculate_distance_from_line(lane_equation, self.node_x, self.node_y)
            lane_id_distance_to_node_dict[center_lane_id] = distance_to_node
            
        sorted_dict = {}
        sorted_keys = sorted(lane_id_distance_to_node_dict, key = lane_id_distance_to_node_dict.get)
        
        for key in sorted_keys:
            sorted_dict[key] = lane_id_distance_to_node_dict[key]
        
        first_lane_id = None
        second_lane_id = None
        first_lane_id = next(iter(sorted_dict))
        first_lane = self.center_lane_dict[first_lane_id]
        first_lane_slope = first_lane.slope 
        first_lane_equation_list = first_lane.equation_list
        a1 = first_lane_equation_list[0]
        b1 = first_lane_equation_list[1]
        c1 = first_lane_equation_list[2]
        for lane_id, distance_to_node in sorted_dict.items():
            if lane_id == first_lane_id:
                continue
            second_lane = self.center_lane_dict[lane_id]
            second_lane_slope = second_lane.slope 
            angle_between_two_lanes = math.degrees(math.atan((second_lane_slope - first_lane_slope)/(1 + (second_lane_slope * first_lane_slope))))
            angle_between_two_lanes = abs(angle_between_two_lanes)
            
            if angle_between_two_lanes > 45 and angle_between_two_lanes < 135:
                second_lane_id = lane_id
                break
            
        second_lane = self.center_lane_dict[second_lane_id]
        second_lane_equation_list = second_lane.equation_list
        a2 = second_lane_equation_list[0]
        b2 = second_lane_equation_list[1]
        c2 = second_lane_equation_list[2]  
        
        self.virtual_node_left_x = self.scenario_min_x
        self.virtual_node_right_x = self.scenario_max_x
        self.virtual_node_up_y = self.scenario_max_y
        self.virtual_node_down_y = self.scenario_min_y       
        
        if b1 == 0:
            b1 = 0.01
        if b2 == 0:
            b2 = 0.01            
            
        y1_left = (-a1 / b1) * self.virtual_node_left_x + (-c1 / b1)
        y1_right = (-a1 / b1) * self.virtual_node_right_x + (-c1 / b1)
        
        y2_left = (-a2 / b2) * self.virtual_node_left_x + (-c2 / b2)
        y2_right = (-a2 / b2) * self.virtual_node_right_x + (-c2 / b2)        
        
        if y1_left > self.scenario_min_y and y1_left < self.scenario_max_y:
            self.virtual_node_left_y = y1_left
        if y1_right > self.scenario_min_y and y1_right < self.scenario_max_y:
            self.virtual_node_right_y = y1_right          

        if y2_left > self.scenario_min_y and y2_left < self.scenario_max_y:
            self.virtual_node_left_y = y2_left
        if y2_right > self.scenario_min_y and y2_right < self.scenario_max_y:
            self.virtual_node_right_y = y2_right  

        if a1 == 0:
            a1 = 0.01
        if a2 == 0:
            a2 = 0.01 
            
        x1_up = (-b1 / a1) * self.virtual_node_up_y + (-c1 / a1)
        x1_down = (-b1 / a1) * self.virtual_node_down_y + (-c1 / a1)
        
        x2_up = (-b2 / a2) * self.virtual_node_up_y + (-c2 / a2)
        x2_down = (-b2 / a2) * self.virtual_node_down_y + (-c2 / a2)        
        
        if x1_up > self.scenario_min_x and x1_up < self.scenario_max_x:
            self.virtual_node_up_x = x1_up
        if x1_down > self.scenario_min_x and x1_down < self.scenario_max_x:
            self.virtual_node_down_x = x1_down          

        if x2_up > self.scenario_min_x and x2_up < self.scenario_max_x:
            self.virtual_node_up_x = x2_up
        if x2_down > self.scenario_min_x and x2_down < self.scenario_max_x:
            self.virtual_node_down_x = x2_down 
        
        self.node_left.append(self.virtual_node_left_x)
        self.node_left.append(self.virtual_node_left_y)

        self.node_right.append(self.virtual_node_right_x)
        self.node_right.append(self.virtual_node_right_y)        
        
        self.node_up.append(self.virtual_node_up_x)
        self.node_up.append(self.virtual_node_up_y)

        self.node_down.append(self.virtual_node_down_x)
        self.node_down.append(self.virtual_node_down_y)     
        
        self.node_list.append(self.node_left)
        self.node_list.append(self.node_right)
        self.node_list.append(self.node_up)
        self.node_list.append(self.node_down)
        
            
class Waymo_scenario_topology_network:
    def __init__(self): 
        self.dict = {}
        self.node_list = []
        self.link_list = []
        self.sdc_track_index = None
        self.number_of_map_features = None

class Waymo_scenario_topology_node:
    def __init__(self):
        self.dict = {}
        self.node_id = None
        self.coordinate = None
        self.inbound_link_list = []
        self.outbound_link_list = []
           

class Waymo_scenario_topology_link:   
    def __init__(self):
        self.dict = {}
        self.link_id = None
        self.link_from_node_id = None
        self.link_to_node_id = None
        self.link_from_node = None
        self.link_to_node = None
        self.lane_list = []
        self.lane_dict = {}
        self.link_x = [] # a list from start node x to end node x
        self.link_y = [] # a list from start node y to end node y

class Waymo_scenario_topology_turn_movement:
    def __init__(self): 
        self.dict = {}
        self.turn_id = None
        self.turn_type = None
        self.from_link_id = None
        self.to_link_id = None
        self.from_link = None
        self.to_link = None
        self.lane_list = []


class Motion_training_agent:
    def __init__(self):
        self.agent_type = None  # unset = 0, vehicle = 1, pedestrian = 2, cyclist = 3, other = 4
        self.agent_id = None
        self.states_array = None       
        self.states_mask_list = [] 
        self.states_list = []
        self.matching_agent_time_list = []
        self.matching_agent_total_frame = None
        self.matching_agent_x_list = []
        self.matching_agent_y_list = []
        self.matching_agent_z_list = []
        self.matching_agent_length_list = []
        self.matching_agent_width_list = []
        self.matching_agent_height_list = []
        self.length_width_height_list = []
        
        self.average_length = None
        self.average_width  = None
        self.average_height = None
              
        self.matching_agent_speed_x_list = []
        self.matching_agent_speed_y_list = []     
        self.matching_agent_speed_list = []
        self.acceleration_list = []
        
        self.matching_agent_lane_id_list = []
        self.matching_agent_lane_type_list = []
        self.matching_agent_link_id_list = []
        self.matching_agent_node_id_list = []
        self.matching_agent_turn_movement_list = []
        self.trajectory_point_list = []
        self.unique_matching_agent_lane_id_list = []
        
        self.distance_to_lane_start_list = []
        self.distance_to_lane_end_list = []
        self.odometer_list = []
        self.polygon = None
        
        # preceeding vehicle in the same lane
        self.preceeding_vehicle_id_list = []
        self.preceeding_lane_id_list = []
        self.preceeding_vehicle_distance_list = []
        self.preceeding_vehicle_distance_to_ego_lane_end_list = []
        self.preceeding_vehicle_location_list = []
        self.preceeding_vehicle_time_headway_list = []
        self.preceeding_vehicle_gap_list = []    # gap - bumper to bumper distance
        self.preceeding_vehicle_speed_list = []
        self.preceeding_vehicle_acc_list = []
        
        # following vehicle in the same lane
        self.following_vehicle_id_list = []
        self.following_lane_id_list = []
        self.following_vehicle_distance_list = []
        self.following_vehicle_distance_to_ego_lane_end_list = []
        self.following_vehicle_location_list = []
        self.following_vehicle_time_headway_list = []
        self.following_vehicle_gap_list = []
        self.following_vehicle_speed_list = []
        self.following_vehicle_acc_list = []
        
        # left lane preceding vehcile 
        self.left_preceeding_lane_id_list = []      
        self.left_lane_id_list = []
        self.left_lane_spacing_list = []
        self.left_preceeding_vehicle_id_list = []
        self.left_preceeding_vehicle_distance_list = []
        self.left_preceeding_vehicle_distance_to_ego_lane_end_list = []
        self.left_preceeding_vehicle_location_list = []
        self.left_preceeding_vehicle_time_headway_list = []
        self.left_preceeding_vehicle_gap_list = []
        self.left_preceeding_vehicle_speed_list = []
        self.left_preceeding_vehicle_acc_list = []
        
        # left lane following vehicle
        self.left_following_lane_id_list = []
        self.left_following_vehicle_id_list = []
        self.left_following_vehicle_distance_list = []
        self.left_following_vehicle_distance_to_ego_lane_end_list = []
        self.left_following_vehicle_location_list = []
        self.left_following_vehicle_time_headway_list = []
        self.left_following_vehicle_speed_list = []
        self.left_following_vehicle_acc_list = []
        self.left_lane_gap_list = []
        self.left_lane_change_list = []
        self.left_following_vehicle_gap_list = []
        
        # right lane preceeding vehicle
        self.right_preceeding_lane_id_list = []
        self.right_lane_gap_list = []
        self.right_preceeding_vehicle_id_list = []
        self.right_preceeding_vehicle_distance_list = []
        self.right_preceeding_vehicle_distance_to_ego_lane_end_list = []
        self.right_preceeding_vehicle_location_list = []
        self.right_preceeding_vehicle_time_headway_list = []
        self.right_preceeding_vehicle_gap_list = []
        self.right_preceeding_vehicle_speed_list = []
        self.right_preceeding_vehicle_acc_list = []
        
        # right lane following vehicle
        self.right_following_lane_id_list = []
        self.right_following_vehicle_id_list = []
        self.right_following_vehicle_distance_list = []
        self.right_following_vehicle_distance_to_ego_lane_end_list = []
        self.right_following_vehicle_location_list = []
        self.right_following_vehicle_time_headway_list = []
        self.right_following_vehicle_speed_list = []
        self.right_following_vehicle_acc_list = []
        self.right_lane_spacing_list = []
        self.right_lane_change_list = []
        self.right_following_vehicle_gap_list = []

        # left lane cloest vehicle
        self.left_parallel_lane_id_list = []
        self.left_parallel_vehicle_id_list = []
        self.left_parallel_vehicle_relative_location_list = []
        self.left_parallel_vehicle_front_bumper_gap_list = []
        self.left_parallel_vehicle_spacing_list = []
        self.left_parallel_vehicle_time_headway_list = []
        self.left_parallel_vehicle_location_list = []
        self.left_parallel_vehicle_speed_list = []
        self.left_parallel_vehicle_acc_list = []

        # right lane cloest vehicle
        self.right_parallel_lane_id_list = []
        self.right_parallel_vehicle_id_list = []
        self.right_parallel_vehicle_relative_location_list = []
        self.right_parallel_vehicle_front_bumper_gap_list = []
        self.right_parallel_vehicle_spacing_list = []
        self.right_parallel_vehicle_location_list = []
        self.right_parallel_vehicle_time_headway_list = []
        self.right_parallel_vehicle_speed_list = []
        self.right_parallel_vehicle_acc_list = []

        # signal 
        self.current_lane_signal_light_shape_list = []
        self.current_lane_signal_light_type_list = []
        self.current_lane_signal_light_state_list = []
        self.current_lane_signal_light_distance_list = []
        
        # left right lane spacing 
        self.left_lane_spacing_list = []
        self.right_lane_spacing_list = []
        self.left_lane_id_list = []
        self.right_lane_id_list = []
        
        # movement / motion type
        self.movement_type_list = []
        self.curvature_list = []
        self.grade_list = []
        self.merging_lane_list = []
        self.diverging_lane_list = []
        self.crossing_lane_list = []
        
        self.grade_of_lane_list = []
        self.curvature_of_lane_list = []
        
        self.grade_from_origin_to_destination = None
        self.curvature_from_origin_to_destination = None
        
        self.origin_link_id = None
        self.destination_link_id = None
        self.origin_lane_id = None
        self.destination_lane_id = None       
        self.link_path_list = []
        self.lane_path_list = []
        
        self.scenario_id = None
        self.dataset_id = None
        
        self.map_matching_lane_id = None
        self.is_straight = False
        self.is_left_turn = False
        self.is_right_turn = False
        self.is_u_turn = False
        self.total_frame = None
        
        self.distance_to_lane_end_at_beginning = None
        self.distance_to_lane_start_at_ending = None
        self.ROW_list = []
        #states_array = np.empty((0,13), dtype = float)
   
    
   
'''
# map features related classes
class Waymo_center_lane:
    lane_id = None
    lane_type = None    # undefined, freeway, surface_street, bike_line
    speed_limit = None   
    entry_lines_list = []  # A list of IDs for lanes that this lane may be entered from    
    exit_lanes_list  = [] # A list of IDs for lanes that this lane may exit to   
    left_boundary_segment_list = []   
    right_boundary_segment_list = []  
    
    left_boundary_point_x_list = []   
    left_boundary_point_y_list = []   
    right_boundary_point_x_list = []  
    right_boundary_point_y_list = []  
     
    left_neighbors_segment_list = []  
    right_neighbors_segment_list = []     
    points_list = []
    point_x_list = []
    point_y_list = []
    
    left_neighbors_id_list = []
    right_neighbors_id_list = []
    entry_lane_id_list = []
    exit_lane_id_list = []   
'''

# map features related classes
class Center_lane_point:
    def __init__(self): 
        self.dict = {}
        self.point_id = None
        self.point_coordinate = None
        self.point_type = None
        self.inbound_lane_id_set = []
        self.outbound_lane_id_set = []
        self.crossing_lane_id_list = []


# map features related classes
class Waymo_center_lane:
    def __init__(self): 
        self.dict = {}
        self.lane_id = None
        self.lane_type = None    # undefined, freeway, surface_street, bike_line
        self.speed_limit = None
        self.is_straight = False
        self.is_straight_turn = False
        self.is_right_turn = False
        self.is_left_turn = False
        self.is_u_turn = False
        self.is_main_lane = False
        self.lane_angle = None
        self.radius = None
        self.polygon = None
        self.polygon_for_map_matching = None
        self.crossing_lane_id_list = []
        
        self.entry_lines_list = []  # A list of IDs for lanes that this lane may be entered from    
        self.exit_lanes_list  = [] # A list of IDs for lanes that this lane may exit to   
        self.left_boundary_segment_list = []
        self.right_boundary_segment_list = []

        self.left_boundary_point_list = []
        self.right_boundary_point_list = []
        self.left_boundary_point_x_list = []
        self.left_boundary_point_y_list = []
        self.left_boundary_point_z_list = []
        self.right_boundary_point_x_list = []
        self.right_boundary_point_y_list = []
        self.right_boundary_point_z_list = []
         
        self.left_neighbors_segment_list = []
        self.right_neighbors_segment_list = []
        self.points_list = []
        self.points_xyz_list = []
        self.point_x_list = []
        self.point_y_list = []
        self.point_z_list = []
        
        self.left_neighbors_id_list = []
        self.right_neighbors_id_list = []
        self.entry_lane_id_list = []
        self.exit_lane_id_list = []
        self.slope = None
        self.equation_list = []
        self.lane_length = None
        self.link_id = None
        self.turn_movement_id = None
        self.node_id = None
        self.lane_width = None
        self.lane_connection_point_id_a = None
        self.lane_connection_point_id_b = None
        self.lane_direction = None
        self.lane_average_grade = None
        self.lane_average_curvature = None

        self.signal_light_color_list = []
        self.signal_light_color_dict = {}
        
    def calculate_lane_length(self):
        center_lane_first_x = self.point_x_list[0]
        center_lane_first_y = self.point_y_list[0]
        center_lane_last_x  = self.point_x_list[-1]
        center_lane_last_y  = self.point_y_list[-1]
        self.lane_length = calculate_distance_between_two_points(center_lane_first_x, center_lane_first_y, center_lane_last_x, center_lane_last_y) 
        
    def calculate_lane_grade(self):
        center_lane_first_z = self.point_z_list[0]
        center_lane_last_z  = self.point_z_list[-1]
        delta_z = center_lane_last_z - center_lane_first_z
        if self.lane_length != 0:
            self.lane_average_grade = delta_z / self.lane_length
        else:
            self.lane_average_grade = "None"

    def calculate_lane_curvature(self):
        number_of_points = len(self.point_x_list)
        middle_point_index = int(0.5 * number_of_points)
        
        center_lane_first_point  = self.points_list[0]
        center_lane_middle_point = self.points_list[middle_point_index]
        center_lane_last_point   = self.points_list[-1]
        
        self.lane_average_curvature = calculate_lane_curvature(center_lane_first_point, center_lane_middle_point, center_lane_last_point)

    def calculate_and_determine_if_straight_lane_angle(self):
        lane_type = self.lane_type     
        if lane_type == MAP_feature.TYPE_FREEWAY or lane_type == MAP_feature.TYPE_SURFACE_STREET: 
        #if lane_type == MAP_feature.TYPE_FREEWAY :    
        #if lane_type == MAP_feature.TYPE_SURFACE_STREET :  
            self.is_main_lane = True
        else:
            self.is_main_lane = False
            
        number_of_points_in_this_lane = len(self.points_list)
        first_point_index = 0
        last_point_index = number_of_points_in_this_lane - 1
        middle_point_index = int(0.5 * last_point_index)
        
        if number_of_points_in_this_lane < 4:
            first_point_x = self.point_x_list[first_point_index]
            first_point_y = self.point_y_list[first_point_index]
            last_point_x  = self.point_x_list[last_point_index]
            last_point_y  = self.point_y_list[last_point_index]        
            middle_point_x = self.point_x_list[middle_point_index]
            middle_point_y = self.point_y_list[middle_point_index] 
            
            self.is_straight = True
            a = first_point_y - last_point_y
            b = last_point_x - first_point_x
            c = first_point_x * last_point_y - last_point_x * first_point_y
            
            if b == 0:
                self.slope = 9999
            else:
                self.slope = -1 * a / b
            
            if a == 0 and b == 0:
                self.is_straight = False
            else:
                min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                #print("distance is: ", min_distance)
                if min_distance < 2:
                    self.is_straight = True   
                    self.lane_angle = np.rad2deg(np.arctan2(last_point_y - first_point_y, last_point_x - first_point_x))
                    self.equation_list.append(a)
                    self.equation_list.append(b)
                    self.equation_list.append(c)                
            
        else:
            first_point_x = self.point_x_list[first_point_index]
            first_point_y = self.point_y_list[first_point_index]
            second_point_x = self.point_x_list[first_point_index + 1]
            second_point_y = self.point_y_list[first_point_index + 1]  
            
            last_two_point_x  = self.point_x_list[last_point_index - 1]
            last_two_point_y  = self.point_y_list[last_point_index - 1]        
            last_point_x  = self.point_x_list[last_point_index]
            last_point_y  = self.point_y_list[last_point_index]
            
            middle_point_x = self.point_x_list[middle_point_index]
            middle_point_y = self.point_y_list[middle_point_index]
     
            #print("first point is: ", first_point_x, first_point_y)  
            #print("last point is: ", last_point_x, last_point_y)
     
            a = first_point_y - last_point_y
            b = last_point_x - first_point_x
            c = first_point_x * last_point_y - last_point_x * first_point_y
            
            if b == 0:
                self.slope = 9999
            else:
                self.slope = -1 * a / b
            
            if a == 0 and b == 0:
                self.is_straight = False
            else:
                min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                #print("distance is: ", min_distance)
                if min_distance < 2:
                    self.is_straight = True   
                    self.lane_angle = np.rad2deg(np.arctan2(last_point_y - first_point_y, last_point_x - first_point_x))
                    self.equation_list.append(a)
                    self.equation_list.append(b)
                    self.equation_list.append(c)
                     
                else:
                    self.is_straight = False
                    lane_type = self.lane_type     
                    if self.is_main_lane:
                        start_point = [first_point_x, first_point_y]
                        end_point = [last_point_x, last_point_y]
                        middle_point = [middle_point_x, middle_point_y]
                        point_location = where_the_point_is(start_point, end_point, middle_point)
                        center_point, radius = get_circle_from_three_points(start_point, end_point, middle_point)
                        self.radius = radius
                        
                        #slope1 = (second_point_y - first_point_y) / (second_point_x - first_point_x)
                        #slope2 = (last_two_point_y - last_point_y) / (last_two_point_x - last_point_x)
                        
                        angle1 = np.rad2deg(np.arctan2(second_point_y - first_point_y, second_point_x - first_point_x))
                        angle2 = np.rad2deg(np.arctan2(last_two_point_y - last_point_y, last_two_point_x - last_point_x))
                        
                        if abs(abs(angle2) - abs(angle1)) < 20:                                          
                        #if radius < 5:
                            self.is_u_turn = True
                            #print(" --- u turn is : ", self.lane_id)
                        else:
                            if point_location == "left":
                                self.is_right_turn = True
                            if point_location == "right":
                                self.is_left_turn = True                        
          
# for lane boundary
class Waymo_boundary_segments:
    def __init__(self):
        self.lane_start_index = 0
        self.lane_end_index = 0
        self.boundary_feature_id = 0       
        self.boundary_type = 0
        self.is_straight = False
        self.slope = None
        self.lane_angle = None
        self.equation_list = []
        self.radius = None
        self.is_left_turn = None
        self.is_right_turn = None
        self.is_u_turn = None

    
# for lane neighbor lane
class neighbor_lane:
    def __init__(self):    
        self.lane_id = None
        self.start_index = None    # undefined, freeway, surface_street, bike_line
        self.end_index = None   
        self.neighbor_start_index = None
        self.neighbor_end_index = None

    

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
    def __init__(self):  
        self.line_id = None
        self.line_type = None    # undefined, freeway, surface_street, bike_line  
        self.points_list = []
        
        # white line x lists
        self.broken_single_white_x_list = []
        self.solid_single_white_x_list  = []
        self.solid_double_white_x_list  = []
    
        # white line y lists
        self.broken_single_white_y_list = []
        self.solid_single_white_y_list  = []
        self.solid_double_white_y_list  = []

        # white line z lists
        self.broken_single_white_z_list = []
        self.solid_single_white_z_list  = []
        self.solid_double_white_z_list  = []
    
        # yellow line x lists    
        self.broken_single_yellow_x_list = []
        self.broken_double_yellow_x_list = []
        self.solid_single_yellow_x_list  = []
        self.solid_double_yellow_x_list  = []  
        self.passing_double_yellow_x_list = []     
    
        # yellow line y lists    
        self.broken_single_yellow_y_list = []
        self.broken_double_yellow_y_list = []
        self.solid_single_yellow_y_list  = []
        self.solid_double_yellow_y_list  = []  
        self.passing_double_yellow_y_list = []    

        # yellow line y lists    
        self.broken_single_yellow_z_list = []
        self.broken_double_yellow_z_list = []
        self.solid_single_yellow_z_list  = []
        self.solid_double_yellow_z_list  = []  
        self.passing_double_yellow_z_list = []           

        # unknown x and y list
        self.unknown_x_list = []
        self.unknown_y_list = [] 
        self.unknown_z_list = [] 
        
        self.point_x_list = []
        self.point_y_list = []
        self.point_z_list = []


class Road_edge:
    def __init__(self):    
        self.line_id = None
        self.line_type = None    # undefined, freeway, surface_street, bike_line  
        self.points_list = []
        self.point_x_list = []
        self.point_y_list = []
        self.point_z_list = []   

class Map_feature:
    def __init__(self):    
        self.feature_id = None
        self.feature_type = None    # undefined, freeway, surface_street, bike_line  
        self.points_list = []
        self.points_xyz_list = []
        self.point_x_list = []
        self.point_y_list = [] 
        self.point_z_list = [] 
        self.is_straight = False
        self.slope = None
        self.lane_angle = None
        self.equation_list = []
        self.radius = None
        self.is_left_turn = None
        self.is_right_turn = None
        self.is_u_turn = None 

    
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

class Waymo_dynamic_map_single_second_state:
    def __init__(self):
        self.time_step = None
        self.lane_id_list = []
        self.lane_signal_state_dict = {}
        
class Waymo_data_analysis_statistics:
    def __init__(self):
        self.number_of_dataset = 0
        self.dataset_id_list = []
        self.number_of_scenario = 0
        self.scenario_id_list = []
        self.number_of_agents = 0
        self.agents_id_list = []
        self.number_of_total_frames = 0
        self.number_of_through_vehicle = 0
        self.through_vehicle_id_list = []
        self.number_of_right_turn_vehicle = 0
        self.right_turn_vehicle_id_list = []
        self.number_of_left_turn_vehicle = 0
        self.left_turn_vehicle_id_list = []
        self.number_of_u_turn_vehicle = 0
        self.u_turn_vehicle_id_list = []
        self.number_of_lane_change_vehicle = 0
        self.lane_change_vehicle_id_list = []
        self.frame_number_list = []
        self.statistic_dict = {}
        
    def summary_statistic_data(self):
        self.statistic_dict["number_of_dataset"] = self.number_of_dataset
        self.statistic_dict["id_of_dataset"] = self.dataset_id_list
        self.statistic_dict["number_of_scenario"] = self.number_of_scenario
        self.statistic_dict["id_of_scenario"] = self.scenario_id_list
        self.statistic_dict["number_of_agents"] = self.number_of_agents
             
        self.statistic_dict["number_of_total_frames"] = self.number_of_total_frames
        self.statistic_dict["number_of_through_vehicle"] = self.number_of_through_vehicle              
        self.statistic_dict["number_of_left_turn_vehicle"] = self.number_of_left_turn_vehicle             
        self.statistic_dict["number_of_right_turn_vehicle"] = self.number_of_right_turn_vehicle           
        self.statistic_dict["number_of_u_turn_vehicle"] = self.number_of_u_turn_vehicle                
        self.statistic_dict["number_of_lane_change_vehicle"] = self.number_of_lane_change_vehicle             
        self.statistic_dict["frame_number_list"] = self.frame_number_list  
        
        self.statistic_dict["id_of_agents"] = self.agents_id_list
        self.statistic_dict["id_of_through_vehicle"] = self.through_vehicle_id_list
        self.statistic_dict["id_of_left_turn_vehicle"] = self.left_turn_vehicle_id_list
        self.statistic_dict["id_of_right_turn_vehicle"] = self.right_turn_vehicle_id_list 
        self.statistic_dict["id_of_u_turn_vehicle"] = self.u_turn_vehicle_id_list 
        self.statistic_dict["id_of_lane_change_vehicle"] = self.lane_change_vehicle_id_list  