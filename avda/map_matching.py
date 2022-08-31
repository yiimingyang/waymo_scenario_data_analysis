#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import csv
import os
import json
from .waymo_training_math_tools import *
from .find_near_vehicles_in_lane import *
from shapely.geometry import Point, Polygon
from .av_data_classes import *
def find_lane_agent_on(scenario, agent_x, agent_y, next_agent_x, next_agent_y):
    
    find_nearest_lane = False
    for center_lane in scenario.center_lane_list:
        center_lane_id = center_lane.lane_id
        center_lane_type = center_lane.lane_type
        center_lane_shape = None

        if center_lane.is_right_turn:
            center_lane_shape = "right_turn"
        if center_lane.is_left_turn:
            center_lane_shape = "left_turn"            
        if center_lane.is_u_turn:
            center_lane_shape = "u_turn"   
        if center_lane.is_straight:
            center_lane_shape = "straight"            
       
        angle_agent = np.rad2deg(np.arctan2(agent_y - next_agent_y, agent_x - next_agent_x))
        
        # if the center lane is a straight lane
        if center_lane_shape == "straight":
            # calculate the straight line equation
            number_of_points_in_center_lane = len(center_lane.points_list)            
            # if the lane is too long, it may not straight, split the lane into some segements   
            number_of_segments = math.ceil(number_of_points_in_center_lane/40)
            first_point_index = 0
            last_point_index = number_of_points_in_center_lane - 1   
            segment_interval = math.ceil(last_point_index/number_of_segments)               
            middle_point_index = 0          
            for i in range(number_of_segments):
                middle_point_index += segment_interval
                if middle_point_index > last_point_index:
                    middle_point_index = last_point_index        
                segment_first_point_x = center_lane.point_x_list[first_point_index]
                segment_first_point_y = center_lane.point_y_list[first_point_index]
                segment_last_point_x  = center_lane.point_x_list[middle_point_index]
                segment_last_point_y  = center_lane.point_y_list[middle_point_index]  
                
                a, b, c = get_straight_equation_from_two_points(segment_first_point_x,
                                                                segment_first_point_y,
                                                                segment_last_point_x,
                                                                segment_last_point_y)
                 
                if a == b == 0:
                    continue              

                is_in_lane = is_agent_in_segment(a, b, c, agent_x, agent_y, segment_first_point_x, segment_first_point_y, segment_last_point_x, segment_last_point_y)
                if is_in_lane > 0:
                    find_nearest_lane = True
                    return center_lane_id, center_lane_shape
                first_point_index = middle_point_index
                
        # if the center lane is a left/right lane
        else:
            points_number = len(center_lane.points_list)
            point_index = 0
            while point_index < points_number:
                point = center_lane.points_list[point_index]
                point_x = point[0]
                point_y = point[1]
                distance = calculate_distance(point_x, point_y, agent_x, agent_y)
                if distance < 3:
                    if point_index > 0:
                        last_point = center_lane.points_list[point_index - 1]
                        last_point_x = last_point[0]
                        last_point_y = last_point[1]
                        angle_lane = np.rad2deg(np.arctan2(last_point_y - point_y, last_point_x - point_x))
                        if abs(abs(angle_lane) - abs(angle_agent)) < 20:
                            find_nearest_lane = True
                            #if center_lane_shape == "right_turn":
                                #print(" ============= lane id: ", center_lane_id)
                                #print(" ============= agent angle: ", angle_agent)
                                #print(" ============= lane angle: ", angle_lane)
                            return center_lane_id, center_lane_shape   
                                                  
                    else:
                        next_point = center_lane.points_list[point_index + 1]
                        next_point_x = next_point[0]
                        next_point_y = next_point[1]  
                        angle_lane = np.rad2deg(np.arctan2(point_y - next_point_y, point_x - next_point_x))
                        if abs(abs(angle_lane) - abs(angle_agent)) < 20:
                            find_nearest_lane = True
                            return center_lane_id, center_lane_shape    
                point_index += 1
               
    if find_nearest_lane == False:
        return None, center_lane_shape


def find_lane_agent_on_v1(scenario, agent_x, agent_y, next_agent_x, next_agent_y, agent_x_list, agent_y_list):
    
    find_nearest_lane = False
    for center_lane in scenario.center_lane_list:
        center_lane_id = center_lane.lane_id
        center_lane_type = center_lane.lane_type
        center_lane_shape = None
        if center_lane.is_straight:
            center_lane_shape = "straight"
        elif center_lane.is_right_turn:
            center_lane_shape = "right_turn"
        elif center_lane.is_left_turn:
            center_lane_shape = "left_turn"            
        elif center_lane.is_u_turn:
            center_lane_shape = "u_turn"   
        else:
            center_lane_shape = "other"
        
        angle_agent = np.rad2deg(np.arctan2(agent_y - next_agent_y, agent_x - next_agent_x))
        
        # if the center lane is a straight lane
        if center_lane_shape == "straight":
            # calculate the straight line equation
            number_of_points_in_center_lane = len(center_lane.points_list)            
            # if the lane is too long, it may not straight, split the lane into some segements   
            number_of_segments = math.ceil(number_of_points_in_center_lane/30)
            first_point_index = 0
            last_point_index = number_of_points_in_center_lane - 1   
            segment_interval = math.ceil(last_point_index/number_of_segments)               
            middle_point_index = 0          
            for i in range(number_of_segments):
                middle_point_index += segment_interval
                if middle_point_index > last_point_index:
                    middle_point_index = last_point_index        
                segment_first_point_x = center_lane.point_x_list[first_point_index]
                segment_first_point_y = center_lane.point_y_list[first_point_index]
                segment_last_point_x  = center_lane.point_x_list[middle_point_index]
                segment_last_point_y  = center_lane.point_y_list[middle_point_index]  
                
                a, b, c = get_straight_equation_from_two_points(segment_first_point_x,
                                                                segment_first_point_y,
                                                                segment_last_point_x,
                                                                segment_last_point_y)
                 
                if a == b == 0:
                    continue              

                is_in_lane = is_agent_in_segment(a, b, c, agent_x, agent_y, segment_first_point_x, segment_first_point_y, segment_last_point_x, segment_last_point_y)
                if is_in_lane > 0:
                    find_nearest_lane = True
                    return center_lane_id, center_lane_shape
                first_point_index = middle_point_index
                
        # if the center lane is a left/right lane
        else:
            number_of_points_in_center_lane = len(center_lane.points_list)
            middle_point_index = int(0.5 * number_of_points_in_center_lane)
            first_lane_point = center_lane.points_list[0]
            middle_lane_point = center_lane.points_list[middle_point_index]
            last_lane_point  = center_lane.points_list[-1]
            
            first_lane_point_x = first_lane_point[0]
            first_lane_point_y = first_lane_point[1]
            middle_lane_point_x = middle_lane_point[0]
            middle_lane_point_y = middle_lane_point[1]
            last_lane_point_x = last_lane_point[0]
            last_lane_point_y = last_lane_point[1]
                      
            agent_location_index = 0
            agent_point_number = len(agent_x_list)
            
            flag_first_match  = False
            flag_middle_match = False
            flag_last_match   = False
            
            while agent_location_index < agent_point_number:
                agent_x = agent_x_list[agent_location_index]
                agent_y = agent_y_list[agent_location_index]
                
                distance_to_first_lane_point  = calculate_distance_between_two_points(first_lane_point_x,  first_lane_point_y,  agent_x, agent_y)
                distance_to_middle_lane_point = calculate_distance_between_two_points(middle_lane_point_x, middle_lane_point_y, agent_x, agent_y)                
                distance_to_last_lane_point   = calculate_distance_between_two_points(last_lane_point_x,   last_lane_point_y,   agent_x, agent_y)
                
                if distance_to_first_lane_point < 2:
                    flag_first_match = True
                if distance_to_last_lane_point < 2:
                    flag_middle_match = True                                          
                if distance_to_last_lane_point < 2:
                    flag_last_match = True   
                    
                agent_location_index += 1
                
            if flag_first_match and flag_middle_match:
                find_nearest_lane = True
                return center_lane_id, center_lane_shape                      
            if flag_first_match and flag_last_match:
                find_nearest_lane = True
                return center_lane_id, center_lane_shape                      
            if flag_middle_match and flag_last_match:
                find_nearest_lane = True
                return center_lane_id, center_lane_shape  
                    
            '''                                
            if flag_first_match and flag_last_match:
                find_nearest_lane = True
                return center_lane_id, center_lane_shape                           
            
            point_index = 0            
                      
            
            points_number = len(center_lane.points_list)
            point_index = 0
            while point_index < points_number:
                point = center_lane.points_list[point_index]
                point_x = point[0]
                point_y = point[1]
                distance = calculate_distance(point_x, point_y, agent_x, agent_y)
                if distance < 2:
                    if point_index > 0:
                        last_point = center_lane.points_list[point_index - 1]
                        last_point_x = last_point[0]
                        last_point_y = last_point[1]
                        angle_lane = np.rad2deg(np.arctan2(last_point_y - point_y, last_point_x - point_x))
                        if abs(abs(angle_lane) - abs(angle_agent)) < 20:
                            find_nearest_lane = True
                            #if center_lane_shape == "right_turn":
                                #print(" ============= lane id: ", center_lane_id)
                                #print(" ============= agent angle: ", angle_agent)
                                #print(" ============= lane angle: ", angle_lane)
                            return center_lane_id, center_lane_shape   
                                                  
                    else:
                        next_point = center_lane.points_list[point_index + 1]
                        next_point_x = next_point[0]
                        next_point_y = next_point[1]  
                        angle_lane = np.rad2deg(np.arctan2(point_y - next_point_y, point_x - next_point_x))
                        if abs(abs(angle_lane) - abs(angle_agent)) < 20:
                            find_nearest_lane = True
                            return center_lane_id, center_lane_shape    
                point_index += 1
             '''
                
    if find_nearest_lane == False:
        return None, center_lane_shape


def map_agent_to_straight_lane(scenario, center_lane, agent_x, agent_y, next_agent_x, next_agent_y):
    
    find_nearest_lane = False
    for center_lane in scenario.center_lane_list:
        center_lane_id = center_lane.lane_id
        center_lane_type = center_lane.lane_type
        center_lane_shape = None
        if center_lane.is_straight:
            center_lane_shape = "straight"
        elif center_lane.is_right_turn:
            center_lane_shape = "right_turn"
        elif center_lane.is_left_turn:
            center_lane_shape = "left_turn"            
        elif center_lane.is_u_turn:
            center_lane_shape = "u_turn"   
        else:
            center_lane_shape = "other"
        
        angle_agent = np.rad2deg(np.arctan2(agent_y - next_agent_y, agent_x - next_agent_x))
        
        # if the center lane is a straight lane
        if center_lane_shape == "straight":
            # calculate the straight line equation
            number_of_points_in_center_lane = len(center_lane.points_list)            
            # if the lane is too long, it may not straight, split the lane into some segements   
            number_of_segments = math.ceil(number_of_points_in_center_lane/30)
            first_point_index = 0
            last_point_index = number_of_points_in_center_lane - 1   
            segment_interval = math.ceil(last_point_index/number_of_segments)               
            middle_point_index = 0          
            for i in range(number_of_segments):
                middle_point_index += segment_interval
                if middle_point_index > last_point_index:
                    middle_point_index = last_point_index        
                segment_first_point_x = center_lane.point_x_list[first_point_index]
                segment_first_point_y = center_lane.point_y_list[first_point_index]
                segment_last_point_x  = center_lane.point_x_list[middle_point_index]
                segment_last_point_y  = center_lane.point_y_list[middle_point_index]  
                
                a, b, c = get_straight_equation_from_two_points(segment_first_point_x,
                                                                segment_first_point_y,
                                                                segment_last_point_x,
                                                                segment_last_point_y)
                 
                if a == b == 0:
                    continue              

                is_in_lane = is_agent_in_segment(a, b, c, agent_x, agent_y, segment_first_point_x, segment_first_point_y, segment_last_point_x, segment_last_point_y)
                if is_in_lane > 0:
                    find_nearest_lane = True
                    return center_lane_id, center_lane_shape
                first_point_index = middle_point_index    

    if find_nearest_lane == False:
        return None, center_lane_shape

def map_agent_to_curve(scenario, center_lane, agent_x_list, agent_y_list):
    
    find_nearest_lane = False

    if center_lane.is_right_turn:
        center_lane_shape = "right_turn"
    elif center_lane.is_left_turn:
        center_lane_shape = "left_turn"            
    elif center_lane.is_u_turn:
        center_lane_shape = "u_turn"   
    else:
        center_lane_shape = "other"
        
    number_of_points_in_center_lane = len(center_lane.points_list)
    middle_point_index = int(0.5 * number_of_points_in_center_lane)
    first_lane_point = center_lane.points_list[0]
    middle_lane_point = center_lane.points_list[middle_point_index]
    last_lane_point  = center_lane.points_list[-1]
    
    first_lane_point_x = first_lane_point[0]
    first_lane_point_y = first_lane_point[1]
    middle_lane_point_x = middle_lane_point[0]
    middle_lane_point_y = middle_lane_point[1]
    last_lane_point_x = last_lane_point[0]
    last_lane_point_y = last_lane_point[1]
              
    agent_location_index = 0
    agent_point_number = len(agent_x_list)
    
    flag_first_match  = False
    flag_middle_match = False
    flag_last_match   = False
    
    while agent_location_index < agent_point_number:
        agent_x = agent_x_list[agent_location_index]
        agent_y = agent_y_list[agent_location_index]
        
        distance_to_first_lane_point  = calculate_distance_between_two_points(first_lane_point_x,  first_lane_point_y,  agent_x, agent_y)
        distance_to_middle_lane_point = calculate_distance_between_two_points(middle_lane_point_x, middle_lane_point_y, agent_x, agent_y)                
        distance_to_last_lane_point   = calculate_distance_between_two_points(last_lane_point_x,   last_lane_point_y,   agent_x, agent_y)
        
        if distance_to_first_lane_point < 2:
            flag_first_match = True
        if distance_to_last_lane_point < 2:
            flag_middle_match = True                                          
        if distance_to_last_lane_point < 2:
            flag_last_match = True   
            
        agent_location_index += 1
        
    if flag_first_match and flag_middle_match:
        find_nearest_lane = True
        return center_lane_id, center_lane_shape                      
    if flag_first_match and flag_last_match:
        find_nearest_lane = True
        return center_lane_id, center_lane_shape                      
    if flag_middle_match and flag_last_match:
        find_nearest_lane = True
        return center_lane_id, center_lane_shape  

    if find_nearest_lane == False:
        return None, center_lane_shape


def map_agent_to_straight_curve_lane(scenario):
    
    for agent in scenario.agents_list: 
        agent_time_list = []
        agent_x_list = []
        agent_y_list = []  
        agent_lane_id_list = []
        agent_lane_type_list = []
        agent_link_id_list = []
        agent_node_id_list = []
        agent_turn_movement_list = []
        
        if agent.states_array.size == 0:
            continue    
        states = agent.states_array
        first_state = states[0]
        state_number = len(states)
        agent_type = first_state[2]
        agent_id = int(first_state[1])
        
        state_index = 0
        time = 0
        while state_index < state_number:
            state = agent.states_array[state_index, :]             
            if state[12] == '-1':
                state_index += 1
                time += 0.1
                continue            
            agent_x = state[3]
            agent_y = state[4]
            agent_z = state[5]
            agent_yaw    = radian_to_degree(state[9])
            agent_length = state[6]
            agent_width  = state[7]
            agent_height = state[8]
            agent_velocity  = math.sqrt(state[10]**2 + state[11]**2)                        
            agent_x_list.append(agent_x)
            agent_y_list.append(agent_y)
            agent_z_list.append(agent_z)

            agent_length_list.append(agent_length)
            agent_width_list.append(agent_width)
            agent_height_list.append(agent_height)            
            
            state_index += 1
            time += 0.1
            agent_time_list.append(time)
        
        # for each center lane, determine if it is straight lane or curve
        
        for center_lane in scenario.center_lane_list:
            center_lane_id = center_lane.lane_id
            center_lane_type = center_lane.lane_type
            center_lane_shape = None
            
            if center_lane.is_straight:
                center_lane_shape = "straight"
                number_of_locations = len(agent_x_list)
                index = 0
                agent_x = None
                agent_y = None
                next_agent_x = None
                next_agent_y = None
                while index < number_of_locations:
                    if index == number_of_locations - 1:
                        next_agent_x = agent_x_list[index]
                        next_agent_y = agent_y_list[index]  
                        agent_x = agent_x_list[index - 1] 
                        agent_y = agent_y_list[index - 1]              
                    else:
                        agent_x = agent_x_list[index]
                        agent_y = agent_y_list[index]  
                        next_agent_x = agent_x_list[index + 1] 
                        next_agent_y = agent_y_list[index + 1]
                                                          
                mateched_lane_id, center_lane_shape = map_agent_to_straight_lane(scenario, center_lane, agent_x, agent_y, next_agent_x, next_agent_y)    
                if mateched_lane_id != None:
                    matched_lane_dict = scenario.lane_dict[mateched_lane_id]
                    matched_link_id = matched_lane_dict["link_id"]
                    matched_turn_movement_id = matched_lane_dict["turn_movement_id"]
                    matched_node_id = matched_lane_dict["node_id"]
                    agent_lane_id_list.append(mateched_lane_id)
                    agent_lane_type_list.append(center_lane_shape)
                    agent_link_id_list.append(matched_link_id)
                    agent_node_id_list.append(matched_node_id)
                    agent_turn_movement_list.append(matched_turn_movement_id) 
                else:
                    agent_lane_id_list.append(" ")
                    agent_lane_type_list.append(" ")
                    agent_link_id_list.append(" ")
                    agent_node_id_list.append(" ")
                    agent_turn_movement_list.append(" ") 
                index += 1 

                agent.matching_agent_time_list = agent_time_list
                agent.matching_agent_x_list = agent_x_list
                agent.matching_agent_y_list = agent_y_list
                agent.matching_agent_z_list = agent_z_list
                
                agent.matching_agent_length_list = agent_length_list
                agent.matching_agent_width_list  = agent_width_list
                agent.matching_agent_height_list = agent_height_list                
                
                agent.matching_agent_lane_id_list = agent_lane_id_list
                agent.matching_agent_lane_type_list = agent_lane_type_list
                agent.matching_agent_link_id_list = agent_link_id_list
                agent.matching_agent_node_id_list = agent_node_id_list
                agent.matching_agent_turn_movement_list = agent_turn_movement_list
        
            else:
                mateched_lane_id, center_lane_shape = map_agent_to_curve(scenario, center_lane, agent_x_list, agent_y_list) 
                if mateched_lane_id != None:
                    matched_lane_dict = scenario.lane_dict[mateched_lane_id]
                    matched_link_id = matched_lane_dict["link_id"]
                    matched_turn_movement_id = matched_lane_dict["turn_movement_id"]
                    matched_node_id = matched_lane_dict["node_id"]
                                     
                    # to-do: use loop to add info to each state
                    agent_lane_id_list.append(mateched_lane_id)
                    agent_lane_type_list.append(center_lane_shape)
                    agent_link_id_list.append(matched_link_id)
                    agent_node_id_list.append(matched_node_id)
                    agent_turn_movement_list.append(matched_turn_movement_id) 
                else:
                    # to-do: use loop to add info to each state
                    agent_lane_id_list.append(" ")
                    agent_lane_type_list.append(" ")
                    agent_link_id_list.append(" ")
                    agent_node_id_list.append(" ")
                    agent_turn_movement_list.append(" ") 
                index += 1 

                agent.matching_agent_time_list = agent_time_list
                agent.matching_agent_x_list = agent_x_list
                agent.matching_agent_y_list = agent_y_list
                agent.matching_agent_z_list = agent_z_list
                agent.matching_agent_lane_id_list = agent_lane_id_list
                agent.matching_agent_lane_type_list = agent_lane_type_list
                agent.matching_agent_link_id_list = agent_link_id_list
                agent.matching_agent_node_id_list = agent_node_id_list
                agent.matching_agent_turn_movement_list = agent_turn_movement_list           




def map_agent_to_lane(scenario):
    
    for agent in scenario.agents_list: 
        agent_time_list = []
        agent_x_list = []
        agent_y_list = []
        agent_z_list = []
        agent_lane_id_list = []
        agent_lane_type_list = []
        agent_link_id_list = []
        agent_node_id_list = []
        agent_turn_movement_list = []
        
        if agent.states_array.size == 0:
            continue    
        states = agent.states_array
        first_state = states[0]
        state_number = len(states)
        agent_type = first_state[2]
        agent_id = int(first_state[1])
        
        state_index = 0
        time = 0
        while state_index < state_number:
            state = agent.states_array[state_index, :]             
            if state[12] == '-1':
                state_index += 1
                time += 0.1
                continue            
            agent_x = state[3]
            agent_y = state[4]
            agent_z = state[5]
            agent_yaw    = radian_to_degree(state[9])
            agent_length = state[6]
            agent_width  = state[7]
            agent_height = state[8]
            agent_velocity  = math.sqrt(state[10]**2 + state[11]**2)                        
            agent_x_list.append(agent_x)
            agent_y_list.append(agent_y)
            state_index += 1
            time += 0.1
            agent_time_list.append(time)
            
        # for each point, find the lane the vehicle is on
        number_of_locations = len(agent_x_list)
        index = 0
        agent_x = None
        agent_y = None
        next_agent_x = None
        next_agent_y = None
        while index < number_of_locations:
            if index == number_of_locations - 1:
                next_agent_x = agent_x_list[index]
                next_agent_y = agent_y_list[index]  
                agent_x = agent_x_list[index - 1] 
                agent_y = agent_y_list[index - 1]              
            else:
                agent_x = agent_x_list[index]
                agent_y = agent_y_list[index]  
                next_agent_x = agent_x_list[index + 1] 
                next_agent_y = agent_y_list[index + 1]
                
            mateched_lane_id, center_lane_shape = find_lane_agent_on(scenario, agent_x, agent_y, next_agent_x, next_agent_y)
            #mateched_lane_id, center_lane_shape = find_lane_agent_on_v1(scenario, agent_x, agent_y, next_agent_x, next_agent_y, agent_x_list, agent_y_list)
            if mateched_lane_id != None:
                matched_lane_dict = scenario.lane_dict[mateched_lane_id]
                matched_link_id = matched_lane_dict["link_id"]
                matched_turn_movement_id = matched_lane_dict["turn_movement_id"]
                matched_node_id = matched_lane_dict["node_id"]
                agent_lane_id_list.append(mateched_lane_id)
                agent_lane_type_list.append(center_lane_shape)
                agent_link_id_list.append(matched_link_id)
                agent_node_id_list.append(matched_node_id)
                agent_turn_movement_list.append(matched_turn_movement_id) 
            else:
                agent_lane_id_list.append(" ")
                agent_lane_type_list.append(" ")
                agent_link_id_list.append(" ")
                agent_node_id_list.append(" ")
                agent_turn_movement_list.append(" ") 
            index += 1
            
        agent.matching_agent_time_list = agent_time_list
        agent.matching_agent_x_list = agent_x_list
        agent.matching_agent_y_list = agent_y_list
        agent.matching_agent_z_list = agent_z_list
        agent.matching_agent_lane_id_list = agent_lane_id_list
        agent.matching_agent_lane_type_list = agent_lane_type_list
        agent.matching_agent_link_id_list = agent_link_id_list
        agent.matching_agent_node_id_list = agent_node_id_list
        agent.matching_agent_turn_movement_list = agent_turn_movement_list
            

'''
Do map matching
'''
def map_agent_to_lane_v1(scenario):
    
    for agent in scenario.agents_list: 
        if agent.states_array.size == 0:
            continue   
        if agent.agent_type == 2 or agent.agent_type == 3:
            continue           
              
        agent_time_list = []
        agent_x_list = []
        agent_y_list = []
        agent_z_list = []
        agent_length_list = []
        agent_width_list = []
        agent_height_list = []
        agent_speed_x_list = []
        agent_speed_y_list = []
        agent_speed_list = []
        agent_acc_list = []
        agent_lane_id_list = []
        agent_lane_type_list = []
        agent_link_id_list = []
        agent_node_id_list = []
        agent_turn_movement_list = []
        agent_trajectory_type = None
        
        if agent.is_straight:
            agent_trajectory_type = "straight"
        
        states = agent.states_array
        first_state = states[0]
        state_number = len(states)
        agent_type = first_state[2]
        agent_id = int(first_state[1])
        
        last_time_step = None
        current_time_step = None
        last_velocity = None
        current_velocity = None
        last_location = None
        current_location = None
        last_z = None
        current_z = None
        
        odometer = 0
        state_index = 0
        time = 0
        total_frame = 0
        acc = 0
        while state_index < state_number:
            current_time_step = scenario.timestep_list[state_index]
            state = agent.states_array[state_index, :]             
            if state[12] == '-1':
                state_index += 1
                time += 0.1
                continue            
            agent_x = state[3]
            agent_y = state[4]
            agent_z = state[5]
            current_z = agent_z
            agent_yaw    = radian_to_degree(state[9])
            agent_length = state[6]
            agent_width  = state[7]
            agent_height = state[8]
            velocity_x = 2.2369 * state[10]  # mps to mph
            velocity_y = 2.2369 * state[11]  # mps to mph
            agent_velocity  = math.sqrt(velocity_x**2 + velocity_y**2)
            current_velocity = agent_velocity
            current_location = (agent_x, agent_y)  
            
            if last_time_step == None:
                acc = "None"
                grade = "None"
                odometer = 0
            else:

                delta_time = current_time_step - last_time_step
                acc = (current_velocity - last_velocity) / delta_time
                if abs(acc) > 10:
                    acc = "None"
    
                delta_z = current_z - last_z
                delta_distance = calculate_distance_between_two_points_v1(current_location, last_location)
                grade = delta_z / delta_distance
                                  
                distance_to_last_location = calculate_distance_between_two_points_v1(current_location, last_location)
                if distance_to_last_location < 5:
                    odometer += distance_to_last_location
                else:
                    odometer += 0
                    
            agent.odometer_list.append(odometer)
            agent.acceleration_list.append(acc)
            agent.grade_list.append(grade)
            agent_x_list.append(agent_x)
            agent_y_list.append(agent_y)
            agent_z_list.append(agent_z)
                       
            agent_length_list.append(agent_length)
            agent_width_list.append(agent_width)
            agent_height_list.append(agent_height)
             
            agent_speed_x_list.append(velocity_x)
            agent_speed_y_list.append(velocity_y)
            agent_speed_list.append(agent_velocity)
            
            last_location = current_location
            last_velocity = current_velocity
            last_time_step = current_time_step
            last_z = current_z
            
            state_index += 1
            time += 0.1
            total_frame += 1
            agent_time_list.append(time)

        #if agent_id == 1562:
        #    print("   --- 1528 acc list is: ", agent.acceleration_list)
        #    print("   --- 1528 velocity x list is: ", agent_speed_x_list)
            
        # for each point, find the lane the vehicle is on
        number_of_locations = len(agent_x_list)
        index = 0
        agent_x = None
        agent_y = None
        next_agent_x = None
        next_agent_y = None
        while index < number_of_locations: 
            agent_x = agent_x_list[index] 
            agent_y = agent_y_list[index]              
            location = Point(agent_x, agent_y)
            
            matched_lane_id_list = []
            matched_lane_type_list = []
            matched_link_id_list = []
            matched_turn_movement_id_list = []
            matched_node_id_list = []
            
            mateched_lane_id = None
            center_lane_shape = None
            for center_lane in scenario.center_lane_list:
                center_lane_id = center_lane.lane_id
                center_lane_type = center_lane.lane_type
                center_lane_shape = None
                if center_lane.is_straight or center_lane.is_straight_turn:
                    center_lane_shape = "THROUGH_LANE"
                elif center_lane.is_right_turn:
                    center_lane_shape = "RIGHT_TURN_LANE"
                elif center_lane.is_left_turn:
                    center_lane_shape = "LEFT_TURN_LANE"            
                elif center_lane.is_u_turn:
                    center_lane_shape = "U_TURN_LANE"   
                else:
                    center_lane_shape = "OTHER"                        
                
                #if agent_id == 1502 and center_lane_id == 287 and index == 24:
                #    print("   --- map-matching 1502 : ", center_lane.polygon_for_map_matching)
                
                lane_polygon = center_lane.polygon_for_map_matching
                if lane_polygon == None:
                    continue
                if location.within(lane_polygon):
                    
                    #if agent_id == 1502 and center_lane_id == 287 and index == 24:
                    #    print("   --- map-matching 1502 is in 287: ")                   
                    
                    if agent_trajectory_type == "straight":
                        mateched_lane_id  = center_lane_id
                        center_lane_shape = center_lane_shape
                        matched_lane_id_list.append(mateched_lane_id)
                        matched_lane_type_list.append("THROUGH_LANE")
                        #matched_lane_type_list.append(center_lane_shape)
                        
                        #if agent_id == 1502 and center_lane_id == 287 and index == 24:
                        #    print("   --- 1502 matched lane id list is : ", matched_lane_id_list)   
                        
                    else:
                        mateched_lane_id  = center_lane_id
                        center_lane_shape = center_lane_shape
                        matched_lane_id_list.append(mateched_lane_id)
                        matched_lane_type_list.append(center_lane_shape)      
                        
                        #if agent_id == 1502 and center_lane_id == 287 and index == 24:
                        #    print("   --- 1502 matched lane id list is : ", matched_lane_id_list)   
                    
            if matched_lane_id_list != []:
                for lane_id in matched_lane_id_list:
                    matched_lane_dict = scenario.lane_dict[lane_id]
                    matched_link_id = matched_lane_dict["link_id"]
                    matched_turn_movement_id = matched_lane_dict["turn_movement_id"]
                    matched_node_id = matched_lane_dict["node_id"]
                    
                    matched_link_id_list.append(matched_link_id)
                    matched_turn_movement_id_list.append(matched_turn_movement_id)
                    matched_node_id_list.append(matched_node_id)
            else:             
                matched_lane_id_list.append("None")
                matched_lane_type_list.append("None")
                matched_link_id_list.append("None")
                matched_node_id_list.append("None")
                matched_turn_movement_id_list.append("None")                 
            index += 1
            
            agent_lane_id_list.append(matched_lane_id_list)
            agent_lane_type_list.append(matched_lane_type_list)
            agent_link_id_list.append(matched_link_id_list)
            agent_node_id_list.append(matched_node_id_list)
            agent_turn_movement_list.append(matched_turn_movement_id_list)
            
        agent.matching_agent_time_list = agent_time_list
        agent.matching_agent_total_frame = total_frame
        agent.matching_agent_x_list = agent_x_list
        agent.matching_agent_y_list = agent_y_list
        agent.matching_agent_z_list = agent_z_list
        
        agent.matching_agent_length_list = agent_length_list
        agent.matching_agent_width_list = agent_width_list
        agent.matching_agent_height_list = agent_height_list       

        agent.matching_agent_speed_x_list = agent_speed_x_list
        agent.matching_agent_speed_y_list = agent_speed_y_list
        agent.matching_agent_speed_list = agent_speed_list
        
        agent.matching_agent_lane_id_list = agent_lane_id_list
        agent.matching_agent_lane_type_list = agent_lane_type_list
        agent.matching_agent_link_id_list = agent_link_id_list
        agent.matching_agent_node_id_list = agent_node_id_list
        agent.matching_agent_turn_movement_list = agent_turn_movement_list



def generate_unique_map_matching_lane_id(scenario):
    # go through each agent and determine the final main map-matching lane id
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type
        
        turn_type = None
        right_turn_count = 0
        left_turn_count = 0
        u_turn_count = 0
        straight_count = 0
        
        right_turn_id_list = []
        left_turn_id_list = []
        u_turn_id_list = []
        straight_id_list = []  
        
        '''
        if agent.agent_id == 4109:
            print("   --- 4109 lane_id_list :", agent.matching_agent_lane_id_list)
            print("   --- 4109 lane_type_list :", agent.matching_agent_lane_type_list)
            print("   --- 4109 lane_id_list length:", len(agent.matching_agent_lane_id_list))
            print("   --- 4109 lane_type_list length:", len(agent.matching_agent_lane_type_list))   
        '''
        
        list_index = -1
        for lane_type_list in agent.matching_agent_lane_type_list:
            list_index += 1
            #print("   --- list index is: ", list_index)
            #if agent.agent_id == 4109:
            #    print("   --- 4109 lane type list :", lane_type_list)
                
            
            lane_id_list = agent.matching_agent_lane_id_list[list_index]
            
            #if agent.agent_id == 4109:
            #    print("   --- 4109 lane id list :", lane_id_list, list_index)
                
            if len(lane_id_list) == 1:
                continue
            
            #if agent.agent_id == 4109:
            #    print("   --- 4109 lane type list :", lane_type_list, list_index)
            
            index = -1
            for lane_type in lane_type_list:
                index += 1
                lane_id = lane_id_list[index]
                if lane_type == "RIGHT_TURN_LANE":
                    right_turn_count += 1
                    right_turn_id_list.append(lane_id)
                if lane_type == "LEFT_TURN_LANE":
                    left_turn_count += 1
                    left_turn_id_list.append(lane_id)
                if lane_type == "U_TURN_LANE":
                    u_turn_count += 1        
                    u_turn_id_list.append(lane_id)
                if lane_type == "THROUGH_LANE":
                    straight_count += 1
                    straight_id_list.append(lane_id)
                    
                
        if right_turn_count > 15:
            turn_type = "right_turn"
            agent.map_matching_lane_id = find_most_frequent_element(right_turn_id_list)
        elif left_turn_count > 15:
            turn_type = "left_turn"
            agent.map_matching_lane_id = find_most_frequent_element(left_turn_id_list)
        elif u_turn_count > 15:
            turn_type = "u_turn"
            agent.map_matching_lane_id = find_most_frequent_element(u_turn_id_list)
        else:
            turn_type = "straight"
            if len(straight_id_list) > 0:
                agent.map_matching_lane_id = find_most_frequent_element(straight_id_list)
            else:
                agent.map_matching_lane_id = "None"

        if agent.agent_id == 4109:
            print("   --- 4109 map matching lane id :", agent.map_matching_lane_id)
            
            print("   --- 4109 left turn id list :", left_turn_id_list)
            print("   --- 4109 right turn id list :", right_turn_id_list)
            print("   --- 4109 u turn id list :", u_turn_id_list)
            print("   --- 4109 straight turn id list :", straight_id_list)        
            
            print("   --- 4109 left turn number :", left_turn_count)
            print("   --- 4109 right turn number :", right_turn_count)
            print("   --- 4109 u turn number :", u_turn_count)
            print("   --- 4109 straight turn number :", straight_count)
            
    # go through each agent and determine the final unique map-matching lane id list
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type
        for lane_id_list in agent.matching_agent_lane_id_list:
            if len(lane_id_list) == 1:
                lane_id = lane_id_list[0]
                agent.unique_matching_agent_lane_id_list.append(lane_id)            
            else:
                main_lane_id = agent.map_matching_lane_id
                agent.unique_matching_agent_lane_id_list.append(main_lane_id)

        #if agent.agent_id == 4109:
        #    print("   --- 1st agents 4109 : ", agent.unique_matching_agent_lane_id_list)




def find_merging_diverging_crossing_lane_using_unique_map_matching_lane(scenario):
    # go through each agent and determine the final main map-matching lane id
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type
        
        merging_lane_id_list = []
        diverging_lane_id_list = []
        crossing_lane_id_list = []
               
        list_index = 0
        
        for unique_lane_id in agent.unique_matching_agent_lane_id_list:
            merging_lane_id_list_for_this_lane   = []
            diverging_lane_id_list_for_this_lane = []
            crossing_lane_id_list_for_this_lane  = []  
            
            if unique_lane_id == "None":
                merging_lane_id_list.append("None")
                diverging_lane_id_list.append("None")
                crossing_lane_id_list.append("None")
                continue
            
            unique_lane = scenario.center_lane_dict[unique_lane_id]
          
            if len(unique_lane.crossing_lane_id_list) == 0:
                crossing_lane_id_list_for_this_lane = "None"   
            else:
                crossing_lane_id_list_for_this_lane = unique_lane.crossing_lane_id_list
                       
            # find the exist lane
            if len(unique_lane.exit_lanes_list) == 0:
                merging_lane_id_list_for_this_lane   = "None"
                diverging_lane_id_list_for_this_lane = "None"
            else:
                # find diverging lanes
                diverging_lane_id_list_for_this_lane = unique_lane.exit_lanes_list
                for next_lane_id in unique_lane.exit_lanes_list:
                    next_lane = scenario.center_lane_dict[next_lane_id]
                    # find entry / merging lanes for the diverging lanes
                    if len(next_lane.entry_lines_list) == 0:
                        merging_lane_id_list_for_this_lane = "None"
                    else:
                        if len(next_lane.entry_lines_list) > 1:
                            if unique_lane_id in next_lane.entry_lines_list:
                                for merging_lane_id in next_lane.entry_lines_list:
                                    if merging_lane_id != unique_lane_id:
                                        merging_lane_id_list_for_this_lane.append(merging_lane_id)
                    
            merging_lane_id_list.append(merging_lane_id_list_for_this_lane)
            diverging_lane_id_list.append(diverging_lane_id_list_for_this_lane)
            crossing_lane_id_list.append(crossing_lane_id_list_for_this_lane)
            
        agent.merging_lane_list = merging_lane_id_list
        agent.diverging_lane_list = diverging_lane_id_list
        agent.crossing_lane_list = crossing_lane_id_list



'''
find left right lane gap for each agent
'''
def find_agent_left_right_lane_spacing(scenario):
    # go through each agent and find the left lane spacing and right lane spacing
    for agent in scenario.agents_list:
        if agent.states_array.size == 0:
            continue   
        if agent.agent_type == 2 or agent.agent_type == 3:
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
            agent_z = state[5]        
        
            agent_location = (agent_x, agent_y)
            left_lane_spacing = 9999
            right_lane_spacing = 9999
            left_lane_spacing_id = "None"
            right_lane_spacing_id = "None"
            matched_lane_id = agent.unique_matching_agent_lane_id_list[state_index]
            if matched_lane_id == "None":
                agent.left_lane_spacing_list.append("None")
                agent.right_lane_spacing_list.append("None")
                agent.left_lane_id_list.append("None")
                agent.right_lane_id_list.append("None")
                state_index += 1
                continue
            matched_lane = scenario.center_lane_dict[matched_lane_id]
            left_neighbors_id_list = matched_lane.left_neighbors_id_list
            right_neighbors_id_list = matched_lane.right_neighbors_id_list
            if len(left_neighbors_id_list) == 0:
                left_lane_spacing = "None"
                left_lane_spacing_id = "None"
            if len(right_neighbors_id_list) == 0:
                right_lane_spacing = "None"
                right_lane_spacing_id = "None"
            if len(left_neighbors_id_list) != 0:
                for left_lane_id in left_neighbors_id_list:
                    left_lane = scenario.center_lane_dict[left_lane_id]        
                    left_lane_point_list = left_lane.points_list
                    number_of_points_in_left_lane = len(left_lane.points_list)
                    spacing_point_index, min_distance = find_nearest_point_in_left_or_right_lane(agent_location, left_lane_point_list)
                    if min_distance < left_lane_spacing:
                        left_lane_spacing = min_distance   
                        left_lane_spacing_id = left_lane_id
            if len(right_neighbors_id_list) != 0:
                for right_lane_id in right_neighbors_id_list:
                    right_lane = scenario.center_lane_dict[right_lane_id]        
                    right_lane_point_list = right_lane.points_list
                    number_of_points_in_right_lane = len(right_lane.points_list)
                    spacing_point_index, min_distance = find_nearest_point_in_left_or_right_lane(agent_location, right_lane_point_list)
                    if min_distance < right_lane_spacing:
                        right_lane_spacing = min_distance   
                        right_lane_spacing_id = right_lane_id
                        
            agent.left_lane_spacing_list.append(left_lane_spacing)
            agent.right_lane_spacing_list.append(right_lane_spacing)
            agent.left_lane_id_list.append(left_lane_spacing_id)
            agent.right_lane_id_list.append(right_lane_spacing_id)
            state_index += 1
                    


'''
Find agent lane change behavior
'''
def find_agent_lane_change_behavior(scenario):
    # go through each agent and find the lane change behavior
    for agent in scenario.agents_list:
        if agent.states_array.size == 0:
            continue   
        if agent.agent_type == 2 or agent.agent_type == 3:
            continue
        states = agent.states_array
        first_state = states[0]
        state_number = len(states)
        agent_type = first_state[2]
        agent_id = int(first_state[1])
        
        state_index = 0
        last_lane_id = None
        current_lane_id = None
        left_lane_change = "False"
        right_lane_change = "False"         
        while state_index < state_number:
            state = agent.states_array[state_index, :]             
            if state[12] == '-1':
                state_index += 1
                continue            

            if state_index == 0 or state_index == state_number:
                left_lane_change = "False"
                right_lane_change = "False"                
            else:
                matched_lane_id = agent.unique_matching_agent_lane_id_list[state_index]
                if matched_lane_id == "None":
                    left_lane_change = "False"
                    right_lane_change = "False"
                    agent.left_lane_change_list.append(left_lane_change)
                    agent.right_lane_change_list.append(right_lane_change)                      
                    state_index += 1
                    continue
                current_lane_id = matched_lane_id
                if state_index == 0:
                    last_lane_id = current_lane_id
                if current_lane_id == last_lane_id:
                    left_lane_change = "False"
                    right_lane_change = "False"
                else:
                    if last_lane_id == "None" or last_lane_id == None:
                        agent.left_lane_change_list.append(left_lane_change)
                        agent.right_lane_change_list.append(right_lane_change)                      
                        state_index += 1
                        continue                        
                    last_matched_lane = scenario.center_lane_dict[last_lane_id]
                    last_left_neighbors_id_list  = last_matched_lane.left_neighbors_id_list
                    last_right_neighbors_id_list = last_matched_lane.right_neighbors_id_list
                    if len(last_left_neighbors_id_list) == 0:
                        left_lane_change = "False"
                    if len(last_right_neighbors_id_list) == 0:
                        right_lane_change = "False"   
                    if len(last_left_neighbors_id_list) != 0:
                        if current_lane_id in last_left_neighbors_id_list:
                            left_lane_change = "True"
                    if len(last_right_neighbors_id_list) != 0:
                        if current_lane_id in last_right_neighbors_id_list:
                            right_lane_change = "True" 
            agent.left_lane_change_list.append(left_lane_change)
            agent.right_lane_change_list.append(right_lane_change)           
            last_lane_id = current_lane_id           
            state_index += 1         
                
      

def calculate_agents_trajectory_type(scenario):
    for agent in scenario.agents_list: 
        agent_trajectory_point_list = agent.trajectory_point_list
        agent_type = agent.agent_type
        agent_id = agent.agent_id        
        if agent.states_array.size == 0:
            continue    
             
        # surrounding pedestrians                         
        if agent_type == 2:           
            continue
        # cyclist
        if agent_type == 3:           
            continue   
          
        number_of_points_in_trajectory = len(agent_trajectory_point_list)
        first_point_index = 0
        last_point_index = number_of_points_in_trajectory - 1
        middle_point_index = int(0.5 * last_point_index)
        
        if number_of_points_in_trajectory < 10:
            continue              
            
        else:
            first_point_x  = agent_trajectory_point_list[first_point_index][0]
            first_point_y  = agent_trajectory_point_list[first_point_index][1]
            second_point_x = agent_trajectory_point_list[first_point_index + 1][0]
            second_point_y = agent_trajectory_point_list[first_point_index + 1][1]
            
            last_two_point_x  = agent_trajectory_point_list[last_point_index - 1][0]
            last_two_point_y  = agent_trajectory_point_list[last_point_index - 1][1]         
            last_point_x  = agent_trajectory_point_list[last_point_index][0]
            last_point_y  = agent_trajectory_point_list[last_point_index][1]  
            
            middle_point_x = agent_trajectory_point_list[middle_point_index][0]
            middle_point_y = agent_trajectory_point_list[middle_point_index][1] 
     
            a = first_point_y - last_point_y
            b = last_point_x - first_point_x
            c = first_point_x * last_point_y - last_point_x * first_point_y
            
            if a == 0 and b == 0:
                agent.is_straight = False
            else:
                min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                #print("distance is: ", min_distance)
                if min_distance < 2:
                    agent.is_straight = True                       
                else:
                    agent.is_straight = False  

                    start_point = [first_point_x, first_point_y]
                    end_point = [last_point_x, last_point_y]
                    middle_point = [middle_point_x, middle_point_y]
                    point_location = where_the_point_is(start_point, end_point, middle_point)
                    center_point, radius = get_circle_from_three_points(start_point, end_point, middle_point)
                
                    angle1 = np.rad2deg(np.arctan2(second_point_y - first_point_y, second_point_x - first_point_x))
                    angle2 = np.rad2deg(np.arctan2(last_two_point_y - last_point_y, last_two_point_x - last_point_x))
                    
                    if abs(abs(angle2) - abs(angle1)) < 20:                                          
                        agent.is_u_turn = True
                    else:
                        if point_location == "right":
                            agent.is_right_turn = True
                        if point_location == "right":
                            agent.is_right_turn = True 


def calculate_distance_in_lane_at_beginning_ending(scenario):
    for agent in scenario.agents_list: 
        location_x_list = agent.matching_agent_x_list
        location_y_list = agent.matching_agent_y_list
        if len(location_x_list) == 0:
            continue
        
        first_location_index = 0
        first_location_x = 0
        first_location_y = 0
        while first_location_x == 0:
            first_location_x = location_x_list[first_location_index]
            first_location_y = location_y_list[first_location_index]
            first_location_index += 1

        first_location_point = (first_location_x, first_location_y)
        
        map_matching_index = 0
        first_map_matching_lane_id = None
        while first_map_matching_lane_id == None and map_matching_index < len(agent.unique_matching_agent_lane_id_list) or \
            first_map_matching_lane_id == "None" and map_matching_index < len(agent.unique_matching_agent_lane_id_list):
            first_map_matching_lane_id = agent.unique_matching_agent_lane_id_list[map_matching_index]
            map_matching_index += 1

        #if agent.agent_id == 4109:
        #    print("   --- 4109 first location ", first_location_point)
        #    print("   --- 4109 last map matching lane id:", first_map_matching_lane_id)

        if first_map_matching_lane_id == None or first_map_matching_lane_id == "None":
            agent.distance_to_lane_end_at_beginning = "None"    
        else:
            first_map_matching_lane = scenario.center_lane_dict[first_map_matching_lane_id]
            first_map_matching_lane_points_list = first_map_matching_lane.points_list
            point_index, point_number = find_nearest_point_in_lane(first_location_point, first_map_matching_lane_points_list)

            #if agent.agent_id == 4109:
            #    print("   --- 4109 first point index :", point_index)
            #    print("   --- 4109 first point number :", point_number)
            
            if point_index != None:
                distance_from_first_point_to_lane_end = 0.5 * (point_number - point_index)    
                agent.distance_to_lane_end_at_beginning = distance_from_first_point_to_lane_end
            else:
                agent.distance_to_lane_end_at_beginning = "None"
        
        last_location_index = -1
        last_location_x = 0
        last_location_y = 0
        while last_location_x == 0:
            last_location_x = location_x_list[last_location_index]
            last_location_y = location_y_list[last_location_index]
            last_location_index = last_location_index - 1
        
        last_location_point = (last_location_x, last_location_y)
        
        map_matching_index = -1
        last_map_matching_lane_id = None
        while last_map_matching_lane_id == None and abs(map_matching_index) <= len(agent.unique_matching_agent_lane_id_list) or \
            last_map_matching_lane_id == "None" and abs(map_matching_index) <= len(agent.unique_matching_agent_lane_id_list):
            last_map_matching_lane_id = agent.unique_matching_agent_lane_id_list[map_matching_index]
            map_matching_index = map_matching_index - 1
        
        #if agent.agent_id == 4109:
        #    print("   --- 4109 last location ", last_location_point)
        #    print("   --- 4109 last map matching lane id:", last_map_matching_lane_id)
        
        if last_map_matching_lane_id == None or last_map_matching_lane_id == "None":
            agent.distance_to_lane_start_at_ending = "None"
        else:
            last_map_matching_lane = scenario.center_lane_dict[last_map_matching_lane_id]
            last_map_matching_lane_points_list = last_map_matching_lane.points_list
            point_index, point_number = find_nearest_point_in_lane(last_location_point, last_map_matching_lane_points_list)
            
            #if agent.agent_id == 4109:
            #    print("   --- 4109 last point index :", point_index)
            #    print("   --- 4109 last point number :", point_number)
                
            if point_index != None:
                distance_from_last_point_to_lane_start = 0.5 * point_index        
                agent.distance_to_lane_start_at_ending = distance_from_last_point_to_lane_start
            else:
                agent.distance_to_lane_end_at_beginning = "None"
        

def calculate_ROW(scenario):
    for agent in scenario.agents_list: 
        record_number = len(agent.matching_agent_time_list)

        # then process the other elements
        record_index = 0
        
        last_signal_light_shape_in_current_lane = "None"
        last_signal_light_type_in_current_lane = "None"
        last_signal_light_state_in_current_lane = "None" 
        last_signal_light_distance_in_current_lane = "None"      
     
        while record_index < record_number: 

            lane_id = agent.unique_matching_agent_lane_id_list[record_index]

            if lane_id == "" or lane_id == None or lane_id == "None":
                agent.ROW_list.append("None")
                record_index += 1
                continue
  
            map_matching_lane = scenario.center_lane_dict[lane_id]            

            signal_light_shape_in_current_lane = agent.current_lane_signal_light_shape_list[record_index]
            signal_light_type_in_current_lane = agent.current_lane_signal_light_type_list[record_index]
            signal_light_state_in_current_lane = agent.current_lane_signal_light_state_list[record_index]
            signal_light_distance_in_current_lane = agent.current_lane_signal_light_distance_list[record_index]
        
            if signal_light_state_in_current_lane == "None":
                signal_light_state_in_current_lane = map_matching_lane.signal_light_color_dict[record_index]
                if signal_light_state_in_current_lane != "None":
                    signal_light_type_in_current_lane = "steady"
                    signal_light_shape_in_current_lane = "round"
                    signal_light_distance_in_current_lane = "None"           

            if signal_light_state_in_current_lane == "green":
                agent.ROW_list.append("protected")
            if signal_light_state_in_current_lane == "red" or signal_light_state_in_current_lane == "yellow":
                agent.ROW_list.append("yield")    
            if signal_light_state_in_current_lane == "None":
                agent.ROW_list.append("None")
                
            record_index += 1


def handle_map_matching_missing_data(scenario, scenario_index, dataset_index):   
    
    scenario_left_turn_number = 0
    scenario_right_turn_number = 0
    scenario_lane_change_number = 0
    
    turn_type_output_number_dict = {}
    turn_type_output_number_dict["right_turn"] = 0
    turn_type_output_number_dict["left_turn"] = 0
    turn_type_output_number_dict["u_turn"] = 0
    turn_type_output_number_dict["straight"] = 0
        
    # go through all the agents in this scenario
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type
        
        if agent_type == -1:
            agent_type = "AV"
        if agent_type == 0:
            agent_type = "UNSET"
        if agent_type == 1:
            agent_type = "HDV"
        if agent_type == 2:
            agent_type = "PEDESTRAIN"
        if agent_type == 3:
            agent_type = "CYCLIST"
        if agent_type == 4:
            agent_type = "OTHER"             

        agent.agent_type = agent_type

        # find turn type / movement type
        turn_type = None
        right_turn_count = 0
        left_turn_count = 0
        u_turn_count = 0
        straight_count = 0
                    
        for lane_type_list in agent.matching_agent_lane_type_list:
            for lane_type in lane_type_list:
                if lane_type == "RIGHT_TURN_LANE":
                    right_turn_count += 1
                if lane_type == "LEFT_TURN_LANE":
                    left_turn_count += 1              
                if lane_type == "U_TURN_LANE":
                    u_turn_count += 1                
                if lane_type == "THROUGH_LANE":
                    straight_count += 1
    
        if right_turn_count > 15:
            turn_type = "RIGHT_TURN"
            scenario_right_turn_number += 1
        elif left_turn_count > 15:
            turn_type = "LEFT_TURN"
            scenario_left_turn_number += 1
        elif u_turn_count > 15:
            turn_type = "U_TURN"
        else:
            turn_type = "THROUGH"
                    
        # find if it is lane change
        has_lane_change = False
        for lane_change in agent.left_lane_change_list:
            if lane_change != "False":
                has_lane_change = True
                break

        for lane_change in agent.right_lane_change_list:
            if lane_change != "False":
                has_lane_change = True
                break           

        if has_lane_change:
            scenario_lane_change_number += 1

        record_number = len(agent.matching_agent_time_list)
        record_index = 0
        
        first_lane_id = None
        last_lane_id = None
        last_map_matching_lane_id = None
        total_frame = 0
        
        origin_link_id = "None"
        destination_link_id = "None"
        origin_lane_id = "None"
        destination_lane_id = "None"
        link_path_list = []
        lane_path_list = []
        
        agent_length_list = []
        agent_width_list = []
        agent_height_list = []
        agent_movement_list = []

        # first process origin link/lane and destination link/lane
        last_lane_id = None
        current_lane_id = None
        lane_id_change = False
        lane_change_flag = False
        
        while record_index < record_number:   
            
            vehicle_length = agent.matching_agent_length_list[record_index] 
            vehicle_width  = agent.matching_agent_width_list[record_index] 
            vehicle_height = agent.matching_agent_height_list[record_index]            
            agent_movement_list.append(turn_type)
            
            x = agent.matching_agent_x_list[record_index]
            y = agent.matching_agent_y_list[record_index]
            z = agent.matching_agent_z_list[record_index]   
            
            if x == 0 and y == 0:
                record_index += 1
                agent.curvature_list.append("None")
                agent.grade_list.append("None")
                continue
            
                           
            lane_id = agent.unique_matching_agent_lane_id_list[record_index]          
            current_lane_id = lane_id
            if lane_id == "" or lane_id == None or lane_id == "None":
                record_index += 1
                agent.curvature_list.append("None")
                continue
            
            map_matching_lane = scenario.center_lane_dict[lane_id]
            
            if last_lane_id != "None":
                if last_lane_id != current_lane_id:
                    lane_id_change = True
                else:
                    lane_id_change = False
            
            curren_lane_curvature = map_matching_lane.lane_average_curvature
            agent.curvature_list.append(curren_lane_curvature)  
                      
            
            if vehicle_length != "" and vehicle_length != None and vehicle_length != "None":
                agent_length_list.append(vehicle_length)
            if vehicle_width != "" and vehicle_width != None and vehicle_width != "None":
                agent_width_list.append(vehicle_width)                
            if vehicle_height != "" and vehicle_height != None and vehicle_height != "None":
                agent_height_list.append(vehicle_height)
  
            total_frame += 1
            
            if first_lane_id == None or origin_link_id == "None":
                first_lane_id = lane_id
                origin_lane_id = lane_id
                first_lane = scenario.center_lane_dict[origin_lane_id]
                origin_link_id = first_lane.link_id
                                       
            if lane_id not in lane_path_list:
                lane_path_list.append(lane_id)
                this_lane = scenario.center_lane_dict[lane_id]
                this_lane_link_id = this_lane.link_id
                if this_lane_link_id != "None" and this_lane_link_id != None and this_lane_link_id != "":
                    if this_lane_link_id not in link_path_list:          
                        link_path_list.append(this_lane_link_id)
            
            destination_lane_id = lane_id
            last_lane = scenario.center_lane_dict[destination_lane_id]
            if last_lane.link_id != "None" and last_lane.link_id != None and last_lane.link_id != "":
                destination_link_id = last_lane.link_id
                            
            last_lane_id = current_lane_id
            record_index += 1
        
        if len(agent_length_list) != 0:
            agent.average_length = calculate_list_average(agent_length_list)
        else:
            agent.average_length = "None"
            
        if len(agent_length_list) != 0: 
            agent.average_width  = calculate_list_average(agent_width_list)
        else:
            agent.average_width = "None"
        if len(agent_length_list) != 0:        
            agent.average_height = calculate_list_average(agent_height_list)
        else:
            agent.average_height = "None"
        
        agent.origin_link_id = origin_link_id
        agent.destination_link_id = destination_link_id
        agent.origin_lane_id = origin_lane_id
        agent.destination_lane_id = destination_lane_id       
        agent.link_path_list = link_path_list
        agent.lane_path_list = lane_path_list
        
        agent.scenario_id = scenario_index
        agent.dataset_id  = dataset_index
        agent.total_frame = total_frame
        agent.movement_type_list = agent_movement_list


                                                                   

def output_map_matching_trajectories_csv(output_file_path, output_statistics_path, scenario, enconding = None, output_turn_type = None, dataset_index = None, scenario_index = None, output_csv_to_one_file = None):   
    
    if output_file_path:
        if not os.path.exists(output_file_path): os.mkdir(output_file_path)

    scenario_left_turn_number = 0
    scenario_right_turn_number = 0
    scenario_lane_change_number = 0
    
    turn_type_output_number_dict = {}
    turn_type_output_number_dict["right_turn"] = 0
    turn_type_output_number_dict["left_turn"] = 0
    turn_type_output_number_dict["u_turn"] = 0
    turn_type_output_number_dict["straight"] = 0
        
    # go through all the agents in this scenario
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type

        # find if the car is standstill
        location_x_list = agent.matching_agent_x_list
        location_unique_x_list = np.unique(location_x_list)
        if len(location_unique_x_list) < 20:
            continue
        
        if agent_type == -1:
            agent_type = "AV"
        if agent_type == 0:
            agent_type = "unset"
        if agent_type == 1:
            agent_type = "HDV"
        if agent_type == 2:
            agent_type = "pedestrain"
        if agent_type == 3:
            agent_type = "cyclist"
        if agent_type == 4:
            agent_type = "other"             

        # find turn type
        turn_type = None
        right_turn_count = 0
        left_turn_count = 0
        u_turn_count = 0
        straight_count = 0
                    
        for lane_type_list in agent.matching_agent_lane_type_list:
            for lane_type in lane_type_list:
                if lane_type == "right_turn":
                    right_turn_count += 1
                if lane_type == "left_turn":
                    left_turn_count += 1              
                if lane_type == "u_turn":
                    u_turn_count += 1                
                if lane_type == "straight":
                    straight_count += 1
    
        if right_turn_count > 15:
            turn_type = "right_turn"
            scenario_right_turn_number += 1
        elif left_turn_count > 15:
            turn_type = "left_turn"
            scenario_left_turn_number += 1
        elif u_turn_count > 15:
            turn_type = "u_turn"
        else:
            turn_type = "straight"
                    
        # find if it is lane change
        has_lane_change = False
        for lane_change in agent.left_lane_change_list:
            if lane_change != "False":
                has_lane_change = True
                break

        for lane_change in agent.right_lane_change_list:
            if lane_change != "False":
                has_lane_change = True
                break           

        if has_lane_change:
            scenario_lane_change_number += 1

        # write other info
        if output_turn_type == None:
            
            output_csv_name = "map_matching_agent_" + agent_type + "_" + str(agent_id) + ".csv"
            output_filepath = os.path.join(output_file_path, output_csv_name)
            
            while True:
                try:
                    if enconding is None:
                        outfile = open(output_filepath, 'w', newline='',errors='ignore')
                    else:
                        outfile = open(output_filepath, 'w', newline='', errors='ignore', encoding=enconding)
                    break
                except PermissionError:
                    print('csv may be locked by other programs. please release it then try again')
                    input()
                    
            writer = csv.writer(outfile)
            title_line = ["waymo_frame_ID", "Total_frames", "waymo_vehicle_id", "v_Class", "local_X", "local_Y", "local_Z", "v_Vel_x", "v_Vel_y", "v_Vel",
                          "v_Length",  "v_width", "v_height", "odometer",
                          "lane_id","position_to_lane_start", 
                          "position_to_lane_end", "signal_light_shape", "signal_light_type", "signal_light_state", "distance_to_signal_light",
                          "preceeding_vehicle_id", "preceeding_vehicle_spacing", "preceeding_vehicle_time_headway", "preceeding_vehicle_distance_to_ego_lane_end", "preceeding_vehicle_gap",
                          "following_vehicle_id", "following_vehicle_spacing", "following_vehicle_time_headway","following_vehicle_distance_to_ego_lane_end","following_vehicle_gap",
                          "left_lane_id", "left_parallel_vehicle_id", "relative_position", "left_parallel_vehicle_gap", "left_parallel_vehicle_spacing",
                          "left_parallel_vehicle_time_headway",
                          "right_lane_id", "right_parallel_vehicle_id", "relative_position", "right_parallel_vehicle_gap", "right_parallel_vehicle_spacing",
                          "right_parallel_vehicle_time_headway",
                          "left_lane_id", "left_lane_preceeding_vehicle_id", "left_lane_preceeding_vehicle_spacing", "left_lane_preceeding_vehicle_to_ego_lane_end", "left_lane_preceeding_vehicle_gap", 
                          "left_lane_preceeding_vehicle_time_headway",
                          "left_lane_following_vehicle_id", "left_lane_following_vehicle_spacing",  "left_lane_following_vehicle_to_ego_lane_end", "left_lane_following_vehicle_gap",
                          "left_lane_following_vehicle_time_headway",
                          "left_lane_spacing", "left_lane_change", 
                          "right_lane_id", "right_lane_preceeding_vehicle_id", "right_lane_preceeding_vehicle_spacing", "right_lane_preceeding_vehicle_to_ego_lane_end", "right_lane_preceeding_vehicle_gap",
                          "right_lane_preceeding_vehicle_time_headway",
                          "right_lane_following_vehicle_id", "right_lane_following_vehicle_spacing","right_lane_following_vehicle_to_ego_lane_end","right_lane_following_vehicle_gap",
                          "right_lane_following_vehicle_time_headway",
                          "right_lane_spacing", "right_lane_change",
                          "lane_id_set", "lane_type", "link_id", "node_id", "turn_movement_id",
                          "origin_link", "destination_link", "link_path", "origin_lane", "destination_lane", "lane_path",
                          "scenario_id", "detaset_id"]
            
            if scenario_index == 0:
                writer.writerow(title_line)
                       
            record_number = len(agent.matching_agent_time_list)
            record_index = 0
            last_map_matching_lane_id = None
            while record_index < record_number:          
                time = agent.matching_agent_time_list[record_index]
                total_frame = agent.matching_agent_total_frame
                x = agent.matching_agent_x_list[record_index]
                y = agent.matching_agent_y_list[record_index]
                z = agent.matching_agent_z_list[record_index]
                if x == 0 and y == 0:
                    record_index += 1
                    continue
                speed_x = agent.matching_agent_speed_x_list[record_index]  
                speed_y = agent.matching_agent_speed_y_list[record_index]  
                speed = agent.matching_agent_speed_list[record_index]  
                vehicle_length = agent.matching_agent_length_list[record_index] 
                vehicle_width = agent.matching_agent_width_list[record_index] 
                vehicle_height = agent.matching_agent_height_list[record_index]
                vehicle_acc = agent.acceleration_list[record_index]
                
                lane_id = agent.unique_matching_agent_lane_id_list[record_index]
                if lane_id == "None" and last_map_matching_lane_id != "None":
                    lane_id = last_map_matching_lane_id 
                position_to_lane_start = agent.distance_to_lane_start_list[record_index]
                position_to_lane_end = agent.distance_to_lane_end_list[record_index]
                odometer = agent.odometer_list[record_index]
                
                # current lane preceding vehicle
                preceeding_vehicle_id = agent.preceeding_vehicle_id_list[record_index]
                preceeding_vehicle_distance_to_ego_vehicle = agent.preceeding_vehicle_distance_list[record_index]
                preceeding_vehicle_time_headway = preceeding_vehicle_distance_to_ego_vehicle / speed
                preceeding_vehicle_distance_to_ego_lane_end = agent.preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                preceeding_vehicle_gap = agent.preceeding_vehicle_gap_list[record_index]
            
                # current lane following vehicle
                following_vehicle_id = agent.following_vehicle_id_list[record_index]
                following_vehicle_distance_to_ego_vehicle = agent.following_vehicle_distance_list[record_index]
                following_vehicle_time_headway = following_vehicle_distance_to_ego_vehicle / speed
                following_vehicle_distance_to_ego_lane_end = agent.following_vehicle_distance_to_ego_lane_end_list[record_index]
                following_vehicle_gap = agent.following_vehicle_gap_list[record_index]
                
                # left lane parallel vehicle
                left_parallel_vehicle_id = agent.left_parallel_vehicle_id_list[record_index]
                left_parallel_vehicle_gap = agent.left_parallel_vehicle_front_bumper_gap_list[record_index]
                left_parallel_vehicle_spacing = agent.left_parallel_vehicle_spacing_list[record_index]
                left_parallel_vehicle_time_headway = left_parallel_vehicle_spacing / speed
                left_parallel_vehicle_location = agent.left_parallel_vehicle_location_list[record_index]
                left_parallel_vehicle_relative = agent.left_parallel_vehicle_relative_location_list[record_index]
                
                left_parallel_vehicle = scenario.agents_dict[left_parallel_vehicle_id]
                left_parallel_vehicle_speed = left_parallel_vehicle.matching_agent_speed_list[left_parallel_vehicle_id]
                left_parallel_vehicle_acc = left_parallel_vehicle.acceleration_list[left_parallel_vehicle_id]
               
                # right lane parallel vehicle
                right_parallel_vehicle_id = agent.right_parallel_vehicle_id_list[record_index]
                right_parallel_vehicle_gap = agent.right_parallel_vehicle_front_bumper_gap_list[record_index]
                right_parallel_vehicle_spacing = agent.right_parallel_vehicle_spacing_list[record_index]
                right_parallel_vehicle_time_headway = right_parallel_vehicle_spacing / speed
                right_parallel_vehicle_location = agent.right_parallel_vehicle_location_list[record_index] 
                right_parallel_vehicle_relative = agent.right_parallel_vehicle_relative_location_list[record_index]
                
                right_parallel_vehicle = scenario.agents_dict[right_parallel_vehicle_id]
                right_parallel_vehicle_speed = right_parallel_vehicle.matching_agent_speed_list[right_parallel_vehicle_id]
                right_parallel_vehicle_acc = right_parallel_vehicle.acceleration_list[right_parallel_vehicle_id]
                
                # left lane preceding vehicle of the left lane parallel vehicle
                left_lane_id = agent.left_lane_id_list[record_index]
                left_lane_preceeding_vehicle_id = agent.left_preceeding_vehicle_id_list[record_index]
                left_lane_preceeding_distance_to_ego_vehicle = agent.left_preceeding_vehicle_distance_list[record_index]
                left_lane_preceeding_distance_to_ego_lane_end = agent.left_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                left_lane_preceeding_gap = agent.left_preceeding_vehicle_gap_list[record_index]
                left_lane_preceeding_time_headway = left_lane_preceeding_gap / left_parallel_vehicle_speed
                
                # left lane following vehicle of the left lane parallel vehicle
                left_lane_following_gap = agent.left_following_vehicle_gap_list[record_index]
                left_lane_following_vehicle_id = agent.left_following_vehicle_id_list[record_index]
                left_lane_following_distance_to_ego_vehicle = agent.left_following_vehicle_distance_list[record_index]
                left_lane_following_vehicle_to_ego_lane_end = agent.left_following_vehicle_distance_to_ego_lane_end_list[record_index]  
                left_lane_following_time_headway = left_lane_following_gap / left_parallel_vehicle_speed
            
                left_lane_spacing = agent.left_lane_spacing_list[record_index]
                left_lane_change = agent.left_lane_change_list[record_index]
            
                # right lane preceding vehicle of the left lane parallel vehicle
                right_lane_id = agent.right_lane_id_list[record_index]
                right_lane_preceeding_vehicle_id = agent.right_preceeding_vehicle_id_list[record_index]
                right_lane_preceeding_distance_to_ego_vehicle = agent.right_preceeding_vehicle_distance_list[record_index]
                right_lane_preceeding_distance_to_ego_lane_end = agent.right_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                right_lane_preceeding_gap = agent.right_preceeding_vehicle_gap_list[record_index]
                right_lane_preceeding_time_headway = right_lane_preceeding_gap / right_parallel_vehicle_speed
                
                # right lane following vehicle of the left lane parallel vehicle
                right_lane_following_gap = agent.right_following_vehicle_gap_list[record_index]
                right_lane_following_vehicle_id = agent.right_following_vehicle_id_list[record_index]
                right_lane_following_distance_to_ego_vehicle = agent.right_following_vehicle_distance_list[record_index]
                right_lane_following_vehicle_to_ego_lane_end = agent.right_following_vehicle_distance_to_ego_lane_end_list[record_index]
                right_lane_following_time_headway = right_lane_following_gap / right_parallel_vehicle_speed
                
                right_lane_spacing = agent.right_lane_spacing_list[record_index]
                right_lane_change = agent.right_lane_change_list[record_index]
                
                # topology 
                lane_id_list = agent.matching_agent_lane_id_list[record_index]
                lane_type_list = agent.matching_agent_lane_type_list[record_index]
                link_id = agent.matching_agent_link_id_list[record_index]
                node_id = agent.matching_agent_node_id_list[record_index]
                turn_id = agent.matching_agent_turn_movement_list[record_index]
                
                # signal states
                signal_light_shape_in_current_lane = agent.current_lane_signal_light_shape_list[record_index]
                signal_light_type_in_current_lane = agent.current_lane_signal_light_type_list[record_index]
                signal_light_state_in_current_lane = agent.current_lane_signal_light_state_list[record_index]
                signal_light_distance_in_current_lane = agent.current_lane_signal_light_distance_list[record_index]
                
                # total 32 ? elements
                csv_line = [time, total_frame, agent_id, agent_type, x, y, z, speed_x, speed_y, speed,    #7
                            vehicle_length, vehicle_width, vehicle_height, odometer, lane_id, 
                            position_to_lane_start, position_to_lane_end,        #3
                            signal_light_shape_in_current_lane,
                            signal_light_type_in_current_lane,
                            signal_light_state_in_current_lane,
                            signal_light_distance_in_current_lane,
                            preceeding_vehicle_id, preceeding_vehicle_distance_to_ego_vehicle, preceeding_vehicle_distance_to_ego_lane_end, preceeding_vehicle_gap, #3
                            preceeding_vehicle_time_headway,
                            following_vehicle_id, following_vehicle_distance_to_ego_vehicle, following_vehicle_distance_to_ego_lane_end,  following_vehicle_gap,   #3                               
                            following_vehicle_time_headway,
                            left_lane_id, left_parallel_vehicle_id, left_parallel_vehicle_relative, left_parallel_vehicle_gap, left_parallel_vehicle_spacing,
                            left_parallel_vehicle_time_headway,
                            right_lane_id, right_parallel_vehicle_id, right_parallel_vehicle_relative, right_parallel_vehicle_gap, right_parallel_vehicle_spacing, 
                            right_parallel_vehicle_time_headway,
                            left_lane_id, left_lane_preceeding_vehicle_id,   #2
                            left_lane_preceeding_distance_to_ego_vehicle, left_lane_preceeding_distance_to_ego_lane_end,  left_lane_preceeding_gap, #2
                            left_lane_preceeding_time_headway,
                            left_lane_following_vehicle_id, left_lane_following_distance_to_ego_vehicle,  #2
                            left_lane_following_vehicle_to_ego_lane_end, left_lane_following_gap,  #1
                            left_lane_following_time_headway,
                            left_lane_spacing, left_lane_change, #2
                            right_lane_id, right_lane_preceeding_vehicle_id,   #2
                            right_lane_preceeding_distance_to_ego_vehicle, right_lane_preceeding_distance_to_ego_lane_end, right_lane_preceeding_gap,#2
                            right_lane_preceeding_time_headway,
                            right_lane_following_vehicle_id, right_lane_following_distance_to_ego_vehicle,  #2
                            right_lane_following_vehicle_to_ego_lane_end, right_lane_following_gap, #1   
                            right_lane_following_time_headway,
                            right_lane_spacing, right_lane_change,                
                            lane_id_list, lane_type_list, link_id, node_id, turn_id]   #1
                writer.writerow(csv_line)
                record_index += 1
                
        else:           
            if turn_type == output_turn_type:
                turn_type_output_number_dict[turn_type] += 1
                # put right turn together, left turn together ... 
                if output_csv_to_one_file == "True":
                    output_csv_name = None
                    
                    '''
                    if dataset_index != None and scenario_index != None:
                        output_csv_name = "dataset" + str(dataset_index) + "_scenario" + str(scenario_index) + "_" + agent_type + "_" + str(agent_id) + "_" + turn_type + ".csv"
                    else:
                        output_csv_name = "map_matching_agent_" + agent_type + "_" + str(agent_id) + "_" + turn_type + ".csv"
                    '''
                    
                    output_csv_name = "dataset" + str(dataset_index) + "_scenario" + str(scenario_index) + "_" + turn_type + "_total.csv"
                    output_filepath = os.path.join(output_file_path, output_csv_name)
                    
                    while True:
                        try:
                            if enconding is None:
                                outfile = open(output_filepath, 'a+', newline='',errors='ignore')
                            else:
                                outfile = open(output_filepath, 'a+', newline='', errors='ignore', encoding=enconding)
                            break
                        except PermissionError:
                            print('csv may be locked by other programs. please release it then try again')
                            input()
                            
                    writer = csv.writer(outfile)
                    # total 38 elements
                    title_line = ["waymo_frame_ID", "Total_frames", "waymo_vehicle_id", "v_Class", "local_X", "local_Y", "local_Z", "v_Vel_x", "v_Vel_y", "v_Vel",
                                  "v_Length",  "v_width", "v_height", "odometer",
                                  "lane_id","position_to_lane_start", 
                                  "position_to_lane_end", "signal_light_shape", "signal_light_type", "signal_light_state", "distance_to_signal_light",
                                  "preceeding_vehicle_id", "preceeding_vehicle_spacing", "preceeding_vehicle_time_headway", "preceeding_vehicle_distance_to_ego_lane_end", "preceeding_vehicle_gap",
                                  "following_vehicle_id", "following_vehicle_spacing", "following_vehicle_time_headway","following_vehicle_distance_to_ego_lane_end","following_vehicle_gap",
                                  "left_lane_id", "left_parallel_vehicle_id", "relative_position", "left_parallel_vehicle_gap", "left_parallel_vehicle_spacing",
                                  "left_parallel_vehicle_time_headway",
                                  "right_lane_id", "right_parallel_vehicle_id", "relative_position", "right_parallel_vehicle_gap", "right_parallel_vehicle_spacing",
                                  "right_parallel_vehicle_time_headway",
                                  "left_lane_id", "left_lane_preceeding_vehicle_id", "left_lane_preceeding_vehicle_spacing", "left_lane_preceeding_vehicle_to_ego_lane_end", "left_lane_preceeding_vehicle_gap", 
                                  "left_lane_preceeding_vehicle_time_headway",
                                  "left_lane_following_vehicle_id", "left_lane_following_vehicle_spacing",  "left_lane_following_vehicle_to_ego_lane_end", "left_lane_following_vehicle_gap",
                                  "left_lane_following_vehicle_time_headway",
                                  "left_lane_spacing", "left_lane_change", 
                                  "right_lane_id", "right_lane_preceeding_vehicle_id", "right_lane_preceeding_vehicle_spacing", "right_lane_preceeding_vehicle_to_ego_lane_end", "right_lane_preceeding_vehicle_gap",
                                  "right_lane_preceeding_vehicle_time_headway",
                                  "right_lane_following_vehicle_id", "right_lane_following_vehicle_spacing","right_lane_following_vehicle_to_ego_lane_end","right_lane_following_vehicle_gap",
                                  "right_lane_following_vehicle_time_headway",
                                  "right_lane_spacing", "right_lane_change",
                                  "lane_id_set", "lane_type", "link_id", "node_id", "turn_movement_id",
                                  "origin_link", "destination_link", "link_path", "origin_lane", "destination_lane", "lane_path",
                                  "scenario_id", "detaset_id"]
                    
                    if scenario_index == 0:
                        writer.writerow(title_line)
                               
                    record_number = len(agent.matching_agent_time_list)
                    record_index = 0
                    last_map_matching_lane_id = None
                    while record_index < record_number:          
                        time = agent.matching_agent_time_list[record_index]
                        total_frame = agent.matching_agent_total_frame
                        x = agent.matching_agent_x_list[record_index]
                        y = agent.matching_agent_y_list[record_index]
                        z = agent.matching_agent_z_list[record_index]
                        if x == 0 and y == 0:
                            record_index += 1
                            continue
                        speed_x = agent.matching_agent_speed_x_list[record_index]  
                        speed_y = agent.matching_agent_speed_y_list[record_index]  
                        speed = agent.matching_agent_speed_list[record_index]  
                        vehicle_length = agent.matching_agent_length_list[record_index] 
                        vehicle_width = agent.matching_agent_width_list[record_index] 
                        vehicle_height = agent.matching_agent_height_list[record_index]
                        vehicle_acc = agent.acceleration_list[record_index]
                        
                        lane_id = agent.unique_matching_agent_lane_id_list[record_index]
                        if lane_id == "None" and last_map_matching_lane_id != "None":
                            lane_id = last_map_matching_lane_id 
                        if lane_id == "" and last_map_matching_lane_id == "" or lane_id == None and last_map_matching_lane_id == None:
                            record_index += 1
                            continue
                            
                        position_to_lane_start = agent.distance_to_lane_start_list[record_index]
                        position_to_lane_end = agent.distance_to_lane_end_list[record_index]
                        odometer = agent.odometer_list[record_index]
                        
                        # current lane preceding vehicle
                        preceeding_vehicle_id = agent.preceeding_vehicle_id_list[record_index]
                        preceeding_vehicle_distance_to_ego_vehicle = agent.preceeding_vehicle_distance_list[record_index]
                        if preceeding_vehicle_distance_to_ego_vehicle == "None":
                            preceeding_vehicle_time_headway = "None"
                        else:
                            preceeding_vehicle_time_headway = preceeding_vehicle_distance_to_ego_vehicle / speed
                        preceeding_vehicle_distance_to_ego_lane_end = agent.preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                        preceeding_vehicle_gap = agent.preceeding_vehicle_gap_list[record_index]
                    
                        # current lane following vehicle
                        following_vehicle_id = agent.following_vehicle_id_list[record_index]
                        following_vehicle_distance_to_ego_vehicle = agent.following_vehicle_distance_list[record_index]
                        if following_vehicle_distance_to_ego_vehicle == "None":
                            following_vehicle_time_headway = "None"
                        else:                
                            following_vehicle_time_headway = following_vehicle_distance_to_ego_vehicle / speed
                        following_vehicle_distance_to_ego_lane_end = agent.following_vehicle_distance_to_ego_lane_end_list[record_index]
                        following_vehicle_gap = agent.following_vehicle_gap_list[record_index]
                        
                        # left lane parallel vehicle
                        left_parallel_vehicle_id = agent.left_parallel_vehicle_id_list[record_index]
                        left_parallel_vehicle_gap = agent.left_parallel_vehicle_front_bumper_gap_list[record_index]
                        left_parallel_vehicle_spacing = agent.left_parallel_vehicle_spacing_list[record_index]
                        if left_parallel_vehicle_spacing == "None":
                            left_parallel_vehicle_time_headway = "None"
                        else:
                            left_parallel_vehicle_time_headway = left_parallel_vehicle_spacing / speed
                        left_parallel_vehicle_location = agent.left_parallel_vehicle_location_list[record_index]
                        left_parallel_vehicle_relative = agent.left_parallel_vehicle_relative_location_list[record_index]
                        
                        left_parallel_vehicle_speed = "None"
                        left_parallel_vehicle_acc = "None"
                        if left_parallel_vehicle_id != "None":
                            left_parallel_vehicle_id = int(left_parallel_vehicle_id)
                            left_parallel_vehicle = scenario.agents_dict[left_parallel_vehicle_id]
                            left_parallel_vehicle_speed = left_parallel_vehicle.matching_agent_speed_list[record_index]
                            left_parallel_vehicle_acc = left_parallel_vehicle.acceleration_list[record_index]
                       
                        # right lane parallel vehicle
                        right_parallel_vehicle_id = agent.right_parallel_vehicle_id_list[record_index]
                        right_parallel_vehicle_gap = agent.right_parallel_vehicle_front_bumper_gap_list[record_index]
                        right_parallel_vehicle_spacing = agent.right_parallel_vehicle_spacing_list[record_index]
                        if right_parallel_vehicle_spacing == "None":
                            right_parallel_vehicle_time_headway = "None"
                        else:                
                            right_parallel_vehicle_time_headway = right_parallel_vehicle_spacing / speed
                        right_parallel_vehicle_location = agent.right_parallel_vehicle_location_list[record_index] 
                        right_parallel_vehicle_relative = agent.right_parallel_vehicle_relative_location_list[record_index]
                        
                        right_parallel_vehicle_speed = "None"
                        right_parallel_vehicle_acc = "None"
                        if right_parallel_vehicle_id != "None":
                            right_parallel_vehicle_id = int(right_parallel_vehicle_id)
                            right_parallel_vehicle = scenario.agents_dict[right_parallel_vehicle_id]
                            right_parallel_vehicle_speed = right_parallel_vehicle.matching_agent_speed_list[record_index]
                            right_parallel_vehicle_acc = right_parallel_vehicle.acceleration_list[record_index]
                        
                        # left lane preceding vehicle of the left lane parallel vehicle
                        left_lane_id = agent.left_lane_id_list[record_index]
                        left_lane_preceeding_vehicle_id = agent.left_preceeding_vehicle_id_list[record_index]
                        left_lane_preceeding_distance_to_ego_vehicle = agent.left_preceeding_vehicle_distance_list[record_index]
                        left_lane_preceeding_distance_to_ego_lane_end = agent.left_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                        left_lane_preceeding_gap = agent.left_preceeding_vehicle_gap_list[record_index]
                        left_lane_preceeding_time_headway = "None"
                        if left_lane_preceeding_gap != "None" and left_parallel_vehicle_speed != "None":
                            left_lane_preceeding_time_headway = left_lane_preceeding_gap / left_parallel_vehicle_speed
                        
                        # left lane following vehicle of the left lane parallel vehicle
                        left_lane_following_gap = agent.left_following_vehicle_gap_list[record_index]
                        left_lane_following_vehicle_id = agent.left_following_vehicle_id_list[record_index]
                        left_lane_following_distance_to_ego_vehicle = agent.left_following_vehicle_distance_list[record_index]
                        left_lane_following_vehicle_to_ego_lane_end = agent.left_following_vehicle_distance_to_ego_lane_end_list[record_index]  
                        left_lane_following_time_headway = "None"
                        if left_lane_following_gap != "None" and left_parallel_vehicle_speed != "None":                   
                            left_lane_following_time_headway = left_lane_following_gap / left_parallel_vehicle_speed
                    
                        left_lane_spacing = agent.left_lane_spacing_list[record_index]
                        left_lane_change = agent.left_lane_change_list[record_index]
                    
                        # right lane preceding vehicle of the left lane parallel vehicle
                        right_lane_id = agent.right_lane_id_list[record_index]
                        right_lane_preceeding_vehicle_id = agent.right_preceeding_vehicle_id_list[record_index]
                        right_lane_preceeding_distance_to_ego_vehicle = agent.right_preceeding_vehicle_distance_list[record_index]
                        right_lane_preceeding_distance_to_ego_lane_end = agent.right_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                        right_lane_preceeding_gap = agent.right_preceeding_vehicle_gap_list[record_index]
                        right_lane_preceeding_time_headway = "None"
                        if right_lane_preceeding_gap != "None" and right_parallel_vehicle_speed != "None": 
                            right_lane_preceeding_time_headway = right_lane_preceeding_gap / right_parallel_vehicle_speed
                        
                        # right lane following vehicle of the left lane parallel vehicle
                        right_lane_following_gap = agent.right_following_vehicle_gap_list[record_index]
                        right_lane_following_vehicle_id = agent.right_following_vehicle_id_list[record_index]
                        right_lane_following_distance_to_ego_vehicle = agent.right_following_vehicle_distance_list[record_index]
                        right_lane_following_vehicle_to_ego_lane_end = agent.right_following_vehicle_distance_to_ego_lane_end_list[record_index]
                        right_lane_following_time_headway = "None"
                        if right_lane_following_gap != "None" and right_parallel_vehicle_speed != "None": 
                            right_lane_following_time_headway = right_lane_following_gap / right_parallel_vehicle_speed
                        
                        right_lane_spacing = agent.right_lane_spacing_list[record_index]
                        right_lane_change = agent.right_lane_change_list[record_index]
                        
                        # topology 
                        lane_id_list = agent.matching_agent_lane_id_list[record_index]
                        lane_type_list = agent.matching_agent_lane_type_list[record_index]
                        link_id = agent.matching_agent_link_id_list[record_index]
                        node_id = agent.matching_agent_node_id_list[record_index]
                        turn_id = agent.matching_agent_turn_movement_list[record_index]
                        
                        # signal states
                        signal_light_shape_in_current_lane = agent.current_lane_signal_light_shape_list[record_index]
                        signal_light_type_in_current_lane = agent.current_lane_signal_light_type_list[record_index]
                        signal_light_state_in_current_lane = agent.current_lane_signal_light_state_list[record_index]
                        signal_light_distance_in_current_lane = agent.current_lane_signal_light_distance_list[record_index]
                        
                        # total 32 ? elements
                        csv_line = [time, total_frame, agent_id, agent_type, x, y, z, speed_x, speed_y, speed,    #7
                                    vehicle_length, vehicle_width, vehicle_height, odometer, lane_id, 
                                    position_to_lane_start, position_to_lane_end,        #3
                                    signal_light_shape_in_current_lane,
                                    signal_light_type_in_current_lane,
                                    signal_light_state_in_current_lane,
                                    signal_light_distance_in_current_lane,
                                    preceeding_vehicle_id, preceeding_vehicle_distance_to_ego_vehicle, preceeding_vehicle_distance_to_ego_lane_end, preceeding_vehicle_gap, #3
                                    preceeding_vehicle_time_headway,
                                    following_vehicle_id, following_vehicle_distance_to_ego_vehicle, following_vehicle_distance_to_ego_lane_end,  following_vehicle_gap,   #3                               
                                    following_vehicle_time_headway,
                                    left_lane_id, left_parallel_vehicle_id, left_parallel_vehicle_relative, left_parallel_vehicle_gap, left_parallel_vehicle_spacing,
                                    left_parallel_vehicle_time_headway,
                                    right_lane_id, right_parallel_vehicle_id, right_parallel_vehicle_relative, right_parallel_vehicle_gap, right_parallel_vehicle_spacing, 
                                    right_parallel_vehicle_time_headway,
                                    left_lane_id, left_lane_preceeding_vehicle_id,   #2
                                    left_lane_preceeding_distance_to_ego_vehicle, left_lane_preceeding_distance_to_ego_lane_end,  left_lane_preceeding_gap, #2
                                    left_lane_preceeding_time_headway,
                                    left_lane_following_vehicle_id, left_lane_following_distance_to_ego_vehicle,  #2
                                    left_lane_following_vehicle_to_ego_lane_end, left_lane_following_gap,  #1
                                    left_lane_following_time_headway,
                                    left_lane_spacing, left_lane_change, #2
                                    right_lane_id, right_lane_preceeding_vehicle_id,   #2
                                    right_lane_preceeding_distance_to_ego_vehicle, right_lane_preceeding_distance_to_ego_lane_end, right_lane_preceeding_gap,#2
                                    right_lane_preceeding_time_headway,
                                    right_lane_following_vehicle_id, right_lane_following_distance_to_ego_vehicle,  #2
                                    right_lane_following_vehicle_to_ego_lane_end, right_lane_following_gap, #1   
                                    right_lane_following_time_headway,
                                    right_lane_spacing, right_lane_change,                
                                    lane_id_list, lane_type_list, link_id, node_id, turn_id]   #1
                        writer.writerow(csv_line)
                        record_index += 1                    
                # output single csv for different turn types
                else:
                    output_csv_name = None
                    if dataset_index != None and scenario_index != None:
                        output_csv_name = "dataset" + str(dataset_index) + "_scenario" + str(scenario_index) + "_" + agent_type + "_" + str(agent_id) + "_" + turn_type + ".csv"
                    else:
                        output_csv_name = "map_matching_agent_" + agent_type + "_" + str(agent_id) + "_" + turn_type + ".csv"
                    output_filepath = os.path.join(output_file_path, output_csv_name)
                    
                    while True:
                        try:
                            if enconding is None:
                                outfile = open(output_filepath, 'w', newline='',errors='ignore')
                            else:
                                outfile = open(output_filepath, 'w', newline='', errors='ignore', encoding=enconding)
                            break
                        except PermissionError:
                            print('csv may be locked by other programs. please release it then try again')
                            input()
                            
                    writer = csv.writer(outfile)
                    # total 38 elements
                    title_line = ["waymo_frame_ID", "Total_frames", "waymo_vehicle_id", "v_Class", "local_X", "local_Y", "local_Z", "v_Vel_x", "v_Vel_y", "v_Vel",
                                  "v_Length",  "v_width", "v_height", "odometer",
                                  "lane_id","position_to_lane_start", 
                                  "position_to_lane_end", "signal_light_shape", "signal_light_type", "signal_light_state", "distance_to_signal_light",
                                  "preceeding_vehicle_id", "preceeding_vehicle_spacing", "preceeding_vehicle_time_headway", "preceeding_vehicle_distance_to_ego_lane_end", "preceeding_vehicle_gap",
                                  "following_vehicle_id", "following_vehicle_spacing", "following_vehicle_time_headway","following_vehicle_distance_to_ego_lane_end","following_vehicle_gap",
                                  "left_lane_id", "left_parallel_vehicle_id", "relative_position", "left_parallel_vehicle_gap", "left_parallel_vehicle_spacing",
                                  "left_parallel_vehicle_time_headway",
                                  "right_lane_id", "right_parallel_vehicle_id", "relative_position", "right_parallel_vehicle_gap", "right_parallel_vehicle_spacing",
                                  "right_parallel_vehicle_time_headway",
                                  "left_lane_id", "left_lane_preceeding_vehicle_id", "left_lane_preceeding_vehicle_spacing", "left_lane_preceeding_vehicle_to_ego_lane_end", "left_lane_preceeding_vehicle_gap", 
                                  "left_lane_preceeding_vehicle_time_headway",
                                  "left_lane_following_vehicle_id", "left_lane_following_vehicle_spacing",  "left_lane_following_vehicle_to_ego_lane_end", "left_lane_following_vehicle_gap",
                                  "left_lane_following_vehicle_time_headway",
                                  "left_lane_spacing", "left_lane_change", 
                                  "right_lane_id", "right_lane_preceeding_vehicle_id", "right_lane_preceeding_vehicle_spacing", "right_lane_preceeding_vehicle_to_ego_lane_end", "right_lane_preceeding_vehicle_gap",
                                  "right_lane_preceeding_vehicle_time_headway",
                                  "right_lane_following_vehicle_id", "right_lane_following_vehicle_spacing","right_lane_following_vehicle_to_ego_lane_end","right_lane_following_vehicle_gap",
                                  "right_lane_following_vehicle_time_headway",
                                  "right_lane_spacing", "right_lane_change",
                                  "lane_id_set", "lane_type", "link_id", "node_id", "turn_movement_id",
                                  "origin_link", "destination_link", "link_path", "origin_lane", "destination_lane", "lane_path",
                                  "scenario_id", "detaset_id"]
                    
                    if scenario_index == 0:
                        writer.writerow(title_line)
                               
                    record_number = len(agent.matching_agent_time_list)
                    record_index = 0
                    last_map_matching_lane_id = None
                    while record_index < record_number:          
                        time = agent.matching_agent_time_list[record_index]
                        total_frame = agent.matching_agent_total_frame
                        x = agent.matching_agent_x_list[record_index]
                        y = agent.matching_agent_y_list[record_index]
                        z = agent.matching_agent_z_list[record_index]
                        if x == 0 and y == 0:
                            record_index += 1
                            continue
                        speed_x = agent.matching_agent_speed_x_list[record_index]  
                        speed_y = agent.matching_agent_speed_y_list[record_index]  
                        speed = agent.matching_agent_speed_list[record_index]  
                        vehicle_length = agent.matching_agent_length_list[record_index] 
                        vehicle_width = agent.matching_agent_width_list[record_index] 
                        vehicle_height = agent.matching_agent_height_list[record_index]
                        vehicle_acc = agent.acceleration_list[record_index]
                        
                        lane_id = agent.unique_matching_agent_lane_id_list[record_index]
                        if lane_id == "None" and last_map_matching_lane_id != "None":
                            lane_id = last_map_matching_lane_id 
                        if lane_id == "" and last_map_matching_lane_id == "" or lane_id == None and last_map_matching_lane_id == None:
                            record_index += 1
                            continue
                            
                        position_to_lane_start = agent.distance_to_lane_start_list[record_index]
                        position_to_lane_end = agent.distance_to_lane_end_list[record_index]
                        odometer = agent.odometer_list[record_index]
                        
                        # current lane preceding vehicle
                        preceeding_vehicle_id = agent.preceeding_vehicle_id_list[record_index]
                        preceeding_vehicle_distance_to_ego_vehicle = agent.preceeding_vehicle_distance_list[record_index]
                        if preceeding_vehicle_distance_to_ego_vehicle == "None":
                            preceeding_vehicle_time_headway = "None"
                        else:
                            preceeding_vehicle_time_headway = preceeding_vehicle_distance_to_ego_vehicle / speed
                        preceeding_vehicle_distance_to_ego_lane_end = agent.preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                        preceeding_vehicle_gap = agent.preceeding_vehicle_gap_list[record_index]
                    
                        # current lane following vehicle
                        following_vehicle_id = agent.following_vehicle_id_list[record_index]
                        following_vehicle_distance_to_ego_vehicle = agent.following_vehicle_distance_list[record_index]
                        if following_vehicle_distance_to_ego_vehicle == "None":
                            following_vehicle_time_headway = "None"
                        else:                
                            following_vehicle_time_headway = following_vehicle_distance_to_ego_vehicle / speed
                        following_vehicle_distance_to_ego_lane_end = agent.following_vehicle_distance_to_ego_lane_end_list[record_index]
                        following_vehicle_gap = agent.following_vehicle_gap_list[record_index]
                        
                        # left lane parallel vehicle
                        left_parallel_vehicle_id = agent.left_parallel_vehicle_id_list[record_index]
                        left_parallel_vehicle_gap = agent.left_parallel_vehicle_front_bumper_gap_list[record_index]
                        left_parallel_vehicle_spacing = agent.left_parallel_vehicle_spacing_list[record_index]
                        if left_parallel_vehicle_spacing == "None":
                            left_parallel_vehicle_time_headway = "None"
                        else:
                            left_parallel_vehicle_time_headway = left_parallel_vehicle_spacing / speed
                        left_parallel_vehicle_location = agent.left_parallel_vehicle_location_list[record_index]
                        left_parallel_vehicle_relative = agent.left_parallel_vehicle_relative_location_list[record_index]
                        
                        left_parallel_vehicle_speed = "None"
                        left_parallel_vehicle_acc = "None"
                        if left_parallel_vehicle_id != "None":
                            left_parallel_vehicle_id = int(left_parallel_vehicle_id)
                            left_parallel_vehicle = scenario.agents_dict[left_parallel_vehicle_id]
                            left_parallel_vehicle_speed = left_parallel_vehicle.matching_agent_speed_list[record_index]
                            left_parallel_vehicle_acc = left_parallel_vehicle.acceleration_list[record_index]
                       
                        # right lane parallel vehicle
                        right_parallel_vehicle_id = agent.right_parallel_vehicle_id_list[record_index]
                        right_parallel_vehicle_gap = agent.right_parallel_vehicle_front_bumper_gap_list[record_index]
                        right_parallel_vehicle_spacing = agent.right_parallel_vehicle_spacing_list[record_index]
                        if right_parallel_vehicle_spacing == "None":
                            right_parallel_vehicle_time_headway = "None"
                        else:                
                            right_parallel_vehicle_time_headway = right_parallel_vehicle_spacing / speed
                        right_parallel_vehicle_location = agent.right_parallel_vehicle_location_list[record_index] 
                        right_parallel_vehicle_relative = agent.right_parallel_vehicle_relative_location_list[record_index]
                        
                        right_parallel_vehicle_speed = "None"
                        right_parallel_vehicle_acc = "None"
                        if right_parallel_vehicle_id != "None":
                            right_parallel_vehicle_id = int(right_parallel_vehicle_id)
                            right_parallel_vehicle = scenario.agents_dict[right_parallel_vehicle_id]
                            right_parallel_vehicle_speed = right_parallel_vehicle.matching_agent_speed_list[record_index]
                            right_parallel_vehicle_acc = right_parallel_vehicle.acceleration_list[record_index]
                        
                        # left lane preceding vehicle of the left lane parallel vehicle
                        left_lane_id = agent.left_lane_id_list[record_index]
                        left_lane_preceeding_vehicle_id = agent.left_preceeding_vehicle_id_list[record_index]
                        left_lane_preceeding_distance_to_ego_vehicle = agent.left_preceeding_vehicle_distance_list[record_index]
                        left_lane_preceeding_distance_to_ego_lane_end = agent.left_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                        left_lane_preceeding_gap = agent.left_preceeding_vehicle_gap_list[record_index]
                        left_lane_preceeding_time_headway = "None"
                        if left_lane_preceeding_gap != "None" and left_parallel_vehicle_speed != "None":
                            left_lane_preceeding_time_headway = left_lane_preceeding_gap / left_parallel_vehicle_speed
                        
                        # left lane following vehicle of the left lane parallel vehicle
                        left_lane_following_gap = agent.left_following_vehicle_gap_list[record_index]
                        left_lane_following_vehicle_id = agent.left_following_vehicle_id_list[record_index]
                        left_lane_following_distance_to_ego_vehicle = agent.left_following_vehicle_distance_list[record_index]
                        left_lane_following_vehicle_to_ego_lane_end = agent.left_following_vehicle_distance_to_ego_lane_end_list[record_index]  
                        left_lane_following_time_headway = "None"
                        if left_lane_following_gap != "None" and left_parallel_vehicle_speed != "None":                   
                            left_lane_following_time_headway = left_lane_following_gap / left_parallel_vehicle_speed
                    
                        left_lane_spacing = agent.left_lane_spacing_list[record_index]
                        left_lane_change = agent.left_lane_change_list[record_index]
                    
                        # right lane preceding vehicle of the left lane parallel vehicle
                        right_lane_id = agent.right_lane_id_list[record_index]
                        right_lane_preceeding_vehicle_id = agent.right_preceeding_vehicle_id_list[record_index]
                        right_lane_preceeding_distance_to_ego_vehicle = agent.right_preceeding_vehicle_distance_list[record_index]
                        right_lane_preceeding_distance_to_ego_lane_end = agent.right_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                        right_lane_preceeding_gap = agent.right_preceeding_vehicle_gap_list[record_index]
                        right_lane_preceeding_time_headway = "None"
                        if right_lane_preceeding_gap != "None" and right_parallel_vehicle_speed != "None": 
                            right_lane_preceeding_time_headway = right_lane_preceeding_gap / right_parallel_vehicle_speed
                        
                        # right lane following vehicle of the left lane parallel vehicle
                        right_lane_following_gap = agent.right_following_vehicle_gap_list[record_index]
                        right_lane_following_vehicle_id = agent.right_following_vehicle_id_list[record_index]
                        right_lane_following_distance_to_ego_vehicle = agent.right_following_vehicle_distance_list[record_index]
                        right_lane_following_vehicle_to_ego_lane_end = agent.right_following_vehicle_distance_to_ego_lane_end_list[record_index]
                        right_lane_following_time_headway = "None"
                        if right_lane_following_gap != "None" and right_parallel_vehicle_speed != "None": 
                            right_lane_following_time_headway = right_lane_following_gap / right_parallel_vehicle_speed
                        
                        right_lane_spacing = agent.right_lane_spacing_list[record_index]
                        right_lane_change = agent.right_lane_change_list[record_index]
                        
                        # topology 
                        lane_id_list = agent.matching_agent_lane_id_list[record_index]
                        lane_type_list = agent.matching_agent_lane_type_list[record_index]
                        link_id = agent.matching_agent_link_id_list[record_index]
                        node_id = agent.matching_agent_node_id_list[record_index]
                        turn_id = agent.matching_agent_turn_movement_list[record_index]
                        
                        # signal states
                        signal_light_shape_in_current_lane = agent.current_lane_signal_light_shape_list[record_index]
                        signal_light_type_in_current_lane = agent.current_lane_signal_light_type_list[record_index]
                        signal_light_state_in_current_lane = agent.current_lane_signal_light_state_list[record_index]
                        signal_light_distance_in_current_lane = agent.current_lane_signal_light_distance_list[record_index]
                        
                        # total 32 ? elements
                        csv_line = [time, total_frame, agent_id, agent_type, x, y, z, speed_x, speed_y, speed,    #7
                                    vehicle_length, vehicle_width, vehicle_height, odometer, lane_id, 
                                    position_to_lane_start, position_to_lane_end,        #3
                                    signal_light_shape_in_current_lane,
                                    signal_light_type_in_current_lane,
                                    signal_light_state_in_current_lane,
                                    signal_light_distance_in_current_lane,
                                    preceeding_vehicle_id, preceeding_vehicle_distance_to_ego_vehicle, preceeding_vehicle_distance_to_ego_lane_end, preceeding_vehicle_gap, #3
                                    preceeding_vehicle_time_headway,
                                    following_vehicle_id, following_vehicle_distance_to_ego_vehicle, following_vehicle_distance_to_ego_lane_end,  following_vehicle_gap,   #3                               
                                    following_vehicle_time_headway,
                                    left_lane_id, left_parallel_vehicle_id, left_parallel_vehicle_relative, left_parallel_vehicle_gap, left_parallel_vehicle_spacing,
                                    left_parallel_vehicle_time_headway,
                                    right_lane_id, right_parallel_vehicle_id, right_parallel_vehicle_relative, right_parallel_vehicle_gap, right_parallel_vehicle_spacing, 
                                    right_parallel_vehicle_time_headway,
                                    left_lane_id, left_lane_preceeding_vehicle_id,   #2
                                    left_lane_preceeding_distance_to_ego_vehicle, left_lane_preceeding_distance_to_ego_lane_end,  left_lane_preceeding_gap, #2
                                    left_lane_preceeding_time_headway,
                                    left_lane_following_vehicle_id, left_lane_following_distance_to_ego_vehicle,  #2
                                    left_lane_following_vehicle_to_ego_lane_end, left_lane_following_gap,  #1
                                    left_lane_following_time_headway,
                                    left_lane_spacing, left_lane_change, #2
                                    right_lane_id, right_lane_preceeding_vehicle_id,   #2
                                    right_lane_preceeding_distance_to_ego_vehicle, right_lane_preceeding_distance_to_ego_lane_end, right_lane_preceeding_gap,#2
                                    right_lane_preceeding_time_headway,
                                    right_lane_following_vehicle_id, right_lane_following_distance_to_ego_vehicle,  #2
                                    right_lane_following_vehicle_to_ego_lane_end, right_lane_following_gap, #1   
                                    right_lane_following_time_headway,
                                    right_lane_spacing, right_lane_change,                
                                    lane_id_list, lane_type_list, link_id, node_id, turn_id]   #1
                        writer.writerow(csv_line)
                        record_index += 1

            if has_lane_change:
                output_csv_name = None
                if dataset_index != None and scenario_index != None:
                    output_csv_name = "dataset" + str(dataset_index) + "_scenario" + str(scenario_index) + "_" + agent_type + "_" + str(agent_id) + "_lane_change" + ".csv"
                else:
                    output_csv_name = "map_matching_agent_" + agent_type + "_" + str(agent_id) + "_lane_change" + ".csv"
                output_filepath = os.path.join(output_file_path, output_csv_name)
                
                while True:
                    try:
                        if enconding is None:
                            outfile = open(output_filepath, 'w', newline='',errors='ignore')
                        else:
                            outfile = open(output_filepath, 'w', newline='', errors='ignore', encoding=enconding)
                        break
                    except PermissionError:
                        print('csv may be locked by other programs. please release it then try again')
                        input()
                        
                writer = csv.writer(outfile)
                # total 38 elements
                title_line = ["waymo_frame_ID", "Total_frames", "waymo_vehicle_id", "v_Class", "local_X", "local_Y", "local_Z", "v_Vel_x", "v_Vel_y", "v_Vel",
                              "v_Length",  "v_width", "v_height", "odometer",
                              "lane_id","position_to_lane_start", 
                              "position_to_lane_end", "signal_light_shape", "signal_light_type", "signal_light_state", "distance_to_signal_light",
                              "preceeding_vehicle_id", "preceeding_vehicle_spacing", "preceeding_vehicle_time_headway", "preceeding_vehicle_distance_to_ego_lane_end", "preceeding_vehicle_gap",
                              "following_vehicle_id", "following_vehicle_spacing", "following_vehicle_time_headway","following_vehicle_distance_to_ego_lane_end","following_vehicle_gap",
                              "left_lane_id", "left_parallel_vehicle_id", "relative_position", "left_parallel_vehicle_gap", "left_parallel_vehicle_spacing",
                              "left_parallel_vehicle_time_headway",
                              "right_lane_id", "right_parallel_vehicle_id", "relative_position", "right_parallel_vehicle_gap", "right_parallel_vehicle_spacing",
                              "right_parallel_vehicle_time_headway",
                              "left_lane_id", "left_lane_preceeding_vehicle_id", "left_lane_preceeding_vehicle_spacing", "left_lane_preceeding_vehicle_to_ego_lane_end", "left_lane_preceeding_vehicle_gap", 
                              "left_lane_preceeding_vehicle_time_headway",
                              "left_lane_following_vehicle_id", "left_lane_following_vehicle_spacing",  "left_lane_following_vehicle_to_ego_lane_end", "left_lane_following_vehicle_gap",
                              "left_lane_following_vehicle_time_headway",
                              "left_lane_spacing", "left_lane_change", 
                              "right_lane_id", "right_lane_preceeding_vehicle_id", "right_lane_preceeding_vehicle_spacing", "right_lane_preceeding_vehicle_to_ego_lane_end", "right_lane_preceeding_vehicle_gap",
                              "right_lane_preceeding_vehicle_time_headway",
                              "right_lane_following_vehicle_id", "right_lane_following_vehicle_spacing","right_lane_following_vehicle_to_ego_lane_end","right_lane_following_vehicle_gap",
                              "right_lane_following_vehicle_time_headway",
                              "right_lane_spacing", "right_lane_change",
                              "lane_id_set", "lane_type", "link_id", "node_id", "turn_movement_id",
                              "origin_link", "destination_link", "link_path", "origin_lane", "destination_lane", "lane_path",
                              "scenario_id", "detaset_id"]
                
                if scenario_index == 0:
                    writer.writerow(title_line)
                           
                record_number = len(agent.matching_agent_time_list)
                record_index = 0
                last_map_matching_lane_id = None
                while record_index < record_number:          
                    time = agent.matching_agent_time_list[record_index]
                    total_frame = agent.matching_agent_total_frame
                    x = agent.matching_agent_x_list[record_index]
                    y = agent.matching_agent_y_list[record_index]
                    z = agent.matching_agent_z_list[record_index]
                    if x == 0 and y == 0:
                        record_index += 1
                        continue
                    speed_x = agent.matching_agent_speed_x_list[record_index]  
                    speed_y = agent.matching_agent_speed_y_list[record_index]  
                    speed = agent.matching_agent_speed_list[record_index]  
                    vehicle_length = agent.matching_agent_length_list[record_index] 
                    vehicle_width = agent.matching_agent_width_list[record_index] 
                    vehicle_height = agent.matching_agent_height_list[record_index]
                    vehicle_acc = agent.acceleration_list[record_index]
                    
                    lane_id = agent.unique_matching_agent_lane_id_list[record_index]
                    if lane_id == "None" and last_map_matching_lane_id != "None":
                        lane_id = last_map_matching_lane_id 
                    if lane_id == "" and last_map_matching_lane_id == "" or lane_id == None and last_map_matching_lane_id == None:
                        record_index += 1
                        continue
                        
                    position_to_lane_start = agent.distance_to_lane_start_list[record_index]
                    position_to_lane_end = agent.distance_to_lane_end_list[record_index]
                    odometer = agent.odometer_list[record_index]
                    
                    # current lane preceding vehicle
                    preceeding_vehicle_id = agent.preceeding_vehicle_id_list[record_index]
                    preceeding_vehicle_distance_to_ego_vehicle = agent.preceeding_vehicle_distance_list[record_index]
                    if preceeding_vehicle_distance_to_ego_vehicle == "None":
                        preceeding_vehicle_time_headway = "None"
                    else:
                        preceeding_vehicle_time_headway = preceeding_vehicle_distance_to_ego_vehicle / speed
                    preceeding_vehicle_distance_to_ego_lane_end = agent.preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                    preceeding_vehicle_gap = agent.preceeding_vehicle_gap_list[record_index]
                
                    # current lane following vehicle
                    following_vehicle_id = agent.following_vehicle_id_list[record_index]
                    following_vehicle_distance_to_ego_vehicle = agent.following_vehicle_distance_list[record_index]
                    if following_vehicle_distance_to_ego_vehicle == "None":
                        following_vehicle_time_headway = "None"
                    else:                
                        following_vehicle_time_headway = following_vehicle_distance_to_ego_vehicle / speed
                    following_vehicle_distance_to_ego_lane_end = agent.following_vehicle_distance_to_ego_lane_end_list[record_index]
                    following_vehicle_gap = agent.following_vehicle_gap_list[record_index]
                    
                    # left lane parallel vehicle
                    left_parallel_vehicle_id = agent.left_parallel_vehicle_id_list[record_index]
                    left_parallel_vehicle_gap = agent.left_parallel_vehicle_front_bumper_gap_list[record_index]
                    left_parallel_vehicle_spacing = agent.left_parallel_vehicle_spacing_list[record_index]
                    if left_parallel_vehicle_spacing == "None":
                        left_parallel_vehicle_time_headway = "None"
                    else:
                        left_parallel_vehicle_time_headway = left_parallel_vehicle_spacing / speed
                    left_parallel_vehicle_location = agent.left_parallel_vehicle_location_list[record_index]
                    left_parallel_vehicle_relative = agent.left_parallel_vehicle_relative_location_list[record_index]
                    
                    left_parallel_vehicle_speed = "None"
                    left_parallel_vehicle_acc = "None"
                    if left_parallel_vehicle_id != "None":
                        left_parallel_vehicle_id = int(left_parallel_vehicle_id)
                        left_parallel_vehicle = scenario.agents_dict[left_parallel_vehicle_id]
                        left_parallel_vehicle_speed = left_parallel_vehicle.matching_agent_speed_list[record_index]
                        left_parallel_vehicle_acc = left_parallel_vehicle.acceleration_list[record_index]
                   
                    # right lane parallel vehicle
                    right_parallel_vehicle_id = agent.right_parallel_vehicle_id_list[record_index]
                    right_parallel_vehicle_gap = agent.right_parallel_vehicle_front_bumper_gap_list[record_index]
                    right_parallel_vehicle_spacing = agent.right_parallel_vehicle_spacing_list[record_index]
                    if right_parallel_vehicle_spacing == "None":
                        right_parallel_vehicle_time_headway = "None"
                    else:                
                        right_parallel_vehicle_time_headway = right_parallel_vehicle_spacing / speed
                    right_parallel_vehicle_location = agent.right_parallel_vehicle_location_list[record_index] 
                    right_parallel_vehicle_relative = agent.right_parallel_vehicle_relative_location_list[record_index]
                    
                    right_parallel_vehicle_speed = "None"
                    right_parallel_vehicle_acc = "None"
                    if right_parallel_vehicle_id != "None":
                        right_parallel_vehicle_id = int(right_parallel_vehicle_id)
                        right_parallel_vehicle = scenario.agents_dict[right_parallel_vehicle_id]
                        right_parallel_vehicle_speed = right_parallel_vehicle.matching_agent_speed_list[record_index]
                        right_parallel_vehicle_acc = right_parallel_vehicle.acceleration_list[record_index]
                    
                    # left lane preceding vehicle of the left lane parallel vehicle
                    left_lane_id = agent.left_lane_id_list[record_index]
                    left_lane_preceeding_vehicle_id = agent.left_preceeding_vehicle_id_list[record_index]
                    left_lane_preceeding_distance_to_ego_vehicle = agent.left_preceeding_vehicle_distance_list[record_index]
                    left_lane_preceeding_distance_to_ego_lane_end = agent.left_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                    left_lane_preceeding_gap = agent.left_preceeding_vehicle_gap_list[record_index]
                    left_lane_preceeding_time_headway = "None"
                    if left_lane_preceeding_gap != "None" and left_parallel_vehicle_speed != "None":
                        left_lane_preceeding_time_headway = left_lane_preceeding_gap / left_parallel_vehicle_speed
                    
                    # left lane following vehicle of the left lane parallel vehicle
                    left_lane_following_gap = agent.left_following_vehicle_gap_list[record_index]
                    left_lane_following_vehicle_id = agent.left_following_vehicle_id_list[record_index]
                    left_lane_following_distance_to_ego_vehicle = agent.left_following_vehicle_distance_list[record_index]
                    left_lane_following_vehicle_to_ego_lane_end = agent.left_following_vehicle_distance_to_ego_lane_end_list[record_index]  
                    left_lane_following_time_headway = "None"
                    if left_lane_following_gap != "None" and left_parallel_vehicle_speed != "None":                   
                        left_lane_following_time_headway = left_lane_following_gap / left_parallel_vehicle_speed
                
                    left_lane_spacing = agent.left_lane_spacing_list[record_index]
                    left_lane_change = agent.left_lane_change_list[record_index]
                
                    # right lane preceding vehicle of the left lane parallel vehicle
                    right_lane_id = agent.right_lane_id_list[record_index]
                    right_lane_preceeding_vehicle_id = agent.right_preceeding_vehicle_id_list[record_index]
                    right_lane_preceeding_distance_to_ego_vehicle = agent.right_preceeding_vehicle_distance_list[record_index]
                    right_lane_preceeding_distance_to_ego_lane_end = agent.right_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                    right_lane_preceeding_gap = agent.right_preceeding_vehicle_gap_list[record_index]
                    right_lane_preceeding_time_headway = "None"
                    if right_lane_preceeding_gap != "None" and right_parallel_vehicle_speed != "None": 
                        right_lane_preceeding_time_headway = right_lane_preceeding_gap / right_parallel_vehicle_speed
                    
                    # right lane following vehicle of the left lane parallel vehicle
                    right_lane_following_gap = agent.right_following_vehicle_gap_list[record_index]
                    right_lane_following_vehicle_id = agent.right_following_vehicle_id_list[record_index]
                    right_lane_following_distance_to_ego_vehicle = agent.right_following_vehicle_distance_list[record_index]
                    right_lane_following_vehicle_to_ego_lane_end = agent.right_following_vehicle_distance_to_ego_lane_end_list[record_index]
                    right_lane_following_time_headway = "None"
                    if right_lane_following_gap != "None" and right_parallel_vehicle_speed != "None": 
                        right_lane_following_time_headway = right_lane_following_gap / right_parallel_vehicle_speed
                    
                    right_lane_spacing = agent.right_lane_spacing_list[record_index]
                    right_lane_change = agent.right_lane_change_list[record_index]
                    
                    # topology 
                    lane_id_list = agent.matching_agent_lane_id_list[record_index]
                    lane_type_list = agent.matching_agent_lane_type_list[record_index]
                    link_id = agent.matching_agent_link_id_list[record_index]
                    node_id = agent.matching_agent_node_id_list[record_index]
                    turn_id = agent.matching_agent_turn_movement_list[record_index]
                    
                    # signal states
                    signal_light_shape_in_current_lane = agent.current_lane_signal_light_shape_list[record_index]
                    signal_light_type_in_current_lane = agent.current_lane_signal_light_type_list[record_index]
                    signal_light_state_in_current_lane = agent.current_lane_signal_light_state_list[record_index]
                    signal_light_distance_in_current_lane = agent.current_lane_signal_light_distance_list[record_index]
                    
                    # total 32 ? elements
                    csv_line = [time, total_frame, agent_id, agent_type, x, y, z, speed_x, speed_y, speed,    #7
                                vehicle_length, vehicle_width, vehicle_height, odometer, lane_id, 
                                position_to_lane_start, position_to_lane_end,        #3
                                signal_light_shape_in_current_lane,
                                signal_light_type_in_current_lane,
                                signal_light_state_in_current_lane,
                                signal_light_distance_in_current_lane,
                                preceeding_vehicle_id, preceeding_vehicle_distance_to_ego_vehicle, preceeding_vehicle_distance_to_ego_lane_end, preceeding_vehicle_gap, #3
                                preceeding_vehicle_time_headway,
                                following_vehicle_id, following_vehicle_distance_to_ego_vehicle, following_vehicle_distance_to_ego_lane_end,  following_vehicle_gap,   #3                               
                                following_vehicle_time_headway,
                                left_lane_id, left_parallel_vehicle_id, left_parallel_vehicle_relative, left_parallel_vehicle_gap, left_parallel_vehicle_spacing,
                                left_parallel_vehicle_time_headway,
                                right_lane_id, right_parallel_vehicle_id, right_parallel_vehicle_relative, right_parallel_vehicle_gap, right_parallel_vehicle_spacing, 
                                right_parallel_vehicle_time_headway,
                                left_lane_id, left_lane_preceeding_vehicle_id,   #2
                                left_lane_preceeding_distance_to_ego_vehicle, left_lane_preceeding_distance_to_ego_lane_end,  left_lane_preceeding_gap, #2
                                left_lane_preceeding_time_headway,
                                left_lane_following_vehicle_id, left_lane_following_distance_to_ego_vehicle,  #2
                                left_lane_following_vehicle_to_ego_lane_end, left_lane_following_gap,  #1
                                left_lane_following_time_headway,
                                left_lane_spacing, left_lane_change, #2
                                right_lane_id, right_lane_preceeding_vehicle_id,   #2
                                right_lane_preceeding_distance_to_ego_vehicle, right_lane_preceeding_distance_to_ego_lane_end, right_lane_preceeding_gap,#2
                                right_lane_preceeding_time_headway,
                                right_lane_following_vehicle_id, right_lane_following_distance_to_ego_vehicle,  #2
                                right_lane_following_vehicle_to_ego_lane_end, right_lane_following_gap, #1   
                                right_lane_following_time_headway,
                                right_lane_spacing, right_lane_change,                
                                lane_id_list, lane_type_list, link_id, node_id, turn_id]   #1
                    writer.writerow(csv_line)
                    record_index += 1

    output_statistic_csv_name = None
    if dataset_index != None and scenario_index != None:
        output_statistic_csv_name = "dataset" + str(dataset_index) + "_scenario" + str(scenario_index) + "_statistics" + ".csv"
    else:
        output_statistic_csv_name = "statistics" + ".csv"
    output_csv_file_path = os.path.join(output_statistics_path, output_statistic_csv_name)
    print("   ---- output csv file path is: ", output_csv_file_path)
    
    while True:
        try:
            if enconding is None:
                outfile = open(output_csv_file_path, 'w', newline='',errors='ignore')
            else:
                outfile = open(output_csv_file_path, 'w', newline='', errors='ignore', encoding = enconding)
            break
        except PermissionError:
            print('csv may be locked by other programs. please release it then try again')
            input()
            
    writer = csv.writer(outfile)
    title_line = ["scenario_id", "scenario_index", "dataset_id", "left_turn_number", "right_turn_number", "lane_change_number"]  
    writer.writerow(title_line)
    statistic_data_line = [scenario.scenario_id, scenario_index, dataset_index, scenario_left_turn_number, scenario_right_turn_number, scenario_lane_change_number]
    writer.writerow(statistic_data_line)                      
    

    
def output_all_map_matching_trajectories_csv(output_title_line_number, output_file_path, data_statistics_tool,
                                             output_statistics_path, scenario, enconding = None, 
                                             output_turn_type = None, dataset_index = None, 
                                             scenario_index = None, output_csv_to_one_file = None):   
    
    
    if output_file_path:
        if not os.path.exists(output_file_path): os.mkdir(output_file_path)
        
    # go through all the agents in this scenario
    for agent in scenario.agents_list:
        
        find_lane_change_flag = False
        agent_id = agent.agent_id
        agent_type = agent.agent_type
       
        # find if the car is standstill
        location_x_list = agent.matching_agent_x_list
        location_unique_x_list = np.unique(location_x_list)
        if len(location_unique_x_list) < 20:
            continue
        
        total_frame = agent.total_frame
        if total_frame == 0:
            continue
        
        data_statistics_tool.number_of_total_frames += total_frame               
        data_statistics_tool.frame_number_list.append(total_frame)
        
        data_statistics_tool.number_of_agents += 1
        data_statistics_tool.agents_id_list.append(agent_id)
        
        # write other info
        if output_turn_type == None:
            output_csv_name = "dataset" + str(dataset_index) + "_map_matching_output_total.csv"
            #output_csv_name = "dataset" + str(dataset_index) + "_map_matching_output.csv"
            output_filepath = os.path.join(output_file_path, output_csv_name)
            
            while True:
                try:
                    if enconding is None:
                        outfile = open(output_filepath, 'a+', newline='',errors='ignore')
                    else:
                        outfile = open(output_filepath, 'a+', newline='', errors='ignore', encoding=enconding)
                    break
                except PermissionError:
                    print('csv may be locked by other programs. please release it then try again')
                    input()
                    
            writer = csv.writer(outfile)
            
            title_line = [  # part1: self information
                
                          "frame_id", "total_frames", "vehicle_id", "vehicle_class", "local_x", "local_y", "local_z", 
                          "vehicle_velocity_x", "vehicle_velocity_y", "vehicle_velocity", "acceleration",                      
                          "vehicle_length",  "vehicle_width", "vehicle_height", "odometer",
                          "map_matching_lane_id", "lane_curvature", "lane_grade",
                          "distance_to_lane_start", 
                          "distance_to_lane_end",
                          "lane_id_set", "lane_type", "link_id", "node_id", "turn_movement_id",
                          "origin_link", "destination_link", "link_path", "origin_lane", "destination_lane", "lane_path", "movement_type",
                          
                          "distance_to_lane_end_at_beginning",
                          "distance_to_lane_start_at_ending",
                          "ROW",
                          
                          # patt2: surrounding vehicle
                          # signal light
                          "signal_light_shape", "signal_light_type", "signal_light_state", "distance_to_signal_light",
                          
                          # preceding and following vehicle of ego vehicle
                          "preceding_vehicle_id", "preceding_vehicle_spacing_headway", "preceding_vehicle_time_headway", 
                          "preceding_vehicle_distance_to_ego_lane_end", "preceding_vehicle_gap", "preceding_vehicle_velocity", "preceding_vehicle_acceleration",
                                                    
                          "following_vehicle_id", "following_vehicle_spacing_headway", "following_vehicle_time_headway",
                          "following_vehicle_distance_to_ego_lane_end","following_vehicle_gap","following_vehicle_velocity", "following_vehicle_acceleration",
                          
                          # left closest vhicle
                          "left_closest_vehicle_id", "lane_id", "relative_position", "left_closest_vehicle_gap", "left_closest_vehicle_spacing_headway",
                          "left_closest_vehicle_time_headway", "left_closest_vehicle_velocity", "left_closest_vehicle_acceleration",
                          
                          # right closest vehicle
                          "right_closest_vehicle_id", "lane_id", "relative_position", "right_closest_vehicle_gap", "right_closest_vehicle_spacing_headway",
                          "right_closest_vehicle_time_headway","right_closest_vehicle_velocity", "right_closest_vehicle_acceleration",
                          
                          # left lane and corresponding preceding vehicle
                          "left_lane_preceding_vehicle_id", "lane_id", "left_lane_preceding_vehicle_spacing_headway", "left_lane_preceding_vehicle_gap", 
                          "left_lane_preceding_vehicle_time_headway", "left_lane_preceding_vehicle_to_ego_lane_end", 
                          "left_lane_preceding_vehicle_velocity", "left_lane_preceding_vehicle_acceleration",
                          
                          # left lane and corresponding following vehicle
                          "left_lane_following_vehicle_id", "lane_id", "left_lane_following_vehicle_spacing_headway", "left_lane_following_vehicle_gap", 
                          "left_lane_following_vehicle_time_headway", "left_lane_following_vehicle_to_ego_lane_end", 
                          "left_lane_following_vehicle_velocity", "left_lane_following_vehicle_acceleration",
                          "left_lane_spacing", "left_lane_change", 
                          
                          # right lane and corresponding preceding vehicle
                          "right_lane_preceding_vehicle_id", "lane_id", "right_lane_preceding_vehicle_spacing_headway", "right_lane_preceding_vehicle_gap", 
                          "right_lane_preceding_vehicle_time_headway", "right_lane_preceding_vehicle_to_ego_lane_end", 
                          "right_lane_preceding_vehicle_velocity", "right_lane_preceding_vehicle_acceleration",
                          
                          # right lane and corresponding following vehicle
                          "right_lane_following_vehicle_id", "lane_id", "right_lane_following_vehicle_spacing_headway", "right_lane_following_vehicle_gap", 
                          "right_lane_following_vehicle_time_headway", "right_lane_following_vehicle_to_ego_lane_end", 
                          "right_lane_following_vehicle_velocity", "right_lane_following_vehicle_acceleration",
                          "right_lane_spacing", "right_lane_change",
   
                          # part3: topology information and map information
                          "mergeing_lane", "diverging_lane", "crossing_lane", "scenario_id", "dataset_id",
                          "intersection_type"]
            
            if output_title_line_number == 0:
                writer.writerow(title_line)
                output_title_line_number += 1
                       
            record_number = len(agent.matching_agent_time_list)
            scenario_id = scenario_index
            detaset_id = dataset_index

            # then process the other elements
            record_index = 0
            last_signal_light_shape_in_current_lane = "None"
            last_signal_light_type_in_current_lane = "None"
            last_signal_light_state_in_current_lane = "None" 
            last_signal_light_distance_in_current_lane = "None"
            
            last_preceeding_vehicle_id = "None"
            last_preceeding_vehicle_spacing = "None"
            last_preceeding_vehicle_headway = "None"
            last_preceeding_vehicle_to_lane_end = "None"
            last_preceeding_vehicle_gap = "None"

            last_following_vehicle_id = "None"
            last_following_vehicle_spacing = "None"
            last_following_vehicle_headway = "None"
            last_following_vehicle_to_lane_end = "None"
            last_following_vehicle_gap = "None"           
            
            agent_movement_type = agent.movement_type_list[1]
            
            if agent_movement_type == "THROUGH":
                data_statistics_tool.number_of_through_vehicle += 1
                data_statistics_tool.through_vehicle_id_list.append(agent_id)
            if agent_movement_type == "RIGHT_TURN":
                data_statistics_tool.number_of_right_turn_vehicle += 1 
                data_statistics_tool.right_turn_vehicle_id_list.append(agent_id)
            if agent_movement_type == "LEFT_TURN":
                data_statistics_tool.number_of_left_turn_vehicle += 1
                data_statistics_tool.left_turn_vehicle_id_list.append(agent_id)
            if agent_movement_type == "U_TURN":
                data_statistics_tool.number_of_u_turn_vehicle += 1 
                data_statistics_tool.u_turn_vehicle_id_list.append(agent_id)
     
            while record_index < record_number:     
                              
                time = agent.matching_agent_time_list[record_index]
                #total_frame = agent.matching_agent_total_frame
                x = agent.matching_agent_x_list[record_index]
                y = agent.matching_agent_y_list[record_index]
                z = agent.matching_agent_z_list[record_index]
                if x == 0 and y == 0:
                    record_index += 1
                    continue
                speed_x = agent.matching_agent_speed_x_list[record_index]  
                speed_y = agent.matching_agent_speed_y_list[record_index]  
                speed = agent.matching_agent_speed_list[record_index]  
                vehicle_acc = agent.acceleration_list[record_index] 
                #vehicle_length = agent.matching_agent_length_list[record_index] 
                #vehicle_width = agent.matching_agent_width_list[record_index] 
                #vehicle_height = agent.matching_agent_height_list[record_index]
                               
                vehicle_length = agent.average_length
                vehicle_width  = agent.average_width
                vehicle_height = agent.average_height
                
                lane_id = agent.unique_matching_agent_lane_id_list[record_index]
                #if lane_id == "None" and last_map_matching_lane_id != "None":
                #    lane_id = last_map_matching_lane_id 
                    
                if lane_id == "" or lane_id == None or lane_id == "None":
                    record_index += 1
                    continue
                
                '''
                if lane_id == "" and last_map_matching_lane_id == "" or \
                    lane_id == None and last_map_matching_lane_id == None or \
                        lane_id == "None" and last_map_matching_lane_id == "None":
                    record_index += 1
                    continue
                '''
               
                map_matching_lane = scenario.center_lane_dict[lane_id]
               
                position_to_lane_start = agent.distance_to_lane_start_list[record_index]
                position_to_lane_end = agent.distance_to_lane_end_list[record_index]
                odometer = agent.odometer_list[record_index]
                
                # current lane preceding vehicle
                preceeding_lane_id = agent.preceeding_vehicle_id_list[record_index]
                preceeding_vehicle_id = agent.preceeding_vehicle_id_list[record_index]           
                preceeding_vehicle_distance_to_ego_vehicle = agent.preceeding_vehicle_distance_list[record_index]
                preceeding_vehicle_time_headway = "None"
                if preceeding_vehicle_distance_to_ego_vehicle == "None":
                    preceeding_vehicle_time_headway = "None"
                else:
                    try:
                        preceeding_vehicle_time_headway = preceeding_vehicle_distance_to_ego_vehicle / speed
                    except:
                        preceeding_vehicle_time_headway = "inf"
                preceeding_vehicle_distance_to_ego_lane_end = agent.preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                preceeding_vehicle_gap = agent.preceeding_vehicle_gap_list[record_index]
                
                preceeding_vehicle_speed = "None"
                preceeding_vehicle_acc = "None"
                if preceeding_vehicle_id != "None":
                    preceeding_vehicle = scenario.agents_dict[preceeding_vehicle_id]
                    preceeding_vehicle_speed = preceeding_vehicle.matching_agent_speed_list[record_index]
                    preceeding_vehicle_acc = preceeding_vehicle.acceleration_list[record_index]
                
                '''
                if preceeding_vehicle_id == "None":                  
                    preceeding_vehicle_id = last_preceeding_vehicle_id
                    preceeding_vehicle_distance_to_ego_vehicle = last_preceeding_vehicle_spacing
                    preceeding_vehicle_time_headway = last_preceeding_vehicle_headway
                    preceeding_vehicle_distance_to_ego_lane_end = last_preceeding_vehicle_to_lane_end
                    preceeding_vehicle_gap = last_preceeding_vehicle_gap
                '''               
            
                # current lane following vehicle
                following_lane_id = agent.following_vehicle_id_list[record_index]
                following_vehicle_id = agent.following_vehicle_id_list[record_index]
                following_vehicle_distance_to_ego_vehicle = agent.following_vehicle_distance_list[record_index]
                following_vehicle_time_headway = "None"
                if following_vehicle_distance_to_ego_vehicle == "None":
                    following_vehicle_time_headway = "None"
                else:                
                    try:
                        following_vehicle_time_headway = following_vehicle_distance_to_ego_vehicle / speed
                    except:
                        following_vehicle_time_headway = "inf"
                following_vehicle_distance_to_ego_lane_end = agent.following_vehicle_distance_to_ego_lane_end_list[record_index]
                following_vehicle_gap = agent.following_vehicle_gap_list[record_index]
                
                following_vehicle_speed = "None"
                following_vehicle_acc = "None"
                if following_vehicle_id != "None":
                    following_vehicle = scenario.agents_dict[following_vehicle_id]
                    following_vehicle_speed = following_vehicle.matching_agent_speed_list[record_index]
                    following_vehicle_acc = following_vehicle.acceleration_list[record_index]


                '''
                if following_vehicle_id == "None":
                    following_vehicle_id = last_following_vehicle_id
                    following_vehicle_distance_to_ego_vehicle = last_following_vehicle_spacing
                    following_vehicle_time_headway = last_following_vehicle_headway
                    following_vehicle_distance_to_ego_lane_end = last_following_vehicle_to_lane_end
                    following_vehicle_gap = last_following_vehicle_gap
                '''
                
                # left lane parallel vehicle
                left_parallel_lane_id = agent.left_parallel_lane_id_list[record_index]
                left_parallel_vehicle_id = agent.left_parallel_vehicle_id_list[record_index]
                left_parallel_vehicle_gap = agent.left_parallel_vehicle_front_bumper_gap_list[record_index]
                left_parallel_vehicle_spacing = agent.left_parallel_vehicle_spacing_list[record_index]
                left_parallel_vehicle_time_headway = "None"
                if left_parallel_vehicle_spacing == "None":
                    left_parallel_vehicle_time_headway = "None"
                else:
                    try:
                        left_parallel_vehicle_time_headway = left_parallel_vehicle_spacing / speed
                    except:
                        left_parallel_vehicle_time_headway = "inf"
                left_parallel_vehicle_location = agent.left_parallel_vehicle_location_list[record_index]
                left_parallel_vehicle_relative = agent.left_parallel_vehicle_relative_location_list[record_index]              
                
                left_parallel_vehicle_speed = "None"
                left_parallel_vehicle_acc = "None"
                if left_parallel_vehicle_id != "None":
                    left_parallel_vehicle_id = int(left_parallel_vehicle_id)
                    left_parallel_vehicle = scenario.agents_dict[left_parallel_vehicle_id]
                    left_parallel_vehicle_speed = left_parallel_vehicle.matching_agent_speed_list[record_index]
                    left_parallel_vehicle_acc = left_parallel_vehicle.acceleration_list[record_index]
               
                # right lane parallel vehicle
                right_parallel_lane_id = agent.right_parallel_lane_id_list[record_index]
                right_parallel_vehicle_id = agent.right_parallel_vehicle_id_list[record_index]          
                right_parallel_vehicle_gap = agent.right_parallel_vehicle_front_bumper_gap_list[record_index]
                right_parallel_vehicle_spacing = agent.right_parallel_vehicle_spacing_list[record_index]
                right_parallel_vehicle_time_headway = "None"
                if right_parallel_vehicle_spacing == "None":
                    right_parallel_vehicle_time_headway = "None"
                else:
                    try:
                        right_parallel_vehicle_time_headway = right_parallel_vehicle_spacing / speed
                    except:
                        right_parallel_vehicle_time_headway = "inf"
                right_parallel_vehicle_location = agent.right_parallel_vehicle_location_list[record_index] 
                right_parallel_vehicle_relative = agent.right_parallel_vehicle_relative_location_list[record_index]
          
                right_parallel_vehicle_speed = "None"
                right_parallel_vehicle_acc = "None"
                if right_parallel_vehicle_id != "None":
                    right_parallel_vehicle_id = int(right_parallel_vehicle_id)
                    right_parallel_vehicle = scenario.agents_dict[right_parallel_vehicle_id]
                    right_parallel_vehicle_speed = right_parallel_vehicle.matching_agent_speed_list[record_index]
                    right_parallel_vehicle_acc = right_parallel_vehicle.acceleration_list[record_index]
                
                # left lane preceding vehicle of the left lane parallel vehicle
                left_preceeding_lane_id = agent.left_preceeding_lane_id_list[record_index]
                left_lane_preceeding_vehicle_id = agent.left_preceeding_vehicle_id_list[record_index]
                left_lane_preceeding_distance_to_ego_vehicle = agent.left_preceeding_vehicle_distance_list[record_index]
                left_lane_preceeding_distance_to_ego_lane_end = agent.left_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                left_lane_preceeding_gap = agent.left_preceeding_vehicle_gap_list[record_index]
                left_lane_preceeding_time_headway = "None"
                if left_lane_preceeding_gap != "None" and left_parallel_vehicle_speed != "None":
                    try:
                        left_lane_preceeding_time_headway = left_lane_preceeding_gap / left_parallel_vehicle_speed
                    except:
                        left_lane_preceeding_time_headway = "inf"

                left_lane_preceeding_vehicle_speed = "None"
                left_lane_preceeding_vehicle_acc = "None"
                if left_lane_preceeding_vehicle_id != "None":
                    left_lane_preceeding_vehicle_id = int(left_lane_preceeding_vehicle_id)
                    left_lane_preceeding_vehicle = scenario.agents_dict[left_lane_preceeding_vehicle_id]
                    left_lane_preceeding_vehicle_speed = left_lane_preceeding_vehicle.matching_agent_speed_list[record_index]
                    left_lane_preceeding_vehicle_acc = left_lane_preceeding_vehicle.acceleration_list[record_index]
                
                # left lane following vehicle of the left lane parallel vehicle
                left_following_lane_id = agent.left_following_lane_id_list[record_index]
                left_lane_following_gap = agent.left_following_vehicle_gap_list[record_index]
                left_lane_following_vehicle_id = agent.left_following_vehicle_id_list[record_index]
                left_lane_following_distance_to_ego_vehicle = agent.left_following_vehicle_distance_list[record_index]
                left_lane_following_vehicle_to_ego_lane_end = agent.left_following_vehicle_distance_to_ego_lane_end_list[record_index]  
                left_lane_following_time_headway = "None"
                if left_lane_following_gap != "None" and left_parallel_vehicle_speed != "None":    
                    try:
                        left_lane_following_time_headway = left_lane_following_gap / left_parallel_vehicle_speed
                    except:
                        left_lane_following_time_headway = "inf"

                left_lane_following_vehicle_speed = "None"
                left_lane_following_vehicle_acc = "None"
                if left_lane_following_vehicle_id != "None":
                    left_lane_following_vehicle_id = int(left_lane_following_vehicle_id)
                    left_lane_following_vehicle = scenario.agents_dict[left_lane_following_vehicle_id]
                    left_lane_following_vehicle_speed = left_lane_following_vehicle.matching_agent_speed_list[record_index]
                    left_lane_following_vehicle_acc = left_lane_following_vehicle.acceleration_list[record_index]
            
                left_lane_spacing = agent.left_lane_spacing_list[record_index]
                left_lane_change = agent.left_lane_change_list[record_index]
            
                # right lane preceding vehicle of the left lane parallel vehicle
                right_preceeding_lane_id = agent.right_preceeding_lane_id_list[record_index]
                right_lane_preceeding_vehicle_id = agent.right_preceeding_vehicle_id_list[record_index]
                right_lane_preceeding_distance_to_ego_vehicle = agent.right_preceeding_vehicle_distance_list[record_index]
                right_lane_preceeding_distance_to_ego_lane_end = agent.right_preceeding_vehicle_distance_to_ego_lane_end_list[record_index]
                right_lane_preceeding_gap = agent.right_preceeding_vehicle_gap_list[record_index]
                right_lane_preceeding_time_headway = "None"
                if right_lane_preceeding_gap != "None" and right_parallel_vehicle_speed != "None": 
                    try:
                        right_lane_preceeding_time_headway = right_lane_preceeding_gap / right_parallel_vehicle_speed
                    except:
                        right_lane_preceeding_time_headway = "inf"

                right_lane_preceeding_vehicle_speed = "None"
                right_lane_preceeding_vehicle_acc = "None"
                if right_lane_preceeding_vehicle_id != "None":
                    right_lane_preceeding_vehicle_id = int(right_lane_preceeding_vehicle_id)
                    right_lane_preceeding_vehicle = scenario.agents_dict[right_lane_preceeding_vehicle_id]
                    right_lane_preceeding_vehicle_speed = right_lane_preceeding_vehicle.matching_agent_speed_list[record_index]
                    right_lane_preceeding_vehicle_acc = right_lane_preceeding_vehicle.acceleration_list[record_index]
                
                # right lane following vehicle of the left lane parallel vehicle
                right_following_lane_id = agent.right_following_lane_id_list[record_index]
                right_lane_following_gap = agent.right_following_vehicle_gap_list[record_index]
                right_lane_following_vehicle_id = agent.right_following_vehicle_id_list[record_index]
                right_lane_following_distance_to_ego_vehicle = agent.right_following_vehicle_distance_list[record_index]
                right_lane_following_distance_to_ego_lane_end = agent.right_following_vehicle_distance_to_ego_lane_end_list[record_index]
                right_lane_following_time_headway = "None"
                if right_lane_following_gap != "None" and right_parallel_vehicle_speed != "None": 
                    right_lane_following_time_headway = right_lane_following_gap / right_parallel_vehicle_speed

                right_lane_following_vehicle_speed = "None"
                right_lane_following_vehicle_acc = "None"
                if right_lane_following_vehicle_id != "None":
                    right_lane_following_vehicle_id = int(right_lane_following_vehicle_id)
                    right_lane_following_vehicle = scenario.agents_dict[right_lane_following_vehicle_id]
                    right_lane_following_vehicle_speed = right_lane_following_vehicle.matching_agent_speed_list[record_index]
                    right_lane_following_vehicle_acc = right_lane_following_vehicle.acceleration_list[record_index]
                
                right_lane_spacing = agent.right_lane_spacing_list[record_index]
                right_lane_change = agent.right_lane_change_list[record_index]
                
                # topology 
                lane_id_list = agent.matching_agent_lane_id_list[record_index]
                lane_type_list = agent.matching_agent_lane_type_list[record_index]
                link_id = agent.matching_agent_link_id_list[record_index]
                node_id = agent.matching_agent_node_id_list[record_index]
                turn_id = agent.matching_agent_turn_movement_list[record_index]
                
                # signal states
                signal_light_state_in_current_lane = "None"
                signal_light_type_in_current_lane = "None"
                signal_light_shape_in_current_lane = "None"
                signal_light_distance_in_current_lane = "None"
                
                signal_light_shape_in_current_lane = agent.current_lane_signal_light_shape_list[record_index]
                signal_light_type_in_current_lane = agent.current_lane_signal_light_type_list[record_index]
                signal_light_state_in_current_lane = agent.current_lane_signal_light_state_list[record_index]
                signal_light_distance_in_current_lane = agent.current_lane_signal_light_distance_list[record_index]
                
                if signal_light_state_in_current_lane == "None":
                    signal_light_state_in_current_lane = map_matching_lane.signal_light_color_dict[record_index]
                    if signal_light_state_in_current_lane != "None":
                        signal_light_type_in_current_lane = "steady"
                        signal_light_shape_in_current_lane = "round"
                        signal_light_distance_in_current_lane = "None"
                
                '''
                if signal_light_shape_in_current_lane == "None":
                    signal_light_shape_in_current_lane = last_signal_light_shape_in_current_lane
                if signal_light_type_in_current_lane == "None":
                    signal_light_type_in_current_lane = last_signal_light_type_in_current_lane
                if signal_light_state_in_current_lane == "None":
                    signal_light_state_in_current_lane = last_signal_light_state_in_current_lane
                if signal_light_distance_in_current_lane == "None":
                    signal_light_distance_in_current_lane = last_signal_light_distance_in_current_lane
                '''  
                
                #print("   --- record index is: ", record_index)
                #print("   --- movement list is: ", len(agent.movement_type_list))
                    
                agent_movement_type = agent.movement_type_list[record_index]
               
                agent_lane_curvature = agent.curvature_list[record_index]
                agent_lane_grade = agent.grade_list[record_index]
                agent_merging_lane_list = agent.merging_lane_list[record_index]
                agent_diverging_lane_list = agent.diverging_lane_list[record_index]
                agent_crossing_lane_list = agent.crossing_lane_list[record_index] 
               
                origin_link_id = agent.origin_link_id
                destination_link_id = agent.destination_link_id
                origin_lane_id = agent.origin_lane_id
                destination_lane_id = agent.destination_lane_id    
                link_path_list = agent.link_path_list
                lane_path_list = agent.lane_path_list
                
                distance_to_lane_end_at_beginning = agent.distance_to_lane_end_at_beginning
                distance_to_lane_start_at_ending = agent.distance_to_lane_start_at_ending
                ROW = agent.ROW_list[record_index]
                intersection_type = "FOUR-LEG intersection"
                
                if find_lane_change_flag == False:
                    if left_lane_change != "False" or right_lane_change != "False":
                        find_lane_change_flag == True
                        data_statistics_tool.number_of_lane_change_vehicle += 1
                        data_statistics_tool.lane_change_vehicle_id_list.append(agent_id)
                
                # total 32 ? elements
                csv_line = [time, total_frame, agent_id, agent_type, x, y, z, 
                            speed_x, speed_y, speed, vehicle_acc,
                            vehicle_length, vehicle_width, vehicle_height, odometer, lane_id, 
                            agent_lane_curvature,
                            agent_lane_grade,
                            position_to_lane_start, position_to_lane_end,        #3
                            lane_id_list, lane_type_list, link_id, node_id, turn_id,
                            origin_link_id, destination_link_id, link_path_list,
                            origin_lane_id, destination_lane_id, lane_path_list,
                            agent_movement_type,
                            
                            distance_to_lane_end_at_beginning,
                            distance_to_lane_start_at_ending,
                            ROW,
                            
                            signal_light_shape_in_current_lane,
                            signal_light_type_in_current_lane,
                            signal_light_state_in_current_lane,
                            signal_light_distance_in_current_lane,
                            
                            preceeding_vehicle_id, preceeding_vehicle_distance_to_ego_vehicle, preceeding_vehicle_time_headway,
                            preceeding_vehicle_distance_to_ego_lane_end, preceeding_vehicle_gap, 
                            preceeding_vehicle_speed, preceeding_vehicle_acc,
                            
                            following_vehicle_id, following_vehicle_distance_to_ego_vehicle, following_vehicle_time_headway,
                            following_vehicle_distance_to_ego_lane_end, following_vehicle_gap, 
                            following_vehicle_speed, following_vehicle_acc,
                            
                            left_parallel_vehicle_id, left_parallel_lane_id, left_parallel_vehicle_relative, left_parallel_vehicle_gap, 
                            left_parallel_vehicle_spacing, left_parallel_vehicle_time_headway, left_parallel_vehicle_speed,
                            left_parallel_vehicle_acc,
                                                       
                            right_parallel_vehicle_id, right_parallel_lane_id, right_parallel_vehicle_relative, right_parallel_vehicle_gap, 
                            right_parallel_vehicle_spacing, right_parallel_vehicle_time_headway, right_parallel_vehicle_speed,
                            right_parallel_vehicle_acc,
                            
                            left_lane_preceeding_vehicle_id, left_preceeding_lane_id,    #2
                            left_lane_preceeding_distance_to_ego_vehicle, left_lane_preceeding_gap, 
                            left_lane_preceeding_time_headway, left_lane_preceeding_distance_to_ego_lane_end,   #2
                            left_lane_preceeding_vehicle_speed, left_lane_preceeding_vehicle_acc,
                            
                            left_lane_following_vehicle_id, left_following_lane_id,    #2
                            left_lane_following_distance_to_ego_vehicle, left_lane_following_gap, #2
                            left_lane_following_time_headway, left_lane_following_vehicle_to_ego_lane_end,
                            left_lane_following_vehicle_speed, left_lane_following_vehicle_acc,
                            left_lane_spacing, left_lane_change, #2
                                                     
                            right_lane_preceeding_vehicle_id, right_preceeding_lane_id,   #2
                            right_lane_preceeding_distance_to_ego_vehicle, right_lane_preceeding_gap,
                            right_lane_preceeding_time_headway, right_lane_preceeding_distance_to_ego_lane_end, #2
                            right_lane_preceeding_vehicle_speed, right_lane_preceeding_vehicle_acc,
                            
                            right_lane_following_vehicle_id, right_following_lane_id,   #2
                            right_lane_following_distance_to_ego_vehicle, right_lane_following_gap,
                            right_lane_following_time_headway, right_lane_following_distance_to_ego_lane_end, #2
                            right_lane_following_vehicle_speed, right_lane_following_vehicle_acc,
                            right_lane_spacing, right_lane_change, 
                            
                            agent_merging_lane_list,
                            agent_diverging_lane_list,
                            agent_crossing_lane_list,
                            scenario_id,
                            detaset_id,
                            intersection_type]   #1
                
                writer.writerow(csv_line)
                
                last_map_matching_lane_id = lane_id
                last_signal_light_shape_in_current_lane = signal_light_shape_in_current_lane
                last_signal_light_type_in_current_lane = signal_light_type_in_current_lane
                last_signal_light_state_in_current_lane  = signal_light_state_in_current_lane
                last_signal_light_distance_in_current_lane = signal_light_distance_in_current_lane
                
                last_preceeding_vehicle_id = preceeding_vehicle_id
                last_preceeding_vehicle_spacing = preceeding_vehicle_distance_to_ego_vehicle
                last_preceeding_vehicle_headway = preceeding_vehicle_time_headway
                last_preceeding_vehicle_to_lane_end = preceeding_vehicle_distance_to_ego_lane_end
                last_preceeding_vehicle_gap = preceeding_vehicle_gap

                last_following_vehicle_id = following_vehicle_id
                last_following_vehicle_spacing = following_vehicle_distance_to_ego_vehicle
                last_following_vehicle_headway = following_vehicle_time_headway
                last_following_vehicle_to_lane_end = following_vehicle_distance_to_ego_lane_end
                last_following_vehicle_gap = following_vehicle_gap
                 
                record_index += 1    
    output_title_line_number = 1
    
    return output_title_line_number, data_statistics_tool
    
    
def output_map_network_to_json(data_statistics_tool, output_file_path, enconding=None):
    
    if output_file_path:
        if not os.path.exists(output_file_path): os.mkdir(output_file_path)


    output_statistic_name = "AV_dataset_analysis_statistics.json"
    output_filepath = os.path.join(output_file_path, output_statistic_name)          
    
    statistic_dict = data_statistics_tool.statistic_dict
    
    with open(output_filepath, "w") as write_file:
        json.dump(statistic_dict, write_file, indent = 4)
