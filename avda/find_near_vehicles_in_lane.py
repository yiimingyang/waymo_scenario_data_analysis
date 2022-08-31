#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from .waymo_training_math_tools import *

def is_in_range(x1,x2,x0):
    if x0 < x1 and x0 < x2 or x0 > x1 and x0 > x2:
        return False
    else:
        return True

def calculate_distance(x1, y1, x2, y2):
    distance = math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    return distance

def calculate_distance_from_line(a, b, c, vehicle_x, vehicle_y):
    min_distance = abs((a * vehicle_x + b * vehicle_y + c)) / (math.sqrt(a * a + b * b))
    return min_distance

def get_straight_equation_from_two_points(first_point_x, first_point_y, last_point_x, last_point_y):
    a = first_point_y - last_point_y
    b = last_point_x - first_point_x
    c = first_point_x * last_point_y - last_point_x * first_point_y
    return a, b, c
        
def is_agent_in_segment(a, b, c, surrounding_vehicle_state_x, surrounding_vehicle_state_y, first_point_x, first_point_y, last_point_x, last_point_y):
    distance = calculate_distance_from_line(a, b, c, surrounding_vehicle_state_x, surrounding_vehicle_state_y)
    if distance >= 1:
        return False
    if distance < 1:
        if abs(first_point_x - last_point_x) >= abs(first_point_y - last_point_y):
            if is_in_range(first_point_x, last_point_x, surrounding_vehicle_state_x):                
                return True
            else:
                return False
                #agents_on_the_same_lane_distance_to_ego_list.append(distance_to_ego_vehicle)
        if abs(first_point_x - last_point_x) <= abs(first_point_y - last_point_y):
            if is_in_range(first_point_y, last_point_y, surrounding_vehicle_state_y):               
                return True
            else:
                return False
                #agents_on_the_same_lane_distance_to_ego_list.append(distance_to_ego_vehicle)
        

def find_preceeding_following_in_lane(scenario, 
                                      time_index,
                                      ego_vheicle_id,
                                      ego_vheicle_x, 
                                      ego_vheicle_y, 
                                      ego_vheicle_length, 
                                      ego_vheicle_width, 
                                      ego_vheicle_yaw):
    
    # find the lane the vehicle is on
    min_distance = 1000
    ego_lane_id = None
    for center_lane in scenario.center_lane_list:
        for point in center_lane.points_list:
            distance = calculate_distance(point[0], point[1], ego_vheicle_x, ego_vheicle_y)
            if distance < min_distance:
                min_distance = distance
                ego_lane_id = center_lane.lane_id
    #print("ego vehicle mindistance from ego lane: ", min_distance)
    #print("ego lane id is: ", ego_lane_id)

    # find closet vehicles in current lane
    agents_on_the_same_lane_list = []
    agents_on_the_same_lane_distance_to_ego_list = []

    preceeding_vehicle_id = None
    preceeding_vehicle_location = None
    preceeding_to_ego_distance = None
    
    following_vehicle_id = None    
    following_vehicle_location = None
    following_to_ego_distance = None
    
    ego_lane = scenario.center_lane_dict[ego_lane_id]
    #print("lane id is: ", ego_lane.lane_id)
    
    # calculate the straight line equation
    number_of_points_in_the_same_lane = len(ego_lane.points_list)
    
    # if the lane is too long, it may not straight, split the lane into some segements   
    number_of_segments = math.ceil(number_of_points_in_the_same_lane/40)
    first_point_index = 0
    last_point_index = number_of_points_in_the_same_lane - 1  

    segment_interval = math.ceil(last_point_index/number_of_segments)      
    
    middle_point_index = 0
    
    for i in range(number_of_segments):
        middle_point_index += segment_interval
        if middle_point_index > last_point_index:
            middle_point_index = last_point_index        
        segment_first_point_x = ego_lane.point_x_list[first_point_index]
        segment_first_point_y = ego_lane.point_y_list[first_point_index]
        segment_last_point_x  = ego_lane.point_x_list[middle_point_index]
        segment_last_point_y  = ego_lane.point_y_list[middle_point_index]  
        
        a, b, c = get_straight_equation_from_two_points(segment_first_point_x,
                                                        segment_first_point_y,
                                                        segment_last_point_x,
                                                        segment_last_point_y)
         
        if a == b == 0:
            continue
        
        for surrounding_vehicle in scenario.agents_list:  
            
            # get surrounding agents info
            surrounding_vehicle_state = surrounding_vehicle.states_array[time_index, :]
            surrounding_vehicle_id = surrounding_vehicle_state[1]
            if surrounding_vehicle_id == ego_vheicle_id:
                continue      
            surrounding_vehicle_state_x = surrounding_vehicle_state[3]
            surrounding_vehicle_state_y = surrounding_vehicle_state[4]
            surrounding_vehicle_state_heading = radian_to_degree(surrounding_vehicle_state[9])
            
            is_in_lane = is_agent_in_segment(a, b, c, surrounding_vehicle_state_x, surrounding_vehicle_state_y, segment_first_point_x, segment_first_point_y, segment_last_point_x, segment_last_point_y)
            if is_in_lane > 0:
                #print(" --- surround vehicle id in center lane is : ", surrounding_vehicle_id)
                #print(" --- surround vehicle distance from center lane: ", ego_lane_id, " is : ", distance)
                agents_on_the_same_lane_list.append(surrounding_vehicle)
                distance_to_ego_vehicle = calculate_distance(surrounding_vehicle_state_x, surrounding_vehicle_state_y, ego_vheicle_x, ego_vheicle_y)            
                agents_on_the_same_lane_distance_to_ego_list.append(distance_to_ego_vehicle)                
        first_point_index = middle_point_index
    
    agent_index  = 0
    last_agent_preceeding_y = None
    last_agent_following_y = None
    last_agent_preceeding_x = None
    last_agent_following_x = None
    last_agent_to_ego_distance = None
        
    for agent in agents_on_the_same_lane_list:
        agent_state = agent.states_array[time_index, :]
        agent_id = agent_state[1]    
        agent_state_x = agent_state[3]
        agent_state_y = agent_state[4]
        agent_state_heading = radian_to_degree(agent_state[9])   

        if ego_vheicle_yaw > 45 and ego_vheicle_yaw < 135:
            if agent_state_y >= ego_vheicle_y:
                if last_agent_preceeding_y == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_y = agent_state_y
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]                        
                else:                        
                    if agent_state_y <= last_agent_preceeding_y:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_y = agent_state_y
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
            else:
                #print("----- the same lane find following vehicle -----")
                if last_agent_following_y == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_y = agent_state_y
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]                   
                else:
                    if agent_state_y >= last_agent_following_y:
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y) 
                        last_agent_following_y = agent_state_y
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]

        if ego_vheicle_yaw > -135 and ego_vheicle_yaw < -45:
            if agent_state_y <= ego_vheicle_y:
                if last_agent_preceeding_y == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_y = agent_state_y
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]  
                else:
                    if agent_state_y <= last_agent_preceeding_y:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_y = agent_state_y
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
            else:
                if last_agent_following_y == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_y = agent_state_y
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index] 
                else:   
                    if agent_state_y <= last_agent_following_y:
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y) 
                        last_agent_following_y = agent_state_y
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]

        if ego_vheicle_yaw > -45 and ego_vheicle_yaw < 45:
            if agent_state_x >= ego_vheicle_x:
                if last_agent_preceeding_x == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_x = agent_state_x
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]  
                else:
                    if agent_state_x <= last_agent_preceeding_x:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_x = agent_state_x
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
            else:
                if last_agent_following_x == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_x = agent_state_x
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]  
                else:
                    if agent_state_x >= last_agent_following_x:                
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y)  
                        last_agent_following_x = agent_state_x
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]

        if ego_vheicle_yaw > 135 and ego_vheicle_yaw <= 180 \
            or ego_vheicle_yaw >= -180 and ego_vheicle_yaw <= -135:
            if agent_state_x <= ego_vheicle_x:
                if last_agent_preceeding_x == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_x = agent_state_x
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]  
                else:
                    if agent_state_x >= last_agent_preceeding_x:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_x = agent_state_x
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[index]
            else:
                if last_agent_following_x == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_x = agent_state_x
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]  
                else:
                    if agent_state_x <= last_agent_following_x:
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_following_x = agent_state_x
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
        agent_index += 1
        
        '''
        if ego_vheicle_yaw > 45 and ego_vheicle_yaw < 135:
            if agent_state_y > ego_vheicle_y:
                if agent_state_y < last_agent_y:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_y = agent_state_y
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
            else:
                if agent_state_y < last_agent_y:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y) 
                    last_agent_y = agent_state_y
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]

        if ego_vheicle_yaw > -135 and ego_vheicle_yaw < -45:
            if agent_state_y < ego_vheicle_y:
                if agent_state_y < last_agent_y:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_y = agent_state_y
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
            else:
                if agent_state_y < last_agent_y:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y) 
                    last_agent_y = agent_state_y
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]

        if ego_vheicle_yaw > -45 and ego_vheicle_yaw < 45:
            if agent_state_x > ego_vheicle_x:
                if agent_state_x < last_agent_x:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_x = agent_state_x
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
            else:
                if agent_state_x < last_agent_x:                
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)  
                    last_agent_x = agent_state_x
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]

        if ego_vheicle_yaw > 135 and ego_vheicle_yaw <= 180 or ego_vheicle_yaw >= -180 and ego_vheicle_yaw <= -135:
            if agent_state_x < ego_vheicle_x:
                if agent_state_x < last_agent_x:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_x = agent_state_x
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[index]
            else:
                if agent_state_x < last_agent_x:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_x = agent_state_x
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[index]
        agent_index += 1
        '''
       
    return preceeding_vehicle_id, \
           preceeding_vehicle_location, \
           preceeding_to_ego_distance, \
           following_vehicle_id, \
           following_vehicle_location, \
           following_to_ego_distance


            
            