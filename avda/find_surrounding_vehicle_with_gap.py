#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from .waymo_training_math_tools import *

def calculate_surrounding_vehicle_with_lane_change_gap(scenario):
    for agent in scenario.agents_list:
        # if agent is not vehicle
        if agent.states_array.size == 0 or agent.agent_type == 2 or agent.agent_type == 3:
            continue   

        agent_time_list = []
        agent_x_list = []
        agent_y_list = []  
        agent_speed_list = []
        agent_lane_id_list = []
        agent_lane_type_list = []
        agent_link_id_list = []
        agent_node_id_list = []
        agent_turn_movement_list = []
        agent_map_matching_lane_id_list = agent.unique_matching_agent_lane_id_list
        #print("unique map matching lane id list size is: ", len(agent_map_matching_lane_id_list))
        
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
            
            # stage1: find the position the vehicle on the lane on certain timestep
            map_matching_lane_id = agent_map_matching_lane_id_list[state_index]
            left_lane_spacing = agent.left_lane_spacing_list[state_index]
            right_lane_spacing = agent.right_lane_spacing_list[state_index]
            agent_location = (agent_x, agent_y)
            agent_shape = (agent_length, agent_width, agent_height)
            agent.length_width_height_list.append(agent_shape)
            
            if map_matching_lane_id == None or map_matching_lane_id == ' ' or map_matching_lane_id == 'None':
                state_index += 1
                agent.distance_to_lane_start_list.append("None")
                agent.distance_to_lane_end_list.append("None")
                
                # left lane parallel vehicle
                agent.left_parallel_lane_id_list.append("None")
                agent.left_parallel_vehicle_id_list.append("None")
                agent.left_parallel_vehicle_relative_location_list.append("None")
                agent.left_parallel_vehicle_front_bumper_gap_list.append("None")
                agent.left_parallel_vehicle_spacing_list.append("None")
                agent.left_parallel_vehicle_location_list.append("None")
                agent.left_parallel_vehicle_speed_list.append("None")
                agent.left_parallel_vehicle_acc_list.append("None")               
        
                # right lane parallel vehicle
                agent.right_parallel_lane_id_list.append("None")
                agent.right_parallel_vehicle_id_list.append("None")
                agent.right_parallel_vehicle_relative_location_list.append("None")
                agent.right_parallel_vehicle_front_bumper_gap_list.append("None")
                agent.right_parallel_vehicle_spacing_list.append("None")
                agent.right_parallel_vehicle_location_list.append("None")    
                agent.right_parallel_vehicle_speed_list.append("None")
                agent.right_parallel_vehicle_acc_list.append("None")
                
                # current lane preceding vehicle
                agent.preceeding_lane_id_list.append("None")
                agent.preceeding_vehicle_id_list.append("None")
                agent.preceeding_vehicle_distance_list.append("None")
                agent.preceeding_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.preceeding_vehicle_location_list.append("None")
                agent.preceeding_vehicle_gap_list.append("None")
                agent.preceeding_vehicle_speed_list.append("None")
                agent.preceeding_vehicle_acc_list.append("None")

                # current lane following vehicle
                agent.following_lane_id_list.append("None")
                agent.following_vehicle_id_list.append("None")
                agent.following_vehicle_distance_list.append("None")
                agent.following_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.following_vehicle_location_list.append("None")
                agent.following_vehicle_gap_list.append("None")
                agent.following_vehicle_speed_list.append("None")
                agent.following_vehicle_acc_list.append("None")
                
                # left lane preceding vehicle
                agent.left_preceeding_lane_id_list.append("None")
                agent.left_preceeding_vehicle_id_list.append("None")
                agent.left_preceeding_vehicle_distance_list.append("None")              
                agent.left_preceeding_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.left_preceeding_vehicle_location_list.append("None")
                agent.left_preceeding_vehicle_gap_list.append("None")
                agent.left_preceeding_vehicle_speed_list.append("None")
                agent.left_preceeding_vehicle_acc_list.append("None")
                
                # left lane following vehicle
                agent.left_following_lane_id_list.append("None")
                agent.left_following_vehicle_id_list.append("None")
                agent.left_following_vehicle_distance_list.append("None")  
                agent.left_following_vehicle_distance_to_ego_lane_end_list.append("None")            
                agent.left_following_vehicle_location_list.append("None")
                agent.left_following_vehicle_gap_list.append("None")
                agent.left_following_vehicle_speed_list.append("None")
                agent.left_following_vehicle_acc_list.append("None")
                #agent.left_lane_id_list.append("None")
                
                # right lane preceding vehicle
                agent.right_preceeding_lane_id_list.append("None")
                agent.right_preceeding_vehicle_id_list.append("None")
                agent.right_preceeding_vehicle_distance_list.append("None")
                agent.right_preceeding_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.right_preceeding_vehicle_location_list.append("None")
                agent.right_preceeding_vehicle_gap_list.append("None")
                agent.right_preceeding_vehicle_speed_list.append("None")
                agent.right_preceeding_vehicle_acc_list.append("None")
                
                # right lane following vehicle
                agent.right_following_lane_id_list.append("None")
                agent.right_following_vehicle_id_list.append("None")
                agent.right_following_vehicle_distance_list.append("None")
                agent.right_following_vehicle_distance_to_ego_lane_end_list.append("None") 
                agent.right_following_vehicle_location_list.append("None")
                agent.right_following_vehicle_gap_list.append("None")
                agent.right_following_vehicle_speed_list.append("None")
                agent.right_following_vehicle_acc_list.append("None")
                
                #agent.right_lane_id_list.append("None")               
                continue      
            
            map_matching_lane = scenario.center_lane_dict[map_matching_lane_id]
            map_matching_lane_point_list = map_matching_lane.points_list           
            location = (agent_x, agent_y)
            current_lane_start_point = map_matching_lane_point_list[0]
            current_lane_end_point = map_matching_lane_point_list[-1]
            
            point_index, number_of_point_in_lane = find_nearest_point_in_lane(location, map_matching_lane_point_list)
            
            #if agent_id == 1552:
            #    if map_matching_lane_id == 156:
            #        print("   ---  point_index : ", point_index)
            #        print("   ---  point in lane : ", number_of_point_in_lane)
            
            if point_index != None and point_index <= number_of_point_in_lane:
                agent_to_lane_start_distance = 0.5 * point_index
                agent_to_lane_end_distance = 0.5 * (number_of_point_in_lane - point_index)
                agent.distance_to_lane_start_list.append(agent_to_lane_start_distance)
                agent.distance_to_lane_end_list.append(agent_to_lane_end_distance)
            else:
                agent.distance_to_lane_start_list.append("None")
                agent.distance_to_lane_end_list.append("None")               
             
            # stage2: find the preceeding and following vehicles on the same lane on each timestep     
            agents_on_the_same_lane_list = []
            agents_on_the_same_lane_distance_to_ego_list = []
            for surrounding_vehicle in scenario.agents_list:  
                if surrounding_vehicle.states_array.size == 0 or surrounding_vehicle.agent_type == 2 or surrounding_vehicle.agent_type == 3:
                    continue                   
                # get surrounding agents info
                surrounding_vehicle_state = surrounding_vehicle.states_array[state_index, :]
                surrounding_vehicle_id = surrounding_vehicle_state[1]
                surrounding_vehicle_length = surrounding_vehicle_state[6]
                if surrounding_vehicle_id == agent_id:
                    continue      
                surrounding_vehicle_map_matching_lane_id_list = surrounding_vehicle.unique_matching_agent_lane_id_list
                if len(surrounding_vehicle_map_matching_lane_id_list) != 0:
                    surrounding_vehicle_map_matching_lane_id = surrounding_vehicle_map_matching_lane_id_list[state_index]
                    if surrounding_vehicle_map_matching_lane_id == map_matching_lane_id:                        
                        surrounding_vehicle_state_x = surrounding_vehicle_state[3]
                        surrounding_vehicle_state_y = surrounding_vehicle_state[4]
                        surrounding_vehicle_state_heading = radian_to_degree(surrounding_vehicle_state[9])
                        distance_to_ego_vehicle = calculate_distance_between_two_points(surrounding_vehicle_state_x, surrounding_vehicle_state_y, agent_x, agent_y) 
                        agents_on_the_same_lane_list.append(surrounding_vehicle)               
                        agents_on_the_same_lane_distance_to_ego_list.append(distance_to_ego_vehicle)   
                                              
            preceeding_vehicle_id, \
            preceeding_vehicle_location, \
            preceeding_to_ego_distance, \
            following_vehicle_id, \
            following_vehicle_location, \
            following_to_ego_distance, \
            preceeding_lane_id, \
            following_lane_id    = find_preceeding_following_vehicle_in_lane(state_index,
                                                    location,
                                                    agent_yaw, 
                                                    agents_on_the_same_lane_list, 
                                                    agents_on_the_same_lane_distance_to_ego_list)                
                       
            if preceeding_vehicle_id != None:
                
                preceeding_vehicle = scenario.agents_dict[preceeding_vehicle_id]
                preceeding_vehicle_state = preceeding_vehicle.states_array[state_index, :]              
                preceeding_vehicle_length = preceeding_vehicle_state[6]
                preceeding_vehicle_speed  = math.sqrt(preceeding_vehicle_state[10]**2 + preceeding_vehicle_state[11]**2) 
                
                agent.preceeding_vehicle_id_list.append(preceeding_vehicle_id)
                agent.preceeding_vehicle_distance_list.append(preceeding_to_ego_distance)
                distance_to_current_lane_end_point = calculate_distance_between_two_points_v1(current_lane_end_point, preceeding_vehicle_location)
                agent.preceeding_vehicle_distance_to_ego_lane_end_list.append(distance_to_current_lane_end_point)                
                agent.preceeding_vehicle_location_list.append(preceeding_vehicle_location)
                gap_distance = preceeding_to_ego_distance - 0.5 * agent_length - 0.5 * preceeding_vehicle_length
                agent.preceeding_vehicle_gap_list.append(gap_distance)
                agent.preceeding_vehicle_speed_list.append(preceeding_vehicle_speed)
            else:
                agent.preceeding_vehicle_id_list.append("None")
                agent.preceeding_vehicle_distance_list.append("None")
                agent.preceeding_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.preceeding_vehicle_location_list.append("None")
                agent.preceeding_vehicle_gap_list.append("None")
                agent.preceeding_vehicle_speed_list.append("None")
                
            if following_vehicle_id != None:
                
                following_vehicle = scenario.agents_dict[following_vehicle_id]
                following_vehicle_state = following_vehicle.states_array[state_index, :]              
                following_vehicle_length = following_vehicle_state[6]
                following_vehicle_speed  = math.sqrt(following_vehicle_state[10]**2 + following_vehicle_state[11]**2)                 
                
                agent.following_vehicle_id_list.append(following_vehicle_id)
                agent.following_vehicle_distance_list.append(following_to_ego_distance)
                distance_to_current_lane_end_point = calculate_distance_between_two_points_v1(current_lane_end_point, following_vehicle_location)
                agent.following_vehicle_distance_to_ego_lane_end_list.append(distance_to_current_lane_end_point)
                agent.following_vehicle_location_list.append(following_vehicle_location)
                gap_distance = following_to_ego_distance - 0.5 * agent_length - 0.5 * following_vehicle_length
                agent.following_vehicle_gap_list.append(gap_distance)     
                agent.following_vehicle_speed_list.append(following_vehicle_speed)         
            else:
                agent.following_vehicle_id_list.append("None")
                agent.following_vehicle_distance_list.append("None")
                agent.following_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.following_vehicle_location_list.append("None")
                agent.following_vehicle_gap_list.append("None")
                agent.following_vehicle_speed_list.append("None") 
            
            # stage3: find the parallel/cloest vehicle on the left lane on certain timestep                       
            left_neighbors_id_list = map_matching_lane.left_neighbors_id_list

            #if agent_id == 1538:
            #    if state_index == 41:
            #        print(" --- agent yaw is: ", agent_yaw)
                  
            cloest_vehicle_id = None
            cloest_vehicle_location = None
            cloest_to_ego_distance = None
            cloest_vehicle_location = None
            cloest_vehicle_yaw = None
            cloest_vehicle_speed = None
            cloest_vehicle_acc = None
            cloest_vehicle_lane_id = None
            cloest_vehicle_shape = None
            cloest_relative_vehicle_location = None
            
            if len(left_neighbors_id_list) == 0:
                agent.left_parallel_vehicle_id_list.append("None")
                agent.left_parallel_vehicle_relative_location_list.append("None")
                agent.left_parallel_vehicle_front_bumper_gap_list.append("None")
                agent.left_parallel_vehicle_spacing_list.append("None")
                agent.left_parallel_vehicle_location_list.append("None")
                agent.left_parallel_vehicle_speed_list.append("None")
                agent.left_parallel_vehicle_acc_list.append("None")
            else:        
               cloest_vehicle_id = None
               cloest_vehicle_distance = 9999
               cloest_vehicle_gap = 9999
               cloest_vehicle_location = None
               cloest_vehicle_yaw = None
               cloest_vehicle_speed = None
               cloest_vehicle_acc = None
               cloest_vehicle_lane_id = None
               cloest_vehicle_shape = None
               cloest_relative_vehicle_location = None
               
               for left_lane_id in left_neighbors_id_list:
                   left_lane = scenario.center_lane_dict[left_lane_id]                   
                   number_of_points_in_left_lane = len(left_lane.points_list)
                   
                   for surrounding_vehicle in scenario.agents_list:                      
                       # get surrounding agents info
                       if surrounding_vehicle.states_array.size == 0 or surrounding_vehicle.agent_type == 2 or surrounding_vehicle.agent_type == 3:
                           continue                         
                       surrounding_vehicle_state = surrounding_vehicle.states_array[state_index, :]
                       surrounding_vehicle_id = surrounding_vehicle_state[1]
                       if surrounding_vehicle_id == agent_id:
                           continue      
                       surrounding_vehicle_map_matching_lane_id_list = surrounding_vehicle.unique_matching_agent_lane_id_list
                       if len(surrounding_vehicle_map_matching_lane_id_list) != 0:
                           surrounding_vehicle_map_matching_lane_id = surrounding_vehicle_map_matching_lane_id_list[state_index]
                           if surrounding_vehicle_map_matching_lane_id == left_lane_id:   
                               # surrounding vehicle location
                               surrounding_vehicle_state_x = surrounding_vehicle_state[3]
                               surrounding_vehicle_state_y = surrounding_vehicle_state[4]
                               surrounding_vehicle_speed = math.sqrt(surrounding_vehicle_state[10]**2 + surrounding_vehicle_state[11]**2) 
                               # surrounding vehicle shape
                               surrounding_vehicle_length = surrounding_vehicle_state[6]
                               surrounding_vehicle_width = surrounding_vehicle_state[7]
                               surrounding_vehicle_height = surrounding_vehicle_state[8]
                               
                               surrounding_vehicle_location = (surrounding_vehicle_state_x, surrounding_vehicle_state_y)
                               surrounding_vehicle_shape = (surrounding_vehicle_length, surrounding_vehicle_width, surrounding_vehicle_height)
                               
                               surrounding_vehicle_state_heading = radian_to_degree(surrounding_vehicle_state[9])
                               distance_to_ego_vehicle = calculate_distance_between_two_points(surrounding_vehicle_state_x, surrounding_vehicle_state_y, agent_x, agent_y) 
                               if distance_to_ego_vehicle < cloest_vehicle_distance:
                                   cloest_vehicle_distance = distance_to_ego_vehicle
                                   cloest_vehicle_id = surrounding_vehicle_id
                                   relative_location, vehicle_gap = calculate_relative_vehicle_front_bumper_gap(agent_location, agent_yaw, agent_shape, 
                                                                      surrounding_vehicle_location, surrounding_vehicle_state_heading, surrounding_vehicle_shape, distance_to_ego_vehicle, left_lane_spacing)
                                   cloest_vehicle_gap = vehicle_gap
                                   cloest_vehicle_location = surrounding_vehicle_location
                                   cloest_vehicle_lane_id = left_lane_id
                                   cloest_vehicle_shape = surrounding_vehicle_shape
                                   cloest_vehicle_yaw = surrounding_vehicle_state_heading
                                   cloest_vehicle_speed = surrounding_vehicle_speed
                                   cloest_relative_vehicle_location = relative_location
              
            # left lane parallel vehicle
            if cloest_vehicle_id != None:
                agent.left_parallel_lane_id_list.append(cloest_vehicle_lane_id)
                agent.left_parallel_vehicle_id_list.append(cloest_vehicle_id)
                agent.left_parallel_vehicle_front_bumper_gap_list.append(cloest_vehicle_gap)
                agent.left_parallel_vehicle_spacing_list.append(cloest_vehicle_distance)
                agent.left_parallel_vehicle_location_list.append(cloest_vehicle_location)
                agent.left_parallel_vehicle_relative_location_list.append(cloest_relative_vehicle_location)
                agent.left_parallel_vehicle_speed_list.append(cloest_vehicle_speed)
            else:
                agent.left_parallel_lane_id_list.append("None")
                agent.left_parallel_vehicle_id_list.append("None")
                agent.left_parallel_vehicle_front_bumper_gap_list.append("None")
                agent.left_parallel_vehicle_spacing_list.append("None")
                agent.left_parallel_vehicle_location_list.append("None")
                agent.left_parallel_vehicle_relative_location_list.append("None")
                agent.left_parallel_vehicle_speed_list.append("None")
            
                   
            # stage4: find the preceeding and following vehicles of the parallel vehicle above on the left lane on certain timestep                       
            
            agents_on_the_same_left_lane_list = []
            agents_on_the_same_left_lane_distance_to_parallel_vehicle_list = []
            for surrounding_vehicle in scenario.agents_list:  
                if surrounding_vehicle.states_array.size == 0 or surrounding_vehicle.agent_type == 2 or surrounding_vehicle.agent_type == 3:
                    continue                   
                # get surrounding agents info
                surrounding_vehicle_state = surrounding_vehicle.states_array[state_index, :]
                surrounding_vehicle_id = surrounding_vehicle_state[1]
                surrounding_vehicle_length = surrounding_vehicle_state[6]
                               
                if surrounding_vehicle_id == agent_id or surrounding_vehicle_id == cloest_vehicle_id:
                    continue      
                surrounding_vehicle_map_matching_lane_id_list = surrounding_vehicle.unique_matching_agent_lane_id_list
                if len(surrounding_vehicle_map_matching_lane_id_list) != 0:
                    surrounding_vehicle_map_matching_lane_id = surrounding_vehicle_map_matching_lane_id_list[state_index]
                    
                    #if agent_id == 1538:
                    #    if state_index == 71:
                    #        if surrounding_vehicle_id == 1500:
                    #            print(" --- current vehicle id is: ", agent_id)
                    #           print(" --- left vehicle id is: ", surrounding_vehicle_id)
                    #            print(" --- map_matching_lane_id is : ", surrounding_vehicle_map_matching_lane_id)
                    #           print(" --- cloest left lane id is : ", cloest_vehicle_lane_id)
                        
                    if surrounding_vehicle_map_matching_lane_id == cloest_vehicle_lane_id:                        
                        surrounding_vehicle_state_x = surrounding_vehicle_state[3]
                        surrounding_vehicle_state_y = surrounding_vehicle_state[4]
                        surrounding_vehicle_state_heading = radian_to_degree(surrounding_vehicle_state[9])
                        distance_to_left_parallel_vehicle = calculate_distance_between_two_points(surrounding_vehicle_state_x, surrounding_vehicle_state_y, cloest_vehicle_location[0], cloest_vehicle_location[1]) 
                        agents_on_the_same_left_lane_list.append(surrounding_vehicle)               
                        agents_on_the_same_left_lane_distance_to_parallel_vehicle_list.append(distance_to_ego_vehicle) 
            
            preceeding_vehicle_id, \
            preceeding_vehicle_location, \
            preceeding_to_ego_distance, \
            following_vehicle_id, \
            following_vehicle_location, \
            following_to_ego_distance, \
            preceeding_lane_id, \
            following_lane_id    = find_preceeding_following_vehicle_in_lane(state_index,
                                                    cloest_vehicle_location,
                                                    cloest_vehicle_yaw, 
                                                    agents_on_the_same_left_lane_list, 
                                                    agents_on_the_same_left_lane_distance_to_parallel_vehicle_list)
                       
                       
            if preceeding_vehicle_id != None:
                
                preceeding_vehicle = scenario.agents_dict[preceeding_vehicle_id]
                preceeding_vehicle_state = preceeding_vehicle.states_array[state_index, :]              
                preceeding_vehicle_length = preceeding_vehicle_state[6]      
                preceeding_vehicle_speed  = math.sqrt(preceeding_vehicle_state[10]**2 + preceeding_vehicle_state[11]**2) 
                
                agent.left_preceeding_lane_id_list.append(preceeding_lane_id)
                agent.left_preceeding_vehicle_id_list.append(preceeding_vehicle_id)
                agent.left_preceeding_vehicle_distance_list.append(preceeding_to_ego_distance)
                distance_to_current_lane_end_point = calculate_distance_between_two_points_v1(current_lane_end_point, preceeding_vehicle_location)
                agent.left_preceeding_vehicle_distance_to_ego_lane_end_list.append(distance_to_current_lane_end_point)                
                agent.left_preceeding_vehicle_location_list.append(preceeding_vehicle_location)
                gap_distance = preceeding_to_ego_distance - 0.5 * cloest_vehicle_shape[0] - 0.5 * preceeding_vehicle_length
                agent.left_preceeding_vehicle_gap_list.append(gap_distance)
                agent.left_preceeding_vehicle_speed_list.append(preceeding_vehicle_speed)
            else:
                agent.left_preceeding_lane_id_list.append("None")
                agent.left_preceeding_vehicle_id_list.append("None")
                agent.left_preceeding_vehicle_distance_list.append("None")
                agent.left_preceeding_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.left_preceeding_vehicle_location_list.append("None")
                agent.left_preceeding_vehicle_gap_list.append("None")
                agent.left_preceeding_vehicle_speed_list.append("None")
                
            if following_vehicle_id != None:
                
                following_vehicle = scenario.agents_dict[following_vehicle_id]
                following_vehicle_state = following_vehicle.states_array[state_index, :]              
                following_vehicle_length = following_vehicle_state[6]                  
                following_vehicle_speed  = math.sqrt(following_vehicle_state[10]**2 + following_vehicle_state[11]**2) 
                
                agent.left_following_lane_id_list.append(following_lane_id)
                agent.left_following_vehicle_id_list.append(following_vehicle_id)
                agent.left_following_vehicle_distance_list.append(following_to_ego_distance)
                distance_to_current_lane_end_point = calculate_distance_between_two_points_v1(current_lane_end_point, following_vehicle_location)
                agent.left_following_vehicle_distance_to_ego_lane_end_list.append(distance_to_current_lane_end_point)
                agent.left_following_vehicle_location_list.append(following_vehicle_location)
                gap_distance = following_to_ego_distance - 0.5 * cloest_vehicle_shape[0] - 0.5 * following_vehicle_length
                agent.left_following_vehicle_gap_list.append(gap_distance)                
                agent.left_following_vehicle_speed_list.append(following_vehicle_speed)     
            else:
                agent.left_following_lane_id_list.append("None")
                agent.left_following_vehicle_id_list.append("None")
                agent.left_following_vehicle_distance_list.append("None")
                agent.left_following_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.left_following_vehicle_location_list.append("None")
                agent.left_following_vehicle_gap_list.append("None")
                agent.left_following_vehicle_speed_list.append("None")  

            # stage4: find the parallel/cloest vehicle on the right lane on certain timestep                       
            right_neighbors_id_list = map_matching_lane.right_neighbors_id_list
        
            cloest_vehicle_id = None
            cloest_vehicle_location = None
            cloest_to_ego_distance = None
            cloest_vehicle_location = None
            cloest_vehicle_yaw = None
            cloest_vehicle_speed = None
            cloest_vehicle_lane_id = None
            cloest_vehicle_shape = None
            cloest_relative_location = None
            
            if len(right_neighbors_id_list) == 0:
                agent.right_parallel_vehicle_id_list.append("None")
                agent.right_parallel_vehicle_front_bumper_gap_list.append("None")
                agent.right_parallel_vehicle_spacing_list.append("None")
                agent.right_parallel_vehicle_location_list.append("None") 
                agent.right_parallel_vehicle_relative_location_list.append("None")
            else:        
               cloest_vehicle_id = None
               cloest_vehicle_distance = 9999
               cloest_vehicle_gap = 9999
               cloest_vehicle_location = None
               cloest_vehicle_yaw = None
               cloest_vehicle_speed = None
               cloest_vehicle_lane_id = None
               cloest_vehicle_shape = None
               cloest_relative_location = None
               
               for right_lane_id in right_neighbors_id_list:
                   right_lane = scenario.center_lane_dict[right_lane_id]                   
                   number_of_points_in_right_lane = len(right_lane.points_list)
                   
                   for surrounding_vehicle in scenario.agents_list:                      
                       # get surrounding agents info
                       if surrounding_vehicle.states_array.size == 0 or surrounding_vehicle.agent_type == 2 or surrounding_vehicle.agent_type == 3:
                           continue                         
                       surrounding_vehicle_state = surrounding_vehicle.states_array[state_index, :]
                       surrounding_vehicle_id = surrounding_vehicle_state[1]
                       if surrounding_vehicle_id == agent_id:
                           continue      
                       surrounding_vehicle_map_matching_lane_id_list = surrounding_vehicle.unique_matching_agent_lane_id_list
                       if len(surrounding_vehicle_map_matching_lane_id_list) != 0:
                           surrounding_vehicle_map_matching_lane_id = surrounding_vehicle_map_matching_lane_id_list[state_index]
                           if surrounding_vehicle_map_matching_lane_id == right_lane_id:   
                               # surrounding vehicle location
                               surrounding_vehicle_state_x = surrounding_vehicle_state[3]
                               surrounding_vehicle_state_y = surrounding_vehicle_state[4]
                               surrounding_vehicle_speed = math.sqrt(surrounding_vehicle_state[10]**2 + surrounding_vehicle_state[11]**2) 
                               # surrounding vehicle shape
                               surrounding_vehicle_length = surrounding_vehicle_state[6]
                               surrounding_vehicle_width = surrounding_vehicle_state[7]
                               surrounding_vehicle_height = surrounding_vehicle_state[8]
                               
                               surrounding_vehicle_location = (surrounding_vehicle_state_x, surrounding_vehicle_state_y)
                               surrounding_vehicle_shape = (surrounding_vehicle_length, surrounding_vehicle_width, surrounding_vehicle_height)
                               
                               surrounding_vehicle_state_heading = radian_to_degree(surrounding_vehicle_state[9])
                               distance_to_ego_vehicle = calculate_distance_between_two_points(surrounding_vehicle_state_x, surrounding_vehicle_state_y, agent_x, agent_y) 
                               if distance_to_ego_vehicle < cloest_vehicle_distance:
                                   cloest_vehicle_distance = distance_to_ego_vehicle
                                   cloest_vehicle_id = surrounding_vehicle_id
                                   
                                   #if cloest_vehicle_id == 1441 and agent_id == 1416:
                                   #    print(" --- 1416 yaw: ", agent_yaw)
                                   #    print(" --- 1416 right_lane_spacing: ", right_lane_spacing)
                                   
                                   relative_location, vehicle_gap = calculate_relative_vehicle_front_bumper_gap(agent_location, agent_yaw, agent_shape, 
                                                                      surrounding_vehicle_location, surrounding_vehicle_state_heading, surrounding_vehicle_shape, distance_to_ego_vehicle, right_lane_spacing)
                                   cloest_vehicle_gap = vehicle_gap
                                   cloest_vehicle_location = surrounding_vehicle_location
                                   cloest_vehicle_lane_id = right_lane_id
                                   cloest_vehicle_shape = surrounding_vehicle_shape
                                   cloest_vehicle_yaw = surrounding_vehicle_state_heading
                                   cloest_vehicle_speed = surrounding_vehicle_speed
                                   cloest_relative_location = relative_location
              
            # right lane parallel vehicle
            if cloest_vehicle_id != None:
                agent.right_parallel_lane_id_list.append(cloest_vehicle_lane_id)
                agent.right_parallel_vehicle_id_list.append(cloest_vehicle_id)
                agent.right_parallel_vehicle_front_bumper_gap_list.append(cloest_vehicle_gap)
                agent.right_parallel_vehicle_spacing_list.append(cloest_vehicle_distance)
                agent.right_parallel_vehicle_location_list.append(cloest_vehicle_location)
                agent.right_parallel_vehicle_relative_location_list.append(cloest_relative_location)
                agent.right_parallel_vehicle_speed_list.append(cloest_vehicle_speed)
            else:
                agent.right_parallel_lane_id_list.append("None")
                agent.right_parallel_vehicle_id_list.append("None")
                agent.right_parallel_vehicle_front_bumper_gap_list.append("None")
                agent.right_parallel_vehicle_spacing_list.append("None")
                agent.right_parallel_vehicle_location_list.append("None")  
                agent.right_parallel_vehicle_relative_location_list.append("None")
                agent.right_parallel_vehicle_speed_list.append("None")
                    
            # stage6: find the preceeding and following vehicles of the parallel vehicle above on the right lane on certain timestep                       
            
            agents_on_the_same_right_lane_list = []
            agents_on_the_same_right_lane_distance_to_parallel_vehicle_list = []
            for surrounding_vehicle in scenario.agents_list:  
                if surrounding_vehicle.states_array.size == 0 or surrounding_vehicle.agent_type == 2 or surrounding_vehicle.agent_type == 3:
                    continue                   
                # get surrounding agents info
                surrounding_vehicle_state = surrounding_vehicle.states_array[state_index, :]
                surrounding_vehicle_id = surrounding_vehicle_state[1]
                surrounding_vehicle_length = surrounding_vehicle_state[6]
                if surrounding_vehicle_id == agent_id or surrounding_vehicle_id == cloest_vehicle_id:
                    continue      
                surrounding_vehicle_map_matching_lane_id_list = surrounding_vehicle.unique_matching_agent_lane_id_list
                if len(surrounding_vehicle_map_matching_lane_id_list) != 0:
                    surrounding_vehicle_map_matching_lane_id = surrounding_vehicle_map_matching_lane_id_list[state_index]
                    if surrounding_vehicle_map_matching_lane_id == cloest_vehicle_lane_id:                        
                        surrounding_vehicle_state_x = surrounding_vehicle_state[3]
                        surrounding_vehicle_state_y = surrounding_vehicle_state[4]
                        surrounding_vehicle_state_heading = radian_to_degree(surrounding_vehicle_state[9])
                        distance_to_right_parallel_vehicle = calculate_distance_between_two_points(surrounding_vehicle_state_x, surrounding_vehicle_state_y, cloest_vehicle_location[0], cloest_vehicle_location[1]) 
                        agents_on_the_same_right_lane_list.append(surrounding_vehicle)               
                        agents_on_the_same_right_lane_distance_to_parallel_vehicle_list.append(distance_to_ego_vehicle) 
            
            preceeding_vehicle_id, \
            preceeding_vehicle_location, \
            preceeding_to_ego_distance, \
            following_vehicle_id, \
            following_vehicle_location, \
            following_to_ego_distance, \
            preceeding_lane_id, \
            following_lane_id    = find_preceeding_following_vehicle_in_lane(state_index,
                                                    cloest_vehicle_location,
                                                    cloest_vehicle_yaw, 
                                                    agents_on_the_same_right_lane_list, 
                                                    agents_on_the_same_right_lane_distance_to_parallel_vehicle_list)
                       
                       
            if preceeding_vehicle_id != None:
                
                preceeding_vehicle = scenario.agents_dict[preceeding_vehicle_id]
                preceeding_vehicle_state = preceeding_vehicle.states_array[state_index, :]              
                preceeding_vehicle_length = preceeding_vehicle_state[6]      
                preceeding_vehicle_speed  = math.sqrt(preceeding_vehicle_state[10]**2 + preceeding_vehicle_state[11]**2) 
                
                agent.right_preceeding_lane_id_list.append(preceeding_lane_id)
                agent.right_preceeding_vehicle_id_list.append(preceeding_vehicle_id)
                agent.right_preceeding_vehicle_distance_list.append(preceeding_to_ego_distance)
                distance_to_current_lane_end_point = calculate_distance_between_two_points_v1(current_lane_end_point, preceeding_vehicle_location)
                agent.right_preceeding_vehicle_distance_to_ego_lane_end_list.append(distance_to_current_lane_end_point)                
                agent.right_preceeding_vehicle_location_list.append(preceeding_vehicle_location)
                gap_distance = preceeding_to_ego_distance - 0.5 * cloest_vehicle_shape[0] - 0.5 * preceeding_vehicle_length
                agent.right_preceeding_vehicle_gap_list.append(gap_distance)
                agent.right_preceeding_vehicle_speed_list.append(preceeding_vehicle_speed)
            else:
                agent.right_preceeding_lane_id_list.append("None")
                agent.right_preceeding_vehicle_id_list.append("None")
                agent.right_preceeding_vehicle_distance_list.append("None")
                agent.right_preceeding_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.right_preceeding_vehicle_location_list.append("None")
                agent.right_preceeding_vehicle_gap_list.append("None")
                agent.right_preceeding_vehicle_speed_list.append("None")
                
            if following_vehicle_id != None:
                
                following_vehicle = scenario.agents_dict[following_vehicle_id]
                following_vehicle_state = following_vehicle.states_array[state_index, :]              
                following_vehicle_length = following_vehicle_state[6]
                following_vehicle_speed  = math.sqrt(following_vehicle_state[10]**2 + following_vehicle_state[11]**2) 
                
                agent.right_following_lane_id_list.append(following_lane_id)
                agent.right_following_vehicle_id_list.append(following_vehicle_id)
                agent.right_following_vehicle_distance_list.append(following_to_ego_distance)
                distance_to_current_lane_end_point = calculate_distance_between_two_points_v1(current_lane_end_point, following_vehicle_location)
                agent.right_following_vehicle_distance_to_ego_lane_end_list.append(distance_to_current_lane_end_point)
                agent.right_following_vehicle_location_list.append(following_vehicle_location)
                gap_distance = following_to_ego_distance - 0.5 * cloest_vehicle_shape[0] - 0.5 * following_vehicle_length
                agent.right_following_vehicle_gap_list.append(gap_distance)                
                agent.right_following_vehicle_speed_list.append(following_vehicle_speed)  
            else:
                agent.right_following_lane_id_list.append("None")
                agent.right_following_vehicle_id_list.append("None")
                agent.right_following_vehicle_distance_list.append("None")
                agent.right_following_vehicle_distance_to_ego_lane_end_list.append("None")
                agent.right_following_vehicle_location_list.append("None")
                agent.right_following_vehicle_gap_list.append("None")
                agent.right_following_vehicle_speed_list.append("None") 
            
            state_index += 1

