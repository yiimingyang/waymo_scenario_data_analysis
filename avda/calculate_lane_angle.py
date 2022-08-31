#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .waymo_training_math_tools import *
from .av_data_classes import *
 
def calculate_lane_angle(scenario):
    for center_lane in scenario.center_lane_list:
        if center_lane.is_left_turn or center_lane.is_right_turn or center_lane.is_u_turn:
            if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:                
                center_lane_id = center_lane.lane_id
                for entry_lane_id in center_lane.entry_lines_list:
                    entry_lane = scenario.center_lane_dict[entry_lane_id]
                    entry_lane_point_list = entry_lane.points_list
                    first_entry_point = entry_lane_point_list[0]
                    last_entry_point  = entry_lane_point_list[-1]
                    entry_lane_coordinate = [first_entry_point, last_entry_point]
                    for exist_lane_id in center_lane.exit_lanes_list:
                        exist_lane = scenario.center_lane_dict[exist_lane_id]
                        exist_lane_point_list = exist_lane.points_list
                        first_exist_point = exist_lane_point_list[0]
                        last_exist_point  = exist_lane_point_list[-1] 
                        exist_lane_coordinate = [first_exist_point, last_exist_point]
                        from_entry_to_exist_angle = get_angle_from_entry_lane_to_exist_lane(entry_lane_coordinate, exist_lane_coordinate)                  
                        from_entry_to_exist_id = str(entry_lane_id) + "-" + str(exist_lane_id)
                        scenario.lane_angle_dict[from_entry_to_exist_id] = from_entry_to_exist_angle
                        

def calculate_current_lane_to_next_lane_angle(scenario):
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type
        
        if agent.states_array.size == 0:
            continue   
        if agent.agent_type == 2 or agent.agent_type == 3:
            continue           
        
        agent_time_list = []
        agent_x_list = []
        agent_y_list = []
        agent_z_list = []

        states = agent.states_array
        first_state = states[0]
        state_number = len(states)
        
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
            agent_x_list.append(agent_x)
            agent_y_list.append(agent_y)
            state_index += 1
        
        time_step_index = 0
        next_lane_id_list = []
        next_lane_angle_list = []

        for lane_id in agent.unique_matching_agent_lane_id_list:
            if lane_id == "None":
                current_lane_signal_light_shape_list.append("None")
                current_lane_signal_light_type_list.append("None")
                current_lane_signal_light_state_list.append("None")
                current_lane_signal_light_distance_list.append("None")
                current_lane_signal_light_distance_list.append("None")            
            else:
                scenario_signal_light_state = scenario.signal_light_state_dict[time_step_index]
                scenario_signal_lane_id_list = scenario_signal_light_state.lane_id_list
                if lane_id not in scenario_signal_lane_id_list:
                    current_lane_signal_light_shape_list.append("None")
                    current_lane_signal_light_type_list.append("None")
                    current_lane_signal_light_state_list.append("None")
                    current_lane_signal_light_distance_list.append("None")
                    current_lane_signal_light_distance_list.append("None")
