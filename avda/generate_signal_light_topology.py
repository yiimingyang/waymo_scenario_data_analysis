#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from .waymo_training_math_tools import *
from avda import waymo_parse_training_data as transfer_tool

def get_signal_light_type_and_state(signal_state):
    signal_color = None
    singal_shape = None
    signal_show  = None 
    
    if signal_state == 0:
        return signal_color, singal_shape, signal_show
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
    return signal_color, singal_shape, signal_show


def map_and_calculate_distance_signal_light(scenario):
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
                continue            
            agent_x = state[3]
            agent_y = state[4]
            agent_z = state[5] 
            agent_x_list.append(agent_x)
            agent_y_list.append(agent_y)
            state_index += 1
        
        time_step_index = 0
        current_lane_signal_light_shape_list = []
        current_lane_signal_light_type_list = []
        current_lane_signal_light_state_list = []
        current_lane_signal_light_distance_list = []
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
                    continue
                lane_signal_state = scenario_signal_light_state.lane_signal_state_dict[lane_id]
                signal_light_x = lane_signal_state["x"]
                signal_light_y = lane_signal_state["y"]
                signal_light_z = lane_signal_state["z"]
                signal_state = lane_signal_state["signal_state"]
                
                if signal_state == 0:
                    current_lane_signal_light_shape_list.append("None")
                    current_lane_signal_light_type_list.append("None")
                    current_lane_signal_light_state_list.append("None")
                    current_lane_signal_light_distance_list.append("None")                  
                else:
                    vehicle_x = agent_x_list[time_step_index]
                    vheicle_y = agent_y_list[time_step_index]
                    distance_to_signal_light = calculate_distance_between_two_points(signal_light_x, signal_light_y, vehicle_x, vheicle_y)
                    current_lane_signal_light_distance_list.append(distance_to_signal_light)
                    
                    if signal_state == 1:
                        current_lane_signal_light_shape_list.append("arrow")
                        current_lane_signal_light_type_list.append("steady")
                        current_lane_signal_light_state_list.append("red")
                        
                    if signal_state == 2:
                        current_lane_signal_light_shape_list.append("arrow")
                        current_lane_signal_light_type_list.append("steady")
                        current_lane_signal_light_state_list.append("yellow")
                    if signal_state == 3:
                        current_lane_signal_light_shape_list.append("arrow")
                        current_lane_signal_light_type_list.append("steady")
                        current_lane_signal_light_state_list.append("green")
    
                    if signal_state == 4:
                        current_lane_signal_light_shape_list.append("circle")
                        current_lane_signal_light_type_list.append("steady")
                        current_lane_signal_light_state_list.append("red")
                    if signal_state == 5:
                        current_lane_signal_light_shape_list.append("circle")
                        current_lane_signal_light_type_list.append("steady")
                        current_lane_signal_light_state_list.append("yellow")
                    if signal_state == 6:
                        current_lane_signal_light_shape_list.append("circle")
                        current_lane_signal_light_type_list.append("steady")
                        current_lane_signal_light_state_list.append("green")
    
                    if signal_state == 7:
                        current_lane_signal_light_shape_list.append("circle")
                        current_lane_signal_light_type_list.append("flashing")
                        current_lane_signal_light_state_list.append("green")
                    if signal_state == 8:
                        current_lane_signal_light_shape_list.append("circle")
                        current_lane_signal_light_type_list.append("flashing")
                        current_lane_signal_light_state_list.append("yellow")
            time_step_index += 1
                
        agent.current_lane_signal_light_shape_list = current_lane_signal_light_shape_list
        agent.current_lane_signal_light_type_list = current_lane_signal_light_type_list
        agent.current_lane_signal_light_state_list = current_lane_signal_light_state_list
        agent.current_lane_signal_light_distance_list = current_lane_signal_light_distance_list


def get_inbound_turn_lane_id_list_with_turn_type(scenario, direction, turn_type):
    result_list = None
    if turn_type == None:
        if direction == "up":
            result_list = scenario.up_through_turn_lane_id_list + scenario.up_left_turn_lane_id_list + scenario.up_right_turn_lane_id_list
            return result_list
        if direction == "down":
            result_list = scenario.down_through_turn_lane_id_list + scenario.down_left_turn_lane_id_list + scenario.down_right_turn_lane_id_list
            return result_list    
        if direction == "left":
            result_list = scenario.left_through_turn_lane_id_list + scenario.left_left_turn_lane_id_list + scenario.left_right_turn_lane_id_list
            return result_list    
        if direction == "right":
            result_list = scenario.right_through_turn_lane_id_list + scenario.right_left_turn_lane_id_list + scenario.right_right_turn_lane_id_list
            return result_list


def get_inbound_lane_id_list_with_turn_type(scenario, direction, turn_type):
    result_list = None
    if turn_type == None:
        if direction == "up":
            result_list = scenario.lane_up_inbound_id_list
            return result_list
        if direction == "down":
            result_list = scenario.lane_down_inbound_id_list
            return result_list    
        if direction == "left":
            result_list = scenario.lane_left_inbound_id_list
            return result_list    
        if direction == "right":
            result_list = scenario.lane_right_inbound_id_list
            return result_list


def determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, direction1, direction2, direction3 = None, direction4 = None):
    
    green_phase_direction = None
    direction1_inbound_lane_id_list = None
    direction2_inbound_lane_id_list = None
    direction3_inbound_lane_id_list = None
    direction4_inbound_lane_id_list = None

    direction1_inbound_turn_lane_id_list = None
    direction2_inbound_turn_lane_id_list = None
    direction3_inbound_turn_lane_id_list = None
    direction4_inbound_turn_lane_id_list = None
    
    direction1_inbound_lane_id_list = get_inbound_lane_id_list_with_turn_type(scenario, direction1, None)
    direction2_inbound_lane_id_list = get_inbound_lane_id_list_with_turn_type(scenario, direction2, None)
    if direction3 != None:
        direction3_inbound_lane_id_list = get_inbound_lane_id_list_with_turn_type(scenario, direction3, None)
    if direction4 != None:
        direction4_inbound_lane_id_list = get_inbound_lane_id_list_with_turn_type(scenario, direction4, None)
    
    direction1_inbound_turn_lane_id_list = get_inbound_turn_lane_id_list_with_turn_type(scenario, direction1, None)
    direction2_inbound_turn_lane_id_list = get_inbound_turn_lane_id_list_with_turn_type(scenario, direction2, None)
    if direction3 != None:
        direction3_inbound_turn_lane_id_list = get_inbound_turn_lane_id_list_with_turn_type(scenario, direction3, None)
    if direction4 != None:
        direction4_inbound_turn_lane_id_list = get_inbound_turn_lane_id_list_with_turn_type(scenario, direction4, None)

    if direction3 == 'down' and time_step == 1:
        print("   --- frame 1 down direction inbound lane list: ", direction3_inbound_lane_id_list)
        print("   --- frame 1 down direction inbound turn lane list: ", direction3_inbound_turn_lane_id_list)

    #if time_step == 24:
    #    print("   --- direction 1 is: ", direction1)
    #    print("   --- direction 2 is: ", direction2)
    #    print("   --- direction 1 inbound turn lane id is: ", direction1_inbound_lane_id_list)
    #    print("   --- direction 2 inbound turn lane id is: ", direction2_inbound_lane_id_list)
    
    direction_1_stop_vehicle_count = 0
    direction_2_stop_vehicle_count = 0
    direction_3_stop_vehicle_count = 0
    direction_4_stop_vehicle_count = 0    
    
    direction_1_vehicle_count = 0
    direction_2_vehicle_count = 0
    direction_3_vehicle_count = 0
    direction_4_vehicle_count = 0
    
    for agent in scenario.agents_list:
        #if agent.agent_id == 1502 and time_step == 24:
        #    print("   --- agents 1502 at 24 : ", agent.agent_id)
        #    print("   --- agents 1502 state size : ", agent.states_array.size)
        #    print("   --- agents 1502 type : ", agent.agent_type)
        #    print("   --- agents 1502 map matching id : ", agent.unique_matching_agent_lane_id_list[time_step])
        
        if agent.states_array.size == 0:
            continue   
        if agent.agent_type == 2 or agent.agent_type == 3:
            continue       
        map_matching_lane_id = agent.unique_matching_agent_lane_id_list[time_step]
        map_matching_lane_id_list = agent.matching_agent_lane_id_list[time_step]
        if len(map_matching_lane_id_list) == 0:
            continue
        
        #if agent.agent_id == 1418 and time_step == 13:
        #    print("   --- 1418 mapped lane id is : ", map_matching_lane_id)
        #if agent.agent_id == 1502 and time_step == 24:
        #    print("   --- 1502 mapped lane id is : ", map_matching_lane_id)
        #if agent.agent_id == 1509 and time_step == 24:
        #    print("   --- 1509 mapped lane id is : ", map_matching_lane_id)            
        
        x = agent.matching_agent_x_list[time_step]
        y = agent.matching_agent_y_list[time_step]
        z = agent.matching_agent_z_list[time_step]

        speed_x = agent.matching_agent_speed_x_list[time_step]  
        speed_y = agent.matching_agent_speed_y_list[time_step]  
        speed = agent.matching_agent_speed_list[time_step]  

        if agent.agent_id == 227 and time_step == 1:
            print("   --- 227 speed is : ", speed)
        
        if agent.agent_id == 227 and time_step == 1:
            print("   --- up inbound list is : ", direction1_inbound_lane_id_list)        
        
        if direction1_inbound_lane_id_list != None:
            if map_matching_lane_id in direction1_inbound_lane_id_list:
                if speed < 0.5:
                    direction_1_stop_vehicle_count += 1
                #if agent.agent_id == 1418 and time_step == 13:
                #    print("   --- 1418 speed is : ", speed)

        if direction2_inbound_lane_id_list != None:
            if map_matching_lane_id in direction2_inbound_lane_id_list:
                if speed < 0.5:
                    direction_2_stop_vehicle_count += 1        

        if direction3_inbound_lane_id_list != None:
            if map_matching_lane_id in direction3_inbound_lane_id_list:
                if speed < 0.5:
                    direction_3_stop_vehicle_count += 1

        if direction4_inbound_lane_id_list != None:
            if map_matching_lane_id in direction4_inbound_lane_id_list:
                if speed < 0.5:
                    direction_4_stop_vehicle_count += 1

    vehicle_stop_count_list = [direction_1_stop_vehicle_count, direction_2_stop_vehicle_count, direction_3_stop_vehicle_count, direction_4_stop_vehicle_count]

    if direction3 == 'down' and time_step == 1:
        print("   --- frame 13 vehicle stop count list: ", vehicle_stop_count_list)
    
    if direction3 == 'down' and time_step == 1:
        print("   --- frame 13 green phase direction: ", green_phase_direction)
    
    max_value = max(vehicle_stop_count_list)
    max_index = vehicle_stop_count_list.index(max_value)
    if max_index == 0:
        green_phase_direction = direction2
    if max_index == 1:
        green_phase_direction = direction1
    if max_index == 2:
        green_phase_direction = direction4
    if max_index == 3:
        green_phase_direction = direction3  

        '''
        if green_phase_direction == None:
            if direction1_inbound_lane_id_list != None:
                if map_matching_lane_id in direction1_inbound_turn_lane_id_list:
                #common_elements = np.intersect1d(map_matching_lane_id_list, direction1_inbound_turn_lane_id_list)               
                #if len(common_elements) > 0:
                    green_phase_direction = direction1
                    direction_1_vehicle_count += 1
            if direction2_inbound_lane_id_list != None:
                if map_matching_lane_id in direction2_inbound_turn_lane_id_list:
                #common_elements = np.intersect1d(map_matching_lane_id_list, direction2_inbound_turn_lane_id_list)
                #if len(common_elements) > 0:
                    green_phase_direction = direction2
                    direction_2_vehicle_count += 1
            if direction3_inbound_lane_id_list != None:
                if map_matching_lane_id in direction3_inbound_turn_lane_id_list:
                #common_elements = np.intersect1d(map_matching_lane_id_list, direction3_inbound_turn_lane_id_list)
                #if len(common_elements) > 0:
                    green_phase_direction = direction3
                    direction_3_vehicle_count += 1
            if direction4_inbound_lane_id_list != None:
                if map_matching_lane_id in direction4_inbound_turn_lane_id_list:
                #common_elements = np.intersect1d(map_matching_lane_id_list, direction4_inbound_turn_lane_id_list)
                #if len(common_elements) > 0:
                    green_phase_direction = direction4
                    direction_4_vehicle_count += 1
            
        vehicle_count_list = [direction_1_vehicle_count, direction_2_vehicle_count, direction_3_vehicle_count, direction_4_vehicle_count]
        max_value = max(vehicle_count_list)
        max_index = vehicle_count_list.index(max_value)
        if max_index == 0:
            green_phase_direction = direction1
        if max_index == 1:
            green_phase_direction = direction2
        if max_index == 2:
            green_phase_direction = direction3
        if max_index == 3:
            green_phase_direction = direction4  
        '''
     
    return green_phase_direction


def initial_lane_signal_light_state_dict(scenario):
    total_time_step = len(scenario.timestep_list)
    time_index = 0
    while time_index < total_time_step:
        for center_lane in scenario.center_lane_list:
            center_lane.signal_light_color_dict[time_index] = "None"
        time_index += 1


def change_signal_light_state_of_inbound_lanes(scenario, time_step, direction, signal_light_color):

    if direction == "up":
        for center_lane_id in scenario.lane_up_inbound_turn_id_list:
            center_lane = scenario.center_lane_dict[center_lane_id]
            center_lane.signal_light_color_dict[time_step] = signal_light_color          
    if direction == "down":
        for center_lane_id in scenario.lane_down_inbound_turn_id_list:
            center_lane = scenario.center_lane_dict[center_lane_id]
            center_lane.signal_light_color_dict[time_step] = signal_light_color
    if direction == "left":
        for center_lane_id in scenario.lane_left_inbound_turn_id_list:
            center_lane = scenario.center_lane_dict[center_lane_id]
            center_lane.signal_light_color_dict[time_step] = signal_light_color
    if direction == "right":
        for center_lane_id in scenario.lane_right_inbound_turn_id_list:
            center_lane = scenario.center_lane_dict[center_lane_id]
            center_lane.signal_light_color_dict[time_step] = signal_light_color
            

def calculate_and_assign_signal_phase_from_signal_ligth(scenario, time_step, up_signal_phase, down_signal_phase, left_signal_phase, right_signal_phase):
    
    up_direction_color = None
    down_direction_color = None
    left_direction_color = None
    right_direction_color = None
    
    # green condition
    # two direction
    if "green" in right_signal_phase and "green" in down_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")  
        
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1        

    if "green" in right_signal_phase and "green" in up_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")   

        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1    
        
    if "green" in right_signal_phase and "green" in left_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")   
    
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1    
    
    if "green" in left_signal_phase and "green" in up_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")   
        
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        
    if "green" in left_signal_phase and "green" in down_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")           
        
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        
    if "green" in up_signal_phase and "green" in down_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")  

        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        

    # green condition
    # one direction
    if "green" in right_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")  
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red") 
        
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        
    
    if "green" in left_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red") 

        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        
    if "green" in up_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")              

        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1

    if "green" in down_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")              
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red") 
        
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        
    # if we have signal light information of two direction 
    if "red" in right_signal_phase and "red" in down_signal_phase:
        down_direction_color = 1
        right_direction_color = 1
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "up", "left")
        if green_phase_direction == "up":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            
            up_direction_color = 1          
            left_direction_color = 1
          
        if green_phase_direction == "left":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "green")
            
            up_direction_color = 1
            left_direction_color = 1
        
            #if time_step == 24:
            #    print("   --- green phase direction is", green_phase_direction)
                #print("   --- down signal phase is", down_signal_phase)

    if "red" in right_signal_phase and "red" in up_signal_phase:
        up_direction_color = 1
        right_direction_color = 1
        
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "down", "left")
        if green_phase_direction == "down":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")            
            
            down_direction_color = 1
            left_direction_color = 1
                     
            
        if green_phase_direction == "left":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "green")
            
            down_direction_color = 1
            left_direction_color = 1

    if "red" in left_signal_phase and "red" in up_signal_phase:
        up_direction_color = 1
        left_direction_color = 1
        
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "down", "right")
        if green_phase_direction == "down":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            
            
            down_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "right":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "green")
            
            down_direction_color = 1
            right_direction_color = 1

    if "red" in left_signal_phase and "red" in down_signal_phase:
        down_direction_color = 1
        left_direction_color = 1
        
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "up", "right")
        if green_phase_direction == "up":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            
            up_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "right":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "green")
            
            up_direction_color = 1
            right_direction_color = 1

    if "red" in right_signal_phase and "red" in left_signal_phase:
        left_direction_color = 1
        right_direction_color = 1
        
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "green")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "green") 
        
        up_direction_color = 1
        down_direction_color = 1


    if "red" in up_signal_phase and "red" in down_signal_phase:
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "green")
        change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "green")
        
        up_direction_color = 1
        down_direction_color = 1
        left_direction_color = 1
        right_direction_color = 1
        
    # if we have signal light information of only one direction 
    if "red" in right_signal_phase and "red" not in down_signal_phase and "red" not in left_signal_phase  and "red" not in up_signal_phase:
        right_direction_color = 1

        if time_step == 1:
            print("   --- frame 1 ")
            
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "up", "left", "down")
        if green_phase_direction == "up":
            
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            
            up_direction_color = 1
            down_direction_color = 1
            left_direction_color = 1
            
        if green_phase_direction == "left":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            
            up_direction_color = 1
            down_direction_color = 1
            left_direction_color = 1
            
        if green_phase_direction == "down":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            
            up_direction_color = 1
            down_direction_color = 1
            left_direction_color = 1
            

    if "red" in left_signal_phase and "red" not in down_signal_phase and "red" not in right_signal_phase  and "red" not in up_signal_phase:
        left_direction_color = 1
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "up", "right", "down")
        if green_phase_direction == "up":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            
            up_direction_color = 1
            down_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "right":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            
            up_direction_color = 1
            down_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "down":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            
            up_direction_color = 1
            down_direction_color = 1
            right_direction_color = 1
            
            
    if "red" in up_signal_phase and "red" not in down_signal_phase and "red" not in right_signal_phase  and "red" not in left_signal_phase:
        up_direction_color = 1

        if time_step == 1:
            print("   --- frame 1 ")
        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "left", "right", "down")
        if green_phase_direction == "left":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            
            down_direction_color = 1
            left_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "right":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "red")
            
            down_direction_color = 1
            left_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "down":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "down", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red") 
            
            down_direction_color = 1
            left_direction_color = 1
            right_direction_color = 1
            

    if "red" in down_signal_phase and "red" not in up_signal_phase and "red" not in right_signal_phase  and "red" not in left_signal_phase:
        down_direction_color = 1

        green_phase_direction = determine_if_vehicle_on_certain_type_of_lane(scenario, time_step, "left", "right", "down")
        if green_phase_direction == "left":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            
            up_direction_color = 1
            left_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "right":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "red")
            
            up_direction_color = 1
            left_direction_color = 1
            right_direction_color = 1
            
        if green_phase_direction == "up":
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "up", "green")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "left", "red")
            change_signal_light_state_of_inbound_lanes(scenario, time_step, "right", "red")
            
            up_direction_color = 1
            left_direction_color = 1
            right_direction_color = 1

    if up_direction_color == 1 and down_direction_color == 1 and left_direction_color == 1 and right_direction_color == 1:
        return True
    elif up_direction_color == None and down_direction_color == None and left_direction_color == None and right_direction_color == None:
        return True
    else:
        return False

def calculate_four_direction_turn_lane(scenario):
    for center_lane_id in scenario.lane_up_inbound_id_list:
        this_lane = scenario.center_lane_dict[center_lane_id]
        if len(this_lane.exit_lane_id_list) != 0:
            for exist_lane_id in this_lane.exit_lane_id_list:
                exist_lane = scenario.center_lane_dict[exist_lane_id]
                if exist_lane.is_straight_turn or exist_lane.is_left_turn or exist_lane.is_right_turn or exist_lane.is_u_turn:
                    scenario.lane_up_inbound_turn_id_list.append(exist_lane_id)
    
    for center_lane_id in scenario.lane_down_inbound_id_list:
        this_lane = scenario.center_lane_dict[center_lane_id]
        if len(this_lane.exit_lane_id_list) != 0:
            for exist_lane_id in this_lane.exit_lane_id_list:
                exist_lane = scenario.center_lane_dict[exist_lane_id]
                if exist_lane.is_straight_turn or exist_lane.is_left_turn or exist_lane.is_right_turn or exist_lane.is_u_turn:
                    scenario.lane_down_inbound_turn_id_list.append(exist_lane_id)
                    
    for center_lane_id in scenario.lane_left_inbound_id_list:
        this_lane = scenario.center_lane_dict[center_lane_id]
        if len(this_lane.exit_lane_id_list) != 0:
            for exist_lane_id in this_lane.exit_lane_id_list:
                exist_lane = scenario.center_lane_dict[exist_lane_id]
                if exist_lane.is_straight_turn or exist_lane.is_left_turn or exist_lane.is_right_turn or exist_lane.is_u_turn:
                    scenario.lane_left_inbound_turn_id_list.append(exist_lane_id)

    #print(" --- lane right inbound id list is: ", scenario.lane_right_inbound_id_list)
    for center_lane_id in scenario.lane_right_inbound_id_list:
        this_lane = scenario.center_lane_dict[center_lane_id]
        if len(this_lane.exit_lane_id_list) != 0:
            #print("   --- has exist lane id is: ", center_lane_id)
            #if center_lane_id == 209:
            #    print("   --- 209 lane exist lane list :", this_lane.exit_lane_id_list)
            for exist_lane_id in this_lane.exit_lane_id_list:
                exist_lane = scenario.center_lane_dict[exist_lane_id]
                #if center_lane_id == 209:
                    #print("   --- 209 lane exist lane straight :", exist_lane.is_straight_turn)
                    #print("   --- 209 lane exist lane right :", exist_lane.is_right_turn)
                    #print("   --- 209 lane exist lane left :", exist_lane.is_left_turn)
                if exist_lane.is_straight_turn or exist_lane.is_left_turn or exist_lane.is_right_turn or exist_lane.is_u_turn:
                    #print("   --- exist lane id is: ", center_lane_id)
                    scenario.lane_right_inbound_turn_id_list.append(exist_lane_id)                    

def calculate_signal_light_topology_for_each_lane(scenario, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset):

    lane_up_inbound_id_list   = scenario.lane_up_inbound_turn_id_list       
    lane_down_inbound_id_list  = scenario.lane_down_inbound_turn_id_list       
    lane_left_inbound_id_list  = scenario.lane_left_inbound_turn_id_list       
    lane_right_inbound_id_list  = scenario.lane_right_inbound_turn_id_list  

    #print(" --- lane up inbound id list is: ", lane_up_inbound_id_list)
    #print(" --- lane down inbound id list is: ", lane_down_inbound_id_list)
    #print(" --- lane left inbound id list is: ", lane_left_inbound_id_list)
    #print(" --- lane right inbound id list is: ", lane_right_inbound_id_list)
    
    #print("   ---  up through turn lan id list is: ", scenario.up_through_turn_lane_id_list)
    #print("   ---  up left turn lan id list is: ", scenario.up_through_turn_lane_id_list)
    #print("   ---  up right turn lan id list is: ", scenario.up_through_turn_lane_id_list)
    
    #print("   ---  down through turn lan id list is: ", scenario.down_through_turn_lane_id_list)
    #print("   ---  down left turn lan id list is: ", scenario.down_through_turn_lane_id_list)
    #print("   ---  down right turn lan id list is: ", scenario.down_through_turn_lane_id_list)    

    #print("   ---  left through turn lan id list is: ", scenario.left_through_turn_lane_id_list)
    #print("   ---  left left turn lan id list is: ", scenario.left_through_turn_lane_id_list)
    #print("   ---  left right turn lan id list is: ", scenario.left_through_turn_lane_id_list)   
    
    #print("   ---  right through turn lan id list is: ", scenario.right_through_turn_lane_id_list)
    #print("   ---  right left turn lan id list is: ", scenario.right_through_turn_lane_id_list)
    #print("   ---  right right turn lan id list is: ", scenario.right_through_turn_lane_id_list)       
    
    #print("   !!! ---  lane_left_inbound_list is: ", scenario.lane_left_inbound_list)
    
    left_inbound_lane_straight_signal_state_list = []
    left_inbound_lane_left_turn_signal_state_list = []
    left_inbound_lane_right_turn_signal_state_list = []
    
    left_inbound_lane_straight_signal_state_list = []
    left_inbound_lane_left_turn_signal_state_list = []
    left_inbound_lane_right_turn_signal_state_list = []
  
    left_inbound_lane_straight_signal_state_list = []
    left_inbound_lane_left_turn_signal_state_list = []
    left_inbound_lane_right_turn_signal_state_list = []

    left_inbound_lane_straight_signal_state_list = []
    left_inbound_lane_left_turn_signal_state_list = []
    left_inbound_lane_right_turn_signal_state_list = []
    
    total_time_index = len(scenario.signal_light_state_list)
    
    failure_times = 0
    time_index = 0
    while time_index < total_time_index:
        single_step_signal_state = scenario.signal_light_state_dict[time_index]
        lane_id_list_at_this_time = single_step_signal_state.lane_id_list
        signal_state_dict_at_this_time = single_step_signal_state.lane_signal_state_dict
        if len(lane_id_list_at_this_time) == 0:
            left_inbound_lane_straight_signal_state_list.append("None")
            left_inbound_lane_left_turn_signal_state_list.append("None")
            left_inbound_lane_right_turn_signal_state_list.append("None")
            
            left_inbound_lane_straight_signal_state_list.append("None")
            left_inbound_lane_left_turn_signal_state_list.append("None")
            left_inbound_lane_right_turn_signal_state_list.append("None")
          
            left_inbound_lane_straight_signal_state_list.append("None")
            left_inbound_lane_left_turn_signal_state_list.append("None")
            left_inbound_lane_right_turn_signal_state_list.append("None")
        
            left_inbound_lane_straight_signal_state_list.append("None")
            left_inbound_lane_left_turn_signal_state_list.append("None")
            left_inbound_lane_right_turn_signal_state_list.append("None")    
        else:
            up_signal_through_state = None
            up_signal_left_turn_state = None
            up_signal_right_turn_state = None
            
            down_signal_through_state = None
            down_signal_left_turn_state = None
            down_signal_right_turn_state = None
            
            left_signal_through_state = None
            left_signal_left_turn_state = None
            left_signal_right_turn_state = None
            
            right_signal_through_state = None
            right_signal_left_turn_state = None
            right_signal_right_turn_state = None          
            
            for lane_id in lane_id_list_at_this_time:
                #print("   --- lane id is: ",lane_id, " lane type is: ", type(lane_id))
                lane_signal_state = signal_state_dict_at_this_time[lane_id]
                signal_state = lane_signal_state["signal_state"]
                this_lane = scenario.center_lane_dict[lane_id]
                
                if lane_id in lane_up_inbound_id_list:
                    #print("   --- find lane in up: ", lane_id)
                    signal_color, singal_shape, signal_show = get_signal_light_type_and_state(signal_state)
                    if this_lane.is_straight_turn:
                        up_signal_through_state = signal_color,
                    if this_lane.is_left_turn:
                        up_signal_left_turn_state = signal_color
                    if this_lane.is_right_turn:
                        up_signal_right_turn_state = signal_color                            
                            
                if lane_id in lane_down_inbound_id_list:
                    #print("   --- find lane in down: ", lane_id)
                    signal_color, singal_shape, signal_show = get_signal_light_type_and_state(signal_state)
                    if this_lane.is_straight_turn:
                        down_signal_through_state = signal_color
                        #print("   ---  down through color is: ", down_signal_through_state)
                    if this_lane.is_left_turn:
                        down_signal_left_turn_state = signal_color
                        #print("   ---  down left color is: ", down_signal_left_turn_state)
                    if this_lane.is_right_turn:
                        down_signal_right_turn_state = signal_color   
                        #print("   ---  down right color is: ", down_signal_right_turn_state)
                                
                if lane_id in lane_left_inbound_id_list:
                    #print("   --- find lane in left: ", lane_id)
                    signal_color, singal_shape, signal_show = get_signal_light_type_and_state(signal_state)
                    if this_lane.is_straight_turn:
                        left_signal_through_state = signal_color
                    if this_lane.is_left_turn:
                        left_signal_left_turn_state = signal_color
                    if this_lane.is_right_turn:
                        left_signal_right_turn_state = signal_color    
                                
                if lane_id in lane_right_inbound_id_list:
                    #print("   --- find lane in right: ", lane_id)
                    signal_color, singal_shape, signal_show = get_signal_light_type_and_state(signal_state)
                    if this_lane.is_straight_turn:
                        right_signal_through_state = signal_color
                        #print("   ---  right through color is: ", right_signal_through_state)
                    if this_lane.is_left_turn:
                        right_signal_left_turn_state = signal_color
                        #print("   ---  right through color is: ", right_signal_left_turn_state)
                    if this_lane.is_right_turn:
                        right_signal_right_turn_state = signal_color  
                        #print("   ---  right through color is: ", right_signal_right_turn_state)
                        
                    #if time_index == 24:
                    #    print("   --- right_signal_through_state is", signal_color)
                    #    print("   --- right_signal_left_state is", signal_color)
                    #    print("   --- right_signal_right_state is", signal_color)
                    
            up_signal_phase = [up_signal_through_state, up_signal_left_turn_state, up_signal_right_turn_state]
            down_signal_phase = [down_signal_through_state, down_signal_left_turn_state, down_signal_right_turn_state]
            left_signal_phase = [left_signal_through_state, left_signal_left_turn_state, left_signal_right_turn_state]
            right_signal_phase = [right_signal_through_state, right_signal_left_turn_state, right_signal_right_turn_state]
            if time_index == 1:
                print("   --- up signal phase is", up_signal_phase)
                print("   --- down signal phase is", down_signal_phase)
                print("   --- left signal phase is", left_signal_phase)
                print("   --- right signal phase is", right_signal_phase)
            
            calculate_signal_light_success = calculate_and_assign_signal_phase_from_signal_ligth(scenario, time_index, up_signal_phase, down_signal_phase, left_signal_phase, right_signal_phase)
            if calculate_signal_light_success == False:
                failure_times += 1
            
        time_index += 1
    if failure_times > 10:
        return False
    else:
        return True


def summary_signal_light_information(scenario):
    signal_light_id = 0
    
    # north 
    for lane_id in scenario.up_through_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "north_through"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict

        signal_light_id += 1
        
    for lane_id in scenario.up_left_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "north_left"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict

        signal_light_id += 1
    
    for lane_id in scenario.up_right_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "north_right"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)
        
        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict        
        
        signal_light_id += 1        
    
    # south
    for lane_id in scenario.down_through_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "south_through"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
        
    for lane_id in scenario.down_left_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "south_left"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
        
    for lane_id in scenario.down_right_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "south_right"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           

    # east
    for lane_id in scenario.left_through_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "east_through"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
        
    for lane_id in scenario.left_left_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "east_left"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
        
    for lane_id in scenario.left_right_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "east_right"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        scenario.signal_light_list.append(signal_infromation_dict)

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
    
    # west
    for lane_id in scenario.right_through_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "west_through"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        
        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
        
    for lane_id in scenario.right_left_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "west_left"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict
        scenario.signal_light_dict[signal_light_id] = signal_infromation_dict
        
        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1          
        
    for lane_id in scenario.right_right_turn_lane_id_list:
        signal_infromation_dict = {}
        this_lane = scenario.center_lane_dict[lane_id]
        signal_light_location = this_lane.points_list[0]
        signal_infromation_dict["lane_id"] = lane_id
        signal_infromation_dict["lane_type"] = "west_right"
        signal_infromation_dict["link_id"] = this_lane.dict["link_id"]
        signal_infromation_dict["location"] = signal_light_location
        signal_infromation_dict["color"] = this_lane.signal_light_color_dict

        scenario.lane_id_singal_light_list.append(lane_id)
        scenario.lane_id_singal_light_state_list.append(signal_infromation_dict)        
        scenario.lane_singal_light_state_dict[lane_id] = signal_infromation_dict
        
        signal_light_id += 1           
    

def assign_signal_light_to_agents(scenario):
    for agent in scenario.agents_list:
        agent_id = agent.agent_id
        agent_type = agent.agent_type
        
        if agent.states_array.size == 0:
            continue   
        if agent.agent_type == 2 or agent.agent_type == 3:
            continue  
         
        record_number = len(agent.matching_agent_time_list)
        record_index = 0
        while record_index < record_number:
            
            map_matching_lane_id = agent.unique_matching_agent_lane_id_list[record_index]
            if map_matching_lane_id == "" or map_matching_lane_id == None or map_matching_lane_id == "None":
                record_index += 1
                continue
            map_matching_lane = scenario.center_lane_dict[map_matching_lane_id]
        
            if map_matching_lane_id in scenario.lane_id_singal_light_list:
                signal_infromation_dict = scenario.lane_singal_light_state_dict[map_matching_lane_id]
                signal_color_dict = signal_infromation_dict["color"]
                signal_color = signal_color_dict[record_index]
                map_matching_lane.signal_light_color_dict[record_index] = signal_color
                
            for exist_lane_id in map_matching_lane.exit_lane_id_list:
                exist_lane = scenario.center_lane_dict[exist_lane_id]
                if exist_lane_id in scenario.lane_id_singal_light_list:
                    signal_infromation_dict = scenario.lane_singal_light_state_dict[exist_lane_id]
                    signal_color_dict = signal_infromation_dict["color"]        
                    signal_color = signal_color_dict[record_index]
                    map_matching_lane.signal_light_color_dict[record_index] = signal_color
            record_index += 1  
            
            
