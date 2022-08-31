#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from .av_data_classes import *
from .waymo_training_math_tools import *

def calculate_straight_turn_in_crossing(scenario):
    crossing_center_point = (scenario.node_x, scenario.node_y)
    for center_lane in scenario.center_lane_list:
        if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:           
            if center_lane.is_straight:           
                first_point = center_lane.point_list[0]
                second_point = center_lane.point_list[1]
                first_point_x = first_point[0]
                first_point_y = first_point[1]
                second_point_x = second_point[0]
                second_point_y = second_point[1]
                
                last_point = center_lane.point_list[-1]
                last_two_point = center_lane.point_list[-2]
                last_two_point_x  = last_two_point[0]
                last_two_point_y  = last_two_point[1]        
                last_point_x  = last_point[0]
                last_point_y  = last_point[1]
                        
                angle1 = np.rad2deg(np.arctan2(second_point_y - first_point_y, second_point_x - first_point_x))
                angle2 = np.rad2deg(np.arctan2(last_two_point_y - last_point_y, last_two_point_x - last_point_x))                
                is_in_range = is_in_range_for_straight(first_point, last_point, crossing_center_point)    
                if abs(abs(angle2) - abs(angle1)) < 20 and is_in_range: 
                    center_lane.is_straight_turn = True
                   

def generate_lane_level_topology(scenario):
    global_lane_id = 0
    for center_lane in scenario.center_lane_list:
        if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
            
            # find lane direction
            center_lane_id = center_lane.lane_id
            center_lane_link_id = center_lane.link_id
            center_lane_turn_id = center_lane.turn_movement_id
            center_lane_node_id = center_lane.node_id
            center_lane_width = center_lane.center_lane_width
            center_lane_point_list = center_lane.point_list
            first_point_location = center_lane_point_list[0]
            last_point_location = center_lane_point_list[-1]
            
            first_point = Center_lane_point()
            first_point.point_id = global_lane_id
            first_point.point_coordinate = first_point_location
            first_point.point_type = "lane_connection_point_id_a"
            first_point.belonged_lane_list.append(center_lane_id)
            
            first_point.dict["id"] = first_point.point_id
            first_point.dict["coordinate"] = first_point.point_coordinate
            first_point.dict["type"] = "direction"
            first_point.dict["lane"] = first_point.belonged_lane_list
            
            global_lane_id += 1
            
            last_point = Center_lane_point()
            last_point.point_id = global_lane_id
            last_point.point_coordinate = last_point_location
            last_point.point_type = "lane_connection_point_id_b"
            last_point.belonged_lane_list.append(center_lane_id)
            global_lane_id += 1            

            last_point.dict["id"] = last_point.point_id
            last_point.dict["coordinate"] = last_point.point_coordinate
            last_point.dict["type"] = "direction"
            last_point.dict["lane"] = last_point.belonged_lane_list
            
            center_lane.dict["lane_connection_point_id_a"] = first_point.point_id
            center_lane.dict["lane_connection_point_id_b"] = last_point.point_id
            center_lane.dict["lane_direction"] = "AB"
        
            # find lane left lane set and right lane set 
            for left_lane_id in center_lane.left_neighbors_id_list:
                left_lane_dict = {}
                left_lane = scenario.lane_dict[left_lane_id]
                left_lane_point_list = left_lane.point_list
                left_lane_dict["lane_id"]  = left_lane_id
                left_lane_dict["location"] = left_lane_point_list
                left_lane_dict["distance_to_lane_end"] = calculate_distance_between_two_points_v1(first_point_location, left_lane_point_list[-1])
                center_lane.dict["left_lane_set"][left_lane_id] = left_lane_dict
                
            # find lane right lane set and right lane set 
            for right_lane_id in center_lane.right_neighbors_id_list:
                right_lane_dict = {}
                right_lane = scenario.lane_dict[right_lane_id]
                right_lane_point_list = right_lane.point_list
                right_lane_dict["lane_id"]  = right_lane_id
                right_lane_dict["location"] = right_lane_point_list
                right_lane_dict["distance_to_lane_end"] = calculate_distance_between_two_points_v1(first_point_location, right_lane_point_list[-1])
                center_lane.dict["right_lane_set"][right_lane_id] = right_lane_dict
                
            # find all the conflict point of this lane
            if center_lane.is_right_turn or center_lane.is_left_turn or center_lane.is_u_turn or center_lane.is_straight_turn:
                used_lane_id_list = []
                crossing_lane_set_dict = {}
                for other_center_lane in scenario.center_lane_list:
                    if other_center_lane.is_right_turn or other_center_lane.is_left_turn or other_center_lane.is_u_turn or other_center_lane.is_straight_turn:
                        other_center_lane_id = other_center_lane.lane_id
                        conflict_point_x, conflict_point_y, index = find_conflict_point_in_two_list(center_lane.points_list, other_center_lane.points_list)
                        if conflict_point_x != None:
                            crossing_lane_dict = {}
                            crossing_lane_dict["lane_id"] = other_center_lane_id
                            if center_lane.is_right_turn and other_center_lane.is_right_turn:
                                crossing_lane_dict["crossing_type"] = "RIGHT_RIGHT"
                            if center_lane.is_right_turn and other_center_lane.is_left_turn:
                                crossing_lane_dict["crossing_type"] = "RIGHT_LEFT"        
                            if center_lane.is_right_turn and other_center_lane.is_u_turn:
                                crossing_lane_dict["crossing_type"] = "RIGHT_U"                                
                            if center_lane.is_right_turn and other_center_lane.is_straight_turn:
                                crossing_lane_dict["crossing_type"] = "RIGHT_STRAIGHT"    
                                
                            if center_lane.is_left_turn and other_center_lane.is_right_turn:
                                crossing_lane_dict["crossing_type"] = "LEFT_RIGHT"                                
                            if center_lane.is_left_turn and other_center_lane.is_left_turn:
                                crossing_lane_dict["crossing_type"] = "LEFT_LEFT"                                
                            if center_lane.is_left_turn and other_center_lane.is_u_turn:
                                crossing_lane_dict["crossing_type"] = "LEFT_U"                                
                            if center_lane.is_left_turn and other_center_lane.is_straight_turn:
                                crossing_lane_dict["crossing_type"] = "LEFT_STRAIGHT"                                

                            if center_lane.is_u_turn and other_center_lane.is_right_turn:
                                crossing_lane_dict["crossing_type"] = "U_RIGHT"                                
                            if center_lane.is_u_turn and other_center_lane.is_left_turn:
                                crossing_lane_dict["crossing_type"] = "U_LEFT"                                
                            if center_lane.is_u_turn and other_center_lane.is_u_turn:
                                crossing_lane_dict["crossing_type"] = "U_U"                                
                            if center_lane.is_u_turn and other_center_lane.is_straight_turn:
                                crossing_lane_dict["crossing_type"] = "U_STRAIGHT"  

                            if center_lane.is_straight_turn and other_center_lane.is_right_turn:
                                crossing_lane_dict["crossing_type"] = "STRAIGHT_RIGHT"                                
                            if center_lane.is_straight_turn and other_center_lane.is_left_turn:
                                crossing_lane_dict["crossing_type"] = "STRAIGHT_LEFT"                                
                            if center_lane.is_straight_turn and other_center_lane.is_u_turn:
                                crossing_lane_dict["crossing_type"] = "STRAIGHT_U"                                
                            if center_lane.is_straight_turn and other_center_lane.is_straight_turn:
                                crossing_lane_dict["crossing_type"] = "STRAIGHT_STRAIGHT"    
                                
                            crossing_lane_dict["location"] = (conflict_point_x, conflict_point_y)
                            crossing_lane_dict["distance_to_lane_end"] = index * 0.5
                            crossing_lane_dict["rule"] = "PERMITTED"                           
                            crossing_lane_set_dict[other_center_lane_id] = crossing_lane_dict
            center_lane.dict["crossing"] = crossing_lane_set_dict
                        
