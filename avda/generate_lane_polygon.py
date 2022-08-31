#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 26 14:50:07 2022

@author: yimingya
"""

from .waymo_training_math_tools import *
from matplotlib.patches import Polygon
from sympy import Point, Line
import random
import math
from .generate_lane_polygon_from_line import *
from .av_data_classes import *

def find_nearest_point_to_point_list(point0, point_list):
    
    if len(point_list) == 0:
        return None, None
    
    min_distance = 1000
    min_distance_index = 0
    
    index = 0
    for point in point_list:
        distance = calculate_distance_between_two_points_v1(point0, point)
        if distance < min_distance:
            min_distance = distance
            min_distance_index = index
        index += 1
    return min_distance_index, min_distance


# find inbound lane id and outbound lane id for curve/connectors in a crossing
def find_inbound_outbound_lane_for_curve(scenario):
    # go through curve lanes
    for center_lane in scenario.center_lane_list:
        if center_lane.is_left_turn or center_lane.is_right_turn or center_lane.is_u_turn:
            if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:                
                center_lane_id = center_lane.lane_id
                #print(" ---- curve lane id is: ", center_lane_id)
                first_curve_point = center_lane.points_list[0]
                last_curve_point  = center_lane.points_list[-1]                
                # find straight lanes
                for straight_center_lane in scenario.center_lane_list:
                    if straight_center_lane.is_straight:
                        if straight_center_lane.lane_type == MAP_feature.TYPE_FREEWAY or straight_center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
                            straight_center_lane_id = straight_center_lane.lane_id
                            first_point = straight_center_lane.points_list[0]
                            last_point  = straight_center_lane.points_list[-1]
                            first_first_distance = calculate_distance_between_two_points_v1(first_curve_point, first_point)
                            first_last_distance  = calculate_distance_between_two_points_v1(first_curve_point, last_point)
                            last_first_distance  = calculate_distance_between_two_points_v1(last_curve_point, first_point)
                            last_last_distance   = calculate_distance_between_two_points_v1(last_curve_point, last_point)
                            if first_last_distance < 1:
                                center_lane.entry_lines_list.append(straight_center_lane_id) 
                                #print(" ------ entry lane id is: ", straight_center_lane_id)
                            if last_first_distance < 1:
                                center_lane.exit_lanes_list.append(straight_center_lane_id)   
                                #print(" ------ exist lane id is: ", straight_center_lane_id)

                
def calculate_and_determine_if_boundary_straight_or_curve(scenario):
    for center_lane in scenario.center_lane_list:
            
            # find lane direction
            center_lane_id = center_lane.lane_id
           
            # find lane left lane set and right lane set
            for left_segment in center_lane.left_boundary_segment_list:
                left_boundary_line_id = left_segment.boundary_feature_id
                left_boundary_line = scenario.map_feature_dict[left_boundary_line_id]  
                left_boundary_line_point_list = left_boundary_line.points_list
                
                number_of_points_in_this_lane = len(left_boundary_line_point_list)
                first_point_index = 0
                last_point_index = number_of_points_in_this_lane - 1
                middle_point_index = int(0.5 * last_point_index)
                
                if number_of_points_in_this_lane < 4:
                    first_point_x = left_boundary_line_point_list[first_point_index][0]
                    first_point_y = left_boundary_line_point_list[first_point_index][1]
                    last_point_x  = left_boundary_line_point_list[last_point_index][0]
                    last_point_y  = left_boundary_line_point_list[last_point_index][1]      
                    middle_point_x = left_boundary_line_point_list[middle_point_index][0]
                    middle_point_y = left_boundary_line_point_list[middle_point_index][1] 
                    
                    left_boundary_line.is_straight = True
                    a = first_point_y - last_point_y
                    b = last_point_x - first_point_x
                    c = first_point_x * last_point_y - last_point_x * first_point_y
                    
                    if b == 0:
                        left_boundary_line.slope = 9999
                    else:
                        left_boundary_line.slope = -1 * a / b
                    
                    if a == 0 and b == 0:
                        left_boundary_line.is_straight = False
                    else:
                        min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                        #print("distance is: ", min_distance)
                        if min_distance < 2:
                            left_boundary_line.is_straight = True   
                            left_boundary_line.lane_angle = np.rad2deg(np.arctan2(last_point_y - first_point_y, last_point_x - first_point_x))
                            left_boundary_line.equation_list.append(a)
                            left_boundary_line.equation_list.append(b)
                            left_boundary_line.equation_list.append(c)                
                    
                if number_of_points_in_this_lane > 4 and number_of_points_in_this_lane < 100:
                    first_point_x  = left_boundary_line_point_list[first_point_index][0]
                    first_point_y  = left_boundary_line_point_list[first_point_index][1]
                    second_point_x = left_boundary_line_point_list[first_point_index + 1][0]
                    second_point_y = left_boundary_line_point_list[first_point_index + 1][1]
                    
                    last_two_point_x  = left_boundary_line_point_list[last_point_index - 1][0]
                    last_two_point_y  = left_boundary_line_point_list[last_point_index - 1][1]         
                    last_point_x  = left_boundary_line_point_list[last_point_index][0]
                    last_point_y  = left_boundary_line_point_list[last_point_index][1]  
                    
                    middle_point_x = left_boundary_line_point_list[middle_point_index][0]
                    middle_point_y = left_boundary_line_point_list[middle_point_index][1] 
             
                    #print("first point is: ", first_point_x, first_point_y)  
                    #print("last point is: ", last_point_x, last_point_y)
             
                    a = first_point_y - last_point_y
                    b = last_point_x - first_point_x
                    c = first_point_x * last_point_y - last_point_x * first_point_y
                    
                    if b == 0:
                        left_boundary_line.slope = 9999
                    else:
                        left_boundary_line.slope = -1 * a / b
                    
                    if a == 0 and b == 0:
                        left_boundary_line.is_straight = False
                    else:
                        min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                        #print("distance is: ", min_distance)
                        if min_distance < 2:
                            left_boundary_line.is_straight = True   
                            left_boundary_line.lane_angle = np.rad2deg(np.arctan2(last_point_y - first_point_y, last_point_x - first_point_x))
                            left_boundary_line.equation_list.append(a)
                            left_boundary_line.equation_list.append(b)
                            left_boundary_line.equation_list.append(c)
                             
                        else:
                            left_boundary_line.is_straight = False   

                            start_point = [first_point_x, first_point_y]
                            end_point = [last_point_x, last_point_y]
                            middle_point = [middle_point_x, middle_point_y]
                            point_location = where_the_point_is(start_point, end_point, middle_point)
                            center_point, radius = get_circle_from_three_points(start_point, end_point, middle_point)
                            left_boundary_line.radius = radius
                            
                            #slope1 = (second_point_y - first_point_y) / (second_point_x - first_point_x)
                            #slope2 = (last_two_point_y - last_point_y) / (last_two_point_x - last_point_x)
                            
                            angle1 = np.rad2deg(np.arctan2(second_point_y - first_point_y, second_point_x - first_point_x))
                            angle2 = np.rad2deg(np.arctan2(last_two_point_y - last_point_y, last_two_point_x - last_point_x))
                            
                            if abs(abs(angle2) - abs(angle1)) < 20:                                          
                            #if radius < 5:
                                left_boundary_line.is_u_turn = True
                                #print(" --- u turn is : ", self.lane_id)
                            else:
                                if point_location == "left":
                                    left_boundary_line.is_right_turn = True
                                if point_location == "right":
                                    left_boundary_line.is_left_turn = True 

            # find lane right lane set and right lane set
            for right_segment in center_lane.right_boundary_segment_list:
                right_boundary_line_id = right_segment.boundary_feature_id
                right_boundary_line = scenario.map_feature_dict[right_boundary_line_id]  
                right_boundary_line_point_list = right_boundary_line.points_list
                
                number_of_points_in_this_lane = len(right_boundary_line_point_list)
                first_point_index = 0
                last_point_index = number_of_points_in_this_lane - 1
                middle_point_index = int(0.5 * last_point_index)
                
                if number_of_points_in_this_lane < 4:
                    first_point_x = right_boundary_line_point_list[first_point_index][0]
                    first_point_y = right_boundary_line_point_list[first_point_index][1]
                    last_point_x  = right_boundary_line_point_list[last_point_index][0]
                    last_point_y  = right_boundary_line_point_list[last_point_index][1]      
                    middle_point_x = right_boundary_line_point_list[middle_point_index][0]
                    middle_point_y = right_boundary_line_point_list[middle_point_index][1] 
                    
                    right_boundary_line.is_straight = True
                    a = first_point_y - last_point_y
                    b = last_point_x - first_point_x
                    c = first_point_x * last_point_y - last_point_x * first_point_y
                    
                    if b == 0:
                        right_boundary_line.slope = 9999
                    else:
                        right_boundary_line.slope = -1 * a / b
                    
                    if a == 0 and b == 0:
                        right_boundary_line.is_straight = False
                    else:
                        min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                        #print("distance is: ", min_distance)
                        if min_distance < 2:
                            right_boundary_line.is_straight = True   
                            right_boundary_line.lane_angle = np.rad2deg(np.arctan2(last_point_y - first_point_y, last_point_x - first_point_x))
                            right_boundary_line.equation_list.append(a)
                            right_boundary_line.equation_list.append(b)
                            right_boundary_line.equation_list.append(c)                
                    
                if number_of_points_in_this_lane > 4 and number_of_points_in_this_lane < 100:
                    first_point_x  = right_boundary_line_point_list[first_point_index][0]
                    first_point_y  = right_boundary_line_point_list[first_point_index][1]
                    second_point_x = right_boundary_line_point_list[first_point_index + 1][0]
                    second_point_y = right_boundary_line_point_list[first_point_index + 1][1]
                    
                    last_two_point_x  = right_boundary_line_point_list[last_point_index - 1][0]
                    last_two_point_y  = right_boundary_line_point_list[last_point_index - 1][1]         
                    last_point_x  = right_boundary_line_point_list[last_point_index][0]
                    last_point_y  = right_boundary_line_point_list[last_point_index][1]  
                    
                    middle_point_x = right_boundary_line_point_list[middle_point_index][0]
                    middle_point_y = right_boundary_line_point_list[middle_point_index][1] 
             
                    #print("first point is: ", first_point_x, first_point_y)  
                    #print("last point is: ", last_point_x, last_point_y)
             
                    a = first_point_y - last_point_y
                    b = last_point_x - first_point_x
                    c = first_point_x * last_point_y - last_point_x * first_point_y
                    
                    if b == 0:
                        right_boundary_line.slope = 9999
                    else:
                        right_boundary_line.slope = -1 * a / b
                    
                    if a == 0 and b == 0:
                        right_boundary_line.is_straight = False
                    else:
                        min_distance = abs((a * middle_point_x + b * middle_point_y + c)) / (math.sqrt(a * a + b * b))
                        #print("distance is: ", min_distance)
                        if min_distance < 2:
                            right_boundary_line.is_straight = True   
                            right_boundary_line.lane_angle = np.rad2deg(np.arctan2(last_point_y - first_point_y, last_point_x - first_point_x))
                            right_boundary_line.equation_list.append(a)
                            right_boundary_line.equation_list.append(b)
                            right_boundary_line.equation_list.append(c)
                             
                        else:
                            right_boundary_line.is_straight = False  

                            start_point = [first_point_x, first_point_y]
                            end_point = [last_point_x, last_point_y]
                            middle_point = [middle_point_x, middle_point_y]
                            point_location = where_the_point_is(start_point, end_point, middle_point)
                            center_point, radius = get_circle_from_three_points(start_point, end_point, middle_point)
                            right_boundary_line.radius = radius
                            
                            #slope1 = (second_point_y - first_point_y) / (second_point_x - first_point_x)
                            #slope2 = (last_two_point_y - last_point_y) / (last_two_point_x - last_point_x)
                            
                            angle1 = np.rad2deg(np.arctan2(second_point_y - first_point_y, second_point_x - first_point_x))
                            angle2 = np.rad2deg(np.arctan2(last_two_point_y - last_point_y, last_two_point_x - last_point_x))
                            
                            if abs(abs(angle2) - abs(angle1)) < 20:                                          
                            #if radius < 5:
                                right_boundary_line.is_u_turn = True
                                #print(" --- u turn is : ", self.lane_id)
                            else:
                                if point_location == "right":
                                    right_boundary_line.is_right_turn = True
                                if point_location == "right":
                                    right_boundary_line.is_right_turn = True 


def calculate_lane_left_right_boundary(scenario, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset):
    for center_lane in scenario.center_lane_list:
        if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:              
            center_lane_id = center_lane.lane_id
            center_lane_point_list = center_lane.points_list

            first_point = center_lane_point_list[0]
            last_point  = center_lane_point_list[-1]            

            left_boundary_list = None
            right_boundary_list = None
            last_three_point_list = []
            first_three_point_list = []
            extened_center_lane_point_list = []
                    
            # find lane width
            lane_width_in_waymo = None
            left_lane_width_in_waymo = None
            right_lane_width_in_waymo = None
            
            last_left_lane_width_in_waymo = None
            last_right_lane_width_in_waymo = None
            for left_segment in center_lane.left_boundary_segment_list:
                left_boundary_line_id = left_segment.boundary_feature_id
                left_boundary_line = scenario.map_feature_dict[left_boundary_line_id]

                if center_lane.is_straight_turn:
                    if left_boundary_line.is_right_turn or left_boundary_line.is_left_turn:
                        continue                  
                                             
                left_boundary_line_point_list = left_boundary_line.points_list                   
                left_boundary_first_point_index, min_distance_first = find_nearest_point_to_point_list(first_point, left_boundary_line_point_list)
                left_boundary_last_point_index,  min_distance_last  = find_nearest_point_to_point_list(last_point, left_boundary_line_point_list)            
        
                #print("left distance first: ", min_distance_first)
                #print("left distance last: ", min_distance_last)
                #if center_lane_id == 160:
                #    print("   1401 left distance1 index : ", left_boundary_first_point_index)
                #    print("   1401 left distance2 index : ", left_boundary_last_point_index)                    
                #    print("   1401 left distance1: ", min_distance_first)
                #    print("   1401 left distance2: ", min_distance_last)
        
        
                if min_distance_first != None and min_distance_last != None:
                    if min_distance_first < 3 and min_distance_last >= 3:
                        if last_left_lane_width_in_waymo != None:
                            if min_distance_first > last_left_lane_width_in_waymo:
                                left_lane_width_in_waymo = min_distance_first
                                last_left_lane_width_in_waymo = left_lane_width_in_waymo
                        else:
                            left_lane_width_in_waymo = min_distance_first
                            last_left_lane_width_in_waymo = left_lane_width_in_waymo
                    if min_distance_first >= 3 and min_distance_last < 3:
                        if last_left_lane_width_in_waymo != None:
                            if min_distance_last > last_left_lane_width_in_waymo:
                                left_lane_width_in_waymo = min_distance_last
                                last_left_lane_width_in_waymo = left_lane_width_in_waymo
                        else:
                            left_lane_width_in_waymo = min_distance_last
                            last_left_lane_width_in_waymo = left_lane_width_in_waymo
                elif min_distance_first != None and min_distance_last == None:
                    if min_distance_first < 3:
                        left_lane_width_in_waymo = min_distance_first
                elif min_distance_first == None and min_distance_last != None:
                    if min_distance_last < 3:
                        left_lane_width_in_waymo = min_distance_last
                else:
                    left_lane_width_in_waymo = 0.5 * lane_polygon_width

            for right_segment in center_lane.right_boundary_segment_list:
                right_boundary_line_id = right_segment.boundary_feature_id
                right_boundary_line = scenario.map_feature_dict[right_boundary_line_id]                 

                if center_lane.is_straight_turn:
                    if left_boundary_line.is_right_turn or left_boundary_line.is_left_turn:
                        continue                  
                
                right_boundary_line_point_list = right_boundary_line.points_list                   
                right_boundary_first_point_index, min_distance_first = find_nearest_point_to_point_list(first_point, right_boundary_line_point_list)
                right_boundary_last_point_index,  min_distance_last  = find_nearest_point_to_point_list(last_point, right_boundary_line_point_list)            
                
                #if center_lane_id == 160:
                    #print("   1401 right distance1 index : ", right_boundary_first_point_index)
                    #print("   1401 right distance2 index : ", right_boundary_last_point_index)   
                    #print("   1401 right distance1: ", min_distance_first)
                    #print("   1401 right distance2: ", min_distance_last)
                #print("right distance first: ", min_distance_first)
                #print("right distance last: ", min_distance_last)
        
                if min_distance_first != None and min_distance_last != None:
                    if min_distance_first < 3 and min_distance_last >= 3:
                        if last_right_lane_width_in_waymo != None:
                            if min_distance_first > last_right_lane_width_in_waymo:
                                right_lane_width_in_waymo = min_distance_first
                                last_right_lane_width_in_waymo = right_lane_width_in_waymo
                        else:
                            right_lane_width_in_waymo = min_distance_first
                            last_right_lane_width_in_waymo = right_lane_width_in_waymo
                    if min_distance_first >= 3 and min_distance_last < 3:
                        if last_right_lane_width_in_waymo != None:
                            if min_distance_last > last_right_lane_width_in_waymo:
                                right_lane_width_in_waymo = min_distance_last
                                last_right_lane_width_in_waymo = right_lane_width_in_waymo
                        else:
                            right_lane_width_in_waymo = min_distance_last
                            last_right_lane_width_in_waymo = right_lane_width_in_waymo
                elif min_distance_first != None and min_distance_last == None:
                    if min_distance_first < 3:
                        right_lane_width_in_waymo = min_distance_first
                elif min_distance_first == None and min_distance_last != None:
                    if min_distance_last < 3:
                        right_lane_width_in_waymo = min_distance_last   
                else:
                    right_lane_width_in_waymo = 0.5 * lane_polygon_width        
                 
            if left_lane_width_in_waymo == None:
                left_lane_width_in_waymo = 0.5 * lane_polygon_width
            if right_lane_width_in_waymo == None:
                right_lane_width_in_waymo = 0.5 * lane_polygon_width                
                
            left_lane_width_in_waymo = left_lane_width_in_waymo
            right_lane_width_in_waymo = right_lane_width_in_waymo
            
            # add engtry lane last 3 points
            for entry_center_lane_id in center_lane.entry_lane_id_list:
                entry_center_lane = scenario.center_lane_dict[entry_center_lane_id]
                if len(entry_center_lane.points_list) > 4:
                    last_four_points = entry_center_lane.points_list[-4:]
                    if entry_center_lane.points_list[-1] == center_lane_point_list[0]:
                        last_three_point_list = last_four_points[-4:-1]
                        break
                    else:                       
                        last_three_point_list = last_four_points[-3:]
                        break
           
            # add exist lane last 3 points
            for exist_center_lane_id in center_lane.exit_lane_id_list:
                exist_center_lane = scenario.center_lane_dict[exist_center_lane_id]
                if len(exist_center_lane.points_list) > 3:
                    first_four_points = exist_center_lane.points_list[0:4]
                    if exist_center_lane.points_list[0] == center_lane_point_list[-1]:
                        first_three_point_list = first_four_points[1:4]
                        break
                    else:
                        first_three_point_list = first_four_points[0:3]
                        break
                       
            extened_center_lane_point_list = last_three_point_list + center_lane_point_list + first_three_point_list
            if len(extened_center_lane_point_list) > 2:
                if center_lane.is_right_turn:
                    left_boundary_list, right_boundary_list = generate_polygon_from_right_turn_line(extened_center_lane_point_list, right_turn_lane_polygon_width, right_turn_lane_polygon_offset)
                else:
                    left_boundary_list, right_boundary_list = generate_polygon_left_right_from_line(extened_center_lane_point_list, left_lane_width_in_waymo, right_lane_width_in_waymo)
                center_lane.left_boundary_point_list = left_boundary_list
                center_lane.right_boundary_point_list = right_boundary_list           



def calculate_lane_left_right_boundary_straight(scenario):
    for center_lane in scenario.center_lane_list:
        if center_lane.is_straight:
            if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
                
                center_lane_id = center_lane.lane_id
                first_point = center_lane.points_list[0]
                last_point  = center_lane.points_list[-1]
                center_lane_point_list = center_lane.points_list
                
                left_min_distance = None
                right_min_distance = None
                left_boundary_list = []
                right_boundary_list = []
                boundary_list = []    
        
                for left_segment in center_lane.left_boundary_segment_list:

                    left_boundary_line_id = left_segment.boundary_feature_id
                    left_boundary_line = scenario.map_feature_dict[left_boundary_line_id]
                    
                    if center_lane.is_straight_turn:
                        if left_boundary_line.is_right_turn or left_boundary_line.is_left_turn:
                            continue                   
                    
                    left_boundary_line_point_list = left_boundary_line.points_list                   
                    left_boundary_first_point_index, min_distance_first = find_nearest_point_to_point_list(first_point, left_boundary_line_point_list)
                    left_boundary_last_point_index,  min_distance_last  = find_nearest_point_to_point_list(last_point, left_boundary_line_point_list)
                    
                    if min_distance_first != None and min_distance_last != None:
                        left_min_distance = 0.5 * (min_distance_first + min_distance_last)
                        
                    for i in range(left_boundary_first_point_index, left_boundary_last_point_index):
                        left_boundary_list.append(left_boundary_line_point_list[i])
                        
                for right_segment in center_lane.right_boundary_segment_list:

                    right_boundary_line_id = right_segment.boundary_feature_id
                    right_boundary_line = scenario.map_feature_dict[right_boundary_line_id]  
                    
                    if center_lane.is_straight_turn:
                        if right_boundary_line.is_right_turn or right_boundary_line.is_left_turn:
                            continue       
                        
                    right_boundary_line_point_list = right_boundary_line.points_list                   
                    right_boundary_first_point_index, min_distance_first = find_nearest_point_to_point_list(first_point, right_boundary_line_point_list)
                    right_boundary_last_point_index,  min_distance_last  = find_nearest_point_to_point_list(last_point, right_boundary_line_point_list)
    
                    if min_distance_first != None and min_distance_last != None:
                        right_min_distance = 0.5 * (min_distance_first + min_distance_last)

                    for i in range(right_boundary_first_point_index, right_boundary_last_point_index):
                        right_boundary_list.append(right_boundary_line_point_list[i])    
                        
                if len(left_boundary_list) != 0 and len(right_boundary_list) != 0:
                    boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                    
                    #random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                    #polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                    #center_lane.polygon = polygon
                    center_lane.lane_width = left_min_distance + right_min_distance
    
                if len(left_boundary_list) != 0 and len(right_boundary_list) == 0:
                    index = 0
                    for point in left_boundary_list:
                        point_in_lane = center_lane_point_list[index]
                        point_in_right = calculate_the_symmetric_point_of_another_point(point, point_in_lane)
                        right_boundary_list.append(point_in_right)
                        index += 1                
                    
                    #boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                    #random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                    #polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                    #center_lane.polygon = polygon
                    center_lane.lane_width = 2 * left_min_distance
                   
                if len(left_boundary_list) == 0 and len(right_boundary_list) != 0:
                    
                    if len(center_lane_point_list) <= len(right_boundary_list):
                        index = 0
                        for point_in_lane in center_lane_point_list:
                            point = right_boundary_list[index]
                            point_in_left = calculate_the_symmetric_point_of_another_point(point, point_in_lane)
                            left_boundary_list.append(point_in_left)
                            index += 1                     
                    
                    else:
                        index = 0
                        for point in right_boundary_list:
                            point_in_lane = center_lane_point_list[index]
                            point_in_left = calculate_the_symmetric_point_of_another_point(point, point_in_lane)
                            left_boundary_list.append(point_in_left)
                            index += 1    
                                       
                    #boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                    #random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                    #polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                    #center_lane.polygon = polygon
                    center_lane.lane_width = 2 * right_min_distance

                center_lane.left_boundary_point_list = left_boundary_list
                center_lane.right_boundary_point_list = right_boundary_list
                


# to make boundary longer to make sure from one lane to another lane there is no blank space
def extend_generate_straight_lane_polygon_boundary(scenario):
    for center_lane in scenario.center_lane_list:
        if center_lane.is_straight:
        #if True:    
            if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
                center_lane_id = center_lane.lane_id
                left_boundary_list  = center_lane.left_boundary_point_list
                right_boundary_list = center_lane.right_boundary_point_list
                
                #if len(center_lane.entry_lane_id_list) < 2 or len(center_lane.exit_lane_id_list) < 2:
                #    continue
                if left_boundary_list != None and right_boundary_list != None:
                    if len(center_lane.entry_lane_id_list) != 0 and len(center_lane.exit_lane_id_list) != 0:
                        from_lane_id = center_lane.entry_lane_id_list[0]                
                        to_lane_id   = center_lane.exit_lane_id_list[0]
                        from_lane = scenario.center_lane_dict[from_lane_id]
                        to_lane   = scenario.center_lane_dict[to_lane_id]
                        from_lane_left_boundary_list  = from_lane.left_boundary_point_list
                        from_lane_right_boundary_list = from_lane.right_boundary_point_list
                        to_lane_left_boundary_list  = to_lane.left_boundary_point_list
                        to_lane_right_boundary_list = to_lane.right_boundary_point_list
                        
                        if from_lane_left_boundary_list != None and from_lane_right_boundary_list != None and to_lane_left_boundary_list != None and to_lane_right_boundary_list != None:
                            if len(from_lane_left_boundary_list) > 3 and len(from_lane_right_boundary_list) > 3:
                                if len(to_lane_left_boundary_list) > 3 and len(to_lane_right_boundary_list) > 3:
                                    left_boundary_list  = from_lane_left_boundary_list[-3:] + left_boundary_list + to_lane_left_boundary_list[0:3]
                                    right_boundary_list = from_lane_right_boundary_list[-3:] + right_boundary_list + to_lane_right_boundary_list[0:3]
                                
                                boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                                random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                                polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                                center_lane.polygon = polygon
                        
                    if len(center_lane.entry_lane_id_list) != 0 and len(center_lane.exit_lane_id_list) == 0:
                        from_lane_id = center_lane.entry_lane_id_list[0]                
                        from_lane = scenario.center_lane_dict[from_lane_id]
                        from_lane_left_boundary_list  = from_lane.left_boundary_point_list
                        from_lane_right_boundary_list = from_lane.right_boundary_point_list
    
                        if from_lane_left_boundary_list != None and from_lane_right_boundary_list != None:
                            if len(from_lane_left_boundary_list) > 3 and len(from_lane_right_boundary_list) > 3:
                                left_boundary_list  = from_lane_left_boundary_list[-3:] + left_boundary_list
                                right_boundary_list = from_lane_right_boundary_list[-3:] + right_boundary_list
                                
                                boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                                random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                                polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                                center_lane.polygon = polygon                    
                        
                    if len(center_lane.entry_lane_id_list) == 0 and len(center_lane.exit_lane_id_list) != 0:              
                        to_lane_id   = center_lane.exit_lane_id_list[0]
                        to_lane   = scenario.center_lane_dict[to_lane_id]
                        to_lane_left_boundary_list  = to_lane.left_boundary_point_list
                        to_lane_right_boundary_list = to_lane.right_boundary_point_list
    
                        if left_boundary_list != None and right_boundary_list != None and to_lane_left_boundary_list != None and to_lane_right_boundary_list != None:
                            if len(to_lane_left_boundary_list) > 3 and len(to_lane_right_boundary_list) > 3:
                                left_boundary_list  = left_boundary_list + to_lane_left_boundary_list[0:3]
                                right_boundary_list = right_boundary_list + to_lane_right_boundary_list[0:3]
                                
                                boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                                random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                                polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                                center_lane.polygon = polygon
                    
                    if len(center_lane.entry_lane_id_list) == 0 and len(center_lane.exit_lane_id_list) == 0:                                
                        boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                        random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
                        polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                        center_lane.polygon = polygon                



def calculate_lane_polygon_for_plot(scenario):
    for center_lane in scenario.center_lane_list:
        #if center_lane.is_left_turn or center_lane.is_right_turn or center_lane.is_u_turn:
        if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:   
            center_lane_id = center_lane.lane_id
            left_boundary_list  = center_lane.left_boundary_point_list
            right_boundary_list = center_lane.right_boundary_point_list                
            boundary_list = left_boundary_list + list(reversed(right_boundary_list))
            random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
            if len(boundary_list) > 1:
                polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
                center_lane.polygon = polygon 
                    
            '''  
            left_boundary_list = []
            right_boundary_list = []
            from_lane = None
            to_lane = None
            from_lane_left_boundary_last_point = None
            from_lane_right_boundary_last_point = None
            to_lane_left_boundary_first_point = None
            to_lane_right_boundary_first_point = None
            
            if len(center_lane.entry_lines_list) != 0 and len(center_lane.exit_lanes_list) != 0 :
                from_lane_id = center_lane.entry_lines_list[0]                
                to_lane_id = center_lane.exit_lanes_list[0]
                from_lane = scenario.center_lane_dict[from_lane_id]
                to_lane   = scenario.center_lane_dict[to_lane_id]
                
                print("from lane id: ",from_lane.lane_id)
                print("to lane id: ", to_lane.lane_id)
                
                print(" left boundary point list length is: ", len(from_lane.left_boundary_point_list))
                
                from_lane_left_boundary_last_point  = from_lane.left_boundary_point_list[-1]
                from_lane_right_boundary_last_point = from_lane.right_boundary_point_list[-1]
                to_lane_left_boundary_first_point   = to_lane.left_boundary_point_list[0]
                to_lane_right_boundary_first_point  = to_lane.right_boundary_point_list[0]  
                left_boundary_list.append(from_lane_left_boundary_last_point)
                right_boundary_list.append(from_lane_right_boundary_last_point)
          
            number_of_points = len(center_lane.points_list)
            middle_index = int(0.5 * number_of_points)
            first_point = center_lane.points_list[0]
            last_point  = center_lane.points_list[-1]            
            
            curve_shift_distance = 2
            distance = calculate_distance_between_two_points_v1(first_point, last_point)
            
            new_point_for_up_shift = rotate(first_point, last_point, 90)
            new_point_for_down_shift = rotate(first_point, last_point, -90)
            delta_x_for_up_shift   = curve_shift_distance / distance * (new_point_for_up_shift[0] - first_point[0])
            delta_y_for_up_shift   = curve_shift_distance / distance * (new_point_for_up_shift[1] - first_point[1])
            delta_x_for_down_shift = curve_shift_distance / distance * (new_point_for_down_shift[0] - first_point[0])
            delta_y_for_down_shift = curve_shift_distance / distance * (new_point_for_down_shift[1] - first_point[1])
            
            for point in center_lane.points_list:
                x_up = point[0] + delta_x_for_up_shift
                y_up = point[1] + delta_y_for_up_shift
                
                x_down = point[0] + delta_x_for_down_shift
                y_down = point[1] + delta_y_for_down_shift
                
                left_boundary_list.append((x_up, y_up))
                right_boundary_list.append((x_down, y_down))

            if from_lane != None and to_lane != None:
                left_boundary_list.append(to_lane_left_boundary_first_point)
                right_boundary_list.append(to_lane_right_boundary_first_point)

            number_of_points = len(left_boundary_list)
            cut_index = int(0.25 * number_of_points)
            left_boundary_list  = [left_boundary_list[0]] + left_boundary_list[cut_index : -1 * cut_index] + [left_boundary_list[-1]]
            right_boundary_list = [right_boundary_list[0]] + right_boundary_list[cut_index : -1 * cut_index] + [right_boundary_list[-1]]

            boundary_list = left_boundary_list + list(reversed(right_boundary_list))
            center_lane.left_boundary_point_list  = left_boundary_list
            center_lane.right_boundary_point_list = right_boundary_list
            random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
            polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
            center_lane.polygon = polygon
            center_lane.lane_width = 2 

      
    else:
        if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
            number_of_points = len(center_lane.points_list)
            middle_index = int(0.5 * number_of_points)
            first_point = center_lane.points_list[0]
            last_point  = center_lane.points_list[-1]
            
            left_boundary_list = []
            right_boundary_list = []
            
            curve_shift_distance = 2
            distance = calculate_distance_between_two_points_v1(first_point, last_point)
            
            new_point_for_up_shift = rotate(first_point, last_point, 90)
            new_point_for_down_shift = rotate(first_point, last_point, -90)
            delta_x_for_up_shift   = curve_shift_distance / distance * (new_point_for_up_shift[0] - first_point[0])
            delta_y_for_up_shift   = curve_shift_distance / distance * (new_point_for_up_shift[1] - first_point[1])
            delta_x_for_down_shift = curve_shift_distance / distance * (new_point_for_down_shift[0] - first_point[0])
            delta_y_for_down_shift = curve_shift_distance / distance * (new_point_for_down_shift[1] - first_point[1])
            
            for point in center_lane.points_list:
                x_up = point[0] + delta_x_for_up_shift
                y_up = point[1] + delta_y_for_up_shift
                
                x_down = point[0] + delta_x_for_down_shift
                y_down = point[1] + delta_y_for_down_shift
                
                left_boundary_list.append((x_up, y_up))
                right_boundary_list.append((x_down, y_down))

            boundary_list = left_boundary_list + list(reversed(right_boundary_list))
            center_lane.left_boundary_point_list  = left_boundary_list
            center_lane.right_boundary_point_list = right_boundary_list
            random_color = ["#"+''.join([random.choice('0123456789ABCDEF') for i in range(6)])]
            polygon = Polygon(boundary_list, color = random_color[0], alpha = 0.5)
            center_lane.polygon = polygon
            center_lane.lane_width = 2 
    '''
        
        
        

def calculate_lane_polygon_for_map_matching(scenario):
    
    from shapely.geometry import Point, Polygon
    
    for center_lane in scenario.center_lane_list:
        if center_lane.lane_type == MAP_feature.TYPE_FREEWAY or center_lane.lane_type == MAP_feature.TYPE_SURFACE_STREET:
            center_lane_id = center_lane.lane_id             
            left_boundary_list = center_lane.left_boundary_point_list
            right_boundary_list = center_lane.right_boundary_point_list
            if left_boundary_list != None and right_boundary_list != None:
                boundary_list = left_boundary_list + list(reversed(right_boundary_list))
                if len(boundary_list) > 2:
                    center_lane.polygon_for_map_matching = Polygon(boundary_list)   
                else:
                    center_lane.polygon_for_map_matching = None
            