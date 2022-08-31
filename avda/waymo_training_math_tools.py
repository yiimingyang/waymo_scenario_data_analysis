#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math
import numpy as np
import numpy.matlib as npm

def calculate_list_average(list_example):
    return sum(list_example) / len(list_example)

def degree_to_rdian(degree):
    return degree * math.pi* 0.005556

def find_most_frequent_element(List):
    return max(set(List), key = List.count)

def radian_to_degree(radian):
    return radian * 180 / math.pi

def radian_to_360_degree(radian):
    angle_degree = radian * 180 / math.pi
    
    if angle_degree > 360:
        angle_degree = angle_degree - 360
    if angle_degree < -360:
        angle_degree = angle_degree + 360
    
    if angle_degree >= 0 and angle_degree <= 90:
        angle_degree = 90 - angle_degree
    elif angle_degree >= 90 and angle_degree <= 180:
        angle_degree = 360 + 90 - angle_degree    
    else:
        angle_degree = 90 - angle_degree  
    return angle_degree
        
        
def calculate_distance_from_line(equation_list, vehicle_x, vehicle_y):
    a = equation_list[0]
    b = equation_list[1]
    c = equation_list[2]
    min_distance = abs((a * vehicle_x + b * vehicle_y + c)) / (math.sqrt(a * a + b * b))
    return min_distance 



def calculate_distance_between_two_points(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2) 



def calculate_distance_between_two_points_v1(point0, point1):
    return math.sqrt((point1[0] - point0[0])**2 + (point1[1] - point0[1])**2) 



def is_in_range(point1, point2, point0):
    middle_point = (0.5 * (point1[0] + point2[0]), 0.5 * (point1[1] + point2[1]))
    distance = calculate_distance_between_two_points_v1(middle_point, point0)
    if distance < 40:
        return True
    else:
        return False


def rotate(origin, point, angle_degree):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in degrees.
    """
    
    angle = math.radians(angle_degree)
    ox, oy = origin[0], origin[1]
    px, py = point[0], point[1]

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return (qx, qy)


def calculate_the_symmetric_point_of_another_point(point, point0):
    x = 2 * point0[0] - point[0]
    y = 2 * point0[1] - point[1]
    return (x, y)


def is_in_range_for_straight(point1, point2, point0):
    if point0[0] > point1[0] and point0[0] < point2[0] and point0[1] > point1[1] and point0[1] > point2[1]:
        return True
    elif point0[0] > point1[0] and point0[0] < point2[0] and point0[1] < point1[1] and point0[1] < point2[1]:
        return True
    elif point0[0] < point1[0] and point0[0] > point2[0] and point0[1] > point1[1] and point0[1] > point2[1]:
        return True
    elif point0[0] < point1[0] and point0[0] > point2[0] and point0[1] < point1[1] and point0[1] < point2[1]:
        return True
    elif point0[1] > point1[1] and point0[1] < point2[1] and point0[0] > point1[0] and point0[0] > point2[0]:
        return True
    elif point0[1] > point1[1] and point0[1] < point2[1] and point0[0] < point1[0] and point0[0] < point2[0]:
        return True
    elif point0[1] < point1[1] and point0[1] > point2[1] and point0[0] > point1[0] and point0[0] > point2[0]:
        return True
    elif point0[1] < point1[1] and point0[1] > point2[1] and point0[0] < point1[0] and point0[0] < point2[0]:
        return True
    else:
        return False


def is_in_range_for_straight_v1(point1, point2, point0):
  
    if point0[0] > point1[0] and point0[0] > point2[0]:
        return False
    elif point0[0] < point1[0] and point0[0] < point2[0]:
        return False
    elif point0[1] > point1[1] and point0[1] > point2[1]:
        return False
    elif point0[1] < point1[1] and point0[1] < point2[1]:
        return False
    else:
        return True
  
    
def get_circle_from_three_points(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])
    
    if abs(det) < 1.0e-3:
        return (None, np.inf)
    
    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det
    
    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    return ((cx, cy), radius)


def calculate_lane_curvature(p1, p2, p3):
    """
    Returns the center and radius of the circle passing the given 3 points.
    In case the 3 points form a line, returns (None, infinity).
    """
    temp = p2[0] * p2[0] + p2[1] * p2[1]
    bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
    cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
    det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])
    
    if abs(det) < 1.0e-3:
        return (None, np.inf)
    
    # Center of circle
    cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
    cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det
    
    radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
    curvature = 1 / radius
    return curvature


def where_the_point_is(line_point1, line_point2, point):
    aX = line_point1[0]
    aY = line_point1[1]
    bX = line_point2[0]
    bY = line_point2[1]
    cX = point[0]
    cY = point[1]

    val = ((bX - aX)*(cY - aY) - (bY - aY)*(cX - aX))
    thresh = 1
    if val >= thresh:
        return "left"
    elif val <= -thresh:
        return "right"
    else:
        return "on"



def get_bottem_left_point_of_rectangle_origin(center_x, center_y, height, width, angle):
    
    angle = angle * math.pi / 180

    #top right:
    top_right_x = center_x + ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_right_y = center_y + ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
       
    #top left:
    top_left_x = center_x - ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_left_y = center_y - ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
        
    #bottom left:
    bot_left_x = center_x - ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_left_y = center_y - ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))
         
    #bottom right:
    bot_right_x = center_x + ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_right_y = center_y + ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))

    #bottom center:
    bot_center_x = 0.5 * (bot_right_x + bot_left_x)
    bot_center_y = 0.5 * (bot_right_y + bot_left_y)

    return bot_left_x, bot_left_y

def get_bottem_left_point_of_rectangle(center_x, center_y, height, width, angle):
            
    angle = angle * math.pi / 180

    #top right:
    top_right_x = center_x + ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_right_y = center_y + ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
       
    #top left:
    top_left_x = center_x - ((width / 2) * math.cos(angle)) - ((height / 2) * math.sin(angle))
    top_left_y = center_y - ((width / 2) * math.sin(angle)) + ((height / 2) * math.cos(angle))
        
    #bottom left:
    bot_left_x = center_x - ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_left_y = center_y - ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))
         
    #bottom right:
    bot_right_x = center_x + ((width / 2) * math.cos(angle)) + ((height / 2) * math.sin(angle))
    bot_right_y = center_y + ((width / 2) * math.sin(angle)) - ((height / 2) * math.cos(angle))

    #bottom center:
    bot_center_x = 0.5 * (bot_right_x + bot_left_x)
    bot_center_y = 0.5 * (bot_right_y + bot_left_y)
    
    angle = radian_to_degree(angle)
    
    if math.sin(angle) >= 0 and math.cos(angle) >= 0:
        return bot_left_x, bot_left_y
    if math.sin(angle) >= 0 and math.cos(angle) <= 0:
        return bot_left_x, bot_left_y   
    if math.sin(angle) <= 0 and math.cos(angle) >= 0:
        return bot_right_x, bot_right_y
    if math.sin(angle) <= 0 and math.cos(angle) <= 0:
        return bot_right_x, bot_right_y    
       
    
def calculate_arrow_base(yaw_theta, rectangle_x, rectangle_y, rectangle_length, rectangle_width):
    theta = yaw_theta + 90
    if theta >= 360:
        theta = theta - 360
    if theta <= -360:
        theta = theta + 360
    if theta >= 180:
        theta = theta - 360
    if theta <= -180:
        theta = theta + 360        

    theta = theta * math.pi / 180
    yaw_theta = yaw_theta * math.pi / 180
    x = rectangle_x + 0.5 * rectangle_width * math.cos(theta)
    y = rectangle_y + 0.5 * rectangle_width * math.sin(theta)
    dx = (rectangle_length + 1) * math.cos(yaw_theta)
    dy = (rectangle_length + 1) * math.sin(yaw_theta)  
    return x, y,dx,dy


def find_conflict_point_in_two_list(point_list1, point_list2):
    min_distance = 1000
    min_distance_x = None
    min_distance_y = None
    conflict_index = 0
    for point1 in point_list1:
        for point2 in point_list2:
            distance = calculate_distance_between_two_points_v1(point1, point2)
            if distance < min_distance:
                min_distance = distance
                min_distance_x = point1[0]
                min_distance_y = point1[1]
        conflict_index += 1
    if min_distance < 0.5:
        return min_distance_x, min_distance_y, conflict_index
    else:
        return None, None, None
        
def find_nearest_point_in_lane(point, point_list):
    min_distance = 1000
    min_distance_x = None
    min_distance_y = None
    point_index_in_lane = 0
    index = 0
    for point_in_lane in point_list:
        distance = calculate_distance_between_two_points_v1(point, point_in_lane)
        if distance < min_distance:
            min_distance = distance
            min_distance_x = point_in_lane[0]
            min_distance_y = point_in_lane[1]
            point_index_in_lane = index
        index += 1
    if min_distance < 10:
        return point_index_in_lane, len(point_list)
    else:
        return None, None  


def find_nearest_point_in_left_or_right_lane(point, point_list):
    min_distance = 1000
    point_index_in_lane = None
    index = 0
    for point_in_lane in point_list:
        distance = calculate_distance_between_two_points_v1(point, point_in_lane)
        if distance < min_distance:
            min_distance = distance
            point_index_in_lane = index
        index += 1
    return point_index_in_lane, min_distance

    
    
def find_preceeding_following_vehicle_in_lane(time_index, ego_vheicle_location, ego_vheicle_yaw, agents_on_the_same_lane_list, agents_on_the_same_lane_distance_to_ego_list):
    agent_index  = 0
    preceeding_vehicle_id = None
    following_vehicle_id = None
    last_agent_preceeding_y = None
    last_agent_following_y = None
    last_agent_preceeding_x = None
    last_agent_following_x = None
    last_agent_to_ego_distance = None
    last_agent_preceeding_lane_id = None
    last_agent_following_lane_id = None
    preceeding_vehicle_location = None
    following_vehicle_location = None
    preceeding_to_ego_distance = None
    following_to_ego_distance = None
    
    if ego_vheicle_location == None:
        return preceeding_vehicle_id, \
               preceeding_vehicle_location, \
               preceeding_to_ego_distance, \
               following_vehicle_id, \
               following_vehicle_location, \
               following_to_ego_distance, \
               last_agent_preceeding_lane_id, \
               last_agent_following_lane_id        
    
    ego_vheicle_x = ego_vheicle_location[0]
    ego_vheicle_y = ego_vheicle_location[1]
        
    for agent in agents_on_the_same_lane_list:
        agent_state = agent.states_array[time_index, :]
        agent_id = agent_state[1]    
        agent_state_x = agent_state[3]
        agent_state_y = agent_state[4]
        agent_state_heading = radian_to_degree(agent_state[9])
        agent_lane_id = agent.unique_matching_agent_lane_id_list[time_index]

        # up direction
        if ego_vheicle_yaw > 45 and ego_vheicle_yaw < 135:
            if agent_state_y >= ego_vheicle_y:
                if last_agent_preceeding_y == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_y = agent_state_y
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_preceeding_lane_id = agent_lane_id           
                else:                        
                    if agent_state_y <= last_agent_preceeding_y:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_y = agent_state_y
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_preceeding_lane_id = agent_lane_id
            else:
                #print("----- the same lane find following vehicle -----")
                if last_agent_following_y == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_y = agent_state_y
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_following_lane_id = agent_lane_id
                else:
                    if agent_state_y >= last_agent_following_y:
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y) 
                        last_agent_following_y = agent_state_y
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_following_lane_id = agent_lane_id

        # down direction
        if ego_vheicle_yaw > -135 and ego_vheicle_yaw < -45:
            if agent_state_y <= ego_vheicle_y:
                if last_agent_preceeding_y == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_y = agent_state_y
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_preceeding_lane_id = agent_lane_id
                else:
                    if agent_state_y <= last_agent_preceeding_y:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_y = agent_state_y
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_preceeding_lane_id = agent_lane_id
            else:
                if last_agent_following_y == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_y = agent_state_y
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_following_lane_id = agent_lane_id
                else:   
                    if agent_state_y <= last_agent_following_y:
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y) 
                        last_agent_following_y = agent_state_y
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_following_lane_id = agent_lane_id

        # right direction
        if ego_vheicle_yaw > -45 and ego_vheicle_yaw < 45:
            if agent_state_x >= ego_vheicle_x:
                if last_agent_preceeding_x == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_x = agent_state_x
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_preceeding_lane_id = agent_lane_id
                else:
                    if agent_state_x <= last_agent_preceeding_x:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_x = agent_state_x
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_preceeding_lane_id = agent_lane_id
            else:
                if last_agent_following_x == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_x = agent_state_x
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_following_lane_id = agent_lane_id
                else:
                    if agent_state_x >= last_agent_following_x:                
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y)  
                        last_agent_following_x = agent_state_x
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_following_lane_id = agent_lane_id

        # left direction
        if ego_vheicle_yaw > 135 and ego_vheicle_yaw <= 180 \
            or ego_vheicle_yaw >= -180 and ego_vheicle_yaw <= -135:
            if agent_state_x <= ego_vheicle_x:
                if last_agent_preceeding_x == None:
                    preceeding_vehicle_id = agent_id
                    preceeding_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_preceeding_x = agent_state_x
                    preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_preceeding_lane_id = agent_lane_id
                else:
                    if agent_state_x >= last_agent_preceeding_x:
                        preceeding_vehicle_id = agent_id
                        preceeding_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_preceeding_x = agent_state_x
                        preceeding_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_following_lane_id = agent_lane_id
            else:
                if last_agent_following_x == None:
                    following_vehicle_id = agent_id
                    following_vehicle_location = (agent_state_x, agent_state_y)
                    last_agent_following_x = agent_state_x
                    following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                    last_agent_following_lane_id = agent_lane_id
                else:
                    if agent_state_x <= last_agent_following_x:
                        following_vehicle_id = agent_id
                        following_vehicle_location = (agent_state_x, agent_state_y)
                        last_agent_following_x = agent_state_x
                        following_to_ego_distance = agents_on_the_same_lane_distance_to_ego_list[agent_index]
                        last_agent_following_lane_id = agent_lane_id
        agent_index += 1    
        
    return preceeding_vehicle_id, \
           preceeding_vehicle_location, \
           preceeding_to_ego_distance, \
           following_vehicle_id, \
           following_vehicle_location, \
           following_to_ego_distance, \
           last_agent_preceeding_lane_id, \
           last_agent_following_lane_id
           
           
def generate_rectangle_point(center_x, center_y, width, height, angle_degree):

    theta = math.radians(angle_degree)
    bbox = npm.repmat([[center_x], [center_y]], 1, 5) + \
       np.matmul([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]],
                 [[-width / 2, width/ 2, width / 2, -width / 2, width / 2 + 8],
                  [-height / 2, -height / 2, height / 2, height / 2, 0]])
    # add first point
    x1, y1 = bbox[0][0], bbox[1][0]
    # add second point
    x2, y2 = bbox[0][1], bbox[1][1]
    # add third point
    x3, y3 = bbox[0][2], bbox[1][2]
    # add forth point
    x4, y4 = bbox[0][3], bbox[1][3]

    return [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]


def get_angle_from_entry_lane_to_exist_lane(line1, line2):
    # Get directional vectors
    d1 = (line1[1][0] - line1[0][0], line1[1][1] - line1[0][1])
    d2 = (line2[1][0] - line2[0][0], line2[1][1] - line2[0][1])
    # Compute dot product
    p = d1[0] * d2[0] + d1[1] * d2[1]
    # Compute norms
    n1 = math.sqrt(d1[0] * d1[0] + d1[1] * d1[1])
    n2 = math.sqrt(d2[0] * d2[0] + d2[1] * d2[1])
    # Compute angle
    ang = math.acos(p / (n1 * n2))
    # Convert to degrees if you want
    ang = math.degrees(ang)
    return ang


def calculate_relative_vehicle_front_bumper_gap(ego_vheicle_location, ego_vheicle_yaw, ego_vehicle_shape, 
                                   agent_location, agent_yaw, agent_shape, agent_ego_spacing, left_or_right_lane_spacing):
    
    relative_front_bumper_gap = None
    relative_location = None
    ego_vheicle_x = ego_vheicle_location[0]
    ego_vheicle_y = ego_vheicle_location[1]
    ego_vheicle_length = ego_vehicle_shape[0]
    ego_vheicle_width = ego_vehicle_shape[1]
        
    agent_state_x = agent_location[0]
    agent_state_y = agent_location[1]
    agent_state_length = agent_shape[0]
    agent_state_width = agent_shape[1]    

    if left_or_right_lane_spacing >= agent_ego_spacing:
        left_or_right_lane_spacing = 3
    if left_or_right_lane_spacing >= agent_ego_spacing:
        relative_front_bumper_gap = 0
        relative_location = "PARALLEL"

    relative_front_bumper_gap = math.sqrt(abs(agent_ego_spacing**2 - left_or_right_lane_spacing**2))
    #relative_front_bumper_gap = relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length
    #relative_front_bumper_gap = relative_front_bumper_gap + 0.5 * ego_vheicle_length - 0.5 * agent_state_length

    if ego_vheicle_yaw > 360:
        ego_vheicle_yaw = ego_vheicle_yaw - 360
    if ego_vheicle_yaw < -360:
        ego_vheicle_yaw = ego_vheicle_yaw + 360    

    #if ego_vheicle_yaw 

    # up direction
    if ego_vheicle_yaw >= 45 and ego_vheicle_yaw <= 135:
        if agent_state_y >= ego_vheicle_y:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "FRONT"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap - 0.5 * ego_vheicle_length + 0.5 * agent_state_length
            if relative_front_bumper_gap < 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap
        else:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "BEHIND"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap + 0.5 * ego_vheicle_length - 0.5 * agent_state_length
            if relative_front_bumper_gap > 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap            

    # down direction
    if ego_vheicle_yaw >= 225 and ego_vheicle_yaw <= 325 or ego_vheicle_yaw >= -135 and ego_vheicle_yaw <= -45:
        if agent_state_y <= ego_vheicle_y:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "FRONT"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap + 0.5 * ego_vheicle_length - 0.5 * agent_state_length
            if relative_front_bumper_gap < 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap
        else:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "BEHIND"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap - 0.5 * ego_vheicle_length + 0.5 * agent_state_length
            if relative_front_bumper_gap > 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap 

    # right direction
    if ego_vheicle_yaw >= 0 and ego_vheicle_yaw <= 45 or ego_vheicle_yaw >= 325 and ego_vheicle_yaw <= 360 or ego_vheicle_yaw >= -45 and ego_vheicle_yaw <= 45:
        if agent_state_x >= ego_vheicle_x:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "FRONT"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap - 0.5 * ego_vheicle_length + 0.5 * agent_state_length
            if relative_front_bumper_gap < 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap
        else:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "BEHIND"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap + 0.5 * ego_vheicle_length - 0.5 * agent_state_length
            if relative_front_bumper_gap > 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap 

    # left direction
    if ego_vheicle_yaw >= 135 and ego_vheicle_yaw <= 225 or ego_vheicle_yaw >= -225 and ego_vheicle_yaw <= -135:
        if agent_state_x <= ego_vheicle_x:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "FRONT"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap + 0.5 * ego_vheicle_length - 0.5 * agent_state_length
            if relative_front_bumper_gap < 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap
        else:
            if relative_front_bumper_gap - 0.5 * ego_vheicle_length - 0.5 * agent_state_length > 0:
                relative_location = "BEHIND"
            else:
                relative_location = "PARALLEL"
            relative_front_bumper_gap = relative_front_bumper_gap - 0.5 * ego_vheicle_length + 0.5 * agent_state_length
            if relative_front_bumper_gap > 0:
                relative_front_bumper_gap = -1 * relative_front_bumper_gap   

    return relative_location, abs(relative_front_bumper_gap)



def data_interpolation(none_number, first_last_lane_id_list1, first_last_list2, first_last_list3, first_last_list4, first_last_list5):
    index = 0
    list_after_interpolation_1 = []
    list_after_interpolation_2 = []
    list_after_interpolation_3 = []
    list_after_interpolation_4 = []
    list_after_interpolation_5 = []

    list_after_interpolation_1.append(first_last_lane_id_list1[0])
    list_after_interpolation_2.append(first_last_list2[0])
    list_after_interpolation_3.append(first_last_list3[0])
    list_after_interpolation_4.append(first_last_list4[0])
    list_after_interpolation_5.append(first_last_list5[0])
    
    list_interval_2 = (first_last_list2[-1] - first_last_list2[0])/none_number
    list_interval_3 = (first_last_list3[-1] - first_last_list3[0])/none_number
    list_interval_4 = (first_last_list4[-1] - first_last_list4[0])/none_number
    list_interval_5 = (first_last_list5[-1] - first_last_list5[0])/none_number
    
    while index < none_number:
        
        list_after_interpolation_1.append(first_last_lane_id_list1[0])
        
        list_single_value_2 = first_last_list2[0] + list_interval_2
        list_after_interpolation_2.append(list_single_value_2)
        
        list_single_value_3 = first_last_list3[0] + list_interval_3
        list_after_interpolation_3.append(list_single_value_3)
        
        list_single_value_4 = first_last_list4[0] + list_interval_4
        list_after_interpolation_4.append(list_single_value_4)

        list_single_value_5 = first_last_list5[0] + list_interval_5
        list_after_interpolation_5.append(list_single_value_5)
        
        index += 1
    
    list_after_interpolation_1.append(first_last_lane_id_list1[-1])
    list_after_interpolation_2.append(first_last_list2[-1])
    list_after_interpolation_3.append(first_last_list3[-1])
    list_after_interpolation_4.append(first_last_list4[-1])
    list_after_interpolation_5.append(first_last_list5[-1])    
    
    return list_after_interpolation_1,\
        list_after_interpolation_2,\
        list_after_interpolation_3,\
        list_after_interpolation_4,\
        list_after_interpolation_5