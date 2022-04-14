#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import math

def degree_to_rdian(degree):
    return degree * math.pi* 0.005556

def radian_to_degree(radian):
    return radian * 180 / math.pi

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