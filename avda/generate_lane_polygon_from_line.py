# -*- coding: utf-8 -*-


from shapely.geometry import LineString

def generate_polygon_from_line(line_list, polygon_width):
    number_of_points = len(line_list)
    if number_of_points < 3:
        print("      --- not enough points in this line")
        return None, None
    index = 0
    left_boundary_list = []
    right_boundary_list = []
    while index < number_of_points - 1:
        first_point = line_list[index]
        second_point = line_list[index + 1]
        longitudinal_line = LineString([first_point, second_point])
        left = longitudinal_line.parallel_offset(0.5 * polygon_width, 'left')
        right = longitudinal_line.parallel_offset(0.5 * polygon_width, 'right')
        left_point = left.boundary[1]
        right_point = right.boundary[0]  # note the different orientation for right offset
        left_boundary_list.append((left_point.x, left_point.y))
        right_boundary_list.append((right_point.x, right_point.y))
        index += 1
    return left_boundary_list, right_boundary_list

def generate_polygon_from_right_turn_line(line_list, polygon_width, polygon_offset):
    number_of_points = len(line_list)
    if number_of_points < 3:
        print("      --- not enough points in this line")
        return None, None
    index = 0
    left_boundary_list = []
    right_boundary_list = []
    while index < number_of_points - 1:
        first_point = line_list[index]
        second_point = line_list[index + 1]
        longitudinal_line = LineString([first_point, second_point])
        left = longitudinal_line.parallel_offset(0.5 * polygon_width - polygon_offset, 'left')
        right = longitudinal_line.parallel_offset(0.5 * polygon_width + polygon_offset, 'right')
        left_point = left.boundary[1]
        right_point = right.boundary[0]  # note the different orientation for right offset
        left_boundary_list.append((left_point.x, left_point.y))
        right_boundary_list.append((right_point.x, right_point.y))
        index += 1
    return left_boundary_list, right_boundary_list


def generate_polygon_left_right_from_line(line_list, left_polygon_width, right_polygon_width):
    number_of_points = len(line_list)
    if number_of_points < 3:
        print("      --- not enough points in this line")
        return None, None
    index = 0
    left_boundary_list = []
    right_boundary_list = []
    while index < number_of_points - 1:
        first_point = line_list[index]
        second_point = line_list[index + 1]
        longitudinal_line = LineString([first_point, second_point])
        left = longitudinal_line.parallel_offset(left_polygon_width, 'left')
        right = longitudinal_line.parallel_offset(right_polygon_width, 'right')
        left_point = left.boundary[1]
        right_point = right.boundary[0]  # note the different orientation for right offset
        left_boundary_list.append((left_point.x, left_point.y))
        right_boundary_list.append((right_point.x, right_point.y))
        index += 1
    return left_boundary_list, right_boundary_list      