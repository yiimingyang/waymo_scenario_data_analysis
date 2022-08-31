#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from .av_data_classes import *

#av_analysis_parameter_json = "av_analysis_scenarios.json"
av_analysis_parameter_json = "av_analysis_scenarios_debug_origin.json"
av_analysis_parameter = open(av_analysis_parameter_json)
av_analysis_parameter_obj = json.load(av_analysis_parameter)


def read_scenario_file():
    # parse dataset file location
    scenario_parser = Scenario_parser()
    if av_analysis_parameter_obj['waymo_data_file_location'] != None:
        scenario_parser.waymo_data_file_location = av_analysis_parameter_obj['waymo_data_file_location']  
    else:
        scenario_parser.waymo_data_file_location = "training_dataset"
    
    # parse dataset file to be analyzed
    if av_analysis_parameter_obj['waymo_data_file_list'] != None:
        file_list = av_analysis_parameter_obj['waymo_data_file_list']
        file_name_list = []
        for file_dictionary in file_list:
            file_name_list = list(file_dictionary.keys())
            #print(" --- dataset file name list is :", file_name_list)
        for file_name in file_name_list:
            #print(" --- dataset file name is: ",file_name)
            scenario_parser.waymo_dataset_list.append(scenario_parser.waymo_data_file_location + "//" + file_name)
    
    # individual image save path
    if av_analysis_parameter_obj['image_save_path'] != None:
        scenario_parser.image_save_path = av_analysis_parameter_obj['image_save_path']  
    else:
        scenario_parser.image_save_path = "waymo_training_plot"
    
    # gif save path
    if av_analysis_parameter_obj['animation_save_path'] != None:
        scenario_parser.animation_save_path = av_analysis_parameter_obj['animation_save_path']  
    else:
        scenario_parser.animation_save_path = "waymo_training_plot"
    
    # generate individual image or not
    if av_analysis_parameter_obj['generate_individual_images'] == "True":
        scenario_parser.generate_individual_images = True 
    else:
        scenario_parser.generate_individual_images = False
    
    # pixels for an image
    if av_analysis_parameter_obj['size_pixels'] != None:
        scenario_parser.size_pixels = av_analysis_parameter_obj['size_pixels']
    else:
        scenario_parser.size_pixels = 300
    
    # dpi for an image
    if av_analysis_parameter_obj['dpi'] != None:
        scenario_parser.dpi = av_analysis_parameter_obj['dpi']
    else:
        scenario_parser.dpi = 400
   
    # start dataset index to be analyzed
    if av_analysis_parameter_obj['start_dataset_index'] != None:
        scenario_parser.start_dataset_index = av_analysis_parameter_obj['start_dataset_index']
    else:
        scenario_parser.start_dataset_index = 0
    
    # start scenario index to be analyzed
    if av_analysis_parameter_obj['scenario_in_dataset_index'] != None:
        scenario_parser.scenario_in_dataset_index = av_analysis_parameter_obj['scenario_in_dataset_index']
        scenario_parser.scenarios_to_plot_list.append(0)
    else:
        scenario_parser.scenario_in_dataset_index = 0
        scenario_parser.scenarios_to_plot_list = []
    
    # plot size inch
    if av_analysis_parameter_obj['size_inch'] != None:
        scenario_parser.size_inch = av_analysis_parameter_obj['size_inch']
    else:
        scenario_parser.size_inch = 22
    
    # generate aniomation or not
    if av_analysis_parameter_obj['generate_animation'] == "True":
        scenario_parser.generate_animation = True 
    else:
        scenario_parser.generate_animation = False
    
    # animation format
    if av_analysis_parameter_obj['animation_format'] == "GIF":
        scenario_parser.animation_format = 'GIF' 
    
    # save gif or not
    if av_analysis_parameter_obj['gif_save_all'] == "True":
        scenario_parser.gif_save_all = True
    else:
        scenario_parser.gif_save_all = False
    
    # git duration
    if av_analysis_parameter_obj['gif_duration'] != None:
        scenario_parser.gif_duration = av_analysis_parameter_obj['gif_duration']  
    else:
        scenario_parser.gif_duration = 50
    
    # gif loop
    if av_analysis_parameter_obj['gif_loop'] != None:
        scenario_parser.gif_loop = av_analysis_parameter_obj['gif_loop']
    else:
        scenario_parser.gif_loop = 0
    
    # plot lane topology or not
    if av_analysis_parameter_obj['plot_topology'] != None:
        scenario_parser.plot_topology = av_analysis_parameter_obj['plot_topology']    
    else:
        scenario_parser.plot_topology = "False"

    # plot a single image in a scenario or not
    if av_analysis_parameter_obj['plot_single_image'] != None:
        scenario_parser.plot_single_image = av_analysis_parameter_obj['plot_single_image']
        #print("plot single plot is: ", scenario_parser.plot_single_image)
    else:
        scenario_parser.plot_single_image = "False"
        #print("plot single plot is None")

    # agent shape: rectangle or round
    if av_analysis_parameter_obj['agent_shape'] != None:
        scenario_parser.agent_shape = av_analysis_parameter_obj['agent_shape']    
    else:
        scenario_parser.agent_shape = "round"

    # if choose to represent an agent using round, set the radius
    if av_analysis_parameter_obj['agent_round_shape_width'] != None:
        scenario_parser.agent_round_shape_width = av_analysis_parameter_obj['agent_round_shape_width']    
    else:
        scenario_parser.agent_round_shape_width = 6        

    # map line width 
    if av_analysis_parameter_obj['map_line_width'] != None:
        scenario_parser.map_line_width = av_analysis_parameter_obj['map_line_width']    
    else:
        scenario_parser.map_line_width = 0.2    

    # plot straight lane or not
    if av_analysis_parameter_obj['plot_straight_lane'] != None:
        scenario_parser.plot_straight_lane = av_analysis_parameter_obj['plot_straight_lane']    
    else:
        scenario_parser.plot_straight_lane = "False"  

    # plot preceeding and following vehicles in the same lane with sdc 
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_lane'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_lane = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_lane']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_lane = "False"          

    # select the color for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_lane_color'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_lane_color = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_lane_color']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_lane_color = "red"   

    # select the line style for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_lane_style'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_lane_style = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_lane_style']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_lane_style = "solid"  

    # plot preceeding and following vehicles in left lane of sdc
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_left_lane'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_left_lane = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_left_lane']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_left_lane = "False"   

    # select the color for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_left_lane_color'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_left_lane_color = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_left_lane_color']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_left_lane_color = "cyan" 

    # select the color for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_left_lane_style'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_left_lane_style = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_left_lane_style']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_left_lane_style = "solid" 

    # plot preceeding and following vehicles in right lane of sdc
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_right_lane'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_right_lane = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_right_lane']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_right_lane = "False"  

    # select the color for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_right_lane_color'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_right_lane_color = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_right_lane_color']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_right_lane_color = "teal"  

    # select the color for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_right_lane_style'] != None:
        scenario_parser.plot_preceeding_and_following_agents_in_right_lane_style = av_analysis_parameter_obj['plot_preceeding_and_following_agents_in_right_lane_style']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_in_right_lane_style = "solid"  

    # select the line width for preceeding and following vehicles
    if av_analysis_parameter_obj['plot_preceeding_and_following_agents_lane_width'] != None:
        scenario_parser.plot_preceeding_and_following_agents_lane_width = av_analysis_parameter_obj['plot_preceeding_and_following_agents_lane_width']    
    else:
        scenario_parser.plot_preceeding_and_following_agents_lane_width = 3  

    # show agent id or not
    if av_analysis_parameter_obj['show_agents_id'] != None:
        scenario_parser.show_agents_id = av_analysis_parameter_obj['show_agents_id']    
    else:
        scenario_parser.show_agents_id = "False"  

    # set plot image number
    if av_analysis_parameter_obj['plot_image_number'] != None:
        scenario_parser.plot_image_number = av_analysis_parameter_obj['plot_image_number']    
    else:
        scenario_parser.plot_image_number = None 

    # generate video or not
    if av_analysis_parameter_obj['generate_video'] != None:
        scenario_parser.generate_video = av_analysis_parameter_obj['generate_video']    
    else:
        scenario_parser.generate_video = "False" 

    # white line color
    if av_analysis_parameter_obj['white_line_color'] != None:
        scenario_parser.white_line_color = av_analysis_parameter_obj['white_line_color']    
    else:
        scenario_parser.white_line_color = "smokewhite" 

    # yellow line color
    if av_analysis_parameter_obj['yellow_line_color'] != None:
        scenario_parser.yellow_line_color = av_analysis_parameter_obj['yellow_line_color']    
    else:
        scenario_parser.yellow_line_color = "gold" 
        
    # center line color
    if av_analysis_parameter_obj['center_line_color'] != None:
        scenario_parser.center_line_color = av_analysis_parameter_obj['center_line_color']    
    else:
        scenario_parser.center_line_color = "pink"         
        
    # center line color
    if av_analysis_parameter_obj['center_line_color'] != None:
        scenario_parser.center_line_color = av_analysis_parameter_obj['center_line_color']    
    else:
        scenario_parser.center_line_color = "pink"   

    # edge line color
    if av_analysis_parameter_obj['edge_line_color'] != None:
        scenario_parser.edge_line_color = av_analysis_parameter_obj['edge_line_color']    
    else:
        scenario_parser.edge_line_color = "saddlebrone"   

    # crossing line color
    if av_analysis_parameter_obj['crossing_line_color'] != None:
        scenario_parser.crossing_line_color = av_analysis_parameter_obj['crossing_line_color']    
    else:
        scenario_parser.crossing_line_color = "darkorange"   

    # stop sign color
    if av_analysis_parameter_obj['stop_sign_color'] != None:
        scenario_parser.stop_sign_color = av_analysis_parameter_obj['stop_sign_color']    
    else:
        scenario_parser.stop_sign_color = "red"   

    # sdc color
    if av_analysis_parameter_obj['sdc_color'] != None:
        scenario_parser.sdc_color = av_analysis_parameter_obj['sdc_color']    
    else:
        scenario_parser.sdc_color = "red"  

    # vehicle color
    if av_analysis_parameter_obj['vehicle_color'] != None:
        scenario_parser.vehicle_color = av_analysis_parameter_obj['vehicle_color']    
    else:
        scenario_parser.vehicle_color = "blue" 

    # bicycle color
    if av_analysis_parameter_obj['bicycle_color'] != None:
        scenario_parser.bicycle_color = av_analysis_parameter_obj['bicycle_color']    
    else:
        scenario_parser.bicycle_color = "orange" 

    # pedestrian color
    if av_analysis_parameter_obj['pedestrian_color'] != None:
        scenario_parser.pedestrian_color = av_analysis_parameter_obj['pedestrian_color']    
    else:
        scenario_parser.pedestrian_color = "blue" 

    # show agents location/coordinates
    if av_analysis_parameter_obj['show_agents_coordinates'] != None:
        scenario_parser.show_agents_coordinates = av_analysis_parameter_obj['show_agents_coordinates']    
    else:
        scenario_parser.show_agents_coordinates = "False" 

    # show agents id color
    if av_analysis_parameter_obj['agents_id_color'] != None:
        scenario_parser.agents_id_color = av_analysis_parameter_obj['agents_id_color']    
    else:
        scenario_parser.agents_id_color = None 

    # agent id x offset
    if av_analysis_parameter_obj['agents_id_x_offset'] != None:
        scenario_parser.agents_id_x_offset = av_analysis_parameter_obj['agents_id_x_offset']    
    else:
        scenario_parser.agents_id_x_offset = 0 

    # agent id y offset
    if av_analysis_parameter_obj['agents_id_y_offset'] != None:
        scenario_parser.agents_id_y_offset = av_analysis_parameter_obj['agents_id_y_offset']    
    else:
        scenario_parser.agents_id_y_offset = 0 

    # agent coordinates x offset
    if av_analysis_parameter_obj['agents_coordinates_x_offset'] != None:
        scenario_parser.agents_coordinates_x_offset = av_analysis_parameter_obj['agents_coordinates_x_offset']    
    else:
        scenario_parser.agents_coordinates_x_offset = 0 

    # agent coordinates y offset
    if av_analysis_parameter_obj['agents_coordinates_y_offset'] != None:
        scenario_parser.agents_coordinates_y_offset = av_analysis_parameter_obj['agents_coordinates_y_offset']    
    else:
        scenario_parser.agents_coordinates_y_offset = 0 

    # agent coordinates y offset
    if av_analysis_parameter_obj['start_frame_number'] != None:
        scenario_parser.start_frame_number = av_analysis_parameter_obj['start_frame_number']    
    else:
        scenario_parser.start_frame_number = 0 

    # generate mp4
    if av_analysis_parameter_obj['generate_mp4'] != None:
        scenario_parser.generate_mp4 = av_analysis_parameter_obj['generate_mp4']    
    else:
        scenario_parser.generate_mp4 = "False"

    # plot node link
    if av_analysis_parameter_obj['plot_node_link'] != None:
        scenario_parser.plot_node_link = av_analysis_parameter_obj['plot_node_link']    
    else:
        scenario_parser.plot_node_link = "False"

    # plot node shape
    if av_analysis_parameter_obj['plot_node_shape'] != None:
        scenario_parser.plot_node_shape = av_analysis_parameter_obj['plot_node_shape']    
    else:
        scenario_parser.plot_node_shape = "*"

    # plot node size
    if av_analysis_parameter_obj['plot_node_size'] != None:
        scenario_parser.plot_node_size = av_analysis_parameter_obj['plot_node_size']    
    else:
        scenario_parser.plot_node_size = 4

    # plot node color
    if av_analysis_parameter_obj['plot_node_color'] != None:
        scenario_parser.plot_node_color = av_analysis_parameter_obj['plot_node_color']    
    else:
        scenario_parser.plot_node_color = "red"
        
    # plot link shape
    if av_analysis_parameter_obj['plot_link_shape'] != None:
        scenario_parser.plot_link_shape = av_analysis_parameter_obj['plot_link_shape']    
    else:
        scenario_parser.plot_link_shape = "dashed"

    # plot link width
    if av_analysis_parameter_obj['plot_link_width'] != None:
        scenario_parser.plot_link_width = av_analysis_parameter_obj['plot_link_width']    
    else:
        scenario_parser.plot_link_width = 3

    # plot link color
    if av_analysis_parameter_obj['plot_link_color'] != None:
        scenario_parser.plot_link_color = av_analysis_parameter_obj['plot_link_color']    
    else:
        scenario_parser.plot_link_color = "red"        

    # plot link color
    if av_analysis_parameter_obj['plot_lane_boundary'] != None:
        scenario_parser.plot_lane_boundary = av_analysis_parameter_obj['plot_lane_boundary']    
    else:
        scenario_parser.plot_lane_boundary = "False"  

    # plot left right turn
    if av_analysis_parameter_obj['plot_left_right_turn'] != None:
        scenario_parser.plot_left_right_turn = av_analysis_parameter_obj['plot_left_right_turn']    
    else:
        scenario_parser.plot_left_right_turn = "False"  

    # output map network csv
    if av_analysis_parameter_obj['output_map_network_csv'] != None:
        scenario_parser.output_map_network_csv = av_analysis_parameter_obj['output_map_network_csv']    
    else:
        scenario_parser.output_map_network_csv = "True"  

    # output map network json
    if av_analysis_parameter_obj['output_map_network_json'] != None:
        scenario_parser.output_map_network_json = av_analysis_parameter_obj['output_map_network_json']    
    else:
        scenario_parser.output_map_network_json = "True"  

    # output map network json
    if av_analysis_parameter_obj['read_network_data_from_json'] != None:
        scenario_parser.read_network_data_from_json = av_analysis_parameter_obj['read_network_data_from_json']    
    else:
        scenario_parser.read_network_data_from_json = "True"  

    # output map network json
    if av_analysis_parameter_obj['plot_agents_trajectories'] != None:
        scenario_parser.plot_agents_trajectories = av_analysis_parameter_obj['plot_agents_trajectories']    
    else:
        scenario_parser.plot_agents_trajectories = "False" 

    # map matching
    if av_analysis_parameter_obj['do_map_matching'] != None:
        scenario_parser.do_map_matching = av_analysis_parameter_obj['do_map_matching']    
    else:
        scenario_parser.do_map_matching = "False" 

    # output map matching
    if av_analysis_parameter_obj['output_map_matching_csv'] != None:
        scenario_parser.output_map_matching_csv = av_analysis_parameter_obj['output_map_matching_csv']    
    else:
        scenario_parser.output_map_matching_csv = "False" 

    # output right turn
    if av_analysis_parameter_obj['output_map_matching_right_turn'] != None:
        scenario_parser.output_map_matching_right_turn = av_analysis_parameter_obj['output_map_matching_right_turn']    
    else:
        scenario_parser.output_map_matching_right_turn = "False" 

    # output left turn
    if av_analysis_parameter_obj['output_map_matching_left_turn'] != None:
        scenario_parser.output_map_matching_left_turn = av_analysis_parameter_obj['output_map_matching_left_turn']    
    else:
        scenario_parser.output_map_matching_left_turn = "False" 

    # output u turn
    if av_analysis_parameter_obj['output_map_matching_u_turn'] != None:
        scenario_parser.output_map_matching_u_turn = av_analysis_parameter_obj['output_map_matching_u_turn']    
    else:
        scenario_parser.output_map_matching_u_turn = "False" 

    # output straight
    if av_analysis_parameter_obj['output_map_matching_straight'] != None:
        scenario_parser.output_map_matching_straight = av_analysis_parameter_obj['output_map_matching_straight']    
    else:
        scenario_parser.output_map_matching_straight = "False" 

    # output multiple-types-figure
    if av_analysis_parameter_obj['plot_different_types_of_figure'] != None:
        scenario_parser.plot_different_types_of_figure = av_analysis_parameter_obj['plot_different_types_of_figure']    
    else:
        scenario_parser.plot_different_types_of_figure = "False" 

    # output lane and boundaries
    if av_analysis_parameter_obj['plot_lane_and_boundaries'] != None:
        scenario_parser.plot_lane_and_boundaries = av_analysis_parameter_obj['plot_lane_and_boundaries']    
    else:
        scenario_parser.plot_lane_and_boundaries = "False"

    # output lane and boundaries
    if av_analysis_parameter_obj['plot_lane_polygon'] != None:
        scenario_parser.plot_lane_polygon = av_analysis_parameter_obj['plot_lane_polygon']    
    else:
        scenario_parser.plot_lane_polygon = "False"

    # set lane polygon width
    if av_analysis_parameter_obj['lane_polygon_width'] != None:
        scenario_parser.lane_polygon_width = av_analysis_parameter_obj['lane_polygon_width']    
    else:
        scenario_parser.lane_polygon_width = 3

    # set lane polygon width
    if av_analysis_parameter_obj['right_turn_lane_polygon_width'] != None:
        scenario_parser.right_turn_lane_polygon_width = av_analysis_parameter_obj['right_turn_lane_polygon_width']    
    else:
        scenario_parser.right_turn_lane_polygon_width = 7

    # set lane polygon width
    if av_analysis_parameter_obj['right_turn_lane_polygon_offset'] != None:
        scenario_parser.right_turn_lane_polygon_offset = av_analysis_parameter_obj['right_turn_lane_polygon_offset']    
    else:
        scenario_parser.right_turn_lane_polygon_offset = 1.5

    # filtered file path
    if av_analysis_parameter_obj['filtered_left_turn_output_path'] != None:
        scenario_parser.filtered_left_turn_output_path = av_analysis_parameter_obj['filtered_left_turn_output_path']    
    else:
        scenario_parser.filtered_left_turn_output_path = "waymo_training_plot/left_turn"

    if av_analysis_parameter_obj['filtered_right_turn_output_path'] != None:
        scenario_parser.filtered_right_turn_output_path = av_analysis_parameter_obj['filtered_right_turn_output_path']    
    else:
        scenario_parser.filtered_right_turn_output_path = "waymo_training_plot/right_turn"

    if av_analysis_parameter_obj['filtered_u_turn_output_path'] != None:
        scenario_parser.filtered_u_turn_output_path = av_analysis_parameter_obj['filtered_u_turn_output_path']    
    else:
        scenario_parser.filtered_u_turn_output_path = "waymo_training_plot/u_turn"

    if av_analysis_parameter_obj['filtered_straight_output_path'] != None:
        scenario_parser.filtered_straight_output_path = av_analysis_parameter_obj['filtered_straight_output_path']    
    else:
        scenario_parser.filtered_straight_output_path = "waymo_training_plot/straight_turn"

    if av_analysis_parameter_obj['plot_signal_light'] != None:
        scenario_parser.plot_signal_light = av_analysis_parameter_obj['plot_signal_light']    
    else:
        scenario_parser.plot_signal_light = "True"

    if av_analysis_parameter_obj['output_csv_to_one_file'] != None:
        scenario_parser.output_csv_to_one_file = av_analysis_parameter_obj['output_csv_to_one_file']    
    else:
        scenario_parser.output_csv_to_one_file = "True"
         
    return scenario_parser