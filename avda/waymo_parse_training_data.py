#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import waymo_open_dataset
import os.path, pkgutil
import tensorflow as tf
import csv
import json

pkgpath = os.path.dirname(waymo_open_dataset.__file__)
print([name for _, name, _ in pkgutil.iter_modules([pkgpath])])

from waymo_open_dataset.metrics.ops import py_metrics_ops
from waymo_open_dataset.metrics.python import config_util_py as config_util
from waymo_open_dataset.protos import scenario_pb2 

from waymo_open_dataset.metrics.ops import py_metrics_ops
from waymo_open_dataset.metrics.python import config_util_py as config_util
#from waymo_open_dataset.protos import motion_submission

from google.protobuf import text_format
from .av_data_classes import *
from .waymo_training_plot_tools import *
from .generate_lane_polygon import *
from .generate_lane_topology import *
from .calculate_lane_angle import *

# process begin
def process_waymo_training_dataset(waymo_dataset_file, scenarios_to_plot_list, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset):
    
    motion_dataset = tf.data.TFRecordDataset(waymo_dataset_file)
    motion_scenario_data_list = []
    
    # read data from proto``
    # read each scenario/video into motion_scenario_data_list
    total_scenario = 0
    for motion_data in motion_dataset:
        proto_string = motion_data.numpy()
        scenario_proto = scenario_pb2.Scenario()
        scenario_proto.ParseFromString(proto_string)
        motion_scenario_data_list.append(scenario_proto)
        total_scenario += 1
       
    scenarios_summary = Scenarios()
    scenarios_summary.scenarios_file_name = waymo_dataset_file.split("\\")[-1]
    # motion_scenario_data is a list, only has length
    print("   scenarios number in this dataset: ", len(motion_scenario_data_list))
    
    # iterate every scenario
    scenario_number = 0
    #scenarios_to_plot_list = [0,100,200,300]
    # parse each scenario/video
    
    if len(scenarios_to_plot_list) == 0:
        index = 0
        while index < total_scenario:
            scenarios_to_plot_list.append(index)
            index += 1
    
    scenario_index = 0
    for motion_scenario in motion_scenario_data_list:
        if scenario_number not in scenarios_to_plot_list:
            scenario_number += 1
            continue
        scenario_index += 1
        print("      " + str(scenario_index) + ". motion_scenario_id: ")
        print("      " , motion_scenario.scenario_id)
        
        print("      sdc_track_index: ")
        print("      " , motion_scenario.sdc_track_index)
        
        print("      number of map features: ")
        print("      " , len(motion_scenario.map_features))
        
        print("      number of agent tracks: ")
        print("      " , len(motion_scenario.tracks))    
        
        print("      index of scenario: ")
        print("      " , scenario_number)         
        
        scenario = Waymo_motion_training_scenario()
        scenario.scenario_id = motion_scenario.scenario_id
        scenario.timestep_seconds_list = motion_scenario.timestamps_seconds
        scenario.timestep_list = [i for i in range(len(scenario.timestep_seconds_list))]
        scenario.current_time_index = motion_scenario.current_time_index
        scenario.sdc_track_index = motion_scenario.sdc_track_index
        scenario.number_of_map_features = len(motion_scenario.map_features)
        
        # go through each time step
        #print("    time seconds list is: ", scenario.timestep_seconds_list)
        time_index = 0
        for dynamic_map_state in motion_scenario.dynamic_map_states:
            single_time_index = scenario.timestep_seconds_list[time_index]           
            # go through each signal light
            single_step_signal_state = Waymo_dynamic_map_single_second_state()
            for lane_state in dynamic_map_state.lane_states:
                single_signal_dict = {}
                lane_id = lane_state.lane
                signal_state = lane_state.state
                stop_point = lane_state.stop_point
                
                single_signal_dict["x"] = stop_point.x
                single_signal_dict["y"] = stop_point.y
                single_signal_dict["z"] = stop_point.z
                single_signal_dict["signal_state"] = signal_state
                
                single_step_signal_state.time_step = single_time_index
                single_step_signal_state.lane_id_list.append(lane_id)
                single_step_signal_state.lane_signal_state_dict[lane_id] = single_signal_dict
            scenario.signal_light_state_list.append(single_step_signal_state)
            scenario.signal_light_state_dict[time_index] = single_step_signal_state  
            time_index += 1

        #print("   final time index is : ", time_index)
        
        # iterator tracks/agents
        track_index = 0
        for track in motion_scenario.tracks:
            agent = Motion_training_agent()
            agent.agent_id = track.id
            agent.agent_type = track.object_type
            if track_index == scenario.sdc_track_index:
                agent.agent_type = -1
            
            state_index = 0
            states_array = np.empty((0,13), dtype = float)
            for track_state in track.states:
                           
                state = Agent_states()
                state.timestep = scenario.timestep_list[state_index]
                state.agent_id = track.id  
                state.agent_type = agent.agent_type
                state.center_x = track_state.center_x
                state.center_y = track_state.center_y 
                state.center_z = track_state.center_z 
                state.length   = track_state.length
                state.width    = track_state.width    
                state.height   = track_state.height      
                state.heading  = track_state.heading
                state.velocity_x = track_state.velocity_x
                state.velocity_y = track_state.velocity_y
                state.valid = track_state.valid  
                
                #if state.valid == -1:
                #    print(" ================ found valid -1 : ===================== ")
                #    print(" ================ x is : ====================")
                #    print(" ================ y is : ====================")
                #    continue
                
                single_state = np.array([[state.timestep, state.agent_id, state.agent_type, state.center_x, state.center_y, state.center_z,
                                        state.length, state.width, state.height, state.heading, state.velocity_x, state.velocity_y, state.valid]])          
                
                states_array = np.append(states_array, single_state, axis=0) 
                             
                agent.states_list.append(state)
                agent.states_mask_list.append(state.valid)
                if state.valid:
                    scenario.all_agents_x_list.append(state.center_x)
                    scenario.all_agents_y_list.append(state.center_y)
                    agent.trajectory_point_list.append((state.center_x, state.center_y))
                state_index += 1
            
            agent.states_array = states_array
            scenario.agents_list.append(agent)
            scenario.agents_dict[agent.agent_id] = agent
            track_index += 1
        
        # get hd map
        for map_feature in motion_scenario.map_features:
            
            one_of_feature = Map_feature()
            
            feature_id = map_feature.id
            road_line_type = map_feature.WhichOneof('feature_data')
            
            one_of_feature.feature_id = feature_id
            one_of_feature.type = road_line_type
            
            if road_line_type == "lane": # center line                   
                center_lane = Waymo_center_lane()
                center_lane.lane_id = feature_id
                #print("center lane id is: ",feature_id)
                center_lane.lane_type = "lane"            
                center_lane.speed_limit = map_feature.lane.speed_limit_mph
                
                if map_feature.lane.type == map_feature.lane.LaneType.TYPE_UNDEFINED:
                    center_lane.lane_type = MAP_feature.TYPE_UNSET   
                if map_feature.lane.type == map_feature.lane.LaneType.TYPE_FREEWAY:
                    center_lane.lane_type = MAP_feature.TYPE_FREEWAY                
                if map_feature.lane.type == map_feature.lane.LaneType.TYPE_SURFACE_STREET:
                    center_lane.lane_type = MAP_feature.TYPE_SURFACE_STREET  
                if map_feature.lane.type == map_feature.lane.LaneType.TYPE_BIKE_LANE:
                    center_lane.lane_type = MAP_feature.TYPE_BIKE_LANE  
                    
                # for vehicle map matching
                center_lane_position_list = []
                
                for neighbor_lane in map_feature.lane.left_neighbors:
                    center_lane.left_neighbors_id_list.append(neighbor_lane.feature_id)                   
                    
                for neighbor_lane in map_feature.lane.right_neighbors: 
                    center_lane.right_neighbors_id_list.append(neighbor_lane.feature_id)
                    
                for entry_lane in map_feature.lane.entry_lanes: 
                    center_lane.entry_lane_id_list.append(entry_lane)                    
                    
                for exit_lane in map_feature.lane.exit_lanes: 
                    center_lane.exit_lane_id_list.append(exit_lane)   
                                   
                for boundary_segments in map_feature.lane.left_boundaries:
                    center_lane.left_boundary_segment_list.append(boundary_segments)
                    
                for boundary_segments in map_feature.lane.right_boundaries:
                    center_lane.right_boundary_segment_list.append(boundary_segments)
                                
                for point in map_feature.lane.polyline:
                    center_lane.point_x_list.append(point.x)
                    center_lane.point_y_list.append(point.y)
                    center_lane.point_z_list.append(point.z)
                    center_lane.points_list.append((point.x, point.y))
                    center_lane.points_xyz_list.append((point.x, point.y, point.z))
                    scenario.center_lane_x_list.append(point.x)
                    scenario.center_lane_y_list.append(point.y)
                    
                    one_of_feature.point_x_list.append(point.x)
                    one_of_feature.point_y_list.append(point.y)                   
                    one_of_feature.points_list.append((point.x, point.y))
                    one_of_feature.points_xyz_list.append((point.x, point.y, point.z))
                
                center_lane.calculate_and_determine_if_straight_lane_angle()
                center_lane.calculate_lane_length()
                center_lane.calculate_lane_grade()
                center_lane.calculate_lane_curvature()
                scenario.map_features_list.append(center_lane)
                scenario.lane_black_list.extend([center_lane.point_x_list, center_lane.point_y_list])
                scenario.center_lane_list.append(center_lane)
                scenario.center_lane_dict[feature_id] = center_lane
                scenario.map_feature_dict[feature_id] = one_of_feature
                
                #if feature_id == 209:
                #    print(" --- curren lane is: ", feature_id)
                #    print(" --- exist list is: ", center_lane.exit_lane_id_list) 
                
                #print("lane id is: ", center_lane.lane_id)
                #print("left neighbors are: ", center_lane.left_neighbors_id_list)
                #print("right neighbors are: ", center_lane.right_neighbors_id_list)
                            
            if road_line_type == "road_edge":        
                road_edge = Road_edge()
                road_edge.line_id = feature_id
                road_edge.line_type = "road_edge"
                for point in map_feature.road_edge.polyline:
                    road_edge.point_x_list.append(point.x)
                    road_edge.point_y_list.append(point.y)   
                    scenario.road_edge_x_list.append(point.x)
                    scenario.road_edge_y_list.append(point.y)
                    
                    one_of_feature.point_x_list.append(point.x)
                    one_of_feature.point_y_list.append(point.y)                    
                    one_of_feature.points_list.append((point.x, point.y))
                    
                scenario.map_features_list.append(road_edge)
                scenario.road_edge_list.extend([road_edge.point_x_list, road_edge.point_y_list])  
                scenario.road_line_dict[road_edge.line_id] = road_edge
                scenario.map_feature_dict[feature_id] = one_of_feature

            
            if road_line_type == "road_line":          
                road_line = Road_line()
                road_line.line_id = feature_id
                road_line.line_type = "road_line"          
                for point in map_feature.road_line.polyline:
                    
                    # all points lists
                    road_line.point_x_list.append(point.x)
                    road_line.point_y_list.append(point.y)
                    
                    one_of_feature.point_x_list.append(point.x)
                    one_of_feature.point_y_list.append(point.y)                    
                    one_of_feature.points_list.append((point.x, point.y))
                    
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_UNKNOWN:
                        unknown_x_list.append(point.x)
                        unknown_y_list.append(point.y)
                                            
                    # white line lists
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_BROKEN_SINGLE_WHITE:
                        road_line.broken_single_white_x_list.append(point.x)
                        road_line.broken_single_white_y_list.append(point.y)                
                        scenario.broken_single_white_x_list.append(point.x)
                        scenario.broken_single_white_y_list.append(point.y) 
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_SINGLE_WHITE:
                        road_line.solid_single_white_x_list.append(point.x)
                        road_line.solid_single_white_y_list.append(point.y)  
                        scenario.solid_single_white_x_list.append(point.x)
                        scenario.solid_single_white_y_list.append(point.y)  
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_DOUBLE_WHITE:
                        road_line.solid_double_white_x_list.append(point.x)
                        road_line.solid_double_white_y_list.append(point.y)   
                        scenario.solid_double_white_x_list.append(point.x)
                        scenario.solid_double_white_y_list.append(point.y) 
                        
                    # yellow line lists
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_BROKEN_SINGLE_YELLOW:
                        road_line.broken_single_yellow_x_list.append(point.x)
                        road_line.broken_single_yellow_y_list.append(point.y)                       
                        scenario.broken_single_yellow_x_list.append(point.x)
                        scenario.broken_single_yellow_y_list.append(point.y) 
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_BROKEN_DOUBLE_YELLOW:
                        road_line.broken_double_yellow_x_list.append(point.x)
                        road_line.broken_double_yellow_y_list.append(point.y)                       
                        scenario.broken_double_yellow_x_list.append(point.x)
                        scenario.broken_double_yellow_y_list.append(point.y) 
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_SINGLE_YELLOW:
                        road_line.solid_single_yellow_x_list.append(point.x)
                        road_line.solid_single_yellow_y_list.append(point.y)   
                        scenario.solid_single_yellow_x_list.append(point.x)
                        scenario.solid_single_yellow_y_list.append(point.y)  
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_DOUBLE_YELLOW:
                        road_line.solid_double_yellow_x_list.append(point.x)
                        road_line.solid_double_yellow_y_list.append(point.y)    
                        scenario.solid_double_yellow_x_list.append(point.x)
                        scenario.solid_double_yellow_y_list.append(point.y)   
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_PASSING_DOUBLE_YELLOW:
                        road_line.passing_double_yellow_x_list.append(point.x)
                        road_line.passing_double_yellow_y_list.append(point.y)                        
                        scenario.passing_double_yellow_x_list.append(point.x)
                        scenario.passing_double_yellow_y_list.append(point.y)                          
                    
                scenario.road_line_dict[road_line.line_id] = road_line
                scenario.map_feature_dict[feature_id] = one_of_feature

                # white line lists
                scenario.broken_single_white_list.extend([road_line.broken_single_white_x_list, road_line.broken_single_white_y_list])
                scenario.solid_single_white_list.extend([road_line.solid_single_white_x_list, road_line.solid_single_white_y_list])
                scenario.solid_double_white_list.extend([road_line.solid_double_white_x_list, road_line.solid_double_white_y_list])
            
                # yellow line lists    
                scenario.broken_single_yellow_list.extend([road_line.broken_single_yellow_x_list, road_line.broken_single_yellow_y_list])
                scenario.broken_double_yellow_list.extend([road_line.broken_double_yellow_x_list, road_line.broken_double_yellow_y_list])
                scenario.solid_single_yellow_list.extend([road_line.solid_single_yellow_x_list, road_line.solid_single_yellow_y_list])
                scenario.solid_double_yellow_list.extend([road_line.solid_double_yellow_x_list, road_line.solid_double_yellow_y_list])
                scenario.passing_double_yellow_list.extend([road_line.passing_double_yellow_x_list, road_line.passing_double_yellow_y_list])              
                scenario.map_features_list.append(road_line) 
                scenario.road_line_list.append(road_line)
                
                       
            if road_line_type == "stop_sign":        
                stop_sign = Stop_sign()
                stop_sign.stop_sign_id = feature_id
                stop_sign.lstop_sign_type = "stop_sign"
                stop_sign.conrtolled_lane_id = map_feature.stop_sign.lane
                
                point = map_feature.stop_sign.position
                stop_sign.point_x_list.append(point.x)
                stop_sign.point_y_list.append(point.y)
                scenario.stop_sign_list.extend([stop_sign.point_x_list, stop_sign.point_y_list])              
                scenario.map_features_list.append(stop_sign)
                
                                               
            if road_line_type == "crosswalk":      
                crosswalk = Cross_walk()                
                for point in map_feature.crosswalk.polygon:
                    crosswalk.point_x_list.append(point.x)
                    crosswalk.point_y_list.append(point.y)    
                    
                    one_of_feature.point_x_list.append(point.x)
                    one_of_feature.point_y_list.append(point.y)                    
                    one_of_feature.points_list.append((point.x, point.y))
                    
                scenario.cross_walk_list.append(crosswalk)
                scenario.map_features_list.append(crosswalk)
                scenario.map_feature_dict[feature_id] = one_of_feature
                
            if road_line_type == "speed_bump":               
                speed_bump = Speed_bump()
                for point in map_feature.speed_bump.polygon:
                    speed_bump.point_x_list.append(point.x)
                    speed_bump.point_y_list.append(point.y)
                    
                    one_of_feature.point_x_list.append(point.x)
                    one_of_feature.point_y_list.append(point.y)                    
                    one_of_feature.points_list.append((point.x, point.y))
                    
                scenario.map_features_list.append(speed_bump) 
                scenario.map_feature_dict[feature_id] = one_of_feature
        
        
        #scenario = calculate_scenario_attribute(scenario, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset)
        
        #scenario.get_center_lane_x_y_list()
        #scenario.calculate_left_right_lane_boundary_points()
        
        
        # calculate lane boundary
        scenario.calculate_left_right_lane_boundary_points_v2()
        
        # calculate image max and min 
        scenario.calculate_scenario_region()
        
        # calculate virtual nodes and links
        scenario.calculate_nodes_and_links_v1()
        
        # find straight_turn in crossing
        calculate_straight_turn_in_crossing(scenario)
        
        # delete left and right neighbor lanes of lanes(left_turn, right_turn, through, u_turn) within a crossing
        # because there should not be any lane change during passing a crossing
        delete_left_right_neighbor_lane_in_crossing(scenario)

        # find inbound lane and outbound lane of connector/turn lane
        find_inbound_outbound_lane_for_curve(scenario)
        
        # calculate lane angle
        calculate_lane_angle(scenario)
        
        # calculate if the left boundary line and right boundary line shape
        calculate_and_determine_if_boundary_straight_or_curve(scenario)

        # generate boundary line polygon list
        calculate_lane_left_right_boundary(scenario, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset)

        # generate polygon for plotting
        calculate_lane_polygon_for_plot(scenario)
        
        # generate polygon for map matching
        calculate_lane_polygon_for_map_matching(scenario)
        
        # calculate the connection points and analyze the relationship between lanes
        generate_lane_level_topology(scenario)
        
        # calculate the lane relationship corresponding to the intersection directions
        map_turn_lanes_to_four_directions(scenario)
        
        
        scenarios_summary.scenarios_list.append(scenario)                
        scenario_number += 1
        
    return scenarios_summary



def calculate_scenario_attribute(scenario, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset):
    scenario.calculate_left_right_lane_boundary_points_v2()
    
    # calculate image max and min 
    scenario.calculate_scenario_region()
    
    # calculate virtual nodes and links
    scenario.calculate_nodes_and_links_v1()
    
    # find straight_turn in crossing
    calculate_straight_turn_in_crossing(scenario)
    
    # delete left and right neighbor lanes of lanes(left_turn, right_turn, through, u_turn) within a crossing
    # because there should not be any lane change during passing a crossing
    delete_left_right_neighbor_lane_in_crossing(scenario)

    # find inbound lane and outbound lane of connector/turn lane
    find_inbound_outbound_lane_for_curve(scenario)
    
    # calculate lane angle
    calculate_lane_angle(scenario)
    
    # calculate if the left boundary line and right boundary line shape
    calculate_and_determine_if_boundary_straight_or_curve(scenario)

    # generate boundary line polygon list
    calculate_lane_left_right_boundary(scenario, lane_polygon_width, right_turn_lane_polygon_width, right_turn_lane_polygon_offset)

    # generate polygon for plotting
    calculate_lane_polygon_for_plot(scenario)
    
    # generate polygon for map matching
    calculate_lane_polygon_for_map_matching(scenario)
    
    # calculate the connection points and analyze the relationship between lanes
    generate_lane_level_topology(scenario)
    
    # calculate the lane relationship corresponding to the intersection directions
    map_turn_lanes_to_four_directions(scenario)

    return scenario


def output_map_network_to_csv(scenario, scenario_index, output_file_path, enconding=None):
    
    if output_file_path:
        if not os.path.exists(output_file_path): os.mkdir(output_file_path)

    output_scenario_name = "scenario_" + str(scenario_index) + ".csv"
    output_filepath = os.path.join(output_file_path, output_scenario_name)
    
    while True:
        try:
            if enconding is None:
                outfile = open(output_filepath, 'w', newline='',errors='ignore')
            else:
                outfile = open(output_filepath, 'w', newline='', errors='ignore', encoding=enconding)
            break
        except PermissionError:
            print('csv may be locked by other programs. please release it then try again')
            input()
            
    writer = csv.writer(outfile)
    title_line = ["node_id", "node_x", "node_y",
                  "inbound_link_id", "inbound_from_node_id", "inbound_to_node_id", "inbound_lane", "inbound_lane_entry_lanes", "inbound_lane_exist_lanes",
                  "outbound_link_id", "outbound_from_node_id", "outbound_to_node_id" "outbound_lane", "outbound_lane_entry_lanes", "outbound_lane_exist_lanes"]

    # inbound link - inbound lane - entry lanes
    inbound_lane_entry_lane_title = ["node_id", "node_x", "node_y",
                  "inbound_link_id", "inbound_from_node_id", "inbound_to_node_id", "inbound_lane", "inbound_lane_entry_lanes"]
    
    writer.writerow(inbound_lane_entry_lane_title)
    for node_id, node in scenario.node_dict.items():
        node_x = node.coordinate[0]
        node_y = node.coordinate[1]
        for inbound_link in node.inbound_link_list:  
            inbound_link_id = inbound_link.link_id
            inbound_from_node_id = inbound_link.link_from_node_id
            inbound_to_node_id = inbound_link.link_to_node_id
            for inbound_lane in inbound_link.lane_list:
                inbound_lane_id = inbound_lane.lane_id
                for entry_lane_id in inbound_lane.entry_lane_id_list:               
                    line = [node_id, node_x, node_y, inbound_link_id, inbound_from_node_id, inbound_to_node_id, \
                            inbound_lane_id, entry_lane_id]
                    writer.writerow(line)

    # inbound link - inbound lane - exists lanes
    inbound_lane_exist_lane_title = ["node_id", "node_x", "node_y",
                  "inbound_link_id", "inbound_from_node_id", "inbound_to_node_id", "inbound_lane", "inbound_lane_exist_lanes"]
    
    writer.writerow(inbound_lane_exist_lane_title)
    for node_id, node in scenario.node_dict.items():
        node_x = node.coordinate[0]
        node_y = node.coordinate[1]
        for inbound_link in node.inbound_link_list:  
            inbound_link_id = inbound_link.link_id
            inbound_from_node_id = inbound_link.link_from_node_id
            inbound_to_node_id = inbound_link.link_to_node_id
            for inbound_lane in inbound_link.lane_list:
                inbound_lane_id = inbound_lane.lane_id
                for exist_lane_id in inbound_lane.exit_lane_id_list:                
                    line = [node_id, node_x, node_y, inbound_link_id, inbound_from_node_id, inbound_to_node_id, \
                            inbound_lane_id, exist_lane_id]
                    writer.writerow(line)                        

    # outnound link - outbound lane - entry lanes
    outbound_lane_entry_lane_title = ["node_id", "node_x", "node_y",
                  "outbound_link_id", "outbound_from_node_id", "outbound_to_node_id", "outbound_lane", "outbound_lane_entry_lanes"]
    
    writer.writerow(outbound_lane_entry_lane_title)
    for node_id, node in scenario.node_dict.items():
        node_x = node.coordinate[0]
        node_y = node.coordinate[1]
        for outbound_link in node.outbound_link_list:  
            outbound_link_id = outbound_link.link_id
            outbound_from_node_id = outbound_link.link_from_node_id
            outbound_to_node_id = outbound_link.link_to_node_id
            for outbound_lane in outbound_link.lane_list:
                outbound_lane_id = outbound_lane.lane_id
                for entry_lane_id in outbound_lane.entry_lane_id_list:               
                    line = [node_id, node_x, node_y, outbound_link_id, outbound_from_node_id, outbound_to_node_id, \
                            outbound_lane_id, entry_lane_id]
                    writer.writerow(line)

    # outnound lane - exists lanes
    outbound_lane_exists_lane_title = ["node_id", "node_x", "node_y",
                  "outbound_link_id", "outbound_from_node_id", "outbound_to_node_id", "outbound_lane", "outbound_lane_exist_lanes"]
    
    writer.writerow(outbound_lane_exists_lane_title)
    for node_id, node in scenario.node_dict.items():
        node_x = node.coordinate[0]
        node_y = node.coordinate[1]
        for outbound_link in node.outbound_link_list:  
            outbound_link_id = outbound_link.link_id
            outbound_from_node_id = outbound_link.link_from_node_id
            outbound_to_node_id = outbound_link.link_to_node_id
            for outbound_lane in outbound_link.lane_list:
                outbound_lane_id = outbound_lane.lane_id
                for exist_lane_id in outbound_lane.exit_lane_id_list:                
                    line = [node_id, node_x, node_y, outbound_link_id, outbound_from_node_id, outbound_to_node_id, \
                            outbound_lane_id, exist_lane_id]
                    writer.writerow(line)
                    
    outfile.close()    
    
    

def output_map_network_to_json(scenario, scenario_index, output_file_path, enconding=None):
    
    if output_file_path:
        if not os.path.exists(output_file_path): os.mkdir(output_file_path)
    
    # output node data
    output_scenario_name = "scenario_" + str(scenario_index) + "_node.json"
    output_node_filepath = os.path.join(output_file_path, output_scenario_name)
    
    node_dict = scenario.node_dict
    #print(" ================== node_dict is : ==================== ")
    #print(node_dict)
    
    with open(output_node_filepath, "w") as write_node_file:
        json.dump(node_dict, write_node_file, indent = 4)

    # output link data
    output_scenario_name = "scenario_" + str(scenario_index) + "_link.json"
    output_link_filepath = os.path.join(output_file_path, output_scenario_name)
    
    link_dict = scenario.link_dict
    
    with open(output_link_filepath, "w") as write_link_file:
        json.dump(link_dict, write_link_file, indent = 4)    

    # output turn_movement data
    output_scenario_name = "scenario_" + str(scenario_index) + "_turn_movement.json"
    output_turn_filepath = os.path.join(output_file_path, output_scenario_name)
    
    turn_dict = scenario.turn_movement_dict
    
    with open(output_turn_filepath, "w") as write_turn_file:
        json.dump(turn_dict, write_turn_file, indent = 4)    
        
    # output lane data
    output_scenario_name = "scenario_" + str(scenario_index) + "_lane.json"
    output_lane_filepath = os.path.join(output_file_path, output_scenario_name)
    
    lane_dict = scenario.lane_dict
    
    with open(output_lane_filepath, "w") as write_lane_file:
        json.dump(lane_dict, write_lane_file, indent = 4)            

    # output lane angle data
    output_scenario_name = "scenario_" + str(scenario_index) + "_lane_angle.json"
    output_lane_filepath = os.path.join(output_file_path, output_scenario_name)
    
    lane_angle_dict = scenario.lane_angle_dict
    
    with open(output_lane_filepath, "w") as write_lane_angle_file:
        json.dump(lane_angle_dict, write_lane_angle_file, indent = 4)      
