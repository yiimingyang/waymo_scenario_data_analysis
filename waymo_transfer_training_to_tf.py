#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import waymo_open_dataset
import os.path, pkgutil
import tensorflow as tf

pkgpath = os.path.dirname(waymo_open_dataset.__file__)
print([name for _, name, _ in pkgutil.iter_modules([pkgpath])])

from waymo_open_dataset.metrics.ops import py_metrics_ops
from waymo_open_dataset.metrics.python import config_util_py as config_util
from waymo_open_dataset.protos import scenario_pb2 

from waymo_open_dataset.metrics.ops import py_metrics_ops
from waymo_open_dataset.metrics.python import config_util_py as config_util
#from waymo_open_dataset.protos import motion_submission

from google.protobuf import text_format
from .waymo_scenario_classes import *
from .waymo_training_plot_tools import *


# process begin
def process_waymo_training_dataset(waymo_dataset_file, scenarios_to_plot_list):
    
    motion_dataset = tf.data.TFRecordDataset(waymo_dataset_file)
    motion_scenario_data_list = []
    
    # read data from proto
    for motion_data in motion_dataset:
        proto_string = motion_data.numpy()
        scenario_proto = scenario_pb2.Scenario()
        scenario_proto.ParseFromString(proto_string)
        motion_scenario_data_list.append(scenario_proto)    
          
    scenarios_summary = Scenarios()
    scenarios_summary.scenarios_file_name = waymo_dataset_file.split("\\")[-1]
    # motion_scenario_data is a list, only has length
    print("scenarios number in this dataset: ", len(motion_scenario_data_list))
    
    # iterate every scenario
    scenario_number = 0
    #scenarios_to_plot_list = [0,100,200,300]
    for motion_scenario in motion_scenario_data_list:
        if scenario_number not in scenarios_to_plot_list:
            scenario_number += 1
            continue
        
        print("motion_scenario_id: ")
        print(motion_scenario.scenario_id)
        
        print("sdc_track_index: ")
        print(motion_scenario.sdc_track_index)
        
        print("number of map features: ")
        print(len(motion_scenario.map_features))
        
        print("number of agent tracks: ")
        print(len(motion_scenario.tracks))    
        
        scenario = Motion_training_scenario()
        scenario.scenario_id = motion_scenario.scenario_id
        scenario.timestep_seconds_list = motion_scenario.timestamps_seconds
        scenario.timestep_list = [i for i in range(len(scenario.timestep_seconds_list))]
        scenario.current_time_index = motion_scenario.current_time_index
        scenario.sdc_track_index = motion_scenario.sdc_track_index
        scenario.number_of_map_features = len(motion_scenario.map_features)
           
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
                
                single_state = np.array([[state.timestep, state.agent_id, state.agent_type, state.center_x, state.center_y, state.center_z,
                                        state.length, state.width, state.height, state.heading, state.velocity_x, state.velocity_y, state.valid]])          
                
                states_array = np.append(states_array, single_state, axis=0) 
                             
                agent.states_list.append(state)
                agent.states_mask_list.append(state.valid)
                if state.valid:
                    scenario.all_agents_x_list.append(state.center_x)
                    scenario.all_agents_y_list.append(state.center_y)
                state_index += 1
            
            agent.states_array = states_array
            scenario.agents_list.append(agent)
            track_index += 1
           
        for map_feature in motion_scenario.map_features:
            
            feature_id = map_feature.id
            road_line_type = map_feature.WhichOneof('feature_data')
            
            if road_line_type == "lane": # center line                   
                center_lane = Center_lane()
                center_lane.lane_id = feature_id
                #print("center lane id is: ",feature_id)
                center_lane.lane_type = "lane"            
                center_lane.speed_limit = map_feature.lane.speed_limit_mph
                
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
                    
                '''
                for line in map_feature.left_boundaries:
                    center_lane.left_boundary_segment_list.append()
                    
                for line in map_feature.right_boundaries:
                    center_lane.right_boundary_segment_list.append()
                '''
                
                for point in map_feature.lane.polyline:
                    center_lane.point_x_list.append(point.x)
                    center_lane.point_y_list.append(point.y)
                    center_lane.points_list.append((point.x, point.y))
                scenario.map_features_list.append(center_lane)
                scenario.lane_black_list.extend([center_lane.point_x_list, center_lane.point_y_list])
                scenario.center_lane_list.append(center_lane)
                scenario.center_lane_dict[feature_id] = center_lane
                
            
            if road_line_type == "road_edge":        
                road_edge = Road_edge()
                road_edge.line_id = feature_id
                road_edge.line_type = "road_edge"
                for point in map_feature.road_edge.polyline:
                    road_edge.point_x_list.append(point.x)
                    road_edge.point_y_list.append(point.y)   
                scenario.map_features_list.append(road_edge)
                scenario.road_edge_list.extend([road_edge.point_x_list, road_edge.point_y_list])    
            
            
            if road_line_type == "road_line":          
                road_line = Road_line()
                road_line.line_id = feature_id
                road_line.line_type = "road_line"          
                for point in map_feature.road_line.polyline:
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_UNKNOWN:
                        unknown_x_list.append(point.x)
                        unknown_y_list.append(point.y)
                        
                    # white line lists
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_BROKEN_SINGLE_WHITE:
                        road_line.broken_single_white_x_list.append(point.x)
                        road_line.broken_single_white_y_list.append(point.y)                
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_SINGLE_WHITE:
                        road_line.solid_single_white_x_list.append(point.x)
                        road_line.solid_single_white_y_list.append(point.y)  
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_DOUBLE_WHITE:
                        road_line.solid_double_white_x_list.append(point.x)
                        road_line.solid_double_white_y_list.append(point.y)   
                        
                    # yellow line lists
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_BROKEN_SINGLE_YELLOW:
                        road_line.broken_single_yellow_x_list.append(point.x)
                        road_line.broken_single_yellow_y_list.append(point.y)                       
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_BROKEN_DOUBLE_YELLOW:
                        road_line.broken_double_yellow_x_list.append(point.x)
                        road_line.broken_double_yellow_y_list.append(point.y)                       
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_SINGLE_YELLOW:
                        road_line.solid_single_yellow_x_list.append(point.x)
                        road_line.solid_single_yellow_y_list.append(point.y)   
    
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_SOLID_DOUBLE_YELLOW:
                        road_line.solid_double_yellow_x_list.append(point.x)
                        road_line.solid_double_yellow_y_list.append(point.y)    
                        
                    if map_feature.road_line.type == map_feature.road_line.RoadLineType.TYPE_PASSING_DOUBLE_YELLOW:
                        road_line.passing_double_yellow_x_list.append(point.x)
                        road_line.passing_double_yellow_y_list.append(point.y)                        
                                            
                    # all points lists    
                    road_line.point_x_list.append(point.x)
                    road_line.point_y_list.append(point.y)  
    
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
                scenario.cross_walk_list.append(crosswalk)
                scenario.map_features_list.append(crosswalk)
                
            if road_line_type == "speed_bump":               
                speed_bump = Speed_bump()
                for point in map_feature.speed_bump.polygon:
                    speed_bump.point_x_list.append(point.x)
                    speed_bump.point_y_list.append(point.y)   
                scenario.map_features_list.append(speed_bump)       
                          
        scenarios_summary.scenarios_list.append(scenario)                
        scenario_number += 1
        
    return scenarios_summary

