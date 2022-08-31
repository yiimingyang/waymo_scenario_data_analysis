#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gc
import cv2
import os
import glob
from PIL import Image
from os.path import dirname, abspath
import matplotlib.animation as animation
import moviepy.editor as moviepy
import ffmpeg
import shutil
from avda import waymo_training_plot_tools as plot_tool
from avda import waymo_parse_training_data as transfer_tool
from avda import read_network_data_from_json as network_tool
from avda import map_matching as map_matching_tool
from avda import find_surrounding_vehicle_with_gap_v1 as surrounding_tool
from avda import generate_signal_light_topology as signal_light_tool
from .av_data_classes import *

def run_data_analysis(scenario_parameters):

    data_statistics = Waymo_data_analysis_statistics()
    
    '''
    output_csv_name = "Waymo_2022_training.csv"
    outfile = open(output_csv_name, 'w', newline = '', errors = 'ignore')
    title_line = ["dataset_id","dataset_name","scenario_id","scenario_index","intersection_type"]
    writer.writerow(title_line)
    '''
    
    dataset_index = 0
    output_all_csv_title_line_number = 0
    data_set_index = scenario_parameters.start_dataset_index
    dataset_number = len(scenario_parameters.waymo_dataset_list)
    while dataset_index < dataset_number:
    #for data_set in scenario_parameters.waymo_dataset_list:
        data_set = scenario_parameters.waymo_dataset_list[dataset_index]
        print(" 1. --- data set is --- : ", dataset_index )
        if scenario_parameters.generate_individual_images:
            print("   waymo open dataset analysis begin!")
            #scenario_parameters.scenarios_to_plot_list = []
            scenario_parameters.scenarios_to_plot_list = [0, 1, 2, 12, 19]
            #scenario_parameters.scenarios_to_plot_list = [0, 1, 2, 12, 19, 20, 24, 29, 51, 54, 63, 74, 75, 94]
            scenarios_summary = transfer_tool.process_waymo_training_dataset(data_set, 
                                scenario_parameters.scenarios_to_plot_list, 
                                scenario_parameters.lane_polygon_width,
                                scenario_parameters.right_turn_lane_polygon_width,
                                scenario_parameters.right_turn_lane_polygon_offset)
            time_index_list = scenarios_summary.scenarios_list[0].timestep_list
            states_index_list = time_index_list   
            scenario_list = scenarios_summary.scenarios_list
            #print("   scenario list is : ", scenario_list)
            data_statistics.number_of_dataset += 1
            data_statistics.dataset_id_list.append(dataset_index)
            
            scenario_index = 0
            scenario_number = len(scenario_list)
            while scenario_index < scenario_number:
            #for scenario in scenario_list:     
                data_statistics.number_of_scenario += 1
                data_statistics.scenario_id_list.append(scenario_index)
                
                scenario = scenario_list[scenario_index]   
                dataset_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index)
                plot_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "/"
                #plot_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/"
                csv_path  = scenario_parameters.image_save_path + "//"
                isExist_dataset_path = os.path.exists(dataset_path)
                isExist_plot_path = os.path.exists(plot_path)
                isExist_csv_path = os.path.exists(csv_path)
                       
                '''
                if isExist_plot_path:
                    shutil.rmtree(plot_path)  
                '''
                
                if isExist_plot_path:
                    scenario_index += 1
                    continue
                if not isExist_plot_path:             
                  os.makedirs(plot_path)
                  
                if isExist_csv_path:
                    #scenario_index += 1
                    #continue
                    print("")
                if not isExist_csv_path:             
                  os.makedirs(csv_path)                
                  print("The new csv path is created!")

                if scenario_parameters.output_map_network_csv == "True":
                    #transfer_tool.output_map_network_to_csv(scenarios_summary.scenarios_list[0], data_set_index, csv_path)
                    transfer_tool.output_map_network_to_csv(scenario, data_set_index, csv_path)
                    #print(" ============== scenario_index is : ", scenario_index)

                if scenario_parameters.output_map_network_json == "True":
                    #transfer_tool.output_map_network_to_json(scenarios_summary.scenarios_list[0], data_set_index, csv_path)
                    transfer_tool.output_map_network_to_json(scenario, data_set_index, csv_path)
                    #print(" ============== scenario_index is : ", scenario_index)
                
                if scenario_parameters.read_network_data_from_json == "True":
                    json_file_path = csv_path
                    node_json_name = "scenario_" + str(scenario_index) + "_node.json"
                    link_json_name = "scenario_" + str(scenario_index) + "_link.json"
                    turn_json_name = "scenario_" + str(scenario_index) + "_turn_movement.json"
                    lane_json_name = "scenario_" + str(scenario_index) + "_lane.json"
                    network_tool.read_and_parse_network_json_data(json_file_path,
                                                                  scenario_index, 
                                                                  node_json_name, 
                                                                  link_json_name, 
                                                                  turn_json_name, 
                                                                  lane_json_name,
                                                                  scenario_parameters.dpi,
                                                                  scenario_parameters.size_pixels,
                                                                  scenario_parameters.size_inch,
                                                                  scenario_parameters.plot_node_shape,
                                                                  scenario_parameters.plot_node_color,
                                                                  scenario_parameters.plot_node_size,
                                                                  scenario_parameters.plot_link_shape,
                                                                  scenario_parameters.plot_link_color,
                                                                  scenario_parameters.plot_link_width,                                                                  
                                                                  )
                                                                             
                if scenario_parameters.do_map_matching == "True":
                    #map_matching_tool.map_agent_to_straight_curve_lane(scenarios_summary.scenarios_list[0])
                    #map_matching_tool.calculate_agents_trajectory_type(scenarios_summary.scenarios_list[0])
                    map_matching_tool.calculate_agents_trajectory_type(scenario)
                    map_matching_tool.map_agent_to_lane_v1(scenario)
                    map_matching_tool.generate_unique_map_matching_lane_id(scenario)
                    map_matching_tool.find_merging_diverging_crossing_lane_using_unique_map_matching_lane(scenario)
                    map_matching_tool.find_agent_left_right_lane_spacing(scenario)
                    map_matching_tool.find_agent_lane_change_behavior(scenario)
                    #surrounding_tool.calculate_surrounding_vehicle(scenario)
                    surrounding_tool.calculate_surrounding_vehicle_with_lane_change_gap(scenario)
                    #print(" ============== scenario_index is : ", scenario_index)
                    
                    signal_light_tool.initial_lane_signal_light_state_dict(scenario)
                    signal_light_tool.calculate_four_direction_turn_lane(scenario)
                    signal_light_tool.map_and_calculate_distance_signal_light(scenario)
                    calculate_success = signal_light_tool.calculate_signal_light_topology_for_each_lane(scenario, scenario_parameters.lane_polygon_width, scenario_parameters.right_turn_lane_polygon_width, scenario_parameters.right_turn_lane_polygon_offset)
                    #if calculate_success == False:
                        #scenario_index += 1
                        #break
                    signal_light_tool.summary_signal_light_information(scenario)
                    signal_light_tool.assign_signal_light_to_agents(scenario)
                    map_matching_tool.handle_map_matching_missing_data(scenario, scenario_index, dataset_index)
                    map_matching_tool.calculate_distance_in_lane_at_beginning_ending(scenario)
                    map_matching_tool.calculate_ROW(scenario)
                
                output_statistics_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index)
                if scenario_parameters.output_map_matching_csv == "True":
                    csv_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "_map_matching/"
                    #map_matching_tool.output_map_matching_trajectories_csv(csv_path, scenarios_summary.scenarios_list[0])
                    map_matching_tool.output_map_matching_trajectories_csv(csv_path, output_statistics_path, scenario, dataset_index, scenario_index, scenario_parameters.output_csv_to_one_file)                    
                    #print(" ============== scenario_index is : ", scenario_index)

                if scenario_parameters.output_map_matching_right_turn == "True":
                    csv_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "_right_turn_map_matching/"
                    #map_matching_tool.output_map_matching_trajectories_csv(csv_path, scenarios_summary.scenarios_list[0], None, "right_turn")
                    map_matching_tool.output_map_matching_trajectories_csv(scenario_parameters.filtered_right_turn_output_path, output_statistics_path, scenario, None, "right_turn", dataset_index, scenario_index, scenario_parameters.output_csv_to_one_file)
                    #print(" ============== scenario_index is : ", scenario_index)                                              

                if scenario_parameters.output_map_matching_left_turn == "True":
                    csv_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "_left_turn_map_matching/"
                    #map_matching_tool.output_map_matching_trajectories_csv(csv_path, scenarios_summary.scenarios_list[0], None, "left_turn")
                    map_matching_tool.output_map_matching_trajectories_csv(scenario_parameters.filtered_left_turn_output_path, output_statistics_path, scenario, None, "left_turn", dataset_index, scenario_index, scenario_parameters.output_csv_to_one_file)
                    #print(" ============== scenario_index is : ", scenario_index)   

                if scenario_parameters.output_map_matching_u_turn == "True":
                    csv_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "_u_turn_map_matching/"
                    #map_matching_tool.output_map_matching_trajectories_csv(csv_path, scenarios_summary.scenarios_list[0], None, "u_turn")
                    map_matching_tool.output_map_matching_trajectories_csv(scenario_parameters.filtered_u_turn_output_path, output_statistics_path, scenario, None, "u_turn", dataset_index, scenario_index, scenario_parameters.output_csv_to_one_file)
                    #print(" ============== scenario_index is : ", scenario_index)                            

                if scenario_parameters.output_map_matching_straight == "True":
                    csv_path = scenario_parameters.image_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "_straight_map_matching/"
                    #map_matching_tool.output_map_matching_trajectories_csv(csv_path, scenarios_summary.scenarios_list[0], None, "straight")
                    map_matching_tool.output_map_matching_trajectories_csv(scenario_parameters.filtered_straight_output_path, output_statistics_path, scenario, None, "straight", dataset_index, scenario_index, scenario_parameters.output_csv_to_one_file)
                    #print(" ============== scenario_index is : ", scenario_index)   
                
                #print("   before: ",output_all_csv_title_line_number)
                output_all_csv_title_line_number, data_statistics = map_matching_tool.output_all_map_matching_trajectories_csv(output_all_csv_title_line_number, output_statistics_path, data_statistics, output_statistics_path, scenario, None, None, dataset_index, scenario_index)
                #print("   after: ",output_all_csv_title_line_number)

                if True:
                    plot_number = 0
                    for time_index in time_index_list:
                        if time_index < scenario_parameters.start_frame_number:
                            continue
                        single_step_plot_of_scenario = plot_tool.generate_single_step_plot(scenario,
                                                                    time_index_list[time_index],
                                                                    scenario_parameters.dpi,
                                                                    scenario_parameters.size_pixels,
                                                                    scenario_parameters.size_inch,
                                                                    plot_path,
                                                                    scenario_parameters.plot_topology,
                                                                    scenario_parameters.agent_shape,
                                                                    scenario_parameters.agent_round_shape_width,
                                                                    scenario_parameters.map_line_width,
                                                                    scenario_parameters.plot_straight_lane,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_lane,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_lane_color,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_lane_style,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_left_lane,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_left_lane_color,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_left_lane_style,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_right_lane,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_right_lane_color,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_in_right_lane_style,
                                                                    scenario_parameters.plot_preceeding_and_following_agents_lane_width,
                                                                    scenario_parameters.show_agents_id,
                                                                    scenario_parameters.white_line_color,
                                                                    scenario_parameters.yellow_line_color,
                                                                    scenario_parameters.center_line_color,
                                                                    scenario_parameters.edge_line_color,
                                                                    scenario_parameters.crossing_line_color,
                                                                    scenario_parameters.stop_sign_color,
                                                                    scenario_parameters.sdc_color,
                                                                    scenario_parameters.vehicle_color,
                                                                    scenario_parameters.bicycle_color,
                                                                    scenario_parameters.pedestrian_color,
                                                                    scenario_parameters.show_agents_coordinates,
                                                                    scenario_parameters.agents_id_color,
                                                                    scenario_parameters.agents_id_x_offset,
                                                                    scenario_parameters.agents_id_y_offset,
                                                                    scenario_parameters.agents_coordinates_x_offset,
                                                                    scenario_parameters.agents_coordinates_y_offset,
                                                                    scenario_parameters.plot_node_link,
                                                                    scenario_parameters.plot_node_shape,
                                                                    scenario_parameters.plot_node_color,
                                                                    scenario_parameters.plot_node_size,
                                                                    scenario_parameters.plot_link_shape,
                                                                    scenario_parameters.plot_link_color,
                                                                    scenario_parameters.plot_link_width,
                                                                    scenario_parameters.plot_lane_boundary,
                                                                    scenario_parameters.plot_left_right_turn,
                                                                    scenario_parameters.plot_agents_trajectories,
                                                                    scenario_parameters.plot_different_types_of_figure,
                                                                    scenario_parameters.plot_lane_and_boundaries,
                                                                    scenario_parameters.plot_lane_polygon,
                                                                    scenario_parameters.plot_signal_light) 
                        
                        plot_tool.create_save_single_plot(single_step_plot_of_scenario, plot_path, scenario.scenario_id, states_index_list[time_index], scenario_parameters.dpi)
                        #plot_tool.create_save_single_plot(single_step_plot_of_scenario, plot_path, scenario.scenario_id, scenario_index, scenario_parameters.dpi)
                        #print("the " + str(time_index) + "th figure saved")
                        plot_number += 1 
                        if scenario_parameters.plot_single_image == "True":
                            #print("plot single image!")
                            #if plot_number == 1:
                            #    break
                              
                            if scenario_parameters.plot_image_number != None:
                                print("   plot certain image : ", scenario_parameters.plot_image_number)
                                if plot_number == scenario_parameters.plot_image_number:
                                    break                        
                    
                if scenario_parameters.generate_animation == "True":
                    single_images_path = plot_path + "*.png"
                    gif_output_path = scenario_parameters.animation_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + ".gif"
                    
                    single_images = (Image.open(image_file) for image_file in sorted(glob.glob(single_images_path)))
                    first_image = next(single_images)  # extract first image from iterator
                    first_image.save(fp = gif_output_path,
                                     format = scenario_parameters.animation_format,
                                     append_images = single_images,
                                     save_all = scenario_parameters.gif_save_all,
                                     duration = scenario_parameters.gif_duration,
                                     loop = scenario_parameters.gif_loop)
                    first_image = None

                video_name = None
                if scenario_parameters.generate_video == "True":
                    image_folder = plot_path
                    video_output_path = scenario_parameters.animation_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index)
                    #print("-------- video output path is : ", video_output_path)
                    video_name = video_output_path + "scenario_" + str(scenario_index) + "_video.avi"
                    
                    video_images = [img for img in os.listdir(image_folder) if img.endswith(".png")]
                    video_images = sorted(video_images)
            
                    frame = cv2.imread(os.path.join(image_folder, video_images[0]))
                    height, width, layers = frame.shape
                    
                    video = cv2.VideoWriter(video_name, 0, 1, (width,height))
                    
                    for image in video_images:
                        video.write(cv2.imread(os.path.join(video_output_path, image)))
                    
                    cv2.destroyAllWindows()
                    video.release()     
                    
                ''' 
                if scenario_parameters.generate_mp4 == "True":
                    if video_name == None:
                        print("No video clip fpund")
                    else:
                        video_output_path = scenario_parameters.animation_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index)
                        mp4_video_name = video_output_path + "scenario_" + str(scenario_index) + "_video.mp4"
                        video_clip = moviepy.VideoFileClip(video_name)
                        video_clip.write_videofile(mp4_video_name)    
                '''

                if scenario_parameters.generate_mp4 == "True":
                    current_folder = os.getcwd()
                    image_folder = plot_path
                    os.chdir(image_folder)

                    video_output_path = dirname(dirname(abspath(__file__)))
                    #video_output_path = dirname(abspath(__file__))
                    print("current folder is: ")
                    print(video_output_path)
                    #video_output_path = scenario_parameters.animation_save_path + "//" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "//"
                    video_name_mp4 = video_output_path + "/waymo_training_plot" + "/dataset_" + str(data_set_index) + "/" + "video_" + str(scenario_index) + ".mp4"                   
                    #video_name_mp4 = "video_" + str(scenario_index) + ".mp4"
                    generate_mp4_command = "ffmpeg -framerate 1 -i %03d.png -g 5 " + video_name_mp4
                    
                    os.system(generate_mp4_command)
                    
                    os.chdir(current_folder)
                    
                    gc.collect()
                    
                scenario_index += 1
                
                gc.collect()
                
        dataset_index += 1       
    
    '''
    data_statistics.summary_statistic_data()
    map_matching_tool.output_map_network_to_json(data_statistics, scenario_parameters.image_save_path)
    
    time_step_between_0_and_10 = 0
    time_step_between_10_and_20 = 0
    time_step_between_20_and_30 = 0
    time_step_between_30_and_40 = 0
    time_step_between_40_and_50 = 0
    time_step_between_50_and_60 = 0
    time_step_between_60_and_70 = 0
    time_step_between_70_and_80 = 0
    time_step_between_80_and_90 = 0
    
    
    number_of_frames = len(data_statistics.frame_number_list)
    print("   ---  number of frames in frames list: ", number_of_frames)
    for frame in data_statistics.frame_number_list:
        if frame < 10:
            time_step_between_0_and_10 += 1
        if frame >= 10 and frame < 20:
            time_step_between_10_and_20 += 1            
        if frame >= 20 and frame < 30:
            time_step_between_20_and_30 += 1    
        if frame >= 30 and frame < 40:
            time_step_between_30_and_40 += 1
        if frame >= 40 and frame < 50:
            time_step_between_40_and_50 += 1
        if frame >= 50 and frame < 60:
            time_step_between_50_and_60 += 1
        if frame >= 60 and frame < 70:
            time_step_between_60_and_70 += 1
        if frame >= 70 and frame < 80:
            time_step_between_70_and_80 += 1
        if frame >= 80 and frame <= 91:
            time_step_between_80_and_90 += 1
            
    time_step_between_0_and_10_percentage  = 100 * time_step_between_0_and_10  / number_of_frames
    time_step_between_10_and_20_percentage = 100 * time_step_between_10_and_20 / number_of_frames
    time_step_between_20_and_30_percentage = 100 * time_step_between_20_and_30 / number_of_frames
    time_step_between_30_and_40_percentage = 100 * time_step_between_30_and_40 / number_of_frames
    time_step_between_40_and_50_percentage = 100 * time_step_between_40_and_50 / number_of_frames
    time_step_between_50_and_60_percentage = 100 * time_step_between_50_and_60 / number_of_frames
    time_step_between_60_and_70_percentage = 100 * time_step_between_60_and_70 / number_of_frames
    time_step_between_70_and_80_percentage = 100 * time_step_between_70_and_80 / number_of_frames
    time_step_between_80_and_90_percentage = 100 * time_step_between_80_and_90 / number_of_frames

    print("   ---   frame 0-10 : ",  time_step_between_0_and_10)
    print("   ---   frame 10-20 : ", time_step_between_10_and_20)
    print("   ---   frame 20-30 : ", time_step_between_20_and_30)
    print("   ---   frame 30-40 : ", time_step_between_30_and_40)
    print("   ---   frame 40-50 : ", time_step_between_40_and_50)
    print("   ---   frame 50-60 : ", time_step_between_50_and_60)
    print("   ---   frame 60-70 : ", time_step_between_60_and_70)
    print("   ---   frame 70-80 : ", time_step_between_70_and_80)
    print("   ---   frame 80-90 : ", time_step_between_80_and_90)    

    print("   ---   frame 0-10 : ",  time_step_between_0_and_10_percentage)
    print("   ---   frame 10-20 : ", time_step_between_10_and_20_percentage)
    print("   ---   frame 20-30 : ", time_step_between_20_and_30_percentage)
    print("   ---   frame 30-40 : ", time_step_between_30_and_40_percentage)
    print("   ---   frame 40-50 : ", time_step_between_40_and_50_percentage)
    print("   ---   frame 50-60 : ", time_step_between_50_and_60_percentage)
    print("   ---   frame 60-70 : ", time_step_between_60_and_70_percentage)
    print("   ---   frame 70-80 : ", time_step_between_70_and_80_percentage)
    print("   ---   frame 80-90 : ", time_step_between_80_and_90_percentage)
    '''
    