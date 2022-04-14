#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import matplotlib.pyplot as plt
from pathlib import Path
from IPython import display
#from imageio import imread
#from imageio import mimwrite
import glob
from PIL import Image
import matplotlib.animation as animation
from src import waymo_training_math_tools as math_tool
from src import waymo_training_plot_tools as plot_tool
from src import waymo_scenario_classes as class_tool
from src import waymo_transfer_training_to_tf as transfer_tool

#waymo_dataset_file1 = 'training_dataset/training.tfrecord-00000-of-01000'
#waymo_dataset_file2 = 'training_dataset/training.tfrecord-00200-of-01000'
#waymo_dataset_file3 = 'training_dataset/training.tfrecord-00400-of-01000'
#waymo_dataset_file4 = 'training_dataset/training.tfrecord-00600-of-01000'
#waymo_dataset_file5 = 'training_dataset/training.tfrecord-00702-of-01000'
#waymo_dataset_file6 = 'training_dataset/training.tfrecord-00800-of-01000'
#waymo_dataset_file7 = 'training_dataset/training.tfrecord-00999-of-01000'


#waymo_dataset_file8 = 'training_dataset/training.tfrecord-00100-of-01000'
#waymo_dataset_file9 = 'training_dataset/training.tfrecord-00300-of-01000'
waymo_dataset_file10 = 'training_dataset/training.tfrecord-00500-of-01000'
waymo_dataset_file11 = 'training_dataset/training.tfrecord-00700-of-01000'
waymo_dataset_file12 = 'training_dataset/training.tfrecord-00900-of-01000'


waymo_dataset_list = []
#waymo_dataset_list.append(waymo_dataset_file1)
#waymo_dataset_list.append(waymo_dataset_file2)
#waymo_dataset_list.append(waymo_dataset_file3)
#waymo_dataset_list.append(waymo_dataset_file4)
#waymo_dataset_list.append(waymo_dataset_file5)
#waymo_dataset_list.append(waymo_dataset_file6)
#waymo_dataset_list.append(waymo_dataset_file7)


#waymo_dataset_list.append(waymo_dataset_file8)
#waymo_dataset_list.append(waymo_dataset_file9)
waymo_dataset_list.append(waymo_dataset_file10)
waymo_dataset_list.append(waymo_dataset_file11)
waymo_dataset_list.append(waymo_dataset_file12)


generate_individual_images = True
generate_animation = True

def main():    
    data_set_index = 7
    for data_set in waymo_dataset_list:
        if generate_individual_images:
            print("waymo open dataset analysis begin!")
            size_pixels = 300
            dpi = 400
            size_inch = 22
                   
            scenarios_to_plot_list = [0]
            scenarios_summary = transfer_tool.process_waymo_training_dataset(data_set, scenarios_to_plot_list)
            time_index_list = scenarios_summary.scenarios_list[0].timestep_list
            states_index_list = time_index_list   
            scenario_list = scenarios_summary.scenarios_list        
            
            scenario_index = 0       
            for scenario in scenario_list:                     
    
                plot_path = "waymo_training_plot/" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + "/"
                isExist = os.path.exists(plot_path)               
                if isExist:
                    scenario_index += 1
                    continue
                if not isExist:             
                  os.makedirs(plot_path)
                  print("The new directory is created!")
                
                plot_number = 0
                for time_index in time_index_list:
                    single_step_plot_of_scenario = plot_tool.generate_single_step_plot(scenarios_summary.scenarios_list[0], time_index_list[time_index], dpi, size_pixels, size_inch, plot_path)
                    plot_tool.create_save_single_plot(single_step_plot_of_scenario, plot_path, scenarios_summary.scenarios_list[0].scenario_id, states_index_list[time_index], dpi)
                    print("the " + str(time_index) + "th figure saved")
                    #print("plot number is :", plot_number)
                    plot_number += 1
                    #if plot_number > 3:
                        #break           
                    
                if generate_animation:   
                    
                    single_images_path = plot_path + "*.png"
                    gif_output_path = "waymo_training_plot/" + "dataset_" + str(data_set_index) + "/scenario_" + str(scenario_index) + ".gif"
                    
                    single_images = (Image.open(image_file) for image_file in sorted(glob.glob(single_images_path)))
                    first_image = next(single_images)  # extract first image from iterator
                    first_image.save(fp = gif_output_path, 
                                     format = 'GIF', 
                                     append_images = single_images,
                                     save_all = True, 
                                     duration = 50, 
                                     loop = 0)
                    first_image = None
                scenario_index += 1
            data_set_index += 1
        

if __name__ == "__main__":
    main()
    