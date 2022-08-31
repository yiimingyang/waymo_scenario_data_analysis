# -*- coding: utf-8 -*-


import sys
#print('cmd entry:', sys.argv)
import csv
import os
from fnmatch import fnmatch
#import matplotlib.pyplot as plt
import numpy as np
import shutil
import time

def get_hhmmss_from_seconds(seconds_int):
    return time.strftime('%H:%M:%S', time.gmtime(seconds_int))

def get_seconds_from_hhmmss(time_str):
    h, m, s = time_str.split(":")
    return int(h) * 3600  + int(m) * 60 + int(s)

def get_average(lst):
    return sum(lst) / len(lst)

def get_occupation_index(occupation_seconds):
    if occupation_seconds == 47.5:
        return 1
    if occupation_seconds == 55:
        return 2
    if occupation_seconds == 70:
        return 3
    if occupation_seconds == 85:
        return 4    
    if occupation_seconds == 115:
        return 5
    if occupation_seconds == 325:
        return 6    
    if occupation_seconds == 475:
        return 7

def get_demand_index(train_number):
    if train_number == 2:
        return 1
    if train_number == 5:
        return 2
    if train_number == 10:
        return 3
    if train_number == 15:
        return 4    

def get_demand5_index(train_number):
    if train_number == 1:
        return 1
    if train_number == 2:
        return 2
    if train_number == 3:
        return 3


def get_slide_average(queue_list):
    queue_length = len(queue_list)
    if queue_length < 100:
        print("length less than 20")
        return
    else:
        queue_average_list = []
        average_queue_length = queue_length - 100
        for i in range(100, average_queue_length):
            temp_queue_list = queue_list[i-100: i]
            temp_queue_average = get_average(temp_queue_list)
            queue_average_list.append(temp_queue_average)
        return queue_average_list
            

# step0: define class for each folder
class Cards_output_route():
    def __init__(self):
        self.route_id = None
        self.vehicle_id = None
        self.vehicle_departure_time = None
        self.energy_list = []
        self.vt_energy_list = []
        self.travel_time_list = []
        self.stop_time_before_crossing_list = []
        self.total_energy = None
        self.total_vt_energy = None
        self.total_energy_consumption = None
        self.total_travel_time = None
        self.total_stop_time_before_crossing = None      
        self.is_impacted_vehicle = False

        
class Cards_output_routes():
    def __init__(self):
        self.route_id_list = []
        self.vehicle_id_list = []
        self.departure_time_list = []
        self.energy_consumption_list = []
        self.travel_time_list = []
        self.stop_time_before_crossing_list = []




class Cards_output_trajectory():
    def __init__(self):
        self.time_list = []
        self.distance_list = []
        self.velocity_list = []
        self.origin_zone_list = []
        self.location_list = []
        self.speed_limit_list = []
        
        self.link_list = []
        self.energy_list = []
        self.vt_energy_list = []
        self.energy_crossing_list = []
        self.vt_energy_crossing_list = []  
        self.vt_fuel_list = []
        self.vt_fuel_impacted_list = []  
            
        self.travel_time_list = []
        self.stop_time_list = []
        self.stop_time_before_crossing_list = []
        self.stop_impacted_time_list = []
        
        self.is_controlled_vehicle = None
        self.is_impacted_vehicle = None
         
        self.total_energy = 0
        self.total_vt_energy = 0
        self.total_crossing_energy = 0
        self.total_crossing_vt_energy = 0 
        self.total_impacted_energy = 0
        self.total_vt_fuel = 0        
        self.total_impacted_vt_fuel = 0         
        
        self.total_stop_time = 0
        self.total_stop_crossing_time = 0
        self.total_impacted_stop_time = 0
        self.total_travel_time = 0
        self.distance = 0
        self.total_delay_time = 0
        
    
class Cards_output_trajectories():
    def __init__(self):
        self.trajectories_list = []
        self.energy_list = []
        self.vt_energy_list = []
        self.energy_crossing_list = []
        self.energy_crossing_vt_list = [] 
        self.energy_impacted_list = []
        self.vt_fuel_list = []
        self.vt_fuel_impacted_list = []          
        
        self.travel_time_list = []
        self.stop_time_list = []
        self.stop_before_crossing_time_list = []
        self.stop_impacted_time_list = []     
        self.delay_time_list = []
        
        self.number_vehicle = 0
        self.number_controlled_vehicle = 0
        self.percentage_controlled_vehicle = 0
        self.number_impacted_vehicle = 0
        self.percentage_impacted_vehicle = 0
        
        self.total_energy = 0
        self.total_crossing_energy = 0
        self.total_impacted_energy = 0
        self.total_vt_energy = 0      
        self.total_crossing_vt_energy = 0  
        self.total_vt_fuel = 0
        self.total_impacted_vt_fuel = 0         
        self.total_stop_time = 0
        self.total_travel_time = 0
        self.total_delay_time = 0
        
        self.average_energy = 0
        self.average_vt_energy = 0
        self.average_crossing_energy = 0
        self.average_crossing_vt_energy = 0     
        self.average_vt_fuel = 0       
        self.average_impacted_vt_fuel = 0        
        self.average_stop_time = 0
        self.average_impacted_stop_time = []
        self.average_travel_time = 0
        self.average_delay_time = 0


class All_data_summary():
    def __init__(self):
        self.csv_head = ["CAV penetration", "vehicle demand (veh/hour)","train length(mile)",\
            "train speed (mph)","train demand (train/hour)",\
            "average energy consumption (PHEV) (kJ)", "average energy saving (PHEV) (%)",\
            "average energy consumption (ICE) (kJ)", "average energy saving (ICE) (%)",\
            "average energy consumption (PHEV) (near HRI) (kJ)", "average energy saving (PHEV) (near HRI) (%)",\
            "average energy consumption (ICE) (near HRI) (kJ)", "average energy saving (ICE) (near HRI) (%)",\
            "average travel time (seconds)","average travel time saving (%)",\
            "average stopped time (seconds)","average stopped time saving (%)",\
            "number of controlled vehicle (veh)", "percentage of controlled vehicle (%)",\
            "average energy consumption (PHEV) (impacted) (kJ)", "average energy saving (PHEV) (impacted) (%)",\
            "average fuel consumption (ICE) (mL)", "average fuel saving (ICE) (%)",\
            "average fuel consumption (ICE) (impacted) (mL)", "average fuel saving (ICE) (impacted) (%)",\
            "average delay time (seconds)","average delay time saving (%)",\
            "number of impacted vehicle (veh)", "percentage of impacted vehicle (%)",\
            "average impacted stopped time (seconds)","average impacted stopped time saving (%)\n"]
            
        self.folder_file_data_dict = {}
        self.basecase_file_data_dict = {}
        self.p25_file_data_dict = {}
        self.p50_file_data_dict = {}
        self.p75_file_data_dict = {}
        self.p100_file_data_dict = {}

class CSV_lines():
    def __init__(self):
        self.csv_head = ["CAV penetration", "vehicle demand (veh/hour)","train length(mile)",\
            "train speed (mph)","train demand (train/hour)",\
            "average energy consumption (PHEV) (kJ)", "average energy saving (PHEV) (%)",\
            "average energy consumption (ICE) (kJ)", "average energy saving (ICE) (%)",\
            "average energy consumption (PHEV) (near HRI) (kJ)", "average energy saving (PHEV) (near HRI) (%)",\
            "average energy consumption (ICE) (near HRI) (kJ)", "average energy saving (ICE) (near HRI) (%)",\
            "average travel time (seconds)","average travel time saving (%)",\
            "average stopped time (seconds)","average stopped time saving (%)",\
            "number of controlled vehicle (veh)", "percentage of controlled vehicle (%)",\
            "average energy consumption (PHEV) (impacted) (kJ)", "average energy saving (PHEV) (impacted) (%)",\
            "average fuel consumption (ICE) (mL)", "average fuel saving (ICE) (%)",\
            "average fuel consumption (ICE) (impacted) (mL)", "average fuel saving (ICE) (impacted) (%)",\
            "average delay time (seconds)","average delay time saving (%)",\
            "number of impacted vehicle (veh)", "percentage of impacted vehicle (%)",\
            "average impacted stopped time (seconds)","average impacted stopped time saving (%)\n"]
            
        self.csv_lines_list = []
        self.basecase_csv_lines_list = []
        self.p25_csv_lines_list = []
        self.p50_csv_lines_list = []
        self.p75_csv_lines_list = []
        self.p100_csv_lines_list = []

class CSV_plot_lines():
    def __init__(self):
        self.csv_head = ["CAV MPR (%)",\
                         "vehicle demand (veh/hour)",\
                         "train occupation (seconds)",\
                         "train occupation index",\
                         "train demand (train/hour)",\
                         "train demand index",\
                         "average energy saving (PHEV) (%)",\
                         "average energy saving (impacted) (PHEV) (%)",\
                         "average stopped time (seconds)",\
                         "average stopped time (impacted) (seconds)",\
                         "queue length (ft)",\
                         "queue vehicle count (veh)\n"]
            
        self.csv_lines_list = []
        self.basecase_csv_lines_list = []
        self.p25_csv_lines_list = []
        self.p50_csv_lines_list = []
        self.p75_csv_lines_list = []
        self.p100_csv_lines_list = []
    

#current_folder_path = os. getcwd()
#files = os.listdir(current_folder_path + "\\vehicle_files")

'''
collect_data = False
online_analysis = True
offline_analysis = False
'''
'''
dest_cvs_file_collection = current_folder_path + "\\" + "csv_data_collection" 
if collect_data:
    # collect every cvs data
    for cards_output_file_folder in cards_output_file_location_list:
           
        files = os.listdir(cards_output_file_folder)
        pattern = "output_vehicle_trajectory_10*"  
        
        # find csv data files
        for file in files:
            if fnmatch(file, pattern):
                file_path = cards_output_file_folder + "\\" + file
                shutil.copy(file_path, dest_cvs_file_collection)
'''            

# online-analysis model: analyze data bu group/folder
def main():

    link_impacted_by_train_line3_list = [30, 516, 11, 1215, 26, 28]
    lane_impacted_by_train_line3_list = [30, 516, 11, 1215, 26, 28]
    signal_head_id_on_roadway_impacted_by_train_line3_list = [37, 36, 40, 39, 43, 42]
    signal_head_position_on_roadway_impacted_by_train_line3_list = [165.720, 458.758, 1001.014, 1703.378, 3918.901, 3801.469]

    link_impacted_by_train_line2_list = [30, 516, 11, 1215, 26, 28]
    lane_impacted_by_train_line2_list = [30, 516, 11, 1215, 26, 28]
    
    link_impacted_by_train_line1_list = [30, 516, 11, 1215, 26, 28]
    lane_impacted_by_train_line1_list = [30, 516, 11, 1215, 26, 28]
    
    crossing_distance_line3_list = [30, 516, 11, 1215, 26, 28]
    
    route1_link_id_list = [2,521,341,752,701,88,20,433,28,12,5]
    route2_link_id_list = [2,521,341,752,1215,13,719,709,12,5]
    route3_link_id_list = [2,516,29,19,534,550,1069,1057,1053,711,5]
    
    route1_unique_link_id_list = [701,88,20,433,28]
    route2_unique_link_id_list = [1215,13,719,709]
    route3_unique_link_id_list = [516,29,19,534,550,1069,1057,1053,711]    
    
    vehicle_group_file_name1 = "output_vehicle_trajectory_1000"
    vehicle_group_file_name2 = "output_vehicle_trajectory_2000"
    vehicle_group_file_name3 = "output_vehicle_trajectory_3000"
    vehicle_group_file_name4 = "output_vehicle_trajectory_4000"
    vehicle_group_file_name5 = "output_vehicle_trajectory_5000"
    vehicle_group_file_name6 = "output_vehicle_trajectory_6000"
    vehicle_group_file_name7 = "output_vehicle_trajectory_7000"
    vehicle_group_file_name8 = "output_vehicle_trajectory_8000"
    
    group_1_csv_list = []
    group_2_csv_list = []
    group_3_csv_list = []
    group_4_csv_list = []
    group_5_csv_list = []
    group_6_csv_list = []
    group_7_csv_list = []
    group_8_csv_list = []
    
    total_route_list = []
    route1_list = []
    route2_list = []
    route3_list = []
    
    all_vehicle_list = []
    
    current_folder_path = os. getcwd()
    files = os.listdir(current_folder_path)
       
    # find csv data files
    for file in files:
        if vehicle_group_file_name1 in file:
            group_1_csv_list.append(file)
        if vehicle_group_file_name2 in file:
            group_2_csv_list.append(file)
        if vehicle_group_file_name3 in file:
            group_3_csv_list.append(file)
        if vehicle_group_file_name4 in file:
            group_4_csv_list.append(file)
        if vehicle_group_file_name5 in file:
            group_5_csv_list.append(file)
        if vehicle_group_file_name6 in file:
            group_6_csv_list.append(file)
        if vehicle_group_file_name7 in file:
            group_7_csv_list.append(file)
        if vehicle_group_file_name8 in file:
            group_8_csv_list.append(file)

    all_vehicle_list.append(group_1_csv_list)
    all_vehicle_list.append(group_2_csv_list)
    all_vehicle_list.append(group_3_csv_list)
    all_vehicle_list.append(group_4_csv_list)
    all_vehicle_list.append(group_5_csv_list)
    all_vehicle_list.append(group_6_csv_list)
    all_vehicle_list.append(group_7_csv_list)
    all_vehicle_list.append(group_8_csv_list)
            
    for vehicle_group in all_vehicle_list:
        
        all_vehicle_data = Cards_output_trajectories()
        all_route_data = Cards_output_routes()
              
        # process each excel files
        impacted_vehicle_list = []
        for file in vehicle_group:
            
            file_str_list = file.split("_")
            vehicle_id = int(file_str_list[-2])
            vehicle_departure_time = get_hhmmss_from_seconds(int(file_str_list[-1].split(".")[0]))
            single_vehicle_data = Cards_output_trajectory()
            single_route_data = Cards_output_route()
            single_route_data.vehicle_departure_time = vehicle_departure_time
            
            with open(file) as trajectory_file:   
                trajectory_data = csv.reader(trajectory_file)  
                next(trajectory_data)
                trajectory_frames = [row for row in trajectory_data] 
        
            line_index = 0
            #current_second = 0
            current_simulation_time = 0
            last_second = 999999
            last_simulation_time = 0
            #simulation_time_index = 0
            
            for trajectory_frame in trajectory_frames: 
                if len(trajectory_frame) == 0:
                    continue
                time_str = trajectory_frame[0][0:8]
                simulation_time_str = trajectory_frame[2]
                location_str = trajectory_frame[5]
                link_str = trajectory_frame[7]
                origin_zone_str = trajectory_frame[13]
                distance_to_lane_start = trajectory_frame[15]
                energy_str   = trajectory_frame[16]
                vt_energy_str = trajectory_frame[22]
                velocity_str = trajectory_frame[4]
                is_controlled_str = trajectory_frame[18]
                speed_limit_str = trajectory_frame[21]
                fuel_ml_str = trajectory_frame[22]
                impacted_str = trajectory_frame[23]
                
                time_seconds = get_seconds_from_hhmmss(time_str)
                current_second = time_seconds
                simulation_time = float(simulation_time_str)
                current_simulation_time = simulation_time
                
                simulation_time_index = int(10 * simulation_time)
                
                location_ft  = float(location_str)
                link_id = int(link_str)
                origin_zone  = int(origin_zone_str)
                distance_to_lane_start_ft  = float(distance_to_lane_start)
                energy_kj    = float(energy_str)
                vt_energy_kj = float(vt_energy_str)
                velocity_mph = float(velocity_str)
                is_controlled = int(is_controlled_str)
                speed_limit = float(speed_limit_str)
                vt_fuel_ml  = float(fuel_ml_str)
                is_impacted = int(impacted_str)            
                                             
                single_vehicle_data.time_list.append(time_seconds)
                single_vehicle_data.link_list.append(link_id)
                single_vehicle_data.distance_list.append(distance_to_lane_start_ft)
                single_vehicle_data.location_list.append(location_ft)
                single_vehicle_data.origin_zone_list.append(origin_zone)
                single_vehicle_data.energy_list.append(energy_kj)
                single_route_data.energy_list.append(energy_kj)
                single_vehicle_data.vt_energy_list.append(vt_fuel_ml)
                single_route_data.vt_energy_list.append(vt_fuel_ml)
                single_vehicle_data.velocity_list.append(velocity_mph)  
                single_vehicle_data.vt_fuel_list.append(vt_fuel_ml)  
                single_vehicle_data.speed_limit_list.append(speed_limit)
              
                #single_vehicle_data.velocity_list.append(velocity_mph)  
                #single_vehicle_data.velocity_list.append(velocity_mph)  
                
                if current_second > last_second:
                    delta_time = current_second - last_second
                    single_vehicle_data.travel_time_list.append(delta_time)
                    single_route_data.travel_time_list.append(delta_time)                                               

                if velocity_mph <= 0.5:                      
                    single_vehicle_data.stop_time_list.append(0.1)

                if velocity_mph <= 0.5 and link_id in link_impacted_by_train_line3_list:   
                    index = link_impacted_by_train_line3_list.index(link_id)
                    signal_head_id = signal_head_id_on_roadway_impacted_by_train_line3_list[index]
                    signal_position = signal_head_position_on_roadway_impacted_by_train_line3_list[index]
                    if distance_to_lane_start_ft < signal_position:
                        single_vehicle_data.stop_time_before_crossing_list.append(0.1)
                        single_route_data.stop_time_before_crossing_list.append(0.1)
                        single_route_data.is_impacted_vehicle = True
                
                if link_id in route1_unique_link_id_list:
                    single_route_data.route_id = 1
                if link_id in route2_unique_link_id_list:
                    single_route_data.route_id = 2                    
                if link_id in route3_unique_link_id_list:
                    single_route_data.route_id = 3                    
                       
                line_index += 1
                last_second = current_second
                last_simulation_time = current_simulation_time          
            
            single_vehicle_data.total_energy = sum(single_vehicle_data.energy_list)    
            single_vehicle_data.total_vt_energy = sum(single_vehicle_data.vt_energy_list)  
            single_vehicle_data.total_travel_time = sum(single_vehicle_data.travel_time_list)
            single_vehicle_data.total_stop_crossing_time = sum(single_vehicle_data.stop_time_before_crossing_list)

            single_route_data.total_energy = sum(single_route_data.energy_list)
            single_route_data.total_vt_energy = sum(single_route_data.vt_energy_list)
            single_route_data.total_travel_time = sum(single_route_data.travel_time_list)
            single_route_data.total_stop_time_before_crossing = sum(single_route_data.stop_time_before_crossing_list)

            all_vehicle_data.number_vehicle += 1
            all_vehicle_data.energy_list.append(single_vehicle_data.total_energy)
            all_vehicle_data.vt_energy_list.append(single_vehicle_data.total_vt_energy)
            all_vehicle_data.stop_before_crossing_time_list.append(single_vehicle_data.total_stop_crossing_time)
            all_vehicle_data.travel_time_list.append(single_vehicle_data.total_travel_time)
            
            if single_route_data.route_id == 1:
                route1_list.append(single_route_data)
            if single_route_data.route_id == 2:
                route2_list.append(single_route_data)
            if single_route_data.route_id == 3:
                route3_list.append(single_route_data)                

            print("vehicle id:", vehicle_id)
            print("route id:", single_route_data.route_id)
            print("vehicle departure time:", vehicle_departure_time)
            print("vehicle energy:", single_vehicle_data.total_energy)
            print("vehicle vt energy:", single_vehicle_data.total_vt_energy)
            print("vehicle travel time:", single_vehicle_data.total_travel_time)
            print("crossing stop time:", single_vehicle_data.total_stop_crossing_time)
            print(" ")

    total_route_list.append(route1_list)
    total_route_list.append(route2_list)
    total_route_list.append(route3_list)

    for route_list in total_route_list:
        departure_time = 0
        energy = 0
        vt_energy = 0
        travel_time = 0
        stop_before_crossing_time = 0
        first_route = route_list[0]
        route_id = first_route.route_id
        for route in route_list:
            departure_time += get_seconds_from_hhmmss(route.vehicle_departure_time)
            energy += route.total_energy       
            vt_energy += route.total_vt_energy       
            travel_time += route.total_travel_time
            stop_before_crossing_time += route.total_stop_time_before_crossing
        route_average_departure_time = get_hhmmss_from_seconds(int(departure_time / len(route_list)))
        route_average_energy = energy / len(route_list)
        route_average_vt_energy = vt_energy / len(route_list)
        route_average_travel_time = travel_time / len(route_list)
        route_average_stop_time_before_crossing = stop_before_crossing_time / len(route_list)
        
        print("route id:", route_id)
        #print("average vehicle departure time:", route_average_departure_time)
        print("average vehicle energy:", route_average_energy)
        print("average vehicle vt energy:", route_average_vt_energy)
        print("average vehicle travel time:", route_average_travel_time)
        print("average crossing stop time:", route_average_stop_time_before_crossing) 
        print(" ")

    # calculate not impacted vehicle
    for route_list in total_route_list:
        departure_time = 0
        energy = 0
        vt_energy = 0
        travel_time = 0
        stop_before_crossing_time = 0
        first_route = route_list[0]
        route_id = first_route.route_id
        index = 0
        for route in route_list:
            if route.is_impacted_vehicle:
                continue
            departure_time += get_seconds_from_hhmmss(route.vehicle_departure_time)
            energy += route.total_energy
            vt_energy += route.total_vt_energy
            travel_time += route.total_travel_time
            stop_before_crossing_time += route.total_stop_time_before_crossing
            index += 1
            
        if index != 0:            
            route_average_departure_time = get_hhmmss_from_seconds(int(departure_time / len(route_list)))
            route_average_energy = energy / index
            route_average_vt_energy = vt_energy / index
            route_average_travel_time = travel_time / index
            route_average_stop_time_before_crossing = stop_before_crossing_time / index
        
            print("not impacted route id:", route_id)
            #print("not impacted average vehicle departure time:", route_average_departure_time)
            print("not impacted average vehicle energy:", route_average_energy)
            print("not impacted average vehicle vt energy:", route_average_vt_energy)
            print("not impacted average vehicle travel time:", route_average_travel_time)
            print("not impacted average crossing stop time:", route_average_stop_time_before_crossing) 
            print(" ")
        else:
            print("not impacted route id:", route_id)
            #print("not impacted average vehicle departure time:", route_average_departure_time)
            print("not impacted average vehicle energy: - ")
            print("not impacted average vehicle vt energy: - ")
            print("not impacted average vehicle travel time: - ")
            print("not impacted average crossing stop time: - ") 
            print(" ")            

    # calculate impacted vehicle
    for route_list in total_route_list:
        departure_time = 0
        energy = 0
        vt_energy = 0
        travel_time = 0
        stop_before_crossing_time = 0
        first_route = route_list[0]
        route_id = first_route.route_id
        index = 0
        for route in route_list:
            if not route.is_impacted_vehicle:
                continue            
            departure_time += get_seconds_from_hhmmss(route.vehicle_departure_time)
            energy += route.total_energy
            vt_energy += route.total_vt_energy
            travel_time += route.total_travel_time
            stop_before_crossing_time += route.total_stop_time_before_crossing
            index += 1
            
        if index != 0:            
            route_average_departure_time = get_hhmmss_from_seconds(int(departure_time / len(route_list)))
            route_average_energy = energy / index
            route_average_vt_energy = vt_energy / index
            route_average_travel_time = travel_time / index
            route_average_stop_time_before_crossing = stop_before_crossing_time / index
        
            print("impacted route id:", route_id)
            #print("impacted average vehicle departure time:", route_average_departure_time)
            print("impacted average vehicle energy:", route_average_energy)
            print("impacted average vehicle vt energy:", route_average_vt_energy)
            print("impacted average vehicle travel time:", route_average_travel_time)
            print("impacted average crossing stop time:", route_average_stop_time_before_crossing) 
            print(" ")
        else:
            print("impacted route id:", route_id)
            #print("not impacted average vehicle departure time:", route_average_departure_time)
            print("impacted average vehicle energy: - ")
            print("impacted average vehicle vt energy: - ")
            print("impacted average vehicle travel time: - ")
            print("impacted average crossing stop time: - ") 
            print(" ")   

if __name__ == "__main__":
    main()  
         