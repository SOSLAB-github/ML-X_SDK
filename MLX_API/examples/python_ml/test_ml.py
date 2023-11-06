#!/usr/bin/env python3
#-*- coding : utf-8 -*-

import pylibsoslab
import time, csv
from enum import Enum

input_mode = Enum("input_mode", "CONNECT DISCONNECT START QUIT")
mlx = pylibsoslab.MLX()

def check_ml_connection_status():
    is_connected = mlx.is_connected()

    if is_connected is True:
        print("LiDAR ML :: Connection Status :: Connected.")
    else:
        print("LiDAR ML :: Connection Status :: Not Connected.")

def connection_helper():
    print("\n")
    print("----------------------------- Key Input Helper ----------------------------")
    check_ml_connection_status()
    print("c/C: Connect")
    print("d/D: Disconnect")
    print("l/L: PCD Logging Start")
    print("q/Q: Quit")
    print("---------------------------------------------------------------------------")
    print("\n")

def connection_check():
    success = False
    retval = input_mode.QUIT

    user_input = 'q'
    while (success is False):
        user_input = input("Enter userinput : ")

        if (user_input == 'c' or user_input == 'C'):
            retval = input_mode.CONNECT
            success = True
        elif (user_input == 'd' or user_input == 'D'):
            retval = input_mode.DISCONNECT
            success = True
        elif (user_input == 'l' or user_input == 'L'):
            retval = input_mode.START
            success = True
        elif (user_input == 'q' or user_input == 'Q'):
            retval = input_mode.QUIT
            success = True
        else:
            success = False
    
    return retval

def ml_connect():
    print("SOSLAB default ip set-up")
    print("lidar :: ip 192.168.1.10 :: port 2000")
    print("pc    :: ip 0.0.0.0      :: port 0")

	# SOSLAB default ip set-up
	# lidar :: ip 192.168.1.10 :: port 2000
	# pc    :: ip 0.0.0.0      :: port 0
    mlx_ip = pylibsoslab.MLX_IP()
    pc_ip = pylibsoslab.MLX_IP()
    
    mlx_ip.ip_address = "192.168.1.10"
    mlx_ip.port_number = 2000
    
    pc_ip.ip_address = "0.0.0.0"
    pc_ip.port_number = 0

    # connect with default ip set-up
    mlx.connect(mlx_ip, pc_ip)

def ml_disconnect():
    mlx.stop()
    print("LiDAR ML :: stop.")

    mlx.disconnect()
    print("LiDAR ML :: disconnect.")

def ml_logging_start():
    success = mlx.is_connected()
    if (success is False):
        print("LiDAR ML :: Not Connected.")
        print("LiDAR ML :: Connection First.")
        return
    
    total_frame_count = 0
    total_frame_count = int(input("Enter the number of frames to log : "))

    # to get lidar data, must be run
    success = mlx.run()
    if (success is False):
        print("LiDAR ML :: run failed.")
        mlx.disconnect()
        print("LiDAR ML :: disconnect.")
        return
    print("LiDAR ML :: run.")

    # buffer for save lidar data
    lidar_data_buffer = []

    # collecting lidar data.
    print("LiDAR ML :: waiting for collecting data.")
    get_frame_count = 0
    while (get_frame_count < total_frame_count):
        lidar_data = pylibsoslab.MLX_SCENE()

        if (mlx.get_scene(lidar_data)):
            lidar_data_buffer.append(lidar_data)
            get_frame_count += 1
            print(f"LiDAR ML :: collected data count = {get_frame_count}.")
    print("LiDAR ML :: succeed to collecting data.")

    mlx.stop()
    print("LiDAR ML :: stop.")

    # write point-cloud data to .csv
    print("LiDAR ML :: writing point-cloud data to \".csv\".")
    csv_file_prefix = "LiDAR_ML_pcd_"
    frame_id = 1

    for i in range(len(lidar_data_buffer)):
        csv_file_name = csv_file_prefix + str(frame_id) + ".csv"
        frame_id += 1

        f = open(csv_file_name, 'w', newline='')
        wr = csv.writer(f)

        csv_data_list = [['x', 'y', 'z', 'i']]

        scene_data = lidar_data_buffer[i]
        total_idx = scene_data.rows * scene_data.cols
        for j in range(total_idx):
            xyz = scene_data.pointcloud[0][j]
            intensity = scene_data.intensity_image[0][j]
            csv_data_list.append([xyz.x, xyz.y, xyz.z, intensity])
        
        wr.writerows(csv_data_list)
        f.close()
    
    print("LiDAR ML :: wrote pcd data.")

if __name__ == "__main__":
    print("***************************************************************************")
    print("*********************Example :: Lidar data save to .csv********************")
    print(f"***************Version :: {mlx.api_info()}***************")
    print("***************************************************************************")

    user_input = input_mode.QUIT
    success = False
    while (success is False):
        time.sleep(1)
        connection_helper()
        user_input = connection_check()
        if (user_input == input_mode.CONNECT):
            ml_connect()
        elif (user_input == input_mode.DISCONNECT):
            ml_disconnect()
        elif (user_input == input_mode.START):
            ml_logging_start()
        elif (user_input == input_mode.QUIT):
            success = True
        else:
            success = False
            
    ml_disconnect()
    print("LiDAR ML :: done.")
    