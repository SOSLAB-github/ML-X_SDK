#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import importlib.util
from enum import Enum
import time
import cv2
import numpy as np

# Function to import a module from a given file path
def import_module_from_path(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# Import the libsoslab_ml library
# module_path = 'path/to/library.py'
module_path = '../../pylibsoslab/libsoslab_ml.py' 
module_name = 'libsoslab_ml'
libsoslab_ml = import_module_from_path(module_name, module_path)

# Default settings
fps_enable = False
ambient_enable = True
depth_enable = True
intensity_enable = True
depth_completion = True

# Define the input modes
input_mode = Enum("input_mode", "CONNECT DISCONNECT START QUIT COMMAND")

# Define the ML API Class and Data Structure
lidar_object = libsoslab_ml.LidarML()
lidar_data = libsoslab_ml.Scene()

# Function to display the connection helper menu
def connection_helper():
    print("\n")
    print("----------------------------- Key Input Helper ----------------------------")
    if not lidar_object.is_connected():
        print("c/C: Connect")
    else:
        print("d/D: Disconnect")
    print("s/S: Start")
    if lidar_object.is_connected():
        print("-------------------------------- Functions --------------------------------")
        print("1 : Set FPS 10 Enable")
        print("2 : Set Ambient Enable")
        print("3 : Set Depth Enable")
        print("4 : Set Intensity Enable")
        print("5 : Set Depth Completion Enable")
    print("---------------------------------------------------------------------------")
    print("q/Q: Quit")
    print("---------------------------------------------------------------------------")
    print("\n")

# Function to handle user input and set connection status
def connection_check():
    global fps_enable
    global ambient_enable
    global depth_enable
    global intensity_enable
    global depth_completion
    success = False
    retval = input_mode.QUIT

    user_input = 'q'
    while not success:
        user_input = input("Enter user input: ")

        if user_input in ['c', 'C']:
            retval = input_mode.CONNECT
            success = True
        elif user_input in ['d', 'D']:
            retval = input_mode.DISCONNECT
            success = True
        elif user_input in ['s', 'S']:
            retval = input_mode.START
            success = True
        elif user_input in ['q', 'Q']:
            retval = input_mode.QUIT
            success = True
        elif user_input == '1':
            retval = input_mode.COMMAND
            if lidar_object.is_connected():
                lidar_object.stop()
                lidar_object.fps10(fps_enable)
                msg = "On" if fps_enable else "Off"
                print(f"Current FPS 10: {msg}")
                fps_enable = not fps_enable
            success = True
        elif user_input == '2':
            retval = input_mode.COMMAND
            if lidar_object.is_connected():
                lidar_object.stop()
                lidar_object.ambient_enable(ambient_enable)
                msg = "On" if ambient_enable else "Off"
                print(f"Current ambient_enable: {msg}")
                ambient_enable = not ambient_enable
            success = True
        elif user_input == '3':
            retval = input_mode.COMMAND
            if lidar_object.is_connected():
                lidar_object.stop()
                lidar_object.depth_enable(depth_enable)
                msg = "On" if depth_enable else "Off"
                print(f"Current depth_enable: {msg}")
                depth_enable = not depth_enable
            success = True
        elif user_input == '4':
            retval = input_mode.COMMAND
            if lidar_object.is_connected():
                lidar_object.stop()
                lidar_object.intensity_enable(intensity_enable)
                msg = "On" if intensity_enable else "Off"
                print(f"Current intensity_enable: {msg}")
                intensity_enable = not intensity_enable
            success = True
        elif user_input == '5':
            retval = input_mode.COMMAND
            if lidar_object.is_connected():
                lidar_object.stop()
                lidar_object.depth_completion(depth_completion)
                msg = "On" if depth_completion else "Off"
                print(f"Current depth_completion: {msg}")
                depth_completion = not depth_completion
            success = True
        else:
            success = False
    
    return retval

# Function to connect to the ML
def ml_connect():
    print("SOSLAB default IP setup")
    print("lidar :: IP 192.168.1.10 :: port 2000")
    print("pc    :: IP 0.0.0.0      :: port 0")
    
    ml_ip_address = "192.168.1.10"
    ml_port_number = 2000
    
    retval = lidar_object.connect(ml_ip_address, ml_port_number)

# Function to disconnect from the ML
def ml_disconnect():
    lidar_object.stop()
    print("LiDAR ML :: stop.")
    lidar_object.disconnect()
    print("LiDAR ML :: disconnect.")

# Function to convert image data to 8-bit format
def convert_to_8bit(image, min_value, max_value):
    norm_image = (image - min_value) / (max_value - min_value)
    norm_image = np.clip(norm_image, 0, 1)
    return (255 * norm_image).astype(np.uint8)

# Function to start visualization ML data 
def ml_logging_start():
    lidar_object.run()
    print("LiDAR ML :: run.")
    print("q/Q: Quit")

    while True:
        lidar_data = lidar_object.get_scene()

        if lidar_data is not None:
            # Display PointCloud Data
            # pcd_idx = 0
            # print(f"Point X(mm): {lidar_data.pointcloud[pcd_idx].x} / Point Y(mm): {lidar_data.pointcloud[pcd_idx].y} Point Z(mm): {lidar_data.pointcloud[pcd_idx].z}")

            i1 = np.reshape(lidar_data.ambient_image, (lidar_data.rows, 576))
            i2 = np.reshape(lidar_data.depth_image, (lidar_data.rows, lidar_data.cols))
            i3 = np.reshape(lidar_data.intensity_image, (lidar_data.rows, lidar_data.cols))

            i1_8bit = convert_to_8bit(i1, 0, 10000)
            i2_8bit = convert_to_8bit(i2, 0, 10000)
            i3_8bit = convert_to_8bit(i3, 0, 3000)

            i1_8bit_resized = cv2.resize(i1_8bit, (600, 400))
            i2_8bit_resized = cv2.resize(i2_8bit, (600, 400))
            i3_8bit_resized = cv2.resize(i3_8bit, (600, 400))

            combined_image = np.concatenate((i1_8bit_resized, i2_8bit_resized, i3_8bit_resized), axis=1)

            cv2.imshow('ML Images', combined_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    lidar_object.stop()

if __name__ == "__main__":
    print("***************************************************************************")
    print("*********************** Example :: ML Image Viewer ************************")
    print(f"************** Version :: {lidar_object.api_info()} **************")
    print("***************************************************************************")

    user_input = input_mode.QUIT
    success = False
    while not success:
        time.sleep(1)
        connection_helper()
        user_input = connection_check()
        if user_input == input_mode.CONNECT:
            ml_connect()
        elif user_input == input_mode.DISCONNECT:
            ml_disconnect()
        elif user_input == input_mode.START:
            ml_logging_start()
        elif user_input == input_mode.QUIT:
            success = True
        else:
            success = False
            
    ml_disconnect()
    print("LiDAR ML :: done.")