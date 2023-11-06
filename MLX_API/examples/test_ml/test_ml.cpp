/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#include <iostream>

#include "ml/libsoslab_ml.h"

using namespace SOSLAB;

enum input_mode { CONNECT = 1, DISCONNECT, START, QUIT };
auto lidar_ml = std::make_shared<LidarML>();
std::vector<LidarML::scene_t> lidar_data_buffer;
size_t TOTAL_FRAME_COUNT = 0;

void check_ml_connection_status(void)
{
	bool is_connected = lidar_ml->is_connected();
	if (is_connected) {
		std::cerr << "LiDAR ML :: Connection Status :: Connected." << std::endl;
	}
	else {
		std::cerr << "LiDAR ML :: Connection Status :: Not Connected." << std::endl;
	}
}

void connection_helper(void)
{
	std::cerr << std::endl;
	std::cerr << "----------------------------- Key Input Helper ----------------------------" << std::endl;
	check_ml_connection_status();
	std::cerr << "c/C: Connect" << std::endl;
	std::cerr << "d/D: Disconnect" << std::endl;
	std::cerr << "l/L: PCD Logging Start" << std::endl;
	std::cerr << "q/Q: Quit" << std::endl;
	std::cerr << "---------------------------------------------------------------------------" << std::endl;
}

int connection_check(void)
{
	bool success = false;
	int retval = input_mode::QUIT;

	char user_input = 'q';
	while (!success) {
		std::cin >> user_input;
		switch (user_input) {
		case 'c':
		case 'C':
			retval = input_mode::CONNECT;
			success = true;
			break;
		case 'd':
		case 'D':
			retval = input_mode::DISCONNECT;
			success = true;
			break;
		case 'l':
		case 'L':
			retval = input_mode::START;
			success = true;
			break;
		case 'q':
		case 'Q':
			retval = input_mode::QUIT;
			success = true;
			break;
		default:
			success = false;
			break;
		}
	}

	return retval;
}

void ml_connect(void)
{
	std::cerr
	<< "SOSLAB default ip set-up" << std::endl
	<< "lidar :: ip 192.168.1.10 :: port 2000" << std::endl
	<< "pc    :: ip 0.0.0.0      :: port 0" << std::endl;

	// SOSLAB default ip set-up
	// lidar :: ip 192.168.1.10 :: port 2000
	// pc    :: ip 0.0.0.0      :: port 0
	ip_settings_t ip_settings_device;
	ip_settings_t ip_settings_pc;
	ip_settings_pc.ip_address = "0.0.0.0";
	ip_settings_pc.port_number = 0;
	ip_settings_device.ip_address = "192.168.1.10";
	ip_settings_device.port_number = 2000;

	// connect with default ip set-up
	lidar_ml->connect(ip_settings_device, ip_settings_pc);
}

void ml_disconnect(void)
{
	lidar_ml->stop();
	std::cerr << "LiDAR ML :: stop." << std::endl;

	lidar_ml->disconnect();
	std::cerr << "LiDAR ML :: disconnect." << std::endl;
}

void ml_scene_data_callback(void* arg, LidarML::scene_t& scene)
{
	if (lidar_data_buffer.size() < TOTAL_FRAME_COUNT) {
		lidar_data_buffer.push_back(scene);
		std::cerr << "LiDAR ML :: collected data count = " << lidar_data_buffer.size() << "." << std::endl;
	}
}

bool ml_logging_start(void)
{
	bool success = lidar_ml->is_connected();
	if (!success) {
		std::cerr << "LiDAR ML :: Not Connected." << std::endl;
		std::cerr << "LiDAR ML :: Connection First." << std::endl;
		return false;
	}

	std::cerr << "Enter the number of frames to log : ";
	std::cin >> TOTAL_FRAME_COUNT;
	
	// buffer for save lidar data
	lidar_data_buffer.reserve(TOTAL_FRAME_COUNT);

	// to get lidar data, must be register callback function
	lidar_ml->register_scene_callback(ml_scene_data_callback, nullptr);
	// to get lidar data, must be run
	success = lidar_ml->run();
	if (!success) {
		std::cerr << "LiDAR ML :: run failed." << std::endl;
		lidar_ml->disconnect();
		std::cerr << "LiDAR ML :: disconnect." << std::endl;
		return false;
	}
	std::cerr << "LiDAR ML :: run." << std::endl;

	// collecting lidar data.
	std::cerr << "LiDAR ML :: waiting for collecting data." << std::endl;
	while (true) {
		if (lidar_data_buffer.size() == TOTAL_FRAME_COUNT) break;
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	std::cerr << "LiDAR ML :: succeed to collecting data." << std::endl;

	// write point-cloud data to .csv
	std::cerr << "LiDAR ML :: writing point-cloud data to \".csv\"." << std::endl;
	std::string csv_file_prefix("LiDAR_ML_pcd_");
	int frame_id = 0;
	std::ofstream csv_file_control;

	for (const LidarML::scene_t& scene : lidar_data_buffer) {
		std::string csv_file_name = csv_file_prefix + std::to_string(++frame_id) + ".csv";

		if (csv_file_control.is_open()) csv_file_control.close();
		if (!csv_file_control.is_open()) {
			csv_file_control.open(csv_file_name, std::fstream::out | std::fstream::trunc);
			csv_file_control << "x,y,z,i\n";
		}

		point_t xyz = { 0.0, 0.0, 0.0};
		uint16_t intensity = 0;
		uint16_t total_idx = scene.rows * scene.cols;
		for (uint16_t i = 0; i < total_idx; i++) {
			xyz = scene.pointcloud[0][i];
			intensity = scene.intensity_image[0][i];
			csv_file_control << xyz.x << ',' << xyz.y << ',' << xyz.z << ',' << intensity << '\n';
		}
	}

	if (csv_file_control.is_open()) csv_file_control.close();
	std::cerr << "LiDAR ML :: wrote pcd data." << std::endl;

	lidar_ml->stop();
	std::cerr << "LiDAR ML :: stop." << std::endl;

	lidar_data_buffer.clear();

	return true;
}

int main()
{
	std::cerr
		<< "***************************************************************************" << std::endl
		<< "*********************Example :: Lidar data save to .csv********************" << std::endl
		<< "***************Version :: " << LidarML::api_info() << "***************" << std::endl
		<< "***************************************************************************" << std::endl;

	int user_input = input_mode::QUIT;
	bool success = false;
	while (!success) {
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		connection_helper();
		user_input = connection_check();
		switch (user_input) {
		case input_mode::CONNECT:
			ml_connect();
			break;
		case input_mode::DISCONNECT:
			ml_disconnect();
			break;
		case input_mode::START:
			ml_logging_start();
			break;
		case input_mode::QUIT:
			success = true;
			break;
		default:
			break;
		}
	}

	ml_disconnect();
	std::cerr << "LiDAR ML :: done." << std::endl;

	return 0;
}