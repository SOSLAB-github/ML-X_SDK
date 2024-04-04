/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#include "ml/libsoslab_ml.h"
#include "core/json.hpp"
#include "core/logger.h"
#include "core/ext_interfaces.h"

#ifdef _WIN32
#include<direct.h>
#include<io.h>
#include<ctime>
#endif // _WIN32

#ifdef __linux__
#include<sys/stat.h>
#include<time.h>
#endif

using namespace SOSLAB;

typedef nlohmann::json json_t;

class MLX
{
private:
	std::shared_ptr<ExtInterfaceBase> udp_interface_;
	std::shared_ptr<ExtInterfaceBase> tcp_interface_;

	Fifo<json_t> ack_buffer_;

	Fifo<LidarML::scene_t> scene_buffer_;

	Fifo<std::vector<uint8_t>> udp_receive_buffer_;

	std::thread* parser_thread_;

	bool thread_run;

	std::shared_ptr<Logger> recorder_;

	SOSLAB::LidarML::receive_scene_callback_t recv_scene_callback_ = nullptr;
	void* recv_scene_callback_arg_;

public:
	MLX() :
		udp_interface_(new ExtInterface_UDP_client()),
		tcp_interface_(new ExtInterface_TCP_client()),
		udp_receive_buffer_(128),
		parser_thread_(nullptr), thread_run(false),
		scene_buffer_(3),
		ack_buffer_(1)
	{
	}

	~MLX()
	{
	}

	bool connect(const ip_settings_t& device, const ip_settings_t local)
	{
		ExtInterface_TCP_client::IpSettings ip(local.ip_address, local.port_number, device.ip_address, device.port_number, 1000);

		std::shared_ptr<ExtInterface_TCP_client> tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		tcp->registRecvCallback(tcp_receive_callback, this);
		bool rtn = tcp->connect(ip);
		if (rtn != true) {
			return false;
		}

		std::shared_ptr<ExtInterface_UDP_client> udp = std::dynamic_pointer_cast<ExtInterface_UDP_client>(udp_interface_);
		udp->registRecvCallback(udp_receive_callback, this);
		rtn = udp->connect(ip);
		if (rtn != true) {
			return false;
		}

		udp_receive_buffer_.clear();
		thread_run = false;
		scene_buffer_.clear();
		ack_buffer_.clear();

		return true;
	}

	bool is_connected()
	{
		bool retval = false;
		std::shared_ptr<ExtInterface_TCP_client> tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		retval = tcp->is_connected();
		return retval;
	}

	void disconnect()
	{
		bool retval = false;

		std::string json_disconnect = "{\"command\":\"disconnect\"}";
		std::vector<char> json_vector(json_disconnect.begin(), json_disconnect.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		tcp->disconnect();

		std::shared_ptr<ExtInterface_UDP_client> udp = std::dynamic_pointer_cast<ExtInterface_UDP_client>(udp_interface_);
		udp->disconnect();
	}

	bool run()
	{
		bool retval = false;
		udp_receive_buffer_.clear();

		thread_run = true;

		if (parser_thread_ == nullptr) {
			parser_thread_ = new std::thread([&]() { stream_data_parser(); });
		}
		
		std::string json_run = "{\"command\":\"run\"}";
		std::vector<char> json_vector(json_run.begin(), json_run.end());

		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool stop()
	{
		bool retval = false;
		std::string json_run = "{\"command\":\"stop\"}";
		std::vector<char> json_vector(json_run.begin(), json_run.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		udp_receive_buffer_.clear();

		thread_run = false;
		if (parser_thread_ != nullptr) {
			parser_thread_->join();

			delete parser_thread_;
			parser_thread_ = nullptr;
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool record_start(const std::string filepath) {
		bool retval = false;

		Logger::log_props_t logprops = { filepath, 0 };
		if (recorder_ == nullptr) {
			recorder_ = std::make_shared<Logger>();
			if (recorder_->open(logprops)) {
				retval = true;
			}
			else {
				retval = false;
			}
		}

		return retval;
	}

	void record_stop() {
		if (recorder_ != nullptr) {
			recorder_->close();
			recorder_.reset();
		}
	}
	
	bool play_start(const std::string filepath)
	{
		bool retval = false;

		udp_receive_buffer_.clear();

		thread_run = true;

		if (parser_thread_ == nullptr) {
			parser_thread_ = new std::thread([&]() { stream_data_parser(); });
		}

		Logger::log_props_t logprops = { filepath, 1 };
		if (nullptr == recorder_) {
			recorder_ = std::make_shared<Logger>();
		}
		if (!recorder_->is_open()) {
			if (recorder_->open(logprops)) {
				return true;
			}
			else {
				return false;
			}
		}

		return retval;
	}

	void play_stop() {
		if (recorder_ != nullptr) {
			recorder_->close();
		}

		thread_run = false;
		if (parser_thread_ != nullptr) {
			parser_thread_->join();

			delete parser_thread_;
			parser_thread_ = nullptr;
		}

		udp_receive_buffer_.clear();
	}

	bool get_file_size(uint64_t& size) {
		return recorder_->get_total_size(size);
	}

	bool fps10(bool en)
	{
		bool retval = false;

		std::string json_str;
		if (en) {
			json_str = "{\"PL_Reg\":{\"frame_interval\":10}}";
		}
		else {
			json_str = "{\"PL_Reg\":{\"frame_interval\":20}}";
		}

		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool depth_completion(bool en)
	{
		bool retval = false;

		std::string json_str;
		if (en) {
			json_str = "{\"PL_Reg\":{\"depth_complt_en\":1}}";
		}
		else {
			json_str = "{\"PL_Reg\":{\"depth_complt_en\":0}}";
		}

		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool set_flaring_score(float val)
	{
		bool retval = false;

		std::string json_str;
		json_str = "{\"PL_Reg\":{\"flare_rmv_flare_sum\":" + std::to_string(val) + "}}";

		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool ambient_enable(bool en)
	{
		bool retval = false;

		std::string json_str;
		if (en) {
			json_str = "{\"PL_Reg\":{\"packet_ambient_en\":1}}";
		}
		else {
			json_str = "{\"PL_Reg\":{\"packet_ambient_en\":0}}";
		}
		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool depth_enable(bool en)
	{
		bool retval = false;

		std::string json_str;
		if (en) {
			json_str = "{\"PL_Reg\":{\"packet_depth_en\":1}}";
		}
		else {
			json_str = "{\"PL_Reg\":{\"packet_depth_en\":0}}";
		}
		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool intensity_enable(bool en)
	{
		bool retval = false;

		std::string json_str;
		if (en) {
			json_str = "{\"PL_Reg\":{\"packet_intensity_en\":1}}";
		}
		else {
			json_str = "{\"PL_Reg\":{\"packet_intensity_en\":0}}";
		}
		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	bool multi_echo_enable(bool en)
	{
		bool retval = false;

		std::string json_str;
		if (en) {
			json_str = "{\"PL_Reg\":{\"packet_multi_echo_en\":1}}";
		}
		else {
			json_str = "{\"PL_Reg\":{\"packet_multi_echo_en\":0}}";
		}
		std::vector<char> json_vector(json_str.begin(), json_str.end());
		auto tcp = std::dynamic_pointer_cast<ExtInterface_TCP_client>(tcp_interface_);
		if (tcp->is_connected()) {
			retval = tcp->write(reinterpret_cast<char*>(json_vector.data()), json_vector.size());
			if (retval) std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}

		if (retval) retval = get_ack();

		return retval;
	}

	void register_scene_callback(SOSLAB::LidarML::receive_scene_callback_t callback, void* arg)
	{
		recv_scene_callback_ = callback;
		recv_scene_callback_arg_ = arg;
	}

	bool sync_localtime(void)
	{
		bool retval = false;

		std::string header = "MLXTMNTP";
		struct ntp_packet_t {
			char header[8];
			uint64_t time;
		} send_packet;

		header.copy(send_packet.header, 8);

		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
		uint64_t nanosec_since_epoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
		send_packet.time = nanosec_since_epoch;

		auto udp = std::dynamic_pointer_cast<ExtInterface_UDP_client>(udp_interface_);
		if (udp->is_connected()) {
			retval = udp->write((char*)&send_packet, sizeof(ntp_packet_t));
		}

		return retval;
	}

	bool get_scene(SOSLAB::LidarML::scene_t& scene, int idx = -1)
	{
		std::chrono::system_clock::time_point st;
		bool res = false;

		if (idx > -1 && recorder_ != nullptr) {
			scene_buffer_.clear();
			recorder_->read_once(idx);
			st = std::chrono::system_clock::now();
			while (!scene_buffer_.size()) {
				if (std::chrono::duration<double>(std::chrono::system_clock::now() - st).count() > 0.02)
					break;
			}
		}

		if (scene_buffer_.size()) {	// normal_operation
			res = scene_buffer_.pop(scene);
		}

		return res;
	}

private:
	bool get_ack()
	{
		json_t jsn;
		bool res = false;
		std::chrono::seconds json_ack_time_out = std::chrono::seconds{ 3 };
		std::chrono::system_clock::time_point begin = std::chrono::system_clock::now();

		while (!res) {
			res = ack_buffer_.pop(jsn);
			std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
			std::chrono::duration<double, std::ratio<1, 1>> delta = end - begin;
			if (json_ack_time_out < delta) {
				std::string msg = "[ACK] MLX::ACK Time Out.";
				return false;
			}
		}

		if (res) {
			if (jsn.contains("json_ack")) {
				res = jsn.at("json_ack");
			}

			jsn.clear();
			ack_buffer_.clear();
		}

		if (!res) {
			std::string msg = "[ACK] MLX::ACK FAIL.";
		}

		return res;
	}

	/* receive callback */
	static void tcp_receive_callback(void* arg, char* data, std::size_t size)
	{
		static_cast<MLX*>(arg)->tcp_receive_process(data, size);
	}

	void tcp_receive_process(char* data, std::size_t size)
	{
		std::vector<char> json_receive(size);
		memcpy(&json_receive[0], &data[0], size);

		try {
			json_t jsn = json_t::parse(json_receive);
			ack_buffer_.push(jsn);
			jsn.clear();
		}
		catch (const json_t::parse_error&) {
			std::cerr << "LiDAR ML :: json parse error" << std::endl;
		}
		catch (const json_t::invalid_iterator&) {
			std::cerr << "LiDAR ML :: json invalid_iterator error" << std::endl;
		}
		catch (const json_t::type_error&) {
			std::cerr << "LiDAR ML :: json type error" << std::endl;
		}
		catch (const json_t::out_of_range&) {
			std::cerr << "LiDAR ML :: json out_of_range error" << std::endl;
		}
		catch (const json_t::other_error&) {
			std::cerr << "LiDAR ML :: json other error" << std::endl;
		}
	}

	static void udp_receive_callback(void* arg, char* data, std::size_t size)
	{
		static_cast<MLX*>(arg)->udp_receive_process(data, size);
	}

	void udp_receive_process(char* data, std::size_t size)
	{
		udp_receive_buffer_.push(std::vector<uint8_t>(data, data + size));
		
		/* recorder */
		if (recorder_ != nullptr) {
			recorder_->write(reinterpret_cast<const uint8_t*>(data), size);
		}
	}

	void stream_data_parser()
	{
		std::vector<uint8_t> buffer;
		std::shared_ptr<LidarML::scene_t> scene_ = std::make_shared <LidarML::scene_t>();

		int echo_num = 1;
		bool stream_data_init = false;
		bool get = false;

		while (thread_run) {
			if (recorder_ != nullptr) {
				if (recorder_->is_read()) {
					get = recorder_->read_fifo.pop(buffer);
				}
				else {
					get = udp_receive_buffer_.pop(buffer);
				}
			}
			else {
				get = udp_receive_buffer_.pop(buffer);
			}

			if (get) {
				uint8_t* data = buffer.data(); 
				size_t size = buffer.size();

				ml_lidar_packet_t* lidar_pkt = (ml_lidar_packet_t*)data;

				if (strncmp(lidar_pkt->header, "LIDARPKT", sizeof(lidar_pkt->header)) == 0) {
					if (!stream_data_init) {
						if (lidar_pkt->row_number != 0) continue;
						scene_->timestamp.clear();
						scene_->status = 0;
						scene_->frame_id = 0;
						scene_->rows = 56;
						scene_->cols = 192;

						echo_num = lidar_pkt->type.multi_echo + 1;
						if (lidar_pkt->type.depth_completion) {
							scene_->cols = 576;
						}
						else {
							scene_->cols = 192;
						}

						if (!lidar_pkt->type.ambient_disable) {
							scene_->ambient_image.resize(576 * 56);
						}
						else {
							scene_->ambient_image.resize(0);
						}

						// depth
						if (!lidar_pkt->type.depth_disable) {
							scene_->depth_image.resize(echo_num);
							for (int e = 0; e < echo_num; e++) {
								scene_->depth_image[e].resize(scene_->cols * 56);
							}
						}
						else {
							scene_->depth_image.resize(0);
						}
						// intensity
						if (!lidar_pkt->type.intensity_disable) {
							scene_->intensity_image.resize(echo_num);
							for (int e = 0; e < echo_num; e++) {
								scene_->intensity_image[e].resize(scene_->cols * 56);
							}
						}
						else {
							scene_->intensity_image.resize(0);
						}
						// point cloud
						scene_->pointcloud.resize(echo_num);
						for (int e = 0; e < echo_num; e++) {
							scene_->pointcloud[e].resize(scene_->cols * 56);
						}

						stream_data_init = true;
					}

					if (lidar_pkt->row_number == 0) {
						scene_->timestamp.clear();
						scene_->status = 0;
						scene_->frame_id = lidar_pkt->frame_id;
					}

					scene_->timestamp.push_back(lidar_pkt->timestamp);
					scene_->status |= lidar_pkt->status;
					uint8_t row = lidar_pkt->row_number;

					uint32_t received_size = sizeof(_ML_LIDAR_PACKET_);
					// ambient
					if (!lidar_pkt->type.ambient_disable) {
						uint8_t* amb_ptr = data + received_size;
						memcpy(&scene_->ambient_image[row * 576], amb_ptr, 576 * 4);
						received_size += 576 * 4;
					}
					// lidar data
					for (int c = 0; c < (int)scene_->cols; c++) {
						for (int echo = 0; echo < echo_num; echo++) {
							if (!lidar_pkt->type.depth_disable) {
								uint8_t* depth_ptr = (uint8_t*)(data + received_size);
								scene_->depth_image[echo][(scene_->cols * row) + c] = *(uint32_t*)depth_ptr;
								received_size += 4;
							}
							if (!lidar_pkt->type.intensity_disable) {
								uint8_t* intensity_ptr = (uint8_t*)(data + received_size);
								scene_->intensity_image[echo][(scene_->cols * row) + c] = *(uint16_t*)intensity_ptr;
								received_size += 4;
							}
							{
								point_cloud_t* point_cloud = (point_cloud_t*)(data + received_size);
								scene_->pointcloud[echo][(scene_->cols * row) + c].x = (float)point_cloud->x;
								scene_->pointcloud[echo][(scene_->cols * row) + c].y = (float)point_cloud->y;
								scene_->pointcloud[echo][(scene_->cols * row) + c].z = (float)point_cloud->z;
								received_size += sizeof(point_cloud_t);
							}
						}
					}

					if (row == 55) {
						if (recv_scene_callback_ != nullptr)
							recv_scene_callback_(recv_scene_callback_arg_, *scene_);
						scene_buffer_.push(*scene_);
					}
				}
			}
			else {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				std::this_thread::yield();
			}
		}
	}
};

/*
 * Exported APIs
 */
SOSLAB::LidarML::LidarML() : lidar_(new MLX)
{

}

SOSLAB::LidarML::~LidarML()
{
	delete lidar_;
}

std::string SOSLAB::LidarML::api_info()
{
	std::stringstream ss;
	ss << "SOSLAB LiDAR ML-X API v2.3.0 build";
	return ss.str();
}

bool SOSLAB::LidarML::connect(const ip_settings_t ml, const ip_settings_t local)
{
	return static_cast<MLX*>(lidar_)->connect(ml, local);
}

bool SOSLAB::LidarML::is_connected()
{
	return static_cast<MLX*>(lidar_)->is_connected();
}

void SOSLAB::LidarML::disconnect()
{
	static_cast<MLX*>(lidar_)->disconnect();
}

bool SOSLAB::LidarML::run()
{
	return static_cast<MLX*>(lidar_)->run();
}

bool SOSLAB::LidarML::stop()
{
	return static_cast<MLX*>(lidar_)->stop();
}

bool SOSLAB::LidarML::record_start(const std::string filepath)
{
	return static_cast<MLX*>(lidar_)->record_start(filepath);
}

void SOSLAB::LidarML::record_stop()
{
	static_cast<MLX*>(lidar_)->record_stop();
}

bool SOSLAB::LidarML::play_start(const std::string filepath)
{
	return static_cast<MLX*>(lidar_)->play_start(filepath);
}

void SOSLAB::LidarML::play_stop()
{
	static_cast<MLX*>(lidar_)->play_stop();
}

bool SOSLAB::LidarML::get_file_size(uint64_t& size)
{
	return static_cast<MLX*>(lidar_)->get_file_size(size);
}

bool SOSLAB::LidarML::fps10(bool en)
{
	return static_cast<MLX*>(lidar_)->fps10(en);
}

bool SOSLAB::LidarML::depth_completion(bool en)
{
	return static_cast<MLX*>(lidar_)->depth_completion(en);
}

bool SOSLAB::LidarML::set_flaring_score(float val)
{
	return static_cast<MLX*>(lidar_)->set_flaring_score(val);
}

bool SOSLAB::LidarML::ambient_enable(bool en)
{
	return static_cast<MLX*>(lidar_)->ambient_enable(en);
}

bool SOSLAB::LidarML::depth_enable(bool en)
{
	return static_cast<MLX*>(lidar_)->depth_enable(en);
}

bool SOSLAB::LidarML::intensity_enable(bool en)
{
	return static_cast<MLX*>(lidar_)->intensity_enable(en);
}

bool SOSLAB::LidarML::multi_echo_enable(bool en)
{
	return static_cast<MLX*>(lidar_)->multi_echo_enable(en);
}

void SOSLAB::LidarML::register_scene_callback(receive_scene_callback_t callback, void* arg)
{
	static_cast<MLX*>(lidar_)->register_scene_callback(callback, arg);
}

bool SOSLAB::LidarML::get_scene(scene_t& scene)
{
	return static_cast<MLX*>(lidar_)->get_scene(scene);
}

bool SOSLAB::LidarML::get_scene(scene_t& scene, int idx)
{
	return static_cast<MLX*>(lidar_)->get_scene(scene, idx);
}

bool SOSLAB::LidarML::sync_localtime()
{
	return static_cast<MLX*>(lidar_)->sync_localtime();
}
