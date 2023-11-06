/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#include "core/logger.h"

using namespace SOSLAB;

Logger::Logger()
	: is_read_(false), current_pos_(0), out_fifo_(128), is_stop_(true), logger_thread_(nullptr), read_fifo(128),
	extension_type(0), read_lidar_id(0), read_pcap_status(false)
{

}

Logger::~Logger()
{
	close();
}

bool Logger::open(const log_props_t& props)
{
	bool retval = false;

	std::size_t found = props.filepath.find_last_of(".");
	std::string extension = props.filepath.substr(found + 1);
	if (extension == "bin") extension_type = 0;
	else if (extension == "pcap") extension_type = 1;
	else extension_type = -1;
	std::string filepath = props.filepath.substr(0, found);

	if (!fs_.is_open()) {
		is_read_ = props.is_read;
		std::string filename;

		std::ios::openmode mode = std::fstream::binary;
		if (is_read_) {
			mode |= std::fstream::in;
			filename = props.filepath;
		}
		else {
			extension_type = 0;
			mode |= std::fstream::out;
			mode |= std::fstream::trunc;
			std::time_t base_time = std::time(NULL);
			struct tm base_date_local;
#ifdef _WIN32
			localtime_s(&base_date_local, &base_time);
#endif // _WIN32

#ifdef __linux__
			base_date_local = *localtime(&base_time);
#endif
			std::ostringstream oss;
			oss << std::put_time(&base_date_local, "%m-%d-%H-%M-%S");
			filename = filepath + "_" + oss.str() + ".bin";
		}

		try {
			fs_.open(filename, mode);
			out_fifo_.clear();
			retval = true;
		}
		catch (const std::ifstream::failure& e) {
			std::cerr << "PCAP :: exception in reading a file." << std::endl;
			std::cerr << "PCAP :: exception :: " << e.what() << std::endl;
			retval = false;
		}
		catch (const std::exception& e) {
			std::cerr << "PCAP :: exception :: " << e.what() << std::endl;
			retval = false;
		}

		if (!is_read_) {
			if (retval) {
				if (nullptr == logger_thread_) {
					logger_thread_ = new std::thread(thread_callback, this);
				}
			}
		}
		else {
			if (extension_type == 0) loadingprocess();
			else if (extension_type == 1) pcap_loadingprocess();
		}
	}

	return retval;
}

void Logger::close()
{
	if (nullptr != logger_thread_) {
		is_stop_ = true;
		logger_thread_->join();
		delete logger_thread_;
		logger_thread_ = nullptr;
		out_fifo_.clear();
	}

	if (fs_.is_open()) {
		fs_.close();
		fs_.clear();
	}
}

void Logger::write(const uint8_t* data, const uint64_t size)
{
	if (fs_.is_open()) {
		std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
		uint64_t now_tick = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count();

		log_header_ = { now_tick, size };
		std::vector<uint8_t> write_data((uint8_t*)&log_header_, (uint8_t*)&log_header_ + sizeof(log_header_t));
		for (size_t i = 0; i < size; i++) {
			write_data.push_back(data[i]);
		}
		out_fifo_.push(write_data);
	}
}

bool Logger::read(uint8_t* data, const std::size_t size)
{
	bool retval = false;
	if (fs_.is_open()) {
		fs_.read(reinterpret_cast<char*>(data), size);

		retval = bool(fs_);
	}
	return retval;
}

std::size_t Logger::current_pos() const
{
	return current_pos_;
}

bool Logger::is_open() const
{
	return fs_.is_open();
}

bool Logger::is_read()
{
	return is_read_;
}

bool SOSLAB::Logger::loadingprocess()
{

	slot_counter = 0;
	fs_.seekg(0, std::fstream::end);
	std::streampos fsize = fs_.tellg();
	fs_.seekg(0, std::fstream::beg);

	is_stop_ = false;
	current_pos_ = 0;
	data_block_start_idx.clear();

	bool check_frame = false;

	if (is_read_) {
		while ((!is_stop_) && (current_pos_ < fsize)) {
			if (read_fifo.size() < read_fifo.max_size()) {
				std::streampos temp = fs_.tellg();
				fs_.read(reinterpret_cast<char*>(&log_header_), sizeof(log_header_t));
				std::vector<uint8_t> data(log_header_.data_size);
				fs_.read(reinterpret_cast<char*>(data.data()), log_header_.data_size);
				current_pos_ = fs_.tellg();
				SOSLAB::Logger::PARSERTYPE res = parser_frame_index(data);
				if (res == SOSLAB::Logger::PARSERTYPE::START) {
					data_block_start_idx.push_back(temp);
					check_frame = true;
				}
				else if (data_block_start_idx.size() && res == SOSLAB::Logger::PARSERTYPE::SKIPERROR) {
					if (check_frame) {
						data_block_start_idx.pop_back();
						check_frame = false;
					}
				}
			}
		}
	}
	prev_slot = -1;
	total_size = data_block_start_idx.size() - 1;

	data_block_start_idx.push_back(fsize);
	return false;
}

bool SOSLAB::Logger::pcap_loadingprocess()
{
	slot_counter = 0;
	fs_.seekg(0, std::fstream::end);
	std::streampos fsize = fs_.tellg();
	fs_.seekg(0, std::fstream::beg);


	std::string loading_percent = "0%";
	std::cout << "PCAP :: Loading : " << loading_percent << std::endl;;

	current_pos_ = 0;

	data_block_start_idx.clear();

	GLOBAL_HDR global_header;
	PCAP_HDR pcap_header;
	std::vector<uint8_t> ether_header;
	std::vector<uint8_t> ip_header;
	std::vector<uint8_t> udp_header;

	fs_.read(reinterpret_cast<char*>(&global_header), sizeof(GLOBAL_HDR));

	is_stop_ = false;
	current_pos_ = 0;
	std::map<int, std::streampos> start_pos_;
	//data_block_start_idx.clear();
	pcap_block_start_idx.clear();

	std::map<int, std::vector<uint8_t>> data;
	std::map<int, int> check_error;

	bool fragment_on = false;

	int frame = 1;
	unsigned long long file_last_check = 0;
	lidar_id_list.clear();
	std::chrono::system_clock::time_point st;
	int percent = 0;

	if (is_read_) {
		while ((!is_stop_) && (current_pos_ < fsize)) {
			current_pos_ = fs_.tellg();
			file_last_check = fs_.tellg();
			if (file_last_check + sizeof(pcap_pkthdr) > fsize) break;

			fs_.read(reinterpret_cast<char*>(&pcap_header), sizeof(pcap_pkthdr));
			std::vector<uint8_t> temp(pcap_header.len);

			if (pcap_header.len == 0) {
				break;
			}

			file_last_check = fs_.tellg();
			if (file_last_check + pcap_header.len > fsize) break;

			if (percent != static_cast<int>(((float)current_pos_ / (float)fsize) * 100)) {
				percent = (float)current_pos_ / (float)fsize * 100;
				loading_percent = std::to_string(percent) + "%";
				std::cout << "PCAP :: Loading : " << loading_percent << std::endl;;
			}

			fs_.read(reinterpret_cast<char*>(temp.data()), pcap_header.len);
			int ether_header_size = sizeof(ETHER_HDR);
			int ip_header_size = sizeof(IPV4_HDR);
			int udp_header_size = sizeof(UDP_HDR);

			ether_header = std::vector<uint8_t>(temp.begin(), temp.begin() + ether_header_size);
			ip_header = std::vector<uint8_t>(temp.begin() + ether_header_size, temp.begin() + ether_header_size + ip_header_size);
			udp_header = std::vector<uint8_t>(temp.begin() + ether_header_size + ip_header_size, temp.begin() + +ether_header_size + ip_header_size + udp_header_size);
			std::vector<uint8_t> udp_data = std::vector<uint8_t>(temp.begin() + ether_header_size + ip_header_size + udp_header_size, temp.end());

			IPV4_HDR ip_header_struct;
			UDP_HDR udp_header_struct;
			memcpy(reinterpret_cast<char*>(&ip_header_struct), ip_header.data(), sizeof(IPV4_HDR));
			memcpy(reinterpret_cast<char*>(&udp_header_struct), udp_header.data(), sizeof(UDP_HDR));

			if (ip_header_struct.ip_protocol == 6) {
				//TCP
			}
			else if (ip_header_struct.ip_protocol == 17) {
				//UDP
				int iphdrlen = 0, data_size = 0;
				iphdrlen = ip_header_struct.ip_header_len * 4;

				int offset_fragment = (ip_header_struct.ip_frag_offset) * 2048 + (ip_header_struct.ip_frag_offset1) * 8;
				int flag_fragment = ip_header_struct.ip_more_fragment;
				int dont_fragment = ip_header_struct.ip_dont_fragment;
				int start_pos = 0;

				if (dont_fragment == 0) {

					int ip1, ip2, ip3, ip4;

					ip1 = int(ip_header_struct.ip_srcaddr & 0xFF);
					ip2 = int((ip_header_struct.ip_srcaddr >> 8) & 0xFF);
					ip3 = int((ip_header_struct.ip_srcaddr >> 16) & 0xFF);
					ip4 = int((ip_header_struct.ip_srcaddr >> 24) & 0xFF);

					bool check_ip = false;
					for (auto ip : lidar_id_list) {
						if (ip == ip4) {
							check_ip = true;
						}
					}

					if (!check_ip && ip4 != 15) {
						lidar_id_list.push_back(ip4);
						pcap_block_start_idx.insert(std::make_pair(ip4, std::vector<std::streampos>(0)));
						data.insert(std::make_pair(ip4, std::vector< uint8_t>(0)));
						start_pos_.insert(std::make_pair(ip4, std::streampos()));
						check_error.insert(std::make_pair(ip4, 0));
					}

					if (flag_fragment == 1 && offset_fragment == 0) {
						start_pos_[ip4] = (unsigned long long) fs_.tellg() - sizeof(pcap_pkthdr) - pcap_header.len;
						fragment_on = true;
						data[ip4].clear();
						for (int i = 0; i < udp_data.size(); i++) {
							data[ip4].push_back(udp_data[i]);
						}
					}
					else if ((flag_fragment == 0 || flag_fragment == 1) && offset_fragment != 0) {
						for (int i = 0; i < udp_header.size(); i++) {
							data[ip4].push_back(udp_header[i]);
						}
						for (int i = 0; i < udp_data.size(); i++) {
							data[ip4].push_back(udp_data[i]);
						}
					}
					else {

					}

					if (flag_fragment == 0) {
						if (data[ip4].size() != 0) {
							SOSLAB::Logger::PARSERTYPE res = parser_frame_index(data[ip4]);
							if (res == SOSLAB::Logger::PARSERTYPE::START) {
								if (check_error[ip4] > 10) {
									check_error[ip4] = 0;
									pcap_block_start_idx[ip4].push_back(start_pos_[ip4]);
								}

								fragment_on = false;
							}
							else if (data_block_start_idx.size() && res == SOSLAB::Logger::PARSERTYPE::SKIPERROR) {
								check_error[ip4] = check_error[ip4] + 1;
								fragment_on = false;
							}
							else {
								check_error[ip4] = check_error[ip4] + 1;
								fragment_on = false;
							}
						}
						else {
							fragment_on = false;
						}
					}
					else {

					}
				}
				else if (dont_fragment == 1) {
					//status???
				}
				else {
					//non
				}

			}
		}
	}
	prev_slot = -1;

	total_size = std::numeric_limits< uint64_t>::max();
	for (auto const& block : pcap_block_start_idx) {
		if (block.second.size()) {
			total_size = (total_size > block.second.size()) ? block.second.size() : total_size;
		}
	}

	total_size = total_size - 2;

	return true;
}

bool SOSLAB::Logger::read_once(int idx)
{
	std::vector<uint8_t> ether_header;
	std::vector<uint8_t> ip_header;
	std::vector<uint8_t> udp_header;
	std::vector<uint8_t> data;

	std::streampos offset;
	std::streampos end_pos;

	std::streampos fsize;

	int lidar_loop = 0;

	switch (extension_type)
	{
	case 0:
		fs_.seekg(0, std::fstream::end);
		fsize = fs_.tellg();
		fs_.seekg(0, std::fstream::beg);

		current_pos_ = 0;

		offset = data_block_start_idx[idx];
		end_pos = data_block_start_idx[idx + 1];

		fs_.seekg(offset, std::fstream::beg);
		if (is_read_) {
			while ((current_pos_ < end_pos)) {
				if (read_fifo.size() < read_fifo.max_size()) {
					fs_.read(reinterpret_cast<char*>(&log_header_), sizeof(log_header_t));
					std::vector<uint8_t> data(log_header_.data_size);
					fs_.read(reinterpret_cast<char*>(data.data()), log_header_.data_size);
					current_pos_ = fs_.tellg();
					read_fifo.push(data);
				}
				else {
					return false;
				}
			}
		}
		break;
	default:
		fs_.seekg(0, std::fstream::end);
		fsize = fs_.tellg();
		fs_.seekg(0, std::fstream::beg);

		current_pos_ = 0;
		read_fifo.clear();
		read_pcap_status = true;

		for (auto const& block : pcap_block_start_idx) {

			int target_ip = block.first;
			read_lidar_id = lidar_loop;

			offset = block.second[idx];
			end_pos = block.second[idx + 1];

			fs_.seekg(offset, std::fstream::beg);
			current_pos_ = fs_.tellg();
			pcap_pkthdr pcap_header;

			if (is_read_) {
				while ((current_pos_ < end_pos)) {
					if (read_fifo.size() < read_fifo.max_size()) {

						fs_.read(reinterpret_cast<char*>(&pcap_header), sizeof(pcap_pkthdr));
						std::vector<uint8_t> temp(pcap_header.len);
						fs_.read(reinterpret_cast<char*>(temp.data()), pcap_header.len);
						current_pos_ = fs_.tellg();
						int ether_header_size = sizeof(ETHER_HDR);
						int ip_header_size = sizeof(IPV4_HDR);
						int udp_header_size = sizeof(UDP_HDR);

						ether_header = std::vector<uint8_t>(temp.begin(), temp.begin() + ether_header_size);
						ip_header = std::vector<uint8_t>(temp.begin() + ether_header_size, temp.begin() + ether_header_size + ip_header_size);
						udp_header = std::vector<uint8_t>(temp.begin() + ether_header_size + ip_header_size, temp.begin() + +ether_header_size + ip_header_size + udp_header_size);
						std::vector<uint8_t> udp_data = std::vector<uint8_t>(temp.begin() + ether_header_size + ip_header_size + udp_header_size, temp.end());

						IPV4_HDR ip_header_struct;
						UDP_HDR udp_header_struct;
						memcpy(reinterpret_cast<char*>(&ip_header_struct), ip_header.data(), sizeof(IPV4_HDR));
						memcpy(reinterpret_cast<char*>(&udp_header_struct), udp_header.data(), sizeof(UDP_HDR));

						if (ip_header_struct.ip_protocol == 6) {
							//TCP
						}
						else if (ip_header_struct.ip_protocol == 17) {
							//UDP
							int iphdrlen = 0, data_size = 0;
							iphdrlen = ip_header_struct.ip_header_len * 4;

							int offset_fragment = (ip_header_struct.ip_frag_offset) * 2048 + (ip_header_struct.ip_frag_offset1) * 8;
							int flag_fragment = ip_header_struct.ip_more_fragment;
							int dont_fragment = ip_header_struct.ip_dont_fragment;
							int start_pos = 0;

							if (dont_fragment == 0) {
								int ip1, ip2, ip3, ip4;

								ip1 = int(ip_header_struct.ip_srcaddr & 0xFF);
								ip2 = int((ip_header_struct.ip_srcaddr >> 8) & 0xFF);
								ip3 = int((ip_header_struct.ip_srcaddr >> 16) & 0xFF);
								ip4 = int((ip_header_struct.ip_srcaddr >> 24) & 0xFF);

								if (ip4 == target_ip) {
									if (flag_fragment == 1 && offset_fragment == 0) {
										data.clear();
										for (int i = 0; i < udp_data.size(); i++) {
											data.push_back(udp_data[i]);
										}
									}
									else if ((flag_fragment == 0 || flag_fragment == 1) && offset_fragment != 0) {
										for (int i = 0; i < udp_header.size(); i++) {
											data.push_back(udp_header[i]);
										}
										for (int i = 0; i < udp_data.size(); i++) {
											data.push_back(udp_data[i]);
										}
									}
									else {

									}

									if (flag_fragment == 0) {
										if (lidar_loop == 0) {
											read_fifo.push(data);
										}
									}
									else {

									}
								}
							}
							else if (dont_fragment == 1) {
								
							}
							else {
								
							}

						}
						else {

						}
					}
					else {
						read_pcap_status = false;
						return false;
					}
				}
			}
			lidar_loop++;
		}
		read_pcap_status = false;
		break;
	}

	return true;
}

SOSLAB::Logger::PARSERTYPE SOSLAB::Logger::parser_frame_index(std::vector<uint8_t> buffer)
{
	uint8_t* data = buffer.data();
	unsigned int size = buffer.size();
	const unsigned int emb_size = 174 + 2;
	const unsigned int amb_size = 768;
	const unsigned int amb_size2 = 2304;
	const unsigned int sts_size = 130 + 6;

	ml_lidar_packet_t* lidar_pkt = (ml_lidar_packet_t*)data;

	SOSLAB::Logger::PARSERTYPE retval = SOSLAB::Logger::PARSERTYPE::MIDDLE;

	if (strncmp(lidar_pkt->header, "LIDARPKT", sizeof(lidar_pkt->header)) == 0) {
		int slot_num = int(lidar_pkt->row_number);

		if (slot_num == 0) {
			prev_slot = 0;
			retval = SOSLAB::Logger::PARSERTYPE::START;
		}
		else {
			if (prev_slot != -1) {
				int diff = slot_num - prev_slot;
				prev_slot = slot_num;
				if (diff == 1) retval = SOSLAB::Logger::PARSERTYPE::MIDDLE;
				else if (diff == 0) {
					retval = SOSLAB::Logger::PARSERTYPE::MIDDLE;
				}
				else retval = SOSLAB::Logger::PARSERTYPE::SKIPERROR;
			}
		}
	}
	else {
		retval = SOSLAB::Logger::PARSERTYPE::SKIPERROR;
	}

	return retval;
}

bool SOSLAB::Logger::get_recorder_type(MODE type_)
{
	recorder_type = type_;
	return true;
}
void Logger::thread_process()
{
	std::cout << "PCAP :: " <<  __FUNCTION__ << " :: Start." << std::endl;
	is_stop_ = false;
	if (extension_type == 0) {

		current_pos_ = 0;

		std::vector<uint8_t> data;
		while (!is_stop_) {
			if (out_fifo_.pop(data)) {
				fs_.write(reinterpret_cast<const char*>(data.data()), data.size());
			}
			else {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				std::this_thread::yield();
			}
		}
	}

	std::cout << "PCAP :: " << __FUNCTION__ << " :: Done." << std::endl;
}

void Logger::thread_callback(void* arg)
{
	static_cast<Logger*>(arg)->thread_process();
}
