/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include "soslab_typedef.h"
#include "fifo.hpp"

//PCAP Logger
#ifdef __linux__
#include<sys/socket.h>
#include<arpa/inet.h>
// #include<sys/ioctl.h>
#include<sys/time.h>
#include<net/ethernet.h>
#else
#define NOMINMAX
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include<WinSock2.h>
#define SIO_RCVALL _WSAIOW(IOC_VENDOR,1) //this removes the need of mstcpip.h
#endif

namespace SOSLAB
{
    /* Pcap Global Header */
    typedef struct pcap_file_header {
        uint32_t magic;
        uint16_t version_major;
        uint16_t version_minor;
        int thiszone;	/* gmt to local correction */
        uint32_t sigfigs;	/* accuracy of timestamps */
        uint32_t snaplen;	/* max length saved portion of each pkt */
        uint32_t linktype;	/* data link type (LINKTYPE_*) */
    }GLOBAL_HDR;

    /* Pcap Generic Header */
    typedef struct pcap_pkthdr {
        uint32_t tv_sec;	/* time stamp second */
        uint32_t tv_usec;	/* time stamp microseconds */
        uint32_t caplen;	/* length of portion present */
        uint32_t len;	/* length this packet (off wire) */
    }PCAP_HDR;

    /* Ethernet header */
    typedef struct ethernet_header {
        unsigned char dest[6];
        unsigned char source[6];
        unsigned short type;
    } ETHER_HDR;

    /* IPv4 header */
    typedef struct ip_hdr {
        unsigned char ip_header_len : 4; // 4-bit header length (in 32-bit words) normally=5 (Means 20 Bytes may be 24 also)
        unsigned char ip_version : 4; // 4-bit IPv4 version	
        unsigned char ip_tos; // IP type of service	
        unsigned short ip_total_length; // Total length	
        unsigned short ip_id; // Unique identifier	
        unsigned char ip_frag_offset : 5; // Fragment offset field	
        unsigned char ip_more_fragment : 1;
        unsigned char ip_dont_fragment : 1;
        unsigned char ip_reserved_zero : 1;
        unsigned char ip_frag_offset1; //fragment offset	
        unsigned char ip_ttl; // Time to live	
        unsigned char ip_protocol; // Protocol(TCP,UDP etc)	
        unsigned short ip_checksum; // IP checksum	
        unsigned int ip_srcaddr; // Source address	
        unsigned int ip_destaddr; // Source address
    } IPV4_HDR;

    /* UDP header*/
    typedef struct udp_header {
        unsigned short sport;          // Source port
        unsigned short dport;          // Destination port
        unsigned short len;            // Datagram length
        unsigned short crc;            // Checksum
    }UDP_HDR;

    class SOSLAB_EXPORTS Logger
    {
        enum class PARSERTYPE : uint8_t { START, MIDDLE, END, SKIPERROR };
        enum class MODE : uint8_t { RECORD, PLAY, SAVE };
        typedef
#if defined(_MSC_VER)
#pragma pack(push, 1) 
            struct
#else
            struct __attribute__((packed))
#endif
            LOG_HEADER_T
        {
            uint64_t timestamp_usec;
            uint64_t data_size;
        } log_header_t;
#if defined(_MSC_VER)
#pragma pack(pop)
#endif

    public:
        typedef struct LOG_PROPS_T
        {
            std::string filepath;
            uint8_t is_read;
        } log_props_t;

        Logger();
        virtual ~Logger();

        bool open(const log_props_t& props);
        void close();

        void write(const uint8_t* data, const uint64_t size);
        bool read(uint8_t* data, const std::size_t size);

        std::size_t current_pos() const;

        bool is_open() const;
        bool is_read();

        bool loadingprocess();

        SOSLAB::Fifo<std::vector<uint8_t>> read_fifo;

        bool read_pcap_status;

        bool get_total_size(uint64_t& total_size_) {
            total_size_ = total_size;
            return true;
        }
        bool read_once(int idx);
        bool get_recorder_type(MODE type_);
        int extension_type;
        uint8_t read_lidar_id;

        std::vector<int> lidar_id_list;

        //pcap logger
        //linux
#ifdef __linux__
        int sock_raw;
#else
        SOCKET sock_raw;
#endif // __linux__

    private:
        bool is_read_;

        MODE recorder_type;

        std::fstream fs_;
        std::streampos current_pos_;

        uint16_t slot_counter = 0;
        uint64_t total_size;

        int prev_slot = -1 ;

        std::vector<std::streampos> data_block_start_idx;
        std::map<int, std::vector<std::streampos>> pcap_block_start_idx;

        PARSERTYPE parser_frame_index(std::vector<uint8_t> buffer);

        SOSLAB::Fifo<std::vector<uint8_t> > out_fifo_;

        bool is_stop_;
        std::thread* logger_thread_;
        log_header_t log_header_;

        void thread_process();
        static void thread_callback(void* arg);
        bool pcap_loadingprocess();

#ifndef __linux__
        int gettimeofday(struct timeval* tp, struct timezone* tzp)
        {
            // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
            // This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
            // until 00:00:00 January 1, 1970 
            static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

            SYSTEMTIME  system_time;
            FILETIME    file_time;
            uint64_t    time;

            GetSystemTime(&system_time);
            SystemTimeToFileTime(&system_time, &file_time);
            time = ((uint64_t)file_time.dwLowDateTime);
            time += ((uint64_t)file_time.dwHighDateTime) << 32;

            tp->tv_sec = (long)((time - EPOCH) / 10000000L);
            tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
            return 0;
        }
#endif
    };


}   /* namespace SOSLAB */



#endif  /* LOGGER_H_ */
