/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#ifndef EXT_INTERFACES_H_
#define EXT_INTERFACES_H_

#define ASIO_STANDALONE

#include "soslab_typedef.h"
#include "asio.hpp"
#include "logger.h"

namespace SOSLAB {
    /*
     *  Base asynchronous interface class for the external hardware interfaces.
     */
    class SOSLAB_EXPORTS ExtInterfaceBase 
    {
    public:
        typedef void(*receiving_callback_t)(void *arg, char *data, std::size_t size);
        typedef void(*timeout_callback_t)(void *arg, int timeout_ms);

        class SOSLAB_EXPORTS PortBase 
        {
        public:
            PortBase();
        };

        ExtInterfaceBase();
        virtual ~ExtInterfaceBase();

        void registRecvCallback(receiving_callback_t callback, void *callback_arg);
        void registTimeoutCallback(timeout_callback_t callback, void *callback_arg);

        void received(char *data, std::size_t size);
        void timeout(int timeout_ms);

        void start_thread();
        void stop_thread();

        virtual bool connect(PortBase &port) = 0;
        virtual void disconnect() = 0;
        virtual bool write(char *data, std::size_t size) = 0;

    private:
        virtual void receive_thread_process() = 0;
        std::thread *recv_thread_;

        receiving_callback_t recv_callback_;
        void *recv_callback_arg_;

        timeout_callback_t timeout_callback_;
        void *timeout_callback_arg_;
    };



    /*
     *  TCP Client
     */
    class SOSLAB_EXPORTS ExtInterface_TCP_client : public ExtInterfaceBase
    {
    public:
        class SOSLAB_EXPORTS IpSettings : public PortBase
        {
        public:
            IpSettings(std::string addr_local, int port_local, std::string addr_server, int port_server, int timeout_ms);
            ~IpSettings();

            std::string address_local_;
            int port_local_;

            std::string address_server_;
            int port_server_;

            int timeout_ms_;
        };

        ExtInterface_TCP_client();
        virtual ~ExtInterface_TCP_client();

        virtual bool connect(PortBase& _port_ip);
        virtual void disconnect();
        virtual bool write(char* data, std::size_t size);

        bool is_running();
        bool is_connected();

    private:
        virtual void receive_thread_process();
        virtual void run_with_timeout(std::chrono::steady_clock::duration timeout);
        virtual void handle_connect(const std::error_code& error, uint8_t* success_flag);

        void* tcp_client_;
    };



    /*
     *  UDP Client
     */
    class SOSLAB_EXPORTS ExtInterface_UDP_client : public ExtInterfaceBase 
    {
    public:
        class SOSLAB_EXPORTS IpSettings : public PortBase 
        {
        public:
            IpSettings(std::string addr_local, int port_local, std::string addr_server, int port_server, int timeout_ms);
            ~IpSettings();

            std::string address_local_;
            int port_local_;

            std::string address_server_;
            int port_server_;

            int timeout_ms_;
        };

        ExtInterface_UDP_client();
        virtual ~ExtInterface_UDP_client();

        virtual bool connect(PortBase &_port_ip);
        virtual void disconnect();
        virtual bool write(char *data, std::size_t size);

        bool is_running();
		bool is_connected();

    private:
        virtual void receive_thread_process();

        void *udp_client_;
    };

}   /* namespace SOSLAB */


#endif  /* EXT_INTERFACES_H_ */
