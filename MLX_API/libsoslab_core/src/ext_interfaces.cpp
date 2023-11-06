/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#include "core/ext_interfaces.h"

using namespace SOSLAB;

class HandlerBase {
public:
    int is_receiving_;
    int timeout_ms_;

    asio::io_context io_context_;
    
    std::size_t sz_recv_buffer_;
    char* recvbuf_;
    asio::mutable_buffers_1 asio_buffer_;

    HandlerBase(const std::size_t sz_recv_buffer, const int timeout_ms)
        : is_receiving_(false), timeout_ms_(timeout_ms), sz_recv_buffer_(sz_recv_buffer),
        recvbuf_(new char[sz_recv_buffer_]), asio_buffer_(asio::buffer(recvbuf_, sz_recv_buffer_))
    {}

    virtual ~HandlerBase() {
        delete[] recvbuf_;
    }
};


ExtInterfaceBase::PortBase::PortBase() {}

ExtInterfaceBase::ExtInterfaceBase() 
: recv_thread_(nullptr) ,
  recv_callback_(nullptr), recv_callback_arg_(nullptr),
  timeout_callback_(nullptr), timeout_callback_arg_(nullptr)
{

}

ExtInterfaceBase::~ExtInterfaceBase() 
{
	stop_thread();
}

void ExtInterfaceBase::registRecvCallback(receiving_callback_t callback, void* callback_arg) 
{
	recv_callback_ = callback;
	recv_callback_arg_ = callback_arg;
}

void ExtInterfaceBase::registTimeoutCallback(timeout_callback_t callback, void* callback_arg)
{
	timeout_callback_ = callback;
	timeout_callback_arg_ = callback_arg;
}

void ExtInterfaceBase::received(char* data, std::size_t size) 
{
	if (nullptr != recv_callback_) {
		recv_callback_(recv_callback_arg_, data, size);

#ifdef _DBG_PRINT_RECEIVED_SIZE
		std::cout << __FUNCTION__ << " size(" << size << " bytes)" << std::endl;
#endif
	}
}

void ExtInterfaceBase::timeout(int timeout_ms) 
{
	if (nullptr != timeout_callback_) {
		timeout_callback_(timeout_callback_arg_, timeout_ms);
	}
}

void ExtInterfaceBase::start_thread() 
{
	if (nullptr == recv_thread_) {
		recv_thread_ = new std::thread([&]() { receive_thread_process(); });
	}
}

void ExtInterfaceBase::stop_thread() 
{
	if (nullptr != recv_thread_) {
		recv_thread_->join();

		delete recv_thread_;
		recv_thread_ = nullptr;
	}
}



/*
 * TCP
 */
class HandleTcpClient : public HandlerBase
{
public:
    asio::ip::tcp::socket socket_;
    asio::ip::tcp::endpoint ep_device_;

    HandleTcpClient()
        : HandlerBase(65536, 1000), socket_(this->io_context_)
    {}

    ~HandleTcpClient() {}
};


ExtInterface_TCP_client::IpSettings::IpSettings(std::string addr_local, int port_local, std::string addr_server, int port_server, int timeout_ms)
    : address_local_(addr_local), port_local_(port_local), address_server_(addr_server), port_server_(port_server), timeout_ms_(timeout_ms)
{

}

ExtInterface_TCP_client::IpSettings::~IpSettings()
{

}


ExtInterface_TCP_client::ExtInterface_TCP_client()
    :tcp_client_(new HandleTcpClient)
{

}

ExtInterface_TCP_client::~ExtInterface_TCP_client()
{
    delete static_cast<HandleTcpClient*>(tcp_client_);
}

void ExtInterface_TCP_client::run_with_timeout(std::chrono::steady_clock::duration timeout)
{
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    // Restart the io_context, as it may have been left in the "stopped" state
    // by a previous operation.
    tcp->io_context_.restart();

    // Block until the asynchronous operation has completed, or timed out. If
    // the pending asynchronous operation is a composed operation, the deadline
    // applies to the entire operation, rather than individual operations on
    // the socket.
    tcp->io_context_.run_for(timeout);

    // If the asynchronous operation completed successfully then the io_context
    // would have been stopped due to running out of work. If it was not
    // stopped, then the io_context::run_for call must have timed out.
    if (!tcp->io_context_.stopped())
    {
        // Close the socket to cancel the outstanding asynchronous operation.
        tcp->socket_.close();

        // Run the io_context again until the operation completes.
        tcp->io_context_.run();
    }
}

bool ExtInterface_TCP_client::connect(PortBase& _port_ip)
{
    bool retval = false;

    IpSettings& port_ip = static_cast<IpSettings&>(_port_ip);
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);
    const char* server_ip = port_ip.address_server_.c_str();
    std::string server_port_str = std::to_string(port_ip.port_server_);
    const char* server_port = server_port_str.c_str();

    try {
        
        if (tcp->socket_.is_open()) {
            tcp->socket_.close();
        }

        tcp->socket_.open(asio::ip::tcp::v4());

        if (tcp->socket_.is_open()) {
            tcp->socket_.set_option(asio::ip::tcp::socket::reuse_address(true));

            asio::ip::tcp::endpoint ep_local(asio::ip::address::from_string(port_ip.address_local_).to_v4(), port_ip.port_local_);
            tcp->socket_.bind(ep_local);

            // Resolve the host name and service to a list of endpoints.
            auto endpoints = asio::ip::tcp::resolver(tcp->io_context_).resolve(server_ip, server_port);

            // Start the asynchronous operation itself. The lambda that is used as a
            // callback will update the error variable when the operation completes.
            // The blocking_udp_client.cpp example shows how you can use std::bind
            // rather than a lambda.
            uint8_t success_flag;
            tcp->socket_.async_connect(endpoints->endpoint(), std::bind(&ExtInterface_TCP_client::handle_connect, this, std::placeholders::_1, &success_flag));
            // Run the operation until it completes, or until the timeout.
            run_with_timeout(std::chrono::seconds(3));

            // Determine whether a connection was successfully established.
            if (!success_flag) {
                retval = true;
                start_thread();

                std::cout << "LiDAR ML :: TCP :: " << tcp->socket_.local_endpoint() << std::endl;
                port_ip.address_local_ = tcp->socket_.local_endpoint().address().to_string();
                port_ip.port_local_ = tcp->socket_.local_endpoint().port();
            }
        }
        else {
            std::cout << "LiDAR ML :: Socket open error!" <<  std::endl;
        }
    }
    catch (std::exception & e) {
        std::cout << "LiDAR ML :: Receiving socket error :: " << e.what() << std::endl;
    }

    return retval;
}

void ExtInterface_TCP_client::handle_connect(const std::error_code& error, uint8_t* success_flag)
{
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    // The async_connect() function automatically opens the socket at the start
    // of the asynchronous operation. If the socket is closed at this time then
    // the timeout handler must have run first.
    if (!tcp->socket_.is_open()) {
        std::cout << "LiDAR ML :: Connect timed out.\n";
  
        *success_flag = 1;
    }
    // Check if the connect operation failed before the deadline expired.
    else if (error) {
        std::cout << "LiDAR ML :: Connect error :: " << error.message() << "\n";

        //// We need to close the socket used in the previous connection attempt
        //// before starting a new one.
        //tcp->socket_.close();

        *success_flag = 2;
    }
    // Otherwise we have successfully established a connection.
    else  {
        std::cout << "LiDAR ML :: Connected." << "\n";

        *success_flag = 0;
    }
}

void ExtInterface_TCP_client::disconnect()
{
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    try {
        tcp->is_receiving_ = false;
        //tcp->socket_.cancel();
        tcp->socket_.close();
        stop_thread();

        if (tcp->socket_.is_open()) {
            tcp->socket_.close();
        }
    }
    catch (const std::exception & e) {
        //std::cout << e.what() << std::endl;
        UNUSED(e);
    }
}

bool ExtInterface_TCP_client::write(char* data, std::size_t size)
{
    bool retval = false;

    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    try {
        std::error_code ec;
        asio::const_buffers_1 buffer(data, size);
        std::size_t bytes_sent = tcp->socket_.send(buffer);

        if (!ec && 0 < bytes_sent) {
            retval = true;
        }
        else {
            std::cout << "LiDAR ML :: Sending socket error :: " << ec.message() << std::endl;
        }
    }
    catch (std::exception & e) {
        std::cout << "LiDAR ML :: Sending socket error :: " << e.what() << std::endl;
    }

    return retval;
}

bool ExtInterface_TCP_client::is_running()
{
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    return tcp->is_receiving_;
}

bool ExtInterface_TCP_client::is_connected()
{
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    return tcp->socket_.is_open();
}

void ExtInterface_TCP_client::receive_thread_process()
{
    HandleTcpClient* tcp = static_cast<HandleTcpClient*>(tcp_client_);

    auto receive_handler = [&](const std::error_code& _ec, std::size_t _bytes_recvd) {
        if (!_ec && 0 < _bytes_recvd) {
            received(tcp->recvbuf_, _bytes_recvd);
        }
    };

    tcp->is_receiving_ = true;
    while (tcp->is_receiving_) {
        tcp->socket_.async_receive(tcp->asio_buffer_, receive_handler);
        tcp->io_context_.run();
        tcp->io_context_.reset();
    }

    std::cout << "LiDAR ML :: " << __FUNCTION__ << " :: Done." << std::endl;
}



/*
 * UDP 
 */
class HandleUdpClient : public HandlerBase
{
public:
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint ep_device_;

    HandleUdpClient()
        : HandlerBase(65536, 1000), socket_(this->io_context_)
    {}

	~HandleUdpClient() {}
};


ExtInterface_UDP_client::IpSettings::IpSettings(std::string addr_local, int port_local, std::string addr_server, int port_server, int timeout_ms)
: address_local_(addr_local), port_local_(port_local), address_server_(addr_server), port_server_(port_server), timeout_ms_(timeout_ms)
{

}

ExtInterface_UDP_client::IpSettings::~IpSettings()
{

}


ExtInterface_UDP_client::ExtInterface_UDP_client()
: udp_client_(new HandleUdpClient)
{

}

ExtInterface_UDP_client::~ExtInterface_UDP_client()
{
	delete static_cast<HandleUdpClient*>(udp_client_);
}

bool ExtInterface_UDP_client::connect(PortBase& _port_ip)
{
	bool retval = false;

	IpSettings& port_ip = static_cast<IpSettings&>(_port_ip);
	HandleUdpClient* udp = static_cast<HandleUdpClient*>(udp_client_);

    try {
        udp->ep_device_ = asio::ip::udp::endpoint(asio::ip::address::from_string(port_ip.address_server_).to_v4(), port_ip.port_server_);

        asio::ip::udp::endpoint ep_local(asio::ip::address::from_string(port_ip.address_local_).to_v4(), port_ip.port_local_);
        udp->socket_.open(asio::ip::udp::v4());
        udp->socket_.set_option(asio::ip::udp::socket::reuse_address(true));
        udp->socket_.bind(ep_local);

        if (udp->socket_.is_open()) {
            start_thread();
            retval = true;
        }
    }
    catch (std::exception& e) {
        std::cout << "LiDAR ML :: Receiving socket error :: " << e.what() << std::endl;
    }

	return retval;
}

void ExtInterface_UDP_client::disconnect()
{
	HandleUdpClient* udp = static_cast<HandleUdpClient*>(udp_client_);

    try {
        udp->is_receiving_ = false;
        //udp->socket_.cancel();
        udp->socket_.close();
        stop_thread();

        if (udp->socket_.is_open()) {
            udp->socket_.close();
        }
    }
    catch (const std::exception& e) {
        //std::cout << e.what() << std::endl;
        UNUSED(e);
    }
}

bool ExtInterface_UDP_client::write(char* data, std::size_t size)
{
	bool retval = false;

	HandleUdpClient* udp = static_cast<HandleUdpClient*>(udp_client_);

    try {
        std::error_code ec;
        asio::const_buffers_1 buffer(data, size);
        std::size_t bytes_sent = udp->socket_.send_to(buffer, udp->ep_device_, 0, ec);

        if (!ec && 0 < bytes_sent) {
            retval = true;
        }
        else {
            std::cout << "LiDAR ML :: Sending socket error :: " << ec.message() << std::endl;
        }
    }
    catch (std::exception& e) {
        std::cout << "LiDAR ML :: Sending socket error :: " << e.what() << std::endl;
    }

	return retval;
}

bool ExtInterface_UDP_client::is_running()
{
	HandleUdpClient *udp = static_cast<HandleUdpClient*>(udp_client_);

	return udp->is_receiving_;
}

bool ExtInterface_UDP_client::is_connected()
{
	HandleUdpClient *udp = static_cast<HandleUdpClient*>(udp_client_);

	return udp->socket_.is_open();
}

void ExtInterface_UDP_client::receive_thread_process()
{
	HandleUdpClient *udp = static_cast<HandleUdpClient*>(udp_client_);

    auto receive_handler = [&](const std::error_code& _ec, std::size_t _bytes_recvd) {
        if (!_ec && 0 < _bytes_recvd) {
            received(udp->recvbuf_, _bytes_recvd);
        }
    };

	udp->is_receiving_ = true;
	while (udp->is_receiving_) {
        udp->socket_.async_receive(udp->asio_buffer_, receive_handler);
        udp->io_context_.run();
        udp->io_context_.reset();
    }

	std::cout << "LiDAR ML :: " << __FUNCTION__ << " :: Done." << std::endl;
}


