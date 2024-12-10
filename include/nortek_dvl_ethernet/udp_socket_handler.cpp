/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/

#include <nortek_dvl_ethernet/udp_socket_handler.h>

UDPSocketHandler::UDPSocketHandler(const UdpParam& param) 
    : io_service_(), socket_(io_service_), param_(param)
{
    // Initialize the socket
    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::udp::v4(), param_.udp_rx);
    socket_.open(endpoint.protocol());
    socket_.bind(endpoint);

    // Start async receive
    recv_buffer_.resize(param_.buffer_size);
    StartAsyncReceive();

    // Start io_service in a separate thread
    io_thread_ = boost::thread([this]() { io_service_.run(); });
}

UDPSocketHandler::~UDPSocketHandler() 
{
    io_service_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
}

void UDPSocketHandler::StartAsyncReceive() 
{
    recv_buffer_.assign(recv_buffer_.size(), 0);

    socket_.async_receive_from(
        boost::asio::buffer(recv_buffer_), remote_endpoint_,
        [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
            if (!error) {

                //! TODO: need to check if we are receiving data from specific address ? 
                // if(remote_endpoint_.address().to_string() != param_.udp_address) {
                //     printf("UDP socket: Wrong remote address: %s\n", 
                //         remote_endpoint_.address().to_string().c_str());
                //     StartAsyncReceive();
                //     return;
                // }

#ifdef DEBUG
                std::ostringstream oss;
                for (size_t i = 0; i < bytes_transferred; ++i) {
                    oss << std::hex << std::setfill('0') << std::setw(2)
                        << static_cast<int>(static_cast<unsigned char>(recv_buffer_[i])) << " ";
                }
                std::cout<< "Received raw data (hex): " << oss.str() <<std::endl;
#endif
                // send to callback function
                if(callback_) {
                    callback_(recv_buffer_.data(), bytes_transferred);
                }    

                // Start next async receive
                StartAsyncReceive();

                return;
            }
        });
}