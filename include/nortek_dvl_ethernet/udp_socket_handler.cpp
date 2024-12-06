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
    : io_service_(), socket_(io_service_) 
{
    //! DEBUG: check the param
    std::cout<<"param.udp_rx: "<<param.udp_rx<<std::endl;
    std::cout<<"param.udp_tx: "<<param.udp_tx<<std::endl;
    std::cout<<"param.udp_address: "<<param.udp_address<<std::endl;
    std::cout<<"param.buffer_size: "<<param.buffer_size<<std::endl;
            
    // Initialize the socket
    boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::udp::v4(), param.udp_rx);
    socket_.open(endpoint.protocol());
    socket_.bind(endpoint);

    // Start async receive
    recv_buffer_.resize(param.buffer_size);
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
            }
        });
}

// void NortekDvlEthernet::Receive()
// {
//     //! TODO: need resize the buffer ?
//     recv_buffer_.assign(recv_buffer_.size(), 0);

//     socket_->async_receive_from(
//         boost::asio::buffer(recv_buffer_), remote_endpoint_,
//         boost::bind(&NortekDvlEthernet::HandleReceive, this, 
//                      boost::asio::placeholders::error,
//                      boost::asio::placeholders::bytes_transferred));

//     // std::cout<<"Debug: Receive \n";
//     // recv_buffer_.assign(recv_buffer_.size(), 0);
//     // socket_->async_receive(
//     //     boost::asio::buffer(recv_buffer_), 0,
//     //     boost::bind(&NortekDvlEthernet::HandleReceive, this, 
//     //                  boost::asio::placeholders::error,
//     //     boost::asio::placeholders::bytes_transferred));    
// }

// void NortekDvlEthernet::HandleReceive(
//     const boost::system::error_code& error, 
//     std::size_t bytes_transferred)
// {
//     std::cout<<"Debug: HandleReceive \n";

//     // handle the data
//     if (!error || error == boost::asio::error::message_size)
//     {
//         // Print the received message
//         std::cout << "UDP Received " << bytes_transferred << " bytes from "
//                     << remote_endpoint_.address().to_string() << ":"
//                     << remote_endpoint_.port() << std::endl;
//         std::cout << "Message: " << std::string(recv_buffer_.data(), bytes_transferred) << std::endl;

//         num_read_error_ = 0;

//         //! TODO: Store timestamp as soon as received

//         //! TODO: setup callback

//         Receive();

//         return;
//     }    

//     // handle the error
//     num_read_error_ ++;
//     std::cout<<" Read error on socket: " <<error << " " << error.message()<<"\n";
//     std::cout<<num_read_error_ <<" consecutive read errors on port " << socket_->local_endpoint().port()<<"\n";

//     if (num_read_error_ <= 10)
//     {
//         std::cout<<"Retrying read in 0.1s\n";
//         std::this_thread::sleep_for(std::chrono::milliseconds(100));
//         return;
//     }

//     std::cout<<"Too many read errors on port: " <<socket_->local_endpoint().port()<<"\n";
//     std::exit(EXIT_FAILURE);
// }       