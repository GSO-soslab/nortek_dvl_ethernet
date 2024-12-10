/*
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

#ifndef NORTEK_DVL_ETHERNET_UDP_SOCKET_HANDLER_H_
#define NORTEK_DVL_ETHERNET_UDP_SOCKET_HANDLER_H_

// c++
#include <vector>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <functional>

// customized
#include <nortek_dvl_ethernet/parameters.h>

class UDPSocketHandler {
private:
    void StartAsyncReceive();

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::thread io_thread_;
    std::vector<uint8_t> recv_buffer_;

    UdpParam param_;
    
    std::function <void(const uint8_t*, std::size_t)> callback_;

public:
    // Update DataCallback to take a uint8_t (char) array and its size
    using DataCallback = std::function<void(const uint8_t*, std::size_t)>;    

    UDPSocketHandler(const UdpParam& param);

    ~UDPSocketHandler();

    void SetCallback(decltype(callback_) cb) { callback_  = cb;}
};

#endif // NORTEK_DVL_ETHERNET_UDP_SOCKET_HANDLER_H_
