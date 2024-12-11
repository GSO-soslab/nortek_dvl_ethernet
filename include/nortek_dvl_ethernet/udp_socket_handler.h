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

    Authors: Lin Zhao <linzhao@uri.edu>
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

/**
 * @brief This is the UDP handler class for data receiving using socket
 *
 * It read all the data and pass other classes using callback function.
 */
class UDPSocketHandler {
private:
    //! boost io_service for data interface
    boost::asio::io_service io_service_;

    //! boost socket
    boost::asio::ip::udp::socket socket_;

    //! boost endpoint (remote information?)
    boost::asio::ip::udp::endpoint remote_endpoint_;

    //! boost thread for async data reading
    boost::thread io_thread_;

    //! received buffer from UDP socket
    std::vector<uint8_t> recv_buffer_;

    //! UDP parameters
    UdpParam param_;
    
    /**
     * @brief Sets up a callback to receive the raw binary UDP socket data
     * @param[in] data uint8_t type coming data
     * @param[in] size size_t type coming data size
     *
     * This function allows the user to define a callback that will be invoked 
     * whenever new socket data is received. The callback accepts raw binary 
     * data and data size.
     */    
    std::function <void(const uint8_t*, std::size_t)> callback_;

    /**
     * @brief Receive data from UDP socket
     */   
    void StartAsyncReceive();

public:
    /**
     * @brief Default constructor
     * @param[in] param the UDP related parameters
     */  
    UDPSocketHandler(const UdpParam& param);

    /**
     * @brief Default deconstructor
     */ 
    ~UDPSocketHandler();

    /**
     * @brief The registration function to setup the callback for UDP socket data
     * @param[in] callback_  the std::function to pass binary data and buffer size
     */
    void SetCallback(decltype(callback_) cb) { callback_  = cb;}
};

#endif // NORTEK_DVL_ETHERNET_UDP_SOCKET_HANDLER_H_
