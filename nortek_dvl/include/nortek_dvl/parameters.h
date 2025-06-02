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

#ifndef NORTEK_DVL_ETHERNET_PARAMETERS_H
#define NORTEK_DVL_ETHERNET_PARAMETERS_H

#include <string>

// ========================================================================= //
// UDP related
// ========================================================================= //

//! UDP parameters
struct UdpParam
{
    int udp_rx;
    int udp_tx;
    std::string udp_address;
    int buffer_size;
};

#endif // NORTEK_DVL_ETHERNET_PARAMETERS_H