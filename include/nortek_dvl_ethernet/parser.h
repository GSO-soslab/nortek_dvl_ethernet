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

#ifndef NORTEK_DVL_ETHERNET_PARSER_H_
#define NORTEK_DVL_ETHERNET_PARSER_H_

#include <iostream>
#include <functional>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <nortek_dvl_ethernet/nortekdvl1000_structs.h>
#include <nortek_dvl_ethernet/NortekDF2.h>
#include <nortek_dvl_ethernet/NortekDF3.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/date_time/gregorian/gregorian.hpp>

class NortekDVLParser
{
private:

    std::function <void(const nortek_dvl_ethernet::NortekDF2&)> bottom_track_callback_;

    std::function <void(const nortek_dvl_ethernet::NortekDF2&)> water_track_callback_;

    std::function <void(const nortek_dvl_ethernet::NortekDF3&)> current_profile_callback_;

    bool Checksum(uint16_t length, const uint8_t* buffer);

    nortek_dvl_structs::parserID ParseHeader(
        const uint8_t* buffer, 
        const size_t& buffer_size, 
        uint8_t& head_size);

    void ParseTrack(
        const uint8_t* buffer, 
        uint8_t& head_size, 
        const double io_time, 
        nortek_dvl_ethernet::NortekDF2* df2_msg);

    void ParseCurrentProfile(
        const uint8_t* buffer, 
        uint8_t& head_size, 
        const double io_time, 
        nortek_dvl_ethernet::NortekDF3* df3_msg);

    void ToDF2(
        const double io_time, 
        const nortek_dvl_structs::TrackData& data, 
        nortek_dvl_ethernet::NortekDF2* df2_msg);    

    void ToDF3(
        const double io_time, 
        const nortek_dvl_structs::ProfileData& data, 
        const nortek_dvl_structs::ProfileCells& cells, 
        nortek_dvl_ethernet::NortekDF3* df3_msg);  

    void ToDF3(
        const double io_time, 
        const nortek_dvl_structs::ProfileData& data, 
        const nortek_dvl_structs::ProfileCellsSimple& cells, 
        nortek_dvl_ethernet::NortekDF3* df3_msg);  

    nortek_dvl_structs::ProfileCells* ParseCells(
        const uint8_t* buffer, int size);


public:
    NortekDVLParser() {}

    nortek_dvl_structs::parserID Parse(
        const uint8_t* data, 
        std::size_t size,
        const double io_time);

    void SetCallbackBT(decltype(bottom_track_callback_) cb) { bottom_track_callback_  = cb;}

    void SetCallbackWT(decltype(water_track_callback_) cb) { water_track_callback_  = cb;}

    void SetCallbackCP(decltype(current_profile_callback_) cb) { current_profile_callback_  = cb;}
};

#endif // NORTEK_DVL_ETHERNET_PARSER_H_