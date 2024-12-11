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

/**
 * @brief This is the parser for Nortek DVL1000 Ethernet binary data
 *
 * It get the raw binary from the callback setup of UDP interface. 
 * Then it check the header (e.g., checksum...) and decode to specfic message 
 * such as bottom track, water track and current profile. After that, the data 
 * will be send to wrapper (e.g., ROS) for further usage. 
 * This is non-ROS class, beside use the ROS message struct to pass the data, user
 * can switch another defined struct to pass the data as well. 
 */
class NortekDVLParser
{
private:

    /**
     * @brief Sets up a callback to receive the bottom track data
     * @param[in] message  A ROS message struct nortek_dvl_ethernet::NortekDF2 
     * that contain all the data parsed from bottom track. 
     *
     * This function allows the user to define a callback that will be invoked 
     * whenever new bottom track data is received. The callback accepts a 
     * constant reference to a `NortekDF2` object, which contains the bottom 
     * track data from the Nortek DVL Ethernet device.
     */
    std::function <void(const nortek_dvl_ethernet::NortekDF2&)> 
        bottom_track_callback_;

    /**
     * @brief Sets up a callback to receive the water track data
     * @param[in] message  A ROS message struct nortek_dvl_ethernet::NortekDF2 
     * that contain all the data parsed from water track. 
     *
     * This function allows the user to define a callback that will be invoked 
     * whenever new water track data is received. The callback accepts a 
     * constant reference to a `NortekDF2` object, which contains the water 
     * track data from the Nortek DVL Ethernet device.
     */
    std::function <void(const nortek_dvl_ethernet::NortekDF2&)> 
        water_track_callback_;

    /**
     * @brief Sets up a callback to receive the current profile data
     * @param[in] message  A ROS message struct nortek_dvl_ethernet::NortekDF3 
     * that contain all the data parsed from current profile. 
     *
     * This function allows the user to define a callback that will be invoked 
     * whenever new current profile data is received. The callback accepts a 
     * constant reference to a `NortekDF3` object, which contains the current 
     * profile data from the Nortek DVL Ethernet device.
     */
    std::function <void(const nortek_dvl_ethernet::NortekDF3&)> 
        current_profile_callback_;

    /**
     * @brief Check the checksum of comming data
     * @param[in] length  the data size
     * @param[in] buffer  the coming data buffer
     * @return true if check is successful
     *
     * It's the C-code for Checksum calculations from section Checksum Definitions, 
     * Nortek DVL Integrators Guide. 
     */
    bool Checksum(
        uint16_t length, 
        const uint8_t* buffer);

    /**
     * @brief Parse the header of each DVL binary message
     * @param[in] buffer  the buffer of incoming data
     * @param[in] buffer_size  the buffer size of incoming data 
     * @param[out] head_size  the head size of incoming data
     * @return the header ID defined as nortek_dvl_structs::parserID
     *
     * It check the integrity of the whole data and return which type data it is
     */
    nortek_dvl_structs::parserID ParseHeader(
        const uint8_t* buffer, 
        const size_t& buffer_size, 
        uint8_t& head_size);

    /**
     * @brief Parse the DF21/Df22 data besides the header
     * @param[in] buffer  the buffer of incoming data
     * @param[in] head_size  the head size of incoming data
     * @param[in] io_time  the I/O timestamp, recorded as the parser received this data, 
     *                     it's not the time then the data arrived the computer but close.
     * @param[out] df2_msg  the ROS message data structure of DF21/DF22 data
     *
     * It can be used for bottom track or water track, since they share the 
     * same data structure beside the header ID
     */
    void ParseTrack(
        const uint8_t* buffer, 
        uint8_t& head_size, 
        const double io_time, 
        nortek_dvl_ethernet::NortekDF2* df2_msg);

    /**
     * @brief Parse the DF3 data besides the header
     * @param[in] buffer  the buffer of incoming data
     * @param[in] head_size  the head size of incoming data
     * @param[in] io_time  the I/O timestamp, recorded as the parser received this data, 
     *                     it's not the time then the data arrived the computer but close.
     * @param[out] df3_msg  the ROS message data structure of DF3 data
     *
     * It can be used for current profile
     */
    void ParseCurrentProfile(
        const uint8_t* buffer, 
        uint8_t& head_size, 
        const double io_time, 
        nortek_dvl_ethernet::NortekDF3* df3_msg);

    /**
     * @brief Parse the DF21/DF22 raw data structure to ROS message structure
     * @param[in] io_time  the I/O timestamp, recorded as the parser received this data, 
     *                     it's not the time then the data arrived the computer but close.
     * @param[in] data  the buffer data that aligned with defined structure 
     * @param[out] df2_msg  the ROS message data structure of DF21 data
     *
     * It convert the buffer aligned DF21/DF22 data to the ROS msg
     */
    void ToDF2(
        const double io_time, 
        const nortek_dvl_structs::TrackData& data, 
        nortek_dvl_ethernet::NortekDF2* df2_msg);    

    /**
     * @brief Parse the DF3 raw data structure to ROS message structure
     * @param[in] io_time  the I/O timestamp, recorded as the parser received this data, 
     *                     it's not the time then the data arrived the computer but close.
     * @param[in] data  the profile buffer data that aligned with defined structure 
     * @param[in] cell  the cell buffer data that aligned with defined structure, 
     *                  which is dynamic size 
     * @param[out] df3_msg  the ROS message data structure of DF3 data
     *
     * It convert the buffer aligned DF3 data to the ROS msg, use the dynamic size of cells
     */
    void ToDF3(
        const double io_time, 
        const nortek_dvl_structs::ProfileData& data, 
        const nortek_dvl_structs::ProfileCells& cells, 
        nortek_dvl_ethernet::NortekDF3* df3_msg);  

    /**
     * @brief Parse the DF3 raw data structure to ROS message structure
     * @param[in] io_time  the I/O timestamp, recorded as the parser received this data, 
     *                     it's not the time then the data arrived the computer but close.
     * @param[in] data  the profile buffer data that aligned with defined structure 
     * @param[in] cell  the cell buffer data that aligned with defined structure, 
     *                  which is fixed size 
     * @param[out] df3_msg  the ROS message data structure of DF3 data
     *
     * It convert the buffer aligned DF3 data to the ROS msg, use the fixed size of cells
     */
    void ToDF3(
        const double io_time, 
        const nortek_dvl_structs::ProfileData& data, 
        const nortek_dvl_structs::ProfileCellsSimple& cells, 
        nortek_dvl_ethernet::NortekDF3* df3_msg);  

    /**
     * @brief Parse dynamic size cell data 
     * @param[in] buffer  the buffer start with cells data, removed other previous data
     * @param[in] size  the dynamic cell size
     * @return a pointer of dynamic cell structure of cell data
     *
     */
    nortek_dvl_structs::ProfileCells* ParseCells(
        const uint8_t* buffer, 
        int size);

public:
    /**
     * @brief The default class constructor 
     *
     */
    NortekDVLParser() {}

    /**
     * @brief Parse each coming DVL binary data
     * @param[in] data  the incoming data buffer
     * @param[in] size  the incoming data buffer size
     * @param[in] io_time  the I/O timestamp, recorded as the parser received this data, 
     *                     it's not the time then the data arrived the computer but close.
     *
     * It will be called from other class, when the coming data is ready
     */
    nortek_dvl_structs::parserID Parse(
        const uint8_t* data, 
        std::size_t size,
        const double io_time);

    /**
     * @brief The registration function to setup the callback for bottom track
     * @param[in] bottom_track_callback_  the std::function to pass bottom track data
     *
     * It will be called from other class, when the coming data is ready, to pass the data 
     * to it's registered function
     */
    void SetCallbackBT(decltype(bottom_track_callback_) cb) { 
        bottom_track_callback_  = cb;
    }

    /**
     * @brief The registration function to setup the callback for water track
     * @param[in] water_track_callback_  the std::function to pass water track data
     *
     * It will be called from other class, when the coming data is ready, to pass the data 
     * to it's registered function
     */
    void SetCallbackWT(decltype(water_track_callback_) cb) { 
        water_track_callback_  = cb;
    }

    /**
     * @brief The registration function to setup the callback for current profile
     * @param[in] current_profile_callback_  the std::function to pass current profile data
     *
     * It will be called from other class, when the coming data is ready, to pass the data 
     * to it's registered function
     */
    void SetCallbackCP(decltype(current_profile_callback_) cb) { 
        current_profile_callback_  = cb;
    }
};

#endif // NORTEK_DVL_ETHERNET_PARSER_H_