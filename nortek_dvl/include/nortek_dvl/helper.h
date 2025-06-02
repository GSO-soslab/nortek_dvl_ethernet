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

#ifndef NORTEK_DVL_ETHERNET_HELPER_H_
#define NORTEK_DVL_ETHERNET_HELPER_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>
#include <rclcpp/rclcpp.hpp>
#include "builtin_interfaces/msg/time.hpp"

// double time to stamp (builtin_interfaces::msg::Time)
inline builtin_interfaces::msg::Time DoubleToRosStamp(double time_in_seconds) {
    builtin_interfaces::msg::Time stamp;

    stamp.sec = static_cast<int32_t>(time_in_seconds);
    stamp.nanosec = static_cast<uint32_t>((time_in_seconds - stamp.sec) * 1e9);

    return stamp;
}

// boost time to stamp (builtin_interfaces::msg::Time)
inline builtin_interfaces::msg::Time BoostToRosStamp(const boost::posix_time::ptime& pt) {
    // Define the UNIX epoch as a boost::ptime
    static const boost::posix_time::ptime epoch(
        boost::gregorian::date(1970, 1, 1));

    // Calculate time duration from epoch
    boost::posix_time::time_duration diff = pt - epoch;

    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(diff.total_seconds());
    stamp.nanosec = static_cast<uint32_t>((diff.total_microseconds() % 1000000) * 1000);

    return stamp;
}

// returns duration in seconds as double from two stamp (builtin_interfaces::msg::Time)
inline double DurationFromStamp(
    const builtin_interfaces::msg::Time& start,
    const builtin_interfaces::msg::Time& end)
{
    rclcpp::Time t_start(start);
    rclcpp::Time t_end(end);

    rclcpp::Duration duration = t_end - t_start;
    return duration.seconds();  
}

#endif // NORTEK_DVL_ETHERNET_HELPER_H_