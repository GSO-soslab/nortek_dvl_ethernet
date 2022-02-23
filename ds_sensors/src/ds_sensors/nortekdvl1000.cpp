/**
* Copyright 2018 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ds_sensors/nortekdvl1000.h"
#include "nortekdvl1000_private.h"
#include <sstream>

using namespace boost::posix_time;
using namespace boost::gregorian;

/// ds_sensors/src/ds_sensors/nortekdvl1000.cpp
/// =============================================
/// DATE ------ WHO -------- WHAT ---------------
/// 7-23-19    I Vandor    Created and written

namespace ds_sensors
{
    NortekDvl::NortekDvl() : ds_base::SensorBase(), d_ptr_(std::unique_ptr<NortekDvlPrivate>(new NortekDvlPrivate))
    {
    }

    NortekDvl::NortekDvl(int argc, char* argv[], const std::string& name)
            : ds_base::SensorBase(argc, argv, name), d_ptr_(std::unique_ptr<NortekDvlPrivate>(new NortekDvlPrivate))
    {
    }

    NortekDvl::~NortekDvl() = default;

    double NortekDvl::seconds_from_epoch(ptime const& t) {
        boost::posix_time::ptime const EPOCH(date(1970, 1, 1));
        boost::posix_time::time_duration delta(t-EPOCH);
        return (delta.total_microseconds() / 1000000.0);
    }

    void NortekDvl::msg_to_dvl(ds_sensor_msgs::Dvl* dvl_data,
                               geometry_msgs::TwistStamped* velocity_data,
                               geometry_msgs::PointStamped* depth_data, 
                               double beam_angle, ds_sensor_msgs::NortekDF21* big_msg)
    {
        /******************** Header ********************/
        dvl_data->header = big_msg->header; 
        dvl_data->ds_header = big_msg->ds_header; 
        dvl_data->dvl_time = big_msg->dvl_time;

        /******************** Setting ********************/
        // type
        dvl_data->dvl_type = ds_sensor_msgs::Dvl::DVL_TYPE_PISTON;
        // velocity mode, Bottom tracking or Water tracking 
        if(big_msg->version == 1)
            dvl_data->velocity_mode = dvl_data->DVL_MODE_BOTTOM;  
        else
            dvl_data->velocity_mode = dvl_data->DVL_MODE_WATER;  
        // coordinate, velocity in XYZ on DVL frame
        dvl_data->coordinate_mode = dvl_data->DVL_COORD_INSTRUMENT;

        /******************** Data ********************/
        // fill in velocity and quality (FOM - measurement white noise level)
        dvl_data->velocity.x = big_msg->velX;
        dvl_data->velocity.y = big_msg->velY;
        dvl_data->quality.x = big_msg->fomX;
        dvl_data->quality.y = big_msg->fomY;

        if (big_msg->velZ1 == -32.768f && big_msg->velZ2 != -32.768f) {
            dvl_data->velocity.z = big_msg->velZ2;
            dvl_data->quality.z = big_msg->fomZ2;
        }
        else if (big_msg->velZ1 != -32.768f && big_msg->velZ2 == -32.768f){
            dvl_data->velocity.z = big_msg->velZ1;
            dvl_data->quality.z = big_msg->fomZ1;
        }
        else {
            dvl_data->velocity.z = (big_msg->velZ1 + big_msg->velZ2)/ 2.0;
            dvl_data->quality.z = (big_msg->fomZ1 + big_msg->fomZ2) / 2.0;
        }

        // range: convert vertical distance to range that from beam to target
        dvl_data->range[0] = big_msg->distBeam[0]  / cos(beam_angle);
        dvl_data->range[1] = big_msg->distBeam[1]  / cos(beam_angle);
        dvl_data->range[2] = big_msg->distBeam[2]  / cos(beam_angle);
        dvl_data->range[3] = big_msg->distBeam[3]  / cos(beam_angle);

        // average distance from DVL to the surface, maybe seafloor or lower ice surface 
        dvl_data->avg_altitude = big_msg->altitude_sum / big_msg->good_beams;
        
        // the vehicle speed
        dvl_data->speed_gnd = big_msg->speed_gnd;
        dvl_data->course_gnd = big_msg->course_gnd;

        /******************** Property ********************/
        // good beams
        dvl_data->num_good_beams = big_msg->good_beams;

        // will used for sound speed correction
        dvl_data->speed_sound = big_msg->speed_sound;
        dvl_data->temperature = big_msg->temperature;
    
        /******************** Velocity data ********************/
        velocity_data->header = dvl_data->header;
        velocity_data->twist.linear.x = dvl_data->velocity.x;
        velocity_data->twist.linear.y = dvl_data->velocity.y;
        velocity_data->twist.linear.z = dvl_data->velocity.z;
        
        /******************** Depth data ********************/
        depth_data->header = dvl_data->header;
        depth_data->point.x = 0.0;
        depth_data->point.y = 0.0;
        depth_data->point.z = big_msg->pressure;
    }

    void NortekDvl::msg_to_pointcloud(sensor_msgs::PointCloud2& pc2, 
                                      double beam_angle, ds_sensor_msgs::NortekDF21* big_msg) {

        // setup the pointcloud for 4 points with only XYZ property
        sensor_msgs::PointCloud2Modifier modifier(pc2);
        modifier.setPointCloud2FieldsByString(1, "xyz");    
        modifier.resize(4);   

        // reproject 3D points that the Beam hit on the surface using beam angle and azimuth angle
        // 4 beams location get azimuth angles:
        //        Y 
        //    (4) | (1)
        //   _____|_____ X     
        //        |
        //    (3) | (2)
        double beam_azimuth[] = {M_PI/4.0, -M_PI/4.0, -3.0*M_PI/4.0, 3.0*M_PI/4.0};

        std::vector<Eigen::Vector3d> points;
        for (int i = 0; i < 4; i++) {
            Eigen::Vector3d pt;
            pt(0) = big_msg->distBeam[i]  * tan(beam_angle) * cos(beam_azimuth[i]);
            pt(1) = big_msg->distBeam[i]  * tan(beam_angle) * sin(beam_azimuth[i]);
            pt(2) = big_msg->distBeam[i] ;
            points.push_back(pt);
            
        }

        // setup the points XYZ
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(pc2, "x");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(pc2, "y");
        sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(pc2, "z");

        for (size_t i = 0; i < 4; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {

            const Eigen::Vector3d& point = points.at(i);
            *ros_pc2_x = point(0);
            *ros_pc2_y = point(1);
            *ros_pc2_z = point(2);
        }

        // setup header
        pc2.header = big_msg->header;

        // setup vaild: if not all beams good good, set invaid ?
        if(big_msg->good_beams < 4)
            pc2.is_dense = false;
        else
            pc2.is_dense = true;

        //// TODO: use Beam FOM to filter bad points ?
        //// TODO: is there any intensity from DVL points ?
    }

    void NortekDvl::msg_to_depth(geometry_msgs::PointStamped* depth_data,
                                 ds_sensor_msgs::NortekDF3* big_msg) {
        /******************** Depth data ********************/
        depth_data->header = big_msg->header;
        depth_data->point.x = 0.0;
        depth_data->point.y = 0.0;
        depth_data->point.z = big_msg->pressure;
    }

///*-----------------------------------------------------------------------------*///
///*   DATA CONVERSION PARSERS: generate DF21 from memory parsers                 *///
///*-----------------------------------------------------------------------------*///
    void NortekDvl::bt_to_msg(const ros::Time& io_t, double max_clock_offset, const nortekdvl_structs::bottomtrack& bt, ds_sensor_msgs::NortekDF21* big_msg)
    {
        // type 
        big_msg->dvl_type = ds_sensor_msgs::NortekDF21::DVL_TYPE_PISTON;

        for (int i = 0; i < 4; i++)
        {
            /***** Processed data *****/

            if (bt.distBeam[i] != 0 && bt.velBeam[i] != -32.768f && bt.fomBeam[i] != 10)
                big_msg->good_beams += 1;
            big_msg->altitude_sum += big_msg->distBeam[i];

            /***** Beam Data *****/

            big_msg->velBeam[i] = bt.velBeam[i];
            big_msg->distBeam[i] = bt.distBeam[i];
            big_msg->fomBeam[i] = bt.fomBeam[i];
            big_msg->timeDiff1Beam[i] = bt.timeDiff1Beam[i];
            big_msg->timeDiff2Beam[i] = bt.timeDiff2Beam[i];
            big_msg->timeVelEstBeam[i] = bt.timeVelEstBeam[i];
        }

        /***** information data *****/

        big_msg->version = bt.version;
        big_msg->offsetOfData = bt.data_offset;
        big_msg->serialNumber = bt.serial_num;
        big_msg->year = bt.year;
        //ROS_WARN_STREAM("Not-Adjusted year is: " << static_cast<int>(big_msg->year));
        big_msg->month = bt.month;
        //ROS_WARN_STREAM("Month is: " << static_cast<int>(big_msg->month));
        big_msg->day = bt.day;
        big_msg->hour = bt.hour;
        big_msg->minute = bt.minute;
        big_msg->seconds = bt.seconds;
        big_msg->microSeconds = bt.microseconds; // actually sent as 100-microsecond counts
        big_msg->nBeams = bt.nbeams;
        
        //Basic Error Handling
        big_msg->error = bt.error;
        if (big_msg->error != 0) 
            ROS_WARN_STREAM("Error Message is: "<<big_msg->error);

        //Basic Status Message Handler
        big_msg->status.beam1VelValid  = bt.status.beam1VelValid;
        big_msg->status.beam2VelValid  = bt.status.beam2VelValid;
        big_msg->status.beam3VelValid  = bt.status.beam3VelValid;
        big_msg->status.beam4VelValid  = bt.status.beam4VelValid;
        big_msg->status.beam1DistValid = bt.status.beam1DistValid;
        big_msg->status.beam2DistValid = bt.status.beam2DistValid;
        big_msg->status.beam3DistValid = bt.status.beam3DistValid;
        big_msg->status.beam4DistValid = bt.status.beam4DistValid;
        big_msg->status.beam1FOMValid  = bt.status.beam1FOMValid;
        big_msg->status.beam2FOMValid  = bt.status.beam2FOMValid;
        big_msg->status.beam3FOMValid  = bt.status.beam3FOMValid;
        big_msg->status.beam4FOMValid  = bt.status.beam4FOMValid;
        big_msg->status.xVelValid      = bt.status.xVelValid;
        big_msg->status.yVelValid      = bt.status.yVelValid;
        big_msg->status.z1VelValid     = bt.status.z1VelValid;
        big_msg->status.z2VelValid     = bt.status.z2VelValid;
        big_msg->status.xFOMValid      = bt.status.xFOMValid;
        big_msg->status.yFOMValid      = bt.status.yFOMValid;
        big_msg->status.z1FOMValid     = bt.status.z1FOMValid;
        big_msg->status.z2FOMValid     = bt.status.z2FOMValid;
        big_msg->status.procIdle3      = bt.status.procIdle3;
        big_msg->status.procIdle6      = bt.status.procIdle6;
        big_msg->status.procIdle12     = bt.status.procIdle12;
        big_msg->status.empty          = bt.status.empty;
        big_msg->status.wakeupstate    = bt.status.wakeupstate;
        if (big_msg->status.procIdle12) 
            ROS_WARN_STREAM("DVL Processing Capacity Left Less Then 12%");
        if (big_msg->status.procIdle6) 
            ROS_WARN_STREAM("DVL Processing Capacity Left Less Then 6%");
        if (big_msg->status.procIdle3) 
            ROS_WARN_STREAM("DVL Processing Capacity Left Less Then 3%");

        big_msg->speed_sound = bt.speed_sound;
        big_msg->temperature = bt.temperature;
        big_msg->pressure = bt.pressure;

        /***** XYZ Data *****/

        big_msg->velX = bt.velX;
        big_msg->velY = bt.velY;
        big_msg->velZ1 = bt.velZ1;
        big_msg->velZ2 = bt.velZ2;
        big_msg->fomX = bt.fomX;
        big_msg->fomY = bt.fomY;
        big_msg->fomZ1 = bt.fomZ1;
        big_msg->fomZ2 = bt.fomZ2;
        big_msg->timeDiff1X = bt.timeDiff1X;
        big_msg->timeDiff1Y = bt.timeDiff1Y;
        big_msg->timeDiff1Z1 = bt.timeDiff1Z1;
        big_msg->timeDiff1Z2 = bt.timeDiff1Z2;
        big_msg->timeDiff2X = bt.timeDiff2X;
        big_msg->timeDiff2Y = bt.timeDiff2Y;
        big_msg->timeDiff2Z1 = bt.timeDiff2Z1;
        big_msg->timeDiff2Z2 = bt.timeDiff2Z2;
        big_msg->timeVelEstX = bt.timeVelEstX;
        big_msg->timeVelEstY = bt.timeVelEstY;
        big_msg->timeVelEstZ1 = bt.timeVelEstZ1;
        big_msg->timeVelEstZ2 =  bt.timeVelEstZ2;

        /***** Processed data *****/

        // setup time
        int year = static_cast<int>(big_msg->year) + 1900;
        int month = static_cast<int>(big_msg->month) + 1;
        boost::posix_time::ptime dvltime(boost::gregorian::date(year, month, big_msg->day),
                                         boost::posix_time::hours(big_msg->hour) + boost::posix_time::minutes(big_msg->minute) +
                                         boost::posix_time::seconds(big_msg->seconds) +
                                         boost::posix_time::microseconds(static_cast<int>(big_msg->microSeconds)*100));
        ros::Time dvl_rostime = ros::Time::fromBoost(dvltime);

        big_msg->dvl_time = static_cast<double>(dvl_rostime.sec) + static_cast<double>(dvl_rostime.nsec)*1.0e-9;
        big_msg->speed_gnd = sqrt(big_msg->velX * big_msg->velX + big_msg->velY * big_msg->velY);
        big_msg->course_gnd = atan2(big_msg->velX, big_msg->velY) * 180.0 / M_PI;

        // setup header
        big_msg->header.stamp = dvl_rostime;
        big_msg->ds_header.io_time = io_t; // also fill in the IO time, to handle possible timestamp drift

        // Determine the authoratative timestamp for this message
        ros::Duration dt = io_t - dvl_rostime;
        if (fabs(dt.toSec()) > max_clock_offset) {
          // If the timestamps are wildly different, slam it back into place using I/O time
            // ROS_WARN_STREAM("DVL BT clock differs from CPU clock by " << dt.toSec() 
            //                 << " seconds (threshold: "
            //                 << max_clock_offset <<"); using I/O times");
            big_msg->header.stamp = big_msg->ds_header.io_time;
        }

    }

///*-----------------------------------------------------------------------------*///
///*   CHECKSUM                                                                  *///
///*-----------------------------------------------------------------------------*///
    bool NortekDvl::checksum(uint16_t length, const uint8_t* buffer)
//    Buffer up until checksum is length bytes long.
    {
        uint16_t chksum = 0xB58C;
        uint16_t nbshorts = (length >> 1);
        for (int i = 0; i < nbshorts; i++)
        {
            chksum += *buffer;
            length -= 2;
            buffer++;
        }
        if (length > 0) {
            chksum += ((uint16_t)(*buffer)) << 8;
        }
        return chksum;
    }

    void NortekDvl::setupPublishers()
    {
        SensorBase::setupPublishers();
        DS_D(NortekDvl);
        auto nh = nodeHandle();
        
        // Bottom track
        d->df21_pub_ = nh.advertise<ds_sensor_msgs::NortekDF21>(ros::this_node::getName() + "/df21", 10);
        d->dvl_pub_ = nh.advertise<ds_sensor_msgs::Dvl>(ros::this_node::getName() + "/dvl", 10);
        d->velocity_pub_ = nh.advertise<geometry_msgs::TwistStamped>(ros::this_node::getName() + "/velocity", 10);
        d->depth_pub_ = nh.advertise<geometry_msgs::PointStamped>(ros::this_node::getName() + "/depth", 10);
        d->cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(ros::this_node::getName() + "/pointcloud", 10);

        // current profile
        d->df3_pub_ = nh.advertise<ds_sensor_msgs::NortekDF3>(ros::this_node::getName() + "/df3", 5);

    }

    void NortekDvl::setupParameters()
    {
        SensorBase::setupParameters();

        DS_D(NortekDvl);

        d->beam_angle = ros::param::param<double>("~/beam_angle_deg", 25) * M_PI / 180.0;
        d->max_clock_offset = ros::param::param<double>("max_clock_offset", 0.5);
    }
///*-----------------------------------------------------------------------------*///
///*   TOP FUNCTIONS: receive incoming raw data, create messages, and publish    *///
///*-----------------------------------------------------------------------------*///
/// TOP LEVEL calls parsing functions. If successful, publishes and saves messages
    void NortekDvl::parseReceivedBytes(const ds_core_msgs::RawData& bytes)
    {
        DS_D(NortekDvl);

        uint8_t length;
        auto id = parseHeader(bytes, length);

        switch (id)
        {
        // BT
        case nortekdvl_structs::BT: {
            // parse bottom track
            auto df21 = ds_sensor_msgs::NortekDF21{};
            parseBottomTrack(bytes, length, d->max_clock_offset, &df21);

            // fill in sensor metadata, actually just add frame_id
            FILL_SENSOR_HDR(df21, df21.header.stamp, bytes.ds_header.io_time);

            // encode derived DVL message for navigation application
            auto dvl = ds_sensor_msgs::Dvl{};
            auto velocity = geometry_msgs::TwistStamped{};
            auto depth = geometry_msgs::PointStamped{};
            msg_to_dvl(&dvl, &velocity, &depth, d->beam_angle, &df21);  

            // encode derived pointcloud message for surface visualization or sensor fusion 
            auto cloud = sensor_msgs::PointCloud2{};
            msg_to_pointcloud(cloud, d->beam_angle, &df21);

            // publish messages
            d->df21_pub_.publish(df21);
            d->dvl_pub_.publish(dvl);
            d->velocity_pub_.publish(velocity);
            d->depth_pub_.publish(depth);
            d->cloud_pub_.publish(cloud);

            break;
        }

        case nortekdvl_structs::CP: {
            auto df3 = ds_sensor_msgs::NortekDF3{};

            parseCurrentProfile(bytes, length, d->max_clock_offset, &df3);

            FILL_SENSOR_HDR(df3, df3.header.stamp, bytes.ds_header.io_time);

            auto depth = geometry_msgs::PointStamped{};
            msg_to_depth(&depth, &df3);  

            d->df3_pub_.publish(df3);
            d->depth_pub_.publish(depth);
            //// TODO: 1) frame_id; 2) send derived messages?

            break;
        }
        
        default:

            ROS_ERROR_STREAM("CANNOT PARSE MSGS");

            break;
        }
    }  

    ds_sensors::nortekdvl_structs::parserID 
    NortekDvl::parseHeader(const ds_core_msgs::RawData& bytes, uint8_t& length) {

        if (bytes.data.size() < 2) {
            // Check for zero or small size
            ROS_ERROR_STREAM("BYTES DATA SIZE LESS THAN 2");
            return nortekdvl_structs::ERROR;
        }

        size_t buf_len = bytes.data.size();
        const uint8_t* buffer = bytes.data.data();

        uint32_t sync = (buffer[0]);
        if (sync == 0xa5) {
            // Match on HEADER
            auto *hdr = reinterpret_cast<const ds_sensors::nortekdvl_structs::header *> (buffer);

            length = hdr->header_size;
            auto payload = buffer + length;
            size_t payload_len = hdr->data_size;

            // check header data size
            //// TODO: check header: header size=10, checksum
            if (buf_len < sizeof(hdr)) {
                ROS_ERROR_STREAM("Header is too short. Header is: " << hdr->header_size);
                return nortekdvl_structs::ERROR;
            }
            if (hdr->header_size != 10) {
                ROS_ERROR_STREAM("Header result failure: data number not right: " << hdr->header_size);
                return nortekdvl_structs::ERROR;               
            }

            // check checksum
            //// TODO: check data: checksum
            if (!NortekDvl::checksum(hdr->data_size, buffer)) {
                ROS_ERROR_STREAM("Header result failure: Checksum failed");
                return nortekdvl_structs::ERROR;
            }
            
            // check data id
            if (hdr->headerid == 0x1b) {
                if (payload_len > buf_len) {
                    ROS_ERROR_STREAM("BT Payload length longer that data received");
                }

                return nortekdvl_structs::BT;
            } 
            else if (hdr->headerid == 0x16) {
                if (payload_len > buf_len) {
                    ROS_ERROR_STREAM("CP Payload length longer that data received");
                }
                
                return nortekdvl_structs::CP;
            }
            else {
                ROS_ERROR_STREAM("Header ID Not Recognized: " << hdr->headerid);
                return nortekdvl_structs::ERROR;
            }
        }
        ROS_ERROR_STREAM("Nortek DVL Sync ID not recognized: " << sync);
        return nortekdvl_structs::ERROR;
    }

    void NortekDvl::parseBottomTrack(const ds_core_msgs::RawData& bytes, uint8_t length, 
                                     double max_clock_offset, ds_sensor_msgs::NortekDF21* big_msg) {

        // get buffer
        const uint8_t* buffer = bytes.data.data();
        auto payload = buffer + length;
        // parse into defined struct
        auto *bt = reinterpret_cast<const nortekdvl_structs::bottomtrack*>(payload);
        // convert parse struct into ros message 
        NortekDvl::bt_to_msg(bytes.ds_header.io_time, max_clock_offset, *bt, big_msg);

    }

    void NortekDvl::parseCurrentProfile(const ds_core_msgs::RawData& bytes, uint8_t length, double max_clock_offset, 
                                        ds_sensor_msgs::NortekDF3* big_msg) {
        // get buffer
        const uint8_t* buffer = bytes.data.data();
        auto payload = buffer + length;
        // parse into defined struct
        auto *cp = reinterpret_cast<const nortekdvl_structs::currentprofile*>(payload);
        // convert parse struct into ros message 
        NortekDvl::cp_to_msg(bytes.ds_header.io_time, max_clock_offset, *cp, big_msg);

    }

    void NortekDvl::cp_to_msg(const ros::Time& io_t, double max_clock_offset, 
                              const nortekdvl_structs::currentprofile& cp, ds_sensor_msgs::NortekDF3* big_msg) {
        /***** Information Data*****/

        big_msg->dvl_type                      = ds_sensor_msgs::NortekDF3::DVL_TYPE_PISTON;
        big_msg->version                       = cp.version;
        big_msg->data_offset                   = cp.data_offset;
        big_msg->configuration.pressure        = cp.configuration.pressure;
        big_msg->configuration.temp            = cp.configuration.temp;
        big_msg->configuration.compass         = cp.configuration.compass;
        big_msg->configuration.tilt            = cp.configuration.tilt;
        big_msg->configuration.empty           = cp.configuration.empty;
        big_msg->configuration.velIncluded     = cp.configuration.velIncluded;
        big_msg->configuration.ampIncluded     = cp.configuration.ampIncluded;
        big_msg->configuration.corrIncluded    = cp.configuration.corrIncluded;
        big_msg->configuration.altiIncluded    = cp.configuration.altiIncluded;
        big_msg->configuration.altiRawIncluded = cp.configuration.altiRawIncluded;
        big_msg->configuration.ASTIncluded     = cp.configuration.ASTIncluded;
        big_msg->configuration.echoIncluded    = cp.configuration.echoIncluded;
        big_msg->configuration.ahrsIncluded    = cp.configuration.ahrsIncluded;
        big_msg->configuration.PGoodIncluded   = cp.configuration.PGoodIncluded;
        big_msg->configuration.stdDevIncluded  = cp.configuration.stdDevIncluded;
        big_msg->configuration.unused          = cp.configuration.unused;
        big_msg->serial_number                 = cp.serial_num;

        /***** Sensor Data*****/

        // get DVL system time
        big_msg->year          = cp.year;
        big_msg->month         = cp.month;
        big_msg->day           = cp.day;
        big_msg->hour          = cp.hour;
        big_msg->minute        = cp.minute;
        big_msg->seconds       = cp.seconds;
        big_msg->micro_seconds = cp.microseconds; // actually sent as 100-microsecond counts
        // convert to epoch
        boost::posix_time::ptime dvltime(boost::gregorian::date(static_cast<int>(big_msg->year) + 1900, 
                                                                static_cast<int>(big_msg->month) + 1, 
                                                                big_msg->day),
                                         boost::posix_time::hours(big_msg->hour) + 
                                         boost::posix_time::minutes(big_msg->minute) +
                                         boost::posix_time::seconds(big_msg->seconds) +
                                         boost::posix_time::microseconds(static_cast<int>(big_msg->micro_seconds)*100));
        // convert to ROS time
        ros::Time dvl_rostime = ros::Time::fromBoost(dvltime);
        // different type of sensor data
        big_msg->speed_sound = cp.speed_sound*0.1;
        big_msg->temperature = cp.temperature*0.01;
        big_msg->pressure    = cp.pressure*0.0001; //Bar
        // not inclued for this DVL
        big_msg->heading = cp.heading*0.01;
        big_msg->pitch   = cp.pitch*0.01;
        big_msg->roll    = cp.roll*0.01;
        // beam system
        big_msg->beam_system.num_cells  = cp.beam_system.num_cells;
        big_msg->beam_system.coordinate = cp.beam_system.coordinate;
        big_msg->beam_system.num_beams  = cp.beam_system.num_beams;
        big_msg->cell_size              = cp.cell_size*0.001;
        big_msg->nominal_correlation    = static_cast<int>(cp.nominalCorrelation);
        big_msg->pressure_temperature   = (cp.pressTemp/5.0-4)*0.2;
        big_msg->battery                = cp.battery*0.1;
        // not inclued for this DVL
        big_msg->mag3D[0] = cp.mag3D[0];
        big_msg->mag3D[1] = cp.mag3D[1];
        big_msg->mag3D[2] = cp.mag3D[2];
        big_msg->acc3D[0] = cp.acc3D[0];
        big_msg->acc3D[1] = cp.acc3D[1];
        big_msg->acc3D[2] = cp.acc3D[2];
        // Data Set Description
        big_msg->dataset_description.beamData1 = cp.dataSetDescription.beamData1;
        big_msg->dataset_description.beamData2 = cp.dataSetDescription.beamData2;
        big_msg->dataset_description.beamData3 = cp.dataSetDescription.beamData3;
        big_msg->dataset_description.beamData4 = cp.dataSetDescription.beamData4;
        // different type of sensor data
        big_msg->transmit_energy = cp.transmitEnergy;
        big_msg->power_level     = static_cast<float>(cp.powerlevel);
        big_msg->mag_temperature = cp.magnTemperature;
        big_msg->rtc_temperature = cp.rtcTemperature*0.01;

        big_msg->velocity_scale  = static_cast<int>(cp.velocityScaling);
        double scale_factor      = pow(10, big_msg->velocity_scale);
        big_msg->ambVelocity     = cp.ambVelocity * scale_factor;

        //Basic Error Handling
        big_msg->error = cp.error;
        if (big_msg->error != 0) 
            ROS_WARN_STREAM("Error Message is: "<<big_msg->error);
        // status0
        big_msg->status0.procIdle3      = cp.status0.procIdle3;
        big_msg->status0.procIdle6      = cp.status0.procIdle6;
        big_msg->status0.procIdle12     = cp.status0.procIdle12;
        big_msg->status0.empty          = cp.status0.empty;
        big_msg->status0.stat0inUse     = cp.status0.stat0inUse;
        // status
        big_msg->status.unused1         = cp.status.unused1;
        big_msg->status.bdScaling       = cp.status.bdScaling;
        big_msg->status.unused2         = cp.status.unused2;
        big_msg->status.unused3         = cp.status.unused3;
        big_msg->status.unused4         = cp.status.unused4;
        big_msg->status.echoFrequency   = cp.status.echoFrequency;
        big_msg->status.boostRun        = cp.status.boostRun;
        big_msg->status.telemetry       = cp.status.telemetry;
        big_msg->status.echoIndex       = cp.status.echoIndex;
        big_msg->status.activeConfig    = cp.status.activeConfig;
        big_msg->status.lowVoltSkip     = cp.status.lowVoltSkip;
        big_msg->status.prevWakeupState = cp.status.prevWakeupState;
        big_msg->status.autoOrient      = cp.status.autoOrient;
        big_msg->status.orient          = cp.status.orient;
        big_msg->status.wakeupState     = cp.status.wakeupState;
        // handle blanking scaling
        if(big_msg->status.bdScaling)
            big_msg->blanking           = cp.blanking*0.01;
        else
            big_msg->blanking           = cp.blanking*0.001;
        //// TODO: use this as count for current profile msg?
        big_msg->ensemble_counter       = cp.ensembleCounter;

        /***** Header Data *****/

        big_msg->header.stamp = dvl_rostime;
        big_msg->ds_header.io_time = io_t; // also fill in the IO time, to handle possible timestamp drift
        big_msg->dvl_time = static_cast<double>(dvl_rostime.sec) + static_cast<double>(dvl_rostime.nsec)*1.0e-9;

        // Determine the authoratative timestamp for this message
        ros::Duration dt = io_t - dvl_rostime;
        if (fabs(dt.toSec()) > max_clock_offset) {
          // If the timestamps are wildly different, slam it back into place using I/O time
            // ROS_WARN_STREAM("DVL CP clock differs from CPU clock by " << dt.toSec() 
            //                 << " seconds (threshold: "
            //                 << max_clock_offset <<"); using I/O times");
            big_msg->header.stamp = big_msg->ds_header.io_time;
        }

        /***** Cell Data *****/

        if(big_msg->beam_system.num_beams != 4)
            ROS_WARN_STREAM("Wrong DVL beam number , it's: " << big_msg->beam_system.num_beams);
        if(big_msg->beam_system.num_cells != 20)
            ROS_WARN_STREAM("Wrong DVL cell number , it's: " << big_msg->beam_system.num_cells);


        for(int i=0; i< (int)big_msg->beam_system.num_cells; i++) {
            // prepare the cell
            ds_sensor_msgs::NortekCPCell cell;

            // time
            cell.header_time = big_msg->header.stamp.sec;
            cell.dvl_time    = dvl_rostime.sec;
            // cell id
            cell.num = i+1;
            cell.pos = big_msg->blanking + (i+1) * big_msg->cell_size;
            // velocity data
            if(big_msg->configuration.velIncluded) {
                cell.v_x  = cp.velData[0][i] * scale_factor;
                cell.v_y  = cp.velData[1][i] * scale_factor;
                cell.v_z  = cp.velData[2][i] * scale_factor;
                cell.v_z2 = cp.velData[3][i] * scale_factor;
            }
            // amplitude data
            if(big_msg->configuration.ampIncluded) {
                cell.amp1  = cp.ampData[0][i];
                cell.amp2  = cp.ampData[1][i];
                cell.amp3  = cp.ampData[2][i];
                cell.amp4  = cp.ampData[3][i];
            }
            // correlation data
            if(big_msg->configuration.corrIncluded) {
                cell.cor1  = cp.corData[0][i];
                cell.cor2  = cp.corData[1][i];
                cell.cor3  = cp.corData[2][i];
                cell.cor4  = cp.corData[3][i];                
            }

            big_msg->cells.push_back(cell);
        }


        // std::cout<<"transmitEnergy: " << cp.transmitEnergy
        //          <<", velocityScaling: "<< static_cast<int>(cp.velocityScaling)
        //          <<" powerlevel: "<< static_cast<float>(cp.powerlevel)
        //          <<", magnTemperature: "<<cp.magnTemperature
        //          <<", rtcTemperature: " <<cp.rtcTemperature*0.01
        //          <<"\n";

    }

} // ds_sensors
