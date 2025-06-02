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

#include <nortek_dvl/parser.h>

bool NortekDVLParser::Checksum(
    uint16_t length, 
    const uint8_t* buffer)
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

nortek_dvl_structs::parserID NortekDVLParser::Parse(
    const uint8_t* data, 
    std::size_t size, 
    const double io_time)
{
    uint8_t head_size;
    auto id = ParseHeader(data, size, head_size);    

    switch (id)
    {
        case nortek_dvl_structs::BT: 
        {
            auto bottom_track = nortek_msgs::msg::NortekDF2{};
            ParseTrack(data, head_size, io_time, &bottom_track);

            // send to callback function
            if(bottom_track_callback_) {
              bottom_track_callback_(bottom_track);
            }            

            break;
        }

        case nortek_dvl_structs::CP: 
        {
            auto current_profile = nortek_msgs::msg::NortekDF3{};
            ParseCurrentProfile(data, head_size, io_time, &current_profile);

            // send to callback function
            if(current_profile_callback_) {
              current_profile_callback_(current_profile);
            }       

            break;            
        }

        case nortek_dvl_structs::WT: 
        {
            auto water_track = nortek_msgs::msg::NortekDF2{};
            ParseTrack(data, head_size, io_time, &water_track);

            // send to callback function
            if(water_track_callback_) {
              water_track_callback_(water_track);
            }      

            break;
        }

        default: 
        {
            break;
        }

    }

    return id;
}

nortek_dvl_structs::parserID NortekDVLParser::ParseHeader(
    const uint8_t* buffer, 
    const size_t& buffer_size, 
    uint8_t& head_size) 
{
    if (buffer_size < 2) {
        // Check for zero or small size
        std::cout<<"BYTES DATA SIZE LESS THAN 2\n";
        return nortek_dvl_structs::ERROR;
    }

    uint32_t sync = (buffer[0]);
    if (sync == 0xa5) {
        // Match on HEADER
        auto *hdr = reinterpret_cast<const nortek_dvl_structs::header *> (buffer);

        head_size = hdr->header_size;
        size_t payload_len = hdr->data_size;

        // check header data size
        //// TODO: check header: header size=10, Checksum
        if (buffer_size < sizeof(hdr)) {
            std::cout<<"Error - Header is too short. Header is: " << hdr->header_size<<"\n";
            return nortek_dvl_structs::ERROR;
        }
        if (hdr->header_size != 10) {
            std::cout<<"Error - Header result failure: data number not right: " << hdr->header_size <<"\n";
            return nortek_dvl_structs::ERROR;               
        }

        // check Checksum
        //// TODO: check data: Checksum
        if (!Checksum(hdr->data_size, buffer)) {
            std::cout<<"Error - Header result failure: Checksum failed\n";
            return nortek_dvl_structs::ERROR;
        }
        
        // check data id
        if (hdr->headerid == 0x1b) {
            if (payload_len > buffer_size) {
                std::cout<< "Warning: BT Payload length longer that data received\n";
            }

            // printf("it's BT\n");
            return nortek_dvl_structs::BT;
        } 
        else if (hdr->headerid == 0x16) {
            if (payload_len > buffer_size) {
                std::cout<< "Warning: CP Payload length longer that data received\n";
            }
            
            // printf("it's CP\n");
            return nortek_dvl_structs::CP;
        }
        else if (hdr->headerid == 0x1d) {
            if (payload_len > buffer_size) {
                std::cout<< "Warning: WT Payload length longer that data received\n";
            }
            
            // printf("it's WT\n");
            return nortek_dvl_structs::WT;
        }        
        else {
            std::cout<< "Error - Header ID Not Recognized: " << hdr->headerid <<"\n";
            return nortek_dvl_structs::ERROR;
        }
    }

    std::cout<< "Error - Nortek DVL Sync ID not recognized: " << std::hex << sync <<"\n";
    return nortek_dvl_structs::ERROR;
}

void NortekDVLParser::ParseTrack(
    const uint8_t* buffer, 
    uint8_t& head_size, 
    const double io_time, 
    nortek_msgs::msg::NortekDF2* df2_msg)
{
    // get buffer
    auto payload = buffer + head_size;
    // parse into defined struct
    auto *track = reinterpret_cast<const nortek_dvl_structs::TrackData*>(payload);
    // convert struct into raw msg
    ToDF2(io_time, *track, df2_msg);
}

void NortekDVLParser::ToDF2(
    const double io_time, 
    const nortek_dvl_structs::TrackData& data, 
    nortek_msgs::msg::NortekDF2* df2_msg)
{
    /***** information data *****/

    df2_msg->dvl_type     = nortek_msgs::msg::NortekDF2::DVL_TYPE_PISTON;
    df2_msg->version      = data.version;
    df2_msg->data_offset  = data.data_offset;
    df2_msg->serial_num   = data.serial_num;
    df2_msg->year         = data.year;
    df2_msg->month        = data.month;
    df2_msg->day          = data.day;
    df2_msg->hour         = data.hour;
    df2_msg->minute       = data.minute;
    df2_msg->second       = data.seconds;
    df2_msg->micro_second = data.microseconds; // actually sent as 100-microsecond counts
    df2_msg->beam_num     = data.nbeams;
    
    // Basic Error Handling
    df2_msg->error = data.error;
    if (df2_msg->error != 0) 
        std::cout<< "DVL - Error: "<<df2_msg->error <<std::endl;

    // Basic Status Message Handler
    df2_msg->status.beam1_vel_valid  = data.status.beam1VelValid;
    df2_msg->status.beam2_vel_valid  = data.status.beam2VelValid;
    df2_msg->status.beam3_vel_valid  = data.status.beam3VelValid;
    df2_msg->status.beam4_vel_valid  = data.status.beam4VelValid;
    df2_msg->status.beam1_dist_valid = data.status.beam1DistValid;
    df2_msg->status.beam2_dist_valid = data.status.beam2DistValid;
    df2_msg->status.beam3_dist_valid = data.status.beam3DistValid;
    df2_msg->status.beam4_dist_valid = data.status.beam4DistValid;
    df2_msg->status.beam1_fom_valid  = data.status.beam1FOMValid;
    df2_msg->status.beam2_fom_valid  = data.status.beam2FOMValid;
    df2_msg->status.beam3_fom_valid  = data.status.beam3FOMValid;
    df2_msg->status.beam4_fom_valid  = data.status.beam4FOMValid;
    df2_msg->status.vel_x_valid      = data.status.xVelValid;
    df2_msg->status.vel_y_valid      = data.status.yVelValid;
    df2_msg->status.vel_z1_valid     = data.status.z1VelValid;
    df2_msg->status.vel_z2_valid     = data.status.z2VelValid;
    df2_msg->status.fom_x_valid      = data.status.xFOMValid;
    df2_msg->status.fom_y_valid      = data.status.yFOMValid;
    df2_msg->status.fom_z1_valid     = data.status.z1FOMValid;
    df2_msg->status.fom_z2_valid     = data.status.z2FOMValid;
    df2_msg->status.proc_idle3       = data.status.procIdle3;
    df2_msg->status.proc_idle6       = data.status.procIdle6;
    df2_msg->status.proc_idle12      = data.status.procIdle12;
    df2_msg->status.empty            = data.status.empty;
    df2_msg->status.wakeup_state     = data.status.wakeupstate;
    if (df2_msg->status.proc_idle12) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 12%\n";
    if (df2_msg->status.proc_idle6) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 6%\n";
    if (df2_msg->status.proc_idle3) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 3%\n";

    df2_msg->speed_sound = data.speed_sound;
    df2_msg->temperature = data.temperature;
    df2_msg->pressure    = data.pressure;

    /***** Beam Data *****/
    for (int i = 0; i < 4; i++)
    {
        df2_msg->beam_vel[i]  = data.velBeam[i];
        df2_msg->beam_dist[i] = data.distBeam[i];
        df2_msg->beam_fom[i]  = data.fomBeam[i];
        df2_msg->beam_dt1[i]  = data.timeDiff1Beam[i];
        df2_msg->beam_dt2[i]  = data.timeDiff2Beam[i];
        df2_msg->beam_dura[i] = data.timeVelEstBeam[i];
    }

    /***** XYZ Data *****/

    df2_msg->vel_x         = data.velX;
    df2_msg->vel_y         = data.velY;
    df2_msg->vel_z1        = data.velZ1;
    df2_msg->vel_z2        = data.velZ2;
    df2_msg->fom_x         = data.fomX;
    df2_msg->fom_y         = data.fomY;
    df2_msg->fom_z1        = data.fomZ1;
    df2_msg->fom_z2        = data.fomZ2;
    df2_msg->time_diff1_x   = data.timeDiff1X;
    df2_msg->time_diff1_y   = data.timeDiff1Y;
    df2_msg->time_diff1_z1  = data.timeDiff1Z1;
    df2_msg->time_diff1_z2  = data.timeDiff1Z2;
    df2_msg->time_diff2_x   = data.timeDiff2X;
    df2_msg->time_diff2_y   = data.timeDiff2Y;
    df2_msg->time_diff2_z1  = data.timeDiff2Z1;
    df2_msg->time_diff2_z2  = data.timeDiff2Z2;
    df2_msg->time_dura_x  = data.timeVelEstX;
    df2_msg->time_dura_y  = data.timeVelEstY;
    df2_msg->time_dura_z1 = data.timeVelEstZ1;
    df2_msg->time_dura_z2 = data.timeVelEstZ2;

    /***** Processed data *****/

    double altitude_sum = 0;
    for (int i = 0; i < 4; i++)
    {
        if (df2_msg->beam_dist[i] != 0 && 
            df2_msg->beam_vel[i] != -32.768f && 
            df2_msg->beam_fom[i] != 10){
            df2_msg->good_beams += 1;
            altitude_sum += df2_msg->beam_dist[i];
        }
    }
    df2_msg->altitude = altitude_sum / 4;

    // setup speed ?
    df2_msg->speed_gnd  = sqrt(df2_msg->vel_x * df2_msg->vel_x + df2_msg->vel_y * df2_msg->vel_y);
    df2_msg->course_gnd = atan2(df2_msg->vel_x, df2_msg->vel_y) * 180.0 / M_PI;

    // setup time
    int year = static_cast<int>(df2_msg->year) + 1900;
    int month = static_cast<int>(df2_msg->month) + 1;
    boost::posix_time::ptime dvltime(
        boost::gregorian::date(year, month, df2_msg->day),
        boost::posix_time::hours(df2_msg->hour) + boost::posix_time::minutes(df2_msg->minute) +
        boost::posix_time::seconds(df2_msg->second) +
        boost::posix_time::microseconds(static_cast<int>(df2_msg->micro_second)*100));

    // setup header
    df2_msg->io_time = DoubleToRosStamp(io_time);
    df2_msg->system_time = BoostToRosStamp(dvltime);
    df2_msg->header.stamp = df2_msg->system_time;
    
    // Determine the authoratative timestamp for this message
    double max_clock_offset = 0.5;
    double dt = DurationFromStamp(df2_msg->io_time, df2_msg->system_time);
    if (fabs(dt) > max_clock_offset) {
        df2_msg->header.stamp = df2_msg->io_time;

#ifdef DEBUG
        // If the timestamps are wildly different, use the IO time
        std::cout<<"DVL BT clock differs from CPU clock by " << dt
                        << " seconds (threshold: "
                        << max_clock_offset <<"); using I/O times\n";
#endif
    }
}

void NortekDVLParser::ParseCurrentProfile(
    const uint8_t* buffer, 
    uint8_t& head_size, 
    const double io_time, 
    nortek_msgs::msg::NortekDF3* df3_msg)
{
    // parse the profile data without cells
    auto payload_profile = buffer + head_size;
    auto *profile = reinterpret_cast<const nortek_dvl_structs::ProfileData*>(payload_profile);

    // parse the cells
    auto profile_size = sizeof(nortek_dvl_structs::ProfileData);
    auto payload_cells = buffer + head_size + profile_size;
    auto *cells = ParseCells(payload_cells, profile->beam_system.num_cells);
    // auto *cells = reinterpret_cast<const nortek_dvl_structs::ProfileCellsSimple*>(payload_cells);

    // convert parse struct into ros message 
    ToDF3(io_time, *profile, *cells, df3_msg);
}

nortek_dvl_structs::ProfileCells* NortekDVLParser::ParseCells(
    const uint8_t* buffer, 
    int size) 
{
    // Allocate memory for the struct
    auto* cells = new nortek_dvl_structs::ProfileCells;

    // Calculate initial offsets for each data type
    //! TODO: this is not used?
    size_t velDataSize = 4 * size * sizeof(int16_t);
    size_t ampDataSize = 4 * size * sizeof(uint8_t);
    size_t corDataSize = 4 * size * sizeof(uint8_t);

    // Pointer to the current position in the buffer
    const uint8_t* currentPtr = buffer;

    // Initialize pointers for velData
    for (int i = 0; i < 4; ++i) {
        cells->velData[i] = reinterpret_cast<int16_t*>(const_cast<uint8_t*>(currentPtr));
        currentPtr += size * sizeof(int16_t);
    }

    // Initialize pointers for ampData
    for (int i = 0; i < 4; ++i) {
        cells->ampData[i] = const_cast<uint8_t*>(currentPtr);
        currentPtr += size * sizeof(uint8_t);
    }

    // Initialize pointers for corData
    for (int i = 0; i < 4; ++i) {
        cells->corData[i] = const_cast<uint8_t*>(currentPtr);
        currentPtr += size * sizeof(uint8_t);
    }

    return cells;
}

void NortekDVLParser::ToDF3(
    const double io_time, 
    const nortek_dvl_structs::ProfileData& data, 
    const nortek_dvl_structs::ProfileCells& cells, 
    nortek_msgs::msg::NortekDF3* df3_msg)
{
    /***** Information Data*****/

    df3_msg->dvl_type                       = nortek_msgs::msg::NortekDF3::DVL_TYPE_PISTON;
    df3_msg->version                        = data.version;
    df3_msg->data_offset                    = data.data_offset;
    df3_msg->configuration.pressure_valid   = data.configuration.pressure;
    df3_msg->configuration.temp_valid       = data.configuration.temp;
    df3_msg->configuration.compass_valid    = data.configuration.compass;
    df3_msg->configuration.tilt_valid       = data.configuration.tilt;
    df3_msg->configuration.empty            = data.configuration.empty;
    df3_msg->configuration.vel_exist        = data.configuration.velIncluded;
    df3_msg->configuration.amp_exist        = data.configuration.ampIncluded;
    df3_msg->configuration.cor_exist        = data.configuration.corrIncluded;
    df3_msg->configuration.alt_exist        = data.configuration.altiIncluded;
    df3_msg->configuration.alt_raw_exist    = data.configuration.altiRawIncluded;
    df3_msg->configuration.ast_exist        = data.configuration.ASTIncluded;
    df3_msg->configuration.echo_exist       = data.configuration.echoIncluded;
    df3_msg->configuration.ahrs_exist       = data.configuration.ahrsIncluded;
    df3_msg->configuration.percentage_exist = data.configuration.PGoodIncluded;
    df3_msg->configuration.std_exist        = data.configuration.stdDevIncluded;
    df3_msg->configuration.unused           = data.configuration.unused;
    df3_msg->serial_number                  = data.serial_num;

#ifdef DEBUG
    if (df3_msg->configuration.pressure_valid) { printf("pressure vaild\n"); }
    else { printf("pressure not vaild\n"); }
    if (df3_msg->configuration.temp_valid) { printf("temp vaild\n"); }
    else { printf("temp not vaild\n"); }
    if (df3_msg->configuration.compass_valid) { printf("compass vaild\n"); }
    else { printf("compass not vaild\n"); }
    if (df3_msg->configuration.tilt_valid) { printf("tilt vaild\n"); }
    else { printf("tilt not vaild\n"); }
    if (df3_msg->configuration.vel_exist) { printf("vel Included\n"); }
    else { printf("vel not Included\n"); }
    if (df3_msg->configuration.amp_exist) { printf("amp Included\n"); }
    else { printf("amp not Included\n"); }
    if (df3_msg->configuration.cor_exist) { printf("corr Included\n"); }
    else { printf("corr not Included\n"); }
    if (df3_msg->configuration.alt_exist) { printf("alt Included\n"); }
    else { printf("alt not Included\n"); }
    if(df3_msg->configuration.alt_raw_exist) { printf("alt Raw Included\n"); }
    else { printf("alt Raw not Included\n"); }
    if(df3_msg->configuration.ast_exist) { printf("AST Included\n"); }
    else { printf("AST not Included\n"); }
    if(df3_msg->configuration.echo_exist) { printf("echo Included\n"); }
    else { printf("echo not Included\n"); }    
    if(df3_msg->configuration.ahrs_exist) { printf("ahrs Included\n"); }
    else { printf("ahrs not Included\n"); }        
    if(df3_msg->configuration.percentage_exist) { printf("percentage good Included\n"); }
    else { printf("percentage good not Incldued\n"); }      
    if(df3_msg->configuration.std_exist) { printf("std Dev Included\n"); }
    else { printf("std Dev not Included\n"); }      

    //! NOTE: example for our Nortek DVL 1000
    // pressure vaild
    // temp vaild
    // compass not vaild
    // tilt not vaild
    // velIncluded vaild
    // ampIncluded vaild
    // corrIncluded vaild
    // no altiIncluded
    // no altiRawIncluded
    // no ASTIncluded
    // no echoIncluded
    // no ahrsIncluded
    // no PGoodIncluded
    // no stdDevIncluded       
#endif

    /***** Sensor Data*****/

    // get DVL system time
    df3_msg->year          = data.year;
    df3_msg->month         = data.month;
    df3_msg->day           = data.day;
    df3_msg->hour          = data.hour;
    df3_msg->minute        = data.minute;
    df3_msg->second        = data.seconds;
    df3_msg->micro_second  = data.microseconds; // actually sent as 100-microsecond counts
    // convert to epoch
    boost::posix_time::ptime dvltime(
        boost::gregorian::date(static_cast<int>(df3_msg->year) + 1900, 
                               static_cast<int>(df3_msg->month) + 1, 
                               df3_msg->day),
        boost::posix_time::hours(df3_msg->hour) + 
        boost::posix_time::minutes(df3_msg->minute) +
        boost::posix_time::seconds(df3_msg->second) +
        boost::posix_time::microseconds(static_cast<int>(df3_msg->micro_second)*100));

    // different type of sensor data
    df3_msg->speed_sound = data.speed_sound*0.1;
    df3_msg->temperature = data.temperature*0.01;
    df3_msg->pressure    = data.pressure*0.0001; //Bar

    // our Nortek 1000 not inclued for this DVL
    df3_msg->heading = data.heading*0.01;
    df3_msg->pitch   = data.pitch*0.01;
    df3_msg->roll    = data.roll*0.01;

    // beam system
    df3_msg->beam_system.cell_num   = data.beam_system.num_cells;
    df3_msg->beam_system.coordinate = data.beam_system.coordinate;
    df3_msg->beam_system.beam_num   = data.beam_system.num_beams;
    df3_msg->cell_size              = data.cell_size*0.001;
    df3_msg->nominal_correlation    = static_cast<int>(data.nominalCorrelation);
    df3_msg->pressure_temperature   = (data.pressTemp/5.0-4)*0.2;
    df3_msg->battery                = data.battery*0.1;

    // not inclued for this DVL
    df3_msg->mag_3d[0] = data.mag3D[0];
    df3_msg->mag_3d[1] = data.mag3D[1];
    df3_msg->mag_3d[2] = data.mag3D[2];
    df3_msg->acc_3d[0] = data.acc3D[0];
    df3_msg->acc_3d[1] = data.acc3D[1];
    df3_msg->acc_3d[2] = data.acc3D[2];

    // Data Set Description
    df3_msg->dataset_description.beam_data1 = data.dataSetDescription.beamData1;
    df3_msg->dataset_description.beam_data2 = data.dataSetDescription.beamData2;
    df3_msg->dataset_description.beam_data3 = data.dataSetDescription.beamData3;
    df3_msg->dataset_description.beam_data4 = data.dataSetDescription.beamData4;

    // different type of sensor data
    df3_msg->transmit_energy = data.transmitEnergy;
    df3_msg->power_level     = static_cast<float>(data.powerlevel);
    df3_msg->mag_temperature = data.magnTemperature;
    df3_msg->rtc_temperature = data.rtcTemperature*0.01;

    //! TODO: if Echo Sounder is inclued, this become Number of Echo Sounder Cells ?
    df3_msg->velocity_scale     = static_cast<int>(data.velocityScaling);
    double scale_factor         = pow(10, df3_msg->velocity_scale);
    df3_msg->ambiguity_velocity = data.ambVelocity * scale_factor;

    //Basic Error Handling
    df3_msg->error = data.error;
    if (df3_msg->error != 0) 
        std::cout<< "DVL CP Error Message is: "<<df3_msg->error<<std::endl;
    // status0
    df3_msg->status0.proc_idle3      = data.status0.procIdle3;
    df3_msg->status0.proc_idle6      = data.status0.procIdle6;
    df3_msg->status0.proc_idle12     = data.status0.procIdle12;
    df3_msg->status0.empty          = data.status0.empty;
    df3_msg->status0.use_status0     = data.status0.stat0inUse;
    // status
    df3_msg->status.unused1         = data.status.unused1;
    df3_msg->status.bd_scaling       = data.status.bdScaling;
    df3_msg->status.unused2         = data.status.unused2;
    df3_msg->status.unused3         = data.status.unused3;
    df3_msg->status.unused4         = data.status.unused4;
    df3_msg->status.echo_freq   = data.status.echoFrequency;
    df3_msg->status.boost_running        = data.status.boostRun;
    df3_msg->status.telemetry       = data.status.telemetry;
    df3_msg->status.echo_index       = data.status.echoIndex;
    df3_msg->status.active_config    = data.status.activeConfig;
    df3_msg->status.low_volt_skip     = data.status.lowVoltSkip;
    df3_msg->status.prev_wakeup_state = data.status.prevWakeupState;
    df3_msg->status.auto_orien      = data.status.autoOrient;
    df3_msg->status.orien          = data.status.orient;
    df3_msg->status.wakeup_state     = data.status.wakeupState;
    // handle blanking scaling
    if(df3_msg->status.bd_scaling)
        df3_msg->blanking           = data.blanking*0.01;
    else
        df3_msg->blanking           = data.blanking*0.001;
    //// TODO: use this as count for current profile msg?
    df3_msg->ensemble_counter       = data.ensembleCounter;

    /***** Header Data *****/

    df3_msg->io_time = DoubleToRosStamp(io_time);
    df3_msg->system_time = BoostToRosStamp(dvltime);
    df3_msg->header.stamp = df3_msg->system_time;
    
    // Determine the authoratative timestamp for this message
    double max_clock_offset = 0.5;
    double dt = DurationFromStamp(df3_msg->io_time, df3_msg->system_time);
    if (fabs(dt) > max_clock_offset) {
        df3_msg->header.stamp = df3_msg->io_time;

#ifdef DEBUG
        // If the timestamps are wildly different, use the IO time
        std::cout<<"DVL CP clock differs from CPU clock by " << dt 
                        << " seconds (threshold: "
                        << max_clock_offset <<"); using I/O times\n";
#endif
    }

    /***** Cell Data *****/

    for(int i=0; i< (int)df3_msg->beam_system.cell_num; i++) {
        // prepare the cell
        nortek_msgs::msg::NortekProfileCell cell;
        // time
        cell.header_time = df3_msg->header.stamp.sec;
        cell.dvl_time    = df3_msg->system_time.sec;
        // cell id
        cell.num = i+1;
        cell.pos = df3_msg->blanking + (i+1) * df3_msg->cell_size;
        // velocity data
        if(df3_msg->configuration.vel_exist) {
            cell.vel_x  = cells.velData[0][i] * scale_factor;
            cell.vel_y  = cells.velData[1][i] * scale_factor;
            cell.vel_z1 = cells.velData[2][i] * scale_factor;
            cell.vel_z2 = cells.velData[3][i] * scale_factor;
        }
        // amplitude data
        if(df3_msg->configuration.amp_exist) {
            cell.amp1  = cells.ampData[0][i];
            cell.amp2  = cells.ampData[1][i];
            cell.amp3  = cells.ampData[2][i];
            cell.amp4  = cells.ampData[3][i];
        }
        // correlation data
        if(df3_msg->configuration.cor_exist) {
            cell.cor1  = cells.corData[0][i];
            cell.cor2  = cells.corData[1][i];
            cell.cor3  = cells.corData[2][i];
            cell.cor4  = cells.corData[3][i];                
        }

        df3_msg->cells.push_back(cell);
    }
}   
