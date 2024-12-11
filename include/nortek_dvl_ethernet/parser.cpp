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

#include <nortek_dvl_ethernet/parser.h>

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
            auto bottom_track = nortek_dvl_ethernet::NortekDF2{};
            ParseTrack(data, head_size, io_time, &bottom_track);

            // send to callback function
            if(bottom_track_callback_) {
              bottom_track_callback_(bottom_track);
            }            

            break;
        }

        case nortek_dvl_structs::CP: 
        {
            auto current_profile = nortek_dvl_ethernet::NortekDF3{};
            ParseCurrentProfile(data, head_size, io_time, &current_profile);

            // send to callback function
            if(current_profile_callback_) {
              current_profile_callback_(current_profile);
            }       

            break;            
        }

        case nortek_dvl_structs::WT: 
        {
            auto water_track = nortek_dvl_ethernet::NortekDF2{};
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

    std::cout<< " Error - Nortek DVL Sync ID not recognized: " << sync <<"\n";
    return nortek_dvl_structs::ERROR;
}

void NortekDVLParser::ParseTrack(
    const uint8_t* buffer, 
    uint8_t& head_size, 
    const double io_time, 
    nortek_dvl_ethernet::NortekDF2* df2_msg)
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
    nortek_dvl_ethernet::NortekDF2* df2_msg)
{
    /***** information data *****/

    df2_msg->dvl_type     = nortek_dvl_ethernet::NortekDF2::DVL_TYPE_PISTON;
    df2_msg->version      = data.version;
    df2_msg->offsetOfData = data.data_offset;
    df2_msg->serialNumber = data.serial_num;
    df2_msg->year         = data.year;
    df2_msg->month        = data.month;
    df2_msg->day          = data.day;
    df2_msg->hour         = data.hour;
    df2_msg->minute       = data.minute;
    df2_msg->seconds      = data.seconds;
    df2_msg->microSeconds = data.microseconds; // actually sent as 100-microsecond counts
    df2_msg->nBeams       = data.nbeams;
    
    // Basic Error Handling
    df2_msg->error = data.error;
    if (df2_msg->error != 0) 
        std::cout<< "DVL - Error: "<<df2_msg->error <<std::endl;

    // Basic Status Message Handler
    df2_msg->status.beam1VelValid  = data.status.beam1VelValid;
    df2_msg->status.beam2VelValid  = data.status.beam2VelValid;
    df2_msg->status.beam3VelValid  = data.status.beam3VelValid;
    df2_msg->status.beam4VelValid  = data.status.beam4VelValid;
    df2_msg->status.beam1DistValid = data.status.beam1DistValid;
    df2_msg->status.beam2DistValid = data.status.beam2DistValid;
    df2_msg->status.beam3DistValid = data.status.beam3DistValid;
    df2_msg->status.beam4DistValid = data.status.beam4DistValid;
    df2_msg->status.beam1FOMValid  = data.status.beam1FOMValid;
    df2_msg->status.beam2FOMValid  = data.status.beam2FOMValid;
    df2_msg->status.beam3FOMValid  = data.status.beam3FOMValid;
    df2_msg->status.beam4FOMValid  = data.status.beam4FOMValid;
    df2_msg->status.xVelValid      = data.status.xVelValid;
    df2_msg->status.yVelValid      = data.status.yVelValid;
    df2_msg->status.z1VelValid     = data.status.z1VelValid;
    df2_msg->status.z2VelValid     = data.status.z2VelValid;
    df2_msg->status.xFOMValid      = data.status.xFOMValid;
    df2_msg->status.yFOMValid      = data.status.yFOMValid;
    df2_msg->status.z1FOMValid     = data.status.z1FOMValid;
    df2_msg->status.z2FOMValid     = data.status.z2FOMValid;
    df2_msg->status.procIdle3      = data.status.procIdle3;
    df2_msg->status.procIdle6      = data.status.procIdle6;
    df2_msg->status.procIdle12     = data.status.procIdle12;
    df2_msg->status.empty          = data.status.empty;
    df2_msg->status.wakeupstate    = data.status.wakeupstate;
    if (df2_msg->status.procIdle12) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 12%\n";
    if (df2_msg->status.procIdle6) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 6%\n";
    if (df2_msg->status.procIdle3) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 3%\n";

    df2_msg->speed_sound = data.speed_sound;
    df2_msg->temperature = data.temperature;
    df2_msg->pressure    = data.pressure;

    /***** Beam Data *****/
    for (int i = 0; i < 4; i++)
    {
        df2_msg->velBeam[i]        = data.velBeam[i];
        df2_msg->distBeam[i]       = data.distBeam[i];
        df2_msg->fomBeam[i]        = data.fomBeam[i];
        df2_msg->timeDiff1Beam[i]  = data.timeDiff1Beam[i];
        df2_msg->timeDiff2Beam[i]  = data.timeDiff2Beam[i];
        df2_msg->timeVelEstBeam[i] = data.timeVelEstBeam[i];
    }

    /***** XYZ Data *****/

    df2_msg->velX         = data.velX;
    df2_msg->velY         = data.velY;
    df2_msg->velZ1        = data.velZ1;
    df2_msg->velZ2        = data.velZ2;
    df2_msg->fomX         = data.fomX;
    df2_msg->fomY         = data.fomY;
    df2_msg->fomZ1        = data.fomZ1;
    df2_msg->fomZ2        = data.fomZ2;
    df2_msg->timeDiff1X   = data.timeDiff1X;
    df2_msg->timeDiff1Y   = data.timeDiff1Y;
    df2_msg->timeDiff1Z1  = data.timeDiff1Z1;
    df2_msg->timeDiff1Z2  = data.timeDiff1Z2;
    df2_msg->timeDiff2X   = data.timeDiff2X;
    df2_msg->timeDiff2Y   = data.timeDiff2Y;
    df2_msg->timeDiff2Z1  = data.timeDiff2Z1;
    df2_msg->timeDiff2Z2  = data.timeDiff2Z2;
    df2_msg->timeVelEstX  = data.timeVelEstX;
    df2_msg->timeVelEstY  = data.timeVelEstY;
    df2_msg->timeVelEstZ1 = data.timeVelEstZ1;
    df2_msg->timeVelEstZ2 = data.timeVelEstZ2;

    /***** Processed data *****/

    double altitude_sum = 0;
    for (int i = 0; i < 4; i++)
    {
        if (data.distBeam[i] != 0 && 
            data.velBeam[i] != -32.768f && 
            data.fomBeam[i] != 10){
            df2_msg->good_beams += 1;
            altitude_sum += df2_msg->distBeam[i];
        }
    }
    df2_msg->altitude = altitude_sum / 4;

    // setup speed ?
    df2_msg->speed_gnd  = sqrt(df2_msg->velX * df2_msg->velX + df2_msg->velY * df2_msg->velY);
    df2_msg->course_gnd = atan2(df2_msg->velX, df2_msg->velY) * 180.0 / M_PI;

    // setup time
    int year = static_cast<int>(df2_msg->year) + 1900;
    int month = static_cast<int>(df2_msg->month) + 1;
    boost::posix_time::ptime dvltime(
        boost::gregorian::date(year, month, df2_msg->day),
        boost::posix_time::hours(df2_msg->hour) + boost::posix_time::minutes(df2_msg->minute) +
        boost::posix_time::seconds(df2_msg->seconds) +
        boost::posix_time::microseconds(static_cast<int>(df2_msg->microSeconds)*100));

    // setup header
    df2_msg->io_time = ros::Time().fromSec(io_time);
    df2_msg->system_time = ros::Time::fromBoost(dvltime);
    df2_msg->header.stamp = df2_msg->system_time;
    double max_clock_offset = 0.5;

    // Determine the authoratative timestamp for this message
    ros::Duration dt = df2_msg->io_time - df2_msg->system_time ;
    if (fabs(dt.toSec()) > max_clock_offset) {
        df2_msg->header.stamp = df2_msg->io_time;

#ifdef DEBUG
        // If the timestamps are wildly different, use the IO time
        std::cout<<"DVL BT clock differs from CPU clock by " << dt.toSec() 
                        << " seconds (threshold: "
                        << max_clock_offset <<"); using I/O times\n";
#endif
    }
}

void NortekDVLParser::ParseCurrentProfile(
    const uint8_t* buffer, 
    uint8_t& head_size, 
    const double io_time, 
    nortek_dvl_ethernet::NortekDF3* df3_msg)
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
    nortek_dvl_ethernet::NortekDF3* df3_msg)
{
    /***** Information Data*****/

    df3_msg->dvl_type                      = nortek_dvl_ethernet::NortekDF3::DVL_TYPE_PISTON;
    df3_msg->version                       = data.version;
    df3_msg->data_offset                   = data.data_offset;
    df3_msg->configuration.pressure        = data.configuration.pressure;
    df3_msg->configuration.temp            = data.configuration.temp;
    df3_msg->configuration.compass         = data.configuration.compass;
    df3_msg->configuration.tilt            = data.configuration.tilt;
    df3_msg->configuration.empty           = data.configuration.empty;
    df3_msg->configuration.velIncluded     = data.configuration.velIncluded;
    df3_msg->configuration.ampIncluded     = data.configuration.ampIncluded;
    df3_msg->configuration.corrIncluded    = data.configuration.corrIncluded;
    df3_msg->configuration.altiIncluded    = data.configuration.altiIncluded;
    df3_msg->configuration.altiRawIncluded = data.configuration.altiRawIncluded;
    df3_msg->configuration.ASTIncluded     = data.configuration.ASTIncluded;
    df3_msg->configuration.echoIncluded    = data.configuration.echoIncluded;
    df3_msg->configuration.ahrsIncluded    = data.configuration.ahrsIncluded;
    df3_msg->configuration.PGoodIncluded   = data.configuration.PGoodIncluded;
    df3_msg->configuration.stdDevIncluded  = data.configuration.stdDevIncluded;
    df3_msg->configuration.unused          = data.configuration.unused;
    df3_msg->serial_number                 = data.serial_num;

#ifdef DEBUG
    if (df3_msg->configuration.pressure) { printf("pressure vaild\n"); }
    else { printf("pressure not vaild\n"); }
    if (df3_msg->configuration.temp) { printf("temp vaild\n"); }
    else { printf("temp not vaild\n"); }
    if (df3_msg->configuration.compass) { printf("compass vaild\n"); }
    else { printf("compass not vaild\n"); }
    if (df3_msg->configuration.tilt) { printf("tilt vaild\n"); }
    else { printf("tilt not vaild\n"); }
    if (df3_msg->configuration.velIncluded) { printf("velIncluded vaild\n"); }
    else { printf("velIncluded not vaild\n"); }
    if (df3_msg->configuration.ampIncluded) { printf("ampIncluded vaild\n"); }
    else { printf("ampIncluded not vaild\n"); }
    if (df3_msg->configuration.corrIncluded) { printf("corrIncluded vaild\n"); }
    else { printf("corrIncluded not vaild\n"); }
    if (df3_msg->configuration.altiIncluded) { printf("altiIncluded\n"); }
    else { printf("no altiIncluded\n"); }
    if(df3_msg->configuration.altiRawIncluded) { printf("altiRawIncluded\n"); }
    else { printf("no altiRawIncluded\n"); }
    if(df3_msg->configuration.ASTIncluded) { printf("ASTIncluded\n"); }
    else { printf("no ASTIncluded\n"); }
    if(df3_msg->configuration.echoIncluded) { printf("echoIncluded\n"); }
    else { printf("no echoIncluded\n"); }    
    if(df3_msg->configuration.ahrsIncluded) { printf("ahrsIncluded\n"); }
    else { printf("no ahrsIncluded\n"); }        
    if(df3_msg->configuration.PGoodIncluded) { printf("PGoodIncluded\n"); }
    else { printf("no PGoodIncluded\n"); }      
    if(df3_msg->configuration.stdDevIncluded) { printf("stdDevIncluded\n"); }
    else { printf("no stdDevIncluded\n"); }      

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
    df3_msg->seconds       = data.seconds;
    df3_msg->micro_seconds = data.microseconds; // actually sent as 100-microsecond counts
    // convert to epoch
    boost::posix_time::ptime dvltime(
        boost::gregorian::date(static_cast<int>(df3_msg->year) + 1900, 
                               static_cast<int>(df3_msg->month) + 1, 
                               df3_msg->day),
        boost::posix_time::hours(df3_msg->hour) + 
        boost::posix_time::minutes(df3_msg->minute) +
        boost::posix_time::seconds(df3_msg->seconds) +
        boost::posix_time::microseconds(static_cast<int>(df3_msg->micro_seconds)*100));

    // different type of sensor data
    df3_msg->speed_sound = data.speed_sound*0.1;
    df3_msg->temperature = data.temperature*0.01;
    df3_msg->pressure    = data.pressure*0.0001; //Bar
    // not inclued for this DVL
    df3_msg->heading = data.heading*0.01;
    df3_msg->pitch   = data.pitch*0.01;
    df3_msg->roll    = data.roll*0.01;
    // beam system
    df3_msg->beam_system.num_cells  = data.beam_system.num_cells;
    df3_msg->beam_system.coordinate = data.beam_system.coordinate;
    df3_msg->beam_system.num_beams  = data.beam_system.num_beams;
    df3_msg->cell_size              = data.cell_size*0.001;
    df3_msg->nominal_correlation    = static_cast<int>(data.nominalCorrelation);
    df3_msg->pressure_temperature   = (data.pressTemp/5.0-4)*0.2;
    df3_msg->battery                = data.battery*0.1;
    // not inclued for this DVL
    df3_msg->mag3D[0] = data.mag3D[0];
    df3_msg->mag3D[1] = data.mag3D[1];
    df3_msg->mag3D[2] = data.mag3D[2];
    df3_msg->acc3D[0] = data.acc3D[0];
    df3_msg->acc3D[1] = data.acc3D[1];
    df3_msg->acc3D[2] = data.acc3D[2];
    // Data Set Description
    df3_msg->dataset_description.beamData1 = data.dataSetDescription.beamData1;
    df3_msg->dataset_description.beamData2 = data.dataSetDescription.beamData2;
    df3_msg->dataset_description.beamData3 = data.dataSetDescription.beamData3;
    df3_msg->dataset_description.beamData4 = data.dataSetDescription.beamData4;
    // different type of sensor data
    df3_msg->transmit_energy = data.transmitEnergy;
    df3_msg->power_level     = static_cast<float>(data.powerlevel);
    df3_msg->mag_temperature = data.magnTemperature;
    df3_msg->rtc_temperature = data.rtcTemperature*0.01;

    //! TODO: if Echo Sounder is inclued, this become Number of Echo Sounder Cells ?
    df3_msg->velocity_scale  = static_cast<int>(data.velocityScaling);
    double scale_factor      = pow(10, df3_msg->velocity_scale);
    df3_msg->ambVelocity     = data.ambVelocity * scale_factor;

    //Basic Error Handling
    df3_msg->error = data.error;
    if (df3_msg->error != 0) 
        std::cout<< "DVL CP Error Message is: "<<df3_msg->error<<std::endl;
    // status0
    df3_msg->status0.procIdle3      = data.status0.procIdle3;
    df3_msg->status0.procIdle6      = data.status0.procIdle6;
    df3_msg->status0.procIdle12     = data.status0.procIdle12;
    df3_msg->status0.empty          = data.status0.empty;
    df3_msg->status0.stat0inUse     = data.status0.stat0inUse;
    // status
    df3_msg->status.unused1         = data.status.unused1;
    df3_msg->status.bdScaling       = data.status.bdScaling;
    df3_msg->status.unused2         = data.status.unused2;
    df3_msg->status.unused3         = data.status.unused3;
    df3_msg->status.unused4         = data.status.unused4;
    df3_msg->status.echoFrequency   = data.status.echoFrequency;
    df3_msg->status.boostRun        = data.status.boostRun;
    df3_msg->status.telemetry       = data.status.telemetry;
    df3_msg->status.echoIndex       = data.status.echoIndex;
    df3_msg->status.activeConfig    = data.status.activeConfig;
    df3_msg->status.lowVoltSkip     = data.status.lowVoltSkip;
    df3_msg->status.prevWakeupState = data.status.prevWakeupState;
    df3_msg->status.autoOrient      = data.status.autoOrient;
    df3_msg->status.orient          = data.status.orient;
    df3_msg->status.wakeupState     = data.status.wakeupState;
    // handle blanking scaling
    if(df3_msg->status.bdScaling)
        df3_msg->blanking           = data.blanking*0.01;
    else
        df3_msg->blanking           = data.blanking*0.001;
    //// TODO: use this as count for current profile msg?
    df3_msg->ensemble_counter       = data.ensembleCounter;

    /***** Header Data *****/

    df3_msg->io_time = ros::Time().fromSec(io_time);
    df3_msg->system_time = ros::Time::fromBoost(dvltime);
    df3_msg->header.stamp = df3_msg->system_time;
    double max_clock_offset = 0.5;

    // Determine the authoratative timestamp for this message
    ros::Duration dt = df3_msg->io_time - df3_msg->system_time;
    if (fabs(dt.toSec()) > max_clock_offset) {
        df3_msg->header.stamp = df3_msg->io_time;

#ifdef DEBUG
        // If the timestamps are wildly different, use the IO time
        std::cout<<"DVL CP clock differs from CPU clock by " << dt.toSec() 
                        << " seconds (threshold: "
                        << max_clock_offset <<"); using I/O times\n";
#endif
    }

    /***** Cell Data *****/

    for(int i=0; i< (int)df3_msg->beam_system.num_cells; i++) {
        // prepare the cell
        nortek_dvl_ethernet::NortekProfileCell cell;
        // time
        cell.header_time = df3_msg->header.stamp.sec;
        cell.dvl_time    = df3_msg->system_time.sec;
        // cell id
        cell.num = i+1;
        cell.pos = df3_msg->blanking + (i+1) * df3_msg->cell_size;
        // velocity data
        if(df3_msg->configuration.velIncluded) {
            cell.v_x  = cells.velData[0][i] * scale_factor;
            cell.v_y  = cells.velData[1][i] * scale_factor;
            cell.v_z  = cells.velData[2][i] * scale_factor;
            cell.v_z2 = cells.velData[3][i] * scale_factor;
        }
        // amplitude data
        if(df3_msg->configuration.ampIncluded) {
            cell.amp1  = cells.ampData[0][i];
            cell.amp2  = cells.ampData[1][i];
            cell.amp3  = cells.ampData[2][i];
            cell.amp4  = cells.ampData[3][i];
        }
        // correlation data
        if(df3_msg->configuration.corrIncluded) {
            cell.cor1  = cells.corData[0][i];
            cell.cor2  = cells.corData[1][i];
            cell.cor3  = cells.corData[2][i];
            cell.cor4  = cells.corData[3][i];                
        }

        df3_msg->cells.push_back(cell);
    }
}  

void NortekDVLParser::ToDF3(
    const double io_time, 
    const nortek_dvl_structs::ProfileData& data, 
    const nortek_dvl_structs::ProfileCellsSimple& cells, 
    nortek_dvl_ethernet::NortekDF3* df3_msg)
{
    /***** Information Data*****/

    df3_msg->dvl_type                      = nortek_dvl_ethernet::NortekDF3::DVL_TYPE_PISTON;
    df3_msg->version                       = data.version;
    df3_msg->data_offset                   = data.data_offset;
    df3_msg->configuration.pressure        = data.configuration.pressure;
    df3_msg->configuration.temp            = data.configuration.temp;
    df3_msg->configuration.compass         = data.configuration.compass;
    df3_msg->configuration.tilt            = data.configuration.tilt;
    df3_msg->configuration.empty           = data.configuration.empty;
    df3_msg->configuration.velIncluded     = data.configuration.velIncluded;
    df3_msg->configuration.ampIncluded     = data.configuration.ampIncluded;
    df3_msg->configuration.corrIncluded    = data.configuration.corrIncluded;
    df3_msg->configuration.altiIncluded    = data.configuration.altiIncluded;
    df3_msg->configuration.altiRawIncluded = data.configuration.altiRawIncluded;
    df3_msg->configuration.ASTIncluded     = data.configuration.ASTIncluded;
    df3_msg->configuration.echoIncluded    = data.configuration.echoIncluded;
    df3_msg->configuration.ahrsIncluded    = data.configuration.ahrsIncluded;
    df3_msg->configuration.PGoodIncluded   = data.configuration.PGoodIncluded;
    df3_msg->configuration.stdDevIncluded  = data.configuration.stdDevIncluded;
    df3_msg->configuration.unused          = data.configuration.unused;
    df3_msg->serial_number                 = data.serial_num;

    /***** Sensor Data*****/

    // get DVL system time
    df3_msg->year          = data.year;
    df3_msg->month         = data.month;
    df3_msg->day           = data.day;
    df3_msg->hour          = data.hour;
    df3_msg->minute        = data.minute;
    df3_msg->seconds       = data.seconds;
    df3_msg->micro_seconds = data.microseconds; // actually sent as 100-microsecond counts
    // convert to epoch
    boost::posix_time::ptime dvltime(
        boost::gregorian::date(static_cast<int>(df3_msg->year) + 1900, 
                               static_cast<int>(df3_msg->month) + 1, 
                               df3_msg->day),
        boost::posix_time::hours(df3_msg->hour) + 
        boost::posix_time::minutes(df3_msg->minute) +
        boost::posix_time::seconds(df3_msg->seconds) +
        boost::posix_time::microseconds(static_cast<int>(df3_msg->micro_seconds)*100));

    // different type of sensor data
    df3_msg->speed_sound = data.speed_sound*0.1;
    df3_msg->temperature = data.temperature*0.01;
    df3_msg->pressure    = data.pressure*0.0001; //Bar
    // not inclued for this DVL
    df3_msg->heading = data.heading*0.01;
    df3_msg->pitch   = data.pitch*0.01;
    df3_msg->roll    = data.roll*0.01;
    // beam system
    df3_msg->beam_system.num_cells  = data.beam_system.num_cells;
    df3_msg->beam_system.coordinate = data.beam_system.coordinate;
    df3_msg->beam_system.num_beams  = data.beam_system.num_beams;
    df3_msg->cell_size              = data.cell_size*0.001;
    df3_msg->nominal_correlation    = static_cast<int>(data.nominalCorrelation);
    df3_msg->pressure_temperature   = (data.pressTemp/5.0-4)*0.2;
    df3_msg->battery                = data.battery*0.1;
    // not inclued for this DVL
    df3_msg->mag3D[0] = data.mag3D[0];
    df3_msg->mag3D[1] = data.mag3D[1];
    df3_msg->mag3D[2] = data.mag3D[2];
    df3_msg->acc3D[0] = data.acc3D[0];
    df3_msg->acc3D[1] = data.acc3D[1];
    df3_msg->acc3D[2] = data.acc3D[2];
    // Data Set Description
    df3_msg->dataset_description.beamData1 = data.dataSetDescription.beamData1;
    df3_msg->dataset_description.beamData2 = data.dataSetDescription.beamData2;
    df3_msg->dataset_description.beamData3 = data.dataSetDescription.beamData3;
    df3_msg->dataset_description.beamData4 = data.dataSetDescription.beamData4;
    // different type of sensor data
    df3_msg->transmit_energy = data.transmitEnergy;
    df3_msg->power_level     = static_cast<float>(data.powerlevel);
    df3_msg->mag_temperature = data.magnTemperature;
    df3_msg->rtc_temperature = data.rtcTemperature*0.01;

    df3_msg->velocity_scale  = static_cast<int>(data.velocityScaling);
    double scale_factor      = pow(10, df3_msg->velocity_scale);
    df3_msg->ambVelocity     = data.ambVelocity * scale_factor;

    //Basic Error Handling
    df3_msg->error = data.error;
    if (df3_msg->error != 0) 
        std::cout<< "DVL CP Error Message is: "<<df3_msg->error<<std::endl;
    // status0
    df3_msg->status0.procIdle3      = data.status0.procIdle3;
    df3_msg->status0.procIdle6      = data.status0.procIdle6;
    df3_msg->status0.procIdle12     = data.status0.procIdle12;
    df3_msg->status0.empty          = data.status0.empty;
    df3_msg->status0.stat0inUse     = data.status0.stat0inUse;
    // status
    df3_msg->status.unused1         = data.status.unused1;
    df3_msg->status.bdScaling       = data.status.bdScaling;
    df3_msg->status.unused2         = data.status.unused2;
    df3_msg->status.unused3         = data.status.unused3;
    df3_msg->status.unused4         = data.status.unused4;
    df3_msg->status.echoFrequency   = data.status.echoFrequency;
    df3_msg->status.boostRun        = data.status.boostRun;
    df3_msg->status.telemetry       = data.status.telemetry;
    df3_msg->status.echoIndex       = data.status.echoIndex;
    df3_msg->status.activeConfig    = data.status.activeConfig;
    df3_msg->status.lowVoltSkip     = data.status.lowVoltSkip;
    df3_msg->status.prevWakeupState = data.status.prevWakeupState;
    df3_msg->status.autoOrient      = data.status.autoOrient;
    df3_msg->status.orient          = data.status.orient;
    df3_msg->status.wakeupState     = data.status.wakeupState;
    // handle blanking scaling
    if(df3_msg->status.bdScaling)
        df3_msg->blanking           = data.blanking*0.01;
    else
        df3_msg->blanking           = data.blanking*0.001;
    //// TODO: use this as count for current profile msg?
    df3_msg->ensemble_counter       = data.ensembleCounter;

    /***** Header Data *****/

    df3_msg->io_time = ros::Time().fromSec(io_time);
    df3_msg->system_time = ros::Time::fromBoost(dvltime);
    df3_msg->header.stamp = df3_msg->system_time;
    double max_clock_offset = 0.5;

    // Determine the authoratative timestamp for this message
    ros::Duration dt = df3_msg->io_time - df3_msg->system_time;
    if (fabs(dt.toSec()) > max_clock_offset) {
        df3_msg->header.stamp = df3_msg->io_time;

#ifdef DEBUG
        // If the timestamps are wildly different, use the IO time
        std::cout<<"DVL CP clock differs from CPU clock by " << dt.toSec() 
                        << " seconds (threshold: "
                        << max_clock_offset <<"); using I/O times\n";
#endif
    }

    /***** Cell Data *****/

    //! TODO: change to dynamic cell size
    if(df3_msg->beam_system.num_beams != 4)
        std::cout<<"DVL CP error: Wrong DVL beam number , it's: " << df3_msg->beam_system.num_beams<<std::endl;
    if(df3_msg->beam_system.num_cells != 20)
        std::cout<<"DVL CP error: cell number , it's: " << df3_msg->beam_system.num_cells <<std::endl;

    for(int i=0; i< (int)df3_msg->beam_system.num_cells; i++) {
        // prepare the cell
        nortek_dvl_ethernet::NortekProfileCell cell;
        // time
        cell.header_time = df3_msg->header.stamp.sec;
        cell.dvl_time    = df3_msg->system_time.sec;
        // cell id
        cell.num = i+1;
        cell.pos = df3_msg->blanking + (i+1) * df3_msg->cell_size;
        // velocity data
        if(df3_msg->configuration.velIncluded) {
            cell.v_x  = cells.velData[0][i] * scale_factor;
            cell.v_y  = cells.velData[1][i] * scale_factor;
            cell.v_z  = cells.velData[2][i] * scale_factor;
            cell.v_z2 = cells.velData[3][i] * scale_factor;
        }
        // amplitude data
        if(df3_msg->configuration.ampIncluded) {
            cell.amp1  = cells.ampData[0][i];
            cell.amp2  = cells.ampData[1][i];
            cell.amp3  = cells.ampData[2][i];
            cell.amp4  = cells.ampData[3][i];
        }
        // correlation data
        if(df3_msg->configuration.corrIncluded) {
            cell.cor1  = cells.corData[0][i];
            cell.cor2  = cells.corData[1][i];
            cell.cor3  = cells.corData[2][i];
            cell.cor4  = cells.corData[3][i];                
        }

        df3_msg->cells.push_back(cell);
    }
}    
