#include <nortek_dvl_ethernet/parser.h>

bool NortekDVLParser::Checksum(uint16_t length, const uint8_t* buffer)
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

nortek_dvl_structs::parserID 
NortekDVLParser::Parse(const uint8_t* data, std::size_t size, const double io_time)
{
    uint8_t length;
    auto id = ParseHeader(data, size, length);    

    switch (id)
    {
        case nortek_dvl_structs::BT: 
        {
            auto df21 = nortek_dvl_ethernet::NortekDF21{};
            ParseBottomTrack(data, length, io_time, &df21);

            // send to callback function
            if(df21_callback_) {
              df21_callback_(df21);
            }            

            break;
        }

        case nortek_dvl_structs::CP: 
        {
            auto df3 = nortek_dvl_ethernet::NortekDF3{};
            ParseCurrentProfile(data, length, io_time, &df3);

            // send to callback function
            if(df3_callback_) {
              df3_callback_(df3);
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

nortek_dvl_structs::parserID 
NortekDVLParser::ParseHeader(const uint8_t* buffer, const size_t& buffer_size, uint8_t& length) 
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

        length = hdr->header_size;
        auto payload = buffer + length;
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

            return nortek_dvl_structs::BT;
        } 
        else if (hdr->headerid == 0x16) {
            if (payload_len > buffer_size) {
                std::cout<< "Warning: CP Payload length longer that data received\n";
            }
            
            return nortek_dvl_structs::CP;
        }
        else {
            std::cout<< "Error - Header ID Not Recognized: " << hdr->headerid <<"\n";
            return nortek_dvl_structs::ERROR;
        }
    }

    std::cout<< " Error - Nortek DVL Sync ID not recognized: " << sync <<"\n";
    return nortek_dvl_structs::ERROR;
}

void NortekDVLParser::ParseBottomTrack(
    const uint8_t* buffer, 
    uint8_t& length, 
    const double io_time, 
    nortek_dvl_ethernet::NortekDF21* df21_msg)
{
    // get buffer
    auto payload = buffer + length;
    // parse into defined struct
    auto *bt = reinterpret_cast<const nortek_dvl_structs::bottomtrack*>(payload);
    // convert struct into raw msg
    ToDF21(io_time, *bt, df21_msg);
}

void NortekDVLParser::ToDF21(
    const double io_time, 
    const nortek_dvl_structs::bottomtrack& bt, 
    nortek_dvl_ethernet::NortekDF21* df21_msg)
{
    /***** information data *****/

    df21_msg->dvl_type = nortek_dvl_ethernet::NortekDF21::DVL_TYPE_PISTON;
    df21_msg->version = bt.version;
    df21_msg->offsetOfData = bt.data_offset;
    df21_msg->serialNumber = bt.serial_num;
    df21_msg->year = bt.year;
    df21_msg->month = bt.month;
    df21_msg->day = bt.day;
    df21_msg->hour = bt.hour;
    df21_msg->minute = bt.minute;
    df21_msg->seconds = bt.seconds;
    df21_msg->microSeconds = bt.microseconds; // actually sent as 100-microsecond counts
    df21_msg->nBeams = bt.nbeams;
    
    // Basic Error Handling
    df21_msg->error = bt.error;
    if (df21_msg->error != 0) 
        std::cout<< "DVL - Error: "<<df21_msg->error <<std::endl;

    // Basic Status Message Handler
    df21_msg->status.beam1VelValid  = bt.status.beam1VelValid;
    df21_msg->status.beam2VelValid  = bt.status.beam2VelValid;
    df21_msg->status.beam3VelValid  = bt.status.beam3VelValid;
    df21_msg->status.beam4VelValid  = bt.status.beam4VelValid;
    df21_msg->status.beam1DistValid = bt.status.beam1DistValid;
    df21_msg->status.beam2DistValid = bt.status.beam2DistValid;
    df21_msg->status.beam3DistValid = bt.status.beam3DistValid;
    df21_msg->status.beam4DistValid = bt.status.beam4DistValid;
    df21_msg->status.beam1FOMValid  = bt.status.beam1FOMValid;
    df21_msg->status.beam2FOMValid  = bt.status.beam2FOMValid;
    df21_msg->status.beam3FOMValid  = bt.status.beam3FOMValid;
    df21_msg->status.beam4FOMValid  = bt.status.beam4FOMValid;
    df21_msg->status.xVelValid      = bt.status.xVelValid;
    df21_msg->status.yVelValid      = bt.status.yVelValid;
    df21_msg->status.z1VelValid     = bt.status.z1VelValid;
    df21_msg->status.z2VelValid     = bt.status.z2VelValid;
    df21_msg->status.xFOMValid      = bt.status.xFOMValid;
    df21_msg->status.yFOMValid      = bt.status.yFOMValid;
    df21_msg->status.z1FOMValid     = bt.status.z1FOMValid;
    df21_msg->status.z2FOMValid     = bt.status.z2FOMValid;
    df21_msg->status.procIdle3      = bt.status.procIdle3;
    df21_msg->status.procIdle6      = bt.status.procIdle6;
    df21_msg->status.procIdle12     = bt.status.procIdle12;
    df21_msg->status.empty          = bt.status.empty;
    df21_msg->status.wakeupstate    = bt.status.wakeupstate;
    if (df21_msg->status.procIdle12) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 12%\n";
    if (df21_msg->status.procIdle6) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 6%\n";
    if (df21_msg->status.procIdle3) 
        std::cout<< "DVL warning: Processing Capacity Left Less Then 3%\n";

    df21_msg->speed_sound = bt.speed_sound;
    df21_msg->temperature = bt.temperature;
    df21_msg->pressure = bt.pressure;

    /***** Beam Data *****/
    for (int i = 0; i < 4; i++)
    {
        df21_msg->velBeam[i] = bt.velBeam[i];
        df21_msg->distBeam[i] = bt.distBeam[i];
        df21_msg->fomBeam[i] = bt.fomBeam[i];
        df21_msg->timeDiff1Beam[i] = bt.timeDiff1Beam[i];
        df21_msg->timeDiff2Beam[i] = bt.timeDiff2Beam[i];
        df21_msg->timeVelEstBeam[i] = bt.timeVelEstBeam[i];
    }

    /***** XYZ Data *****/

    df21_msg->velX = bt.velX;
    df21_msg->velY = bt.velY;
    df21_msg->velZ1 = bt.velZ1;
    df21_msg->velZ2 = bt.velZ2;
    df21_msg->fomX = bt.fomX;
    df21_msg->fomY = bt.fomY;
    df21_msg->fomZ1 = bt.fomZ1;
    df21_msg->fomZ2 = bt.fomZ2;
    df21_msg->timeDiff1X = bt.timeDiff1X;
    df21_msg->timeDiff1Y = bt.timeDiff1Y;
    df21_msg->timeDiff1Z1 = bt.timeDiff1Z1;
    df21_msg->timeDiff1Z2 = bt.timeDiff1Z2;
    df21_msg->timeDiff2X = bt.timeDiff2X;
    df21_msg->timeDiff2Y = bt.timeDiff2Y;
    df21_msg->timeDiff2Z1 = bt.timeDiff2Z1;
    df21_msg->timeDiff2Z2 = bt.timeDiff2Z2;
    df21_msg->timeVelEstX = bt.timeVelEstX;
    df21_msg->timeVelEstY = bt.timeVelEstY;
    df21_msg->timeVelEstZ1 = bt.timeVelEstZ1;
    df21_msg->timeVelEstZ2 =  bt.timeVelEstZ2;

    /***** Processed data *****/

    double altitude_sum = 0;
    for (int i = 0; i < 4; i++)
    {
        if (bt.distBeam[i] != 0 && bt.velBeam[i] != -32.768f && bt.fomBeam[i] != 10){
            df21_msg->good_beams += 1;
            altitude_sum += df21_msg->distBeam[i];
        }
    }
    df21_msg->altitude = altitude_sum / 4;

    // setup speed ?
    df21_msg->speed_gnd = sqrt(df21_msg->velX * df21_msg->velX + df21_msg->velY * df21_msg->velY);
    df21_msg->course_gnd = atan2(df21_msg->velX, df21_msg->velY) * 180.0 / M_PI;

    // setup time
    int year = static_cast<int>(df21_msg->year) + 1900;
    int month = static_cast<int>(df21_msg->month) + 1;
    boost::posix_time::ptime dvltime(
        boost::gregorian::date(year, month, df21_msg->day),
        boost::posix_time::hours(df21_msg->hour) + boost::posix_time::minutes(df21_msg->minute) +
        boost::posix_time::seconds(df21_msg->seconds) +
        boost::posix_time::microseconds(static_cast<int>(df21_msg->microSeconds)*100));

    // setup header
    df21_msg->io_time = ros::Time().fromSec(io_time);
    df21_msg->system_time = ros::Time::fromBoost(dvltime);
    df21_msg->header.stamp = df21_msg->system_time;
    double max_clock_offset = 0.5;

    // Determine the authoratative timestamp for this message
    ros::Duration dt = df21_msg->io_time - df21_msg->system_time ;
    if (fabs(dt.toSec()) > max_clock_offset) {
        df21_msg->header.stamp = df21_msg->io_time;

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
    uint8_t& length, 
    const double io_time, 
    nortek_dvl_ethernet::NortekDF3* df3_msg)
{
    auto payload = buffer + length;
    // parse into defined struct
    auto *cp = reinterpret_cast<const nortek_dvl_structs::currentprofile*>(payload);
    // convert parse struct into ros message 
    ToDF3(io_time, *cp, df3_msg);
}

void NortekDVLParser::ToDF3(
    const double io_time, 
    const nortek_dvl_structs::currentprofile& cp, 
    nortek_dvl_ethernet::NortekDF3* df3_msg)
{
    /***** Information Data*****/

    df3_msg->dvl_type                      = nortek_dvl_ethernet::NortekDF3::DVL_TYPE_PISTON;
    df3_msg->version                       = cp.version;
    df3_msg->data_offset                   = cp.data_offset;
    df3_msg->configuration.pressure        = cp.configuration.pressure;
    df3_msg->configuration.temp            = cp.configuration.temp;
    df3_msg->configuration.compass         = cp.configuration.compass;
    df3_msg->configuration.tilt            = cp.configuration.tilt;
    df3_msg->configuration.empty           = cp.configuration.empty;
    df3_msg->configuration.velIncluded     = cp.configuration.velIncluded;
    df3_msg->configuration.ampIncluded     = cp.configuration.ampIncluded;
    df3_msg->configuration.corrIncluded    = cp.configuration.corrIncluded;
    df3_msg->configuration.altiIncluded    = cp.configuration.altiIncluded;
    df3_msg->configuration.altiRawIncluded = cp.configuration.altiRawIncluded;
    df3_msg->configuration.ASTIncluded     = cp.configuration.ASTIncluded;
    df3_msg->configuration.echoIncluded    = cp.configuration.echoIncluded;
    df3_msg->configuration.ahrsIncluded    = cp.configuration.ahrsIncluded;
    df3_msg->configuration.PGoodIncluded   = cp.configuration.PGoodIncluded;
    df3_msg->configuration.stdDevIncluded  = cp.configuration.stdDevIncluded;
    df3_msg->configuration.unused          = cp.configuration.unused;
    df3_msg->serial_number                 = cp.serial_num;

    /***** Sensor Data*****/

    // get DVL system time
    df3_msg->year          = cp.year;
    df3_msg->month         = cp.month;
    df3_msg->day           = cp.day;
    df3_msg->hour          = cp.hour;
    df3_msg->minute        = cp.minute;
    df3_msg->seconds       = cp.seconds;
    df3_msg->micro_seconds = cp.microseconds; // actually sent as 100-microsecond counts
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
    df3_msg->speed_sound = cp.speed_sound*0.1;
    df3_msg->temperature = cp.temperature*0.01;
    df3_msg->pressure    = cp.pressure*0.0001; //Bar
    // not inclued for this DVL
    df3_msg->heading = cp.heading*0.01;
    df3_msg->pitch   = cp.pitch*0.01;
    df3_msg->roll    = cp.roll*0.01;
    // beam system
    df3_msg->beam_system.num_cells  = cp.beam_system.num_cells;
    df3_msg->beam_system.coordinate = cp.beam_system.coordinate;
    df3_msg->beam_system.num_beams  = cp.beam_system.num_beams;
    df3_msg->cell_size              = cp.cell_size*0.001;
    df3_msg->nominal_correlation    = static_cast<int>(cp.nominalCorrelation);
    df3_msg->pressure_temperature   = (cp.pressTemp/5.0-4)*0.2;
    df3_msg->battery                = cp.battery*0.1;
    // not inclued for this DVL
    df3_msg->mag3D[0] = cp.mag3D[0];
    df3_msg->mag3D[1] = cp.mag3D[1];
    df3_msg->mag3D[2] = cp.mag3D[2];
    df3_msg->acc3D[0] = cp.acc3D[0];
    df3_msg->acc3D[1] = cp.acc3D[1];
    df3_msg->acc3D[2] = cp.acc3D[2];
    // Data Set Description
    df3_msg->dataset_description.beamData1 = cp.dataSetDescription.beamData1;
    df3_msg->dataset_description.beamData2 = cp.dataSetDescription.beamData2;
    df3_msg->dataset_description.beamData3 = cp.dataSetDescription.beamData3;
    df3_msg->dataset_description.beamData4 = cp.dataSetDescription.beamData4;
    // different type of sensor data
    df3_msg->transmit_energy = cp.transmitEnergy;
    df3_msg->power_level     = static_cast<float>(cp.powerlevel);
    df3_msg->mag_temperature = cp.magnTemperature;
    df3_msg->rtc_temperature = cp.rtcTemperature*0.01;

    df3_msg->velocity_scale  = static_cast<int>(cp.velocityScaling);
    double scale_factor      = pow(10, df3_msg->velocity_scale);
    df3_msg->ambVelocity     = cp.ambVelocity * scale_factor;

    //Basic Error Handling
    df3_msg->error = cp.error;
    if (df3_msg->error != 0) 
        std::cout<< "DVL CP Error Message is: "<<df3_msg->error<<std::endl;
    // status0
    df3_msg->status0.procIdle3      = cp.status0.procIdle3;
    df3_msg->status0.procIdle6      = cp.status0.procIdle6;
    df3_msg->status0.procIdle12     = cp.status0.procIdle12;
    df3_msg->status0.empty          = cp.status0.empty;
    df3_msg->status0.stat0inUse     = cp.status0.stat0inUse;
    // status
    df3_msg->status.unused1         = cp.status.unused1;
    df3_msg->status.bdScaling       = cp.status.bdScaling;
    df3_msg->status.unused2         = cp.status.unused2;
    df3_msg->status.unused3         = cp.status.unused3;
    df3_msg->status.unused4         = cp.status.unused4;
    df3_msg->status.echoFrequency   = cp.status.echoFrequency;
    df3_msg->status.boostRun        = cp.status.boostRun;
    df3_msg->status.telemetry       = cp.status.telemetry;
    df3_msg->status.echoIndex       = cp.status.echoIndex;
    df3_msg->status.activeConfig    = cp.status.activeConfig;
    df3_msg->status.lowVoltSkip     = cp.status.lowVoltSkip;
    df3_msg->status.prevWakeupState = cp.status.prevWakeupState;
    df3_msg->status.autoOrient      = cp.status.autoOrient;
    df3_msg->status.orient          = cp.status.orient;
    df3_msg->status.wakeupState     = cp.status.wakeupState;
    // handle blanking scaling
    if(df3_msg->status.bdScaling)
        df3_msg->blanking           = cp.blanking*0.01;
    else
        df3_msg->blanking           = cp.blanking*0.001;
    //// TODO: use this as count for current profile msg?
    df3_msg->ensemble_counter       = cp.ensembleCounter;

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
        nortek_dvl_ethernet::NortekCPCell cell;

        // time
        cell.header_time = df3_msg->header.stamp.sec;
        cell.dvl_time    = df3_msg->system_time.sec;
        // cell id
        cell.num = i+1;
        cell.pos = df3_msg->blanking + (i+1) * df3_msg->cell_size;
        // velocity data
        if(df3_msg->configuration.velIncluded) {
            cell.v_x  = cp.velData[0][i] * scale_factor;
            cell.v_y  = cp.velData[1][i] * scale_factor;
            cell.v_z  = cp.velData[2][i] * scale_factor;
            cell.v_z2 = cp.velData[3][i] * scale_factor;
        }
        // amplitude data
        if(df3_msg->configuration.ampIncluded) {
            cell.amp1  = cp.ampData[0][i];
            cell.amp2  = cp.ampData[1][i];
            cell.amp3  = cp.ampData[2][i];
            cell.amp4  = cp.ampData[3][i];
        }
        // correlation data
        if(df3_msg->configuration.corrIncluded) {
            cell.cor1  = cp.corData[0][i];
            cell.cor2  = cp.corData[1][i];
            cell.cor3  = cp.corData[2][i];
            cell.cor4  = cp.corData[3][i];                
        }

        df3_msg->cells.push_back(cell);
    }
}
