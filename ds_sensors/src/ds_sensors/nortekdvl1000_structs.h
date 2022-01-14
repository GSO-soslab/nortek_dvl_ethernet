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
// file:  include/ds_sensors/nortekdvl_structs.h

#ifndef DS_SENSORS_NORTEKDVL_STRUCTS_H
#define DS_SENSORS_NORTEKDVL_STRUCTS_H

namespace ds_sensors {
    namespace nortekdvl_structs
    {

        enum parserID
        {
            ERROR = -1,
            BT = 0,
            CP = 1
        };

    /************************************************************************************/
    /******************************************* Header *********************************/
    /************************************************************************************/
    
        struct header
        {
            uint8_t sync;
            uint8_t header_size;
            uint8_t headerid;
            uint8_t family;
            uint8_t data_size;
            uint8_t data_checksum;
            uint8_t header_checksum;
        } __attribute__((packed));

    /************************************************************************************/
    /************************************* Data: Bottom Track ***************************/
    /************************************************************************************/
        struct BTstatus
        {
            uint32_t beam1VelValid  : 1; // BIT(0)
            uint32_t beam2VelValid  : 1; // BIT(1)
            uint32_t beam3VelValid  : 1; // BIT(2)
            uint32_t beam4VelValid  : 1; // BIT(3)
            uint32_t beam1DistValid : 1; // BIT(4)
            uint32_t beam2DistValid : 1; // BIT(5)
            uint32_t beam3DistValid : 1; // BIT(6)
            uint32_t beam4DistValid : 1; // BIT(7)
            uint32_t beam1FOMValid  : 1; // BIT(8)
            uint32_t beam2FOMValid  : 1; // BIT(9)
            uint32_t beam3FOMValid  : 1; // BIT(10)
            uint32_t beam4FOMValid  : 1; // BIT(11)
            uint32_t xVelValid      : 1; // BIT(12)
            uint32_t yVelValid      : 1; // BIT(13)
            uint32_t z1VelValid     : 1; // BIT(14)
            uint32_t z2VelValid     : 1; // BIT(15)
            uint32_t xFOMValid      : 1; // BIT(16)
            uint32_t yFOMValid      : 1; // BIT(17)
            uint32_t z1FOMValid     : 1; // BIT(18)
            uint32_t z2FOMValid     : 1; // BIT(19)
            uint32_t procIdle3      : 1; // BIT(20)
            uint32_t procIdle6      : 1; // BIT(21)
            uint32_t procIdle12     : 1; // BIT(22)
            uint32_t empty          : 5; // BIT(23-27)
            uint32_t wakeupstate    : 4; // BIT(28-31)
        } __attribute__((packed));

        struct bottomtrack
        {
            uint8_t version;
            uint8_t data_offset;
            uint32_t serial_num;
            uint8_t year;            //Trugger time
            uint8_t month;
            uint8_t day;
            uint8_t hour;
            uint8_t minute;
            uint8_t seconds;
            uint16_t microseconds;
            uint16_t nbeams;
            uint32_t error;
            BTstatus status;
            float speed_sound;       // [m/s]
            float temperature;       // [Celsius]
            float pressure;          // [bar]

            float velBeam[4];        //Velocities for each beam [m/s]
            float distBeam[4];       //Distances for each beam  [m]
            float fomBeam[4];        //Figure of merit for each beam [m/s]
            float timeDiff1Beam[4];  //DT1 for each beam [s]
            float timeDiff2Beam[4];  //DT2 for each beam [s]
            float timeVelEstBeam[4]; //Duration of velocity estimate for each beam [s]

            float velX;              //Velocity X [m/s]
            float velY;              //Velocity Y [m/s]
            float velZ1;             //Velocity Z1 [m/s]
            float velZ2;             //Velocity Z2 [m/s]
            float fomX;              //Figure of Merit X [m/s]
            float fomY;              //Figure of Merit Y [m/s]
            float fomZ1;             //Figure of Merit Z1 [m/s]
            float fomZ2;             //Figure of Merit Z2 [m/s]
            float timeDiff1X;        //Time from trigger to center of bottom echo [s]
            float timeDiff1Y;        //Same as above [s]
            float timeDiff1Z1;       //Same as above [s]
            float timeDiff1Z2;       //Same as above [s]
            float timeDiff2X;        //Time from start of NMEA output msg to center of bottom echo [s]
            float timeDiff2Y;        //Same as above [s]
            float timeDiff2Z1;       //Same as above [s]
            float timeDiff2Z2;       //Same as above [s]
            float timeVelEstX;       //Duration of velocity estimate for each component [s]
            float timeVelEstY;       //Same as above [s]
            float timeVelEstZ1;      //Same as above [s]
            float timeVelEstZ2;      //Same as above [s]
        } __attribute__((packed));

    /******************************************************************************************/
    /********************************* Data: Current Profile **********************************/
    /******************************************************************************************/
        struct CPconfiguration
        {
            uint16_t pressure        : 1; // BIT(0), 1 and inclued in DVL 1000
            uint16_t temp            : 1; // BIT(1), 1 and inclued in DVL 1000
            uint16_t compass         : 1; // BIT(2), 0 and not inclued in DVL 1000
            uint16_t tilt            : 1; // BIT(3), 0 and not inclued in DVL 1000
            uint16_t empty           : 1; // BIT(4)
            uint16_t velIncluded     : 1; // BIT(5), 1 and inclued in DVL 1000
            uint16_t ampIncluded     : 1; // BIT(6), 1 and inclued in DVL 1000
            uint16_t corrIncluded    : 1; // BIT(7), 1 and inclued in DVL 1000
            uint16_t altiIncluded    : 1; // BIT(8), 0 and not inclued in DVL 1000 
            uint16_t altiRawIncluded : 1; // BIT(9), 0 and not inclued in DVL 1000
            uint16_t ASTIncluded     : 1; // BIT(10), 0 and not inclued in DVL 1000
            uint16_t echoIncluded    : 1; // BIT(11), 0 and not inclued in DVL 1000
            uint16_t ahrsIncluded    : 1; // BIT(12), 0 and not inclued in DVL 1000
            uint16_t PGoodIncluded   : 1; // BIT(13), 0 and not inclued in DVL 1000
            uint16_t stdDevIncluded  : 1; // BIT(14), 0 and not inclued in DVL 1000
            uint16_t unused          : 1; // BIT(15)
        } __attribute__((packed));

        struct CPbeams
        {
            uint16_t num_cells  : 10; // BIT(0~9), Number of Cells (NC)
            uint16_t coordinate : 2;  // BIT(10~11), 01=XYZ, 10=BEAM, 11=-
            uint16_t num_beams  : 4;  // BIT(12~15), Number of Beams (NB)
        } __attribute__((packed));

        struct CPdataset
        {   
            uint16_t beamData1  : 4; // BIT(0~3), Physical beam used for 1st data set
            uint16_t beamData2  : 4; // BIT(4~7), Physical beam used for 2nd data set
            uint16_t beamData3  : 4; // BIT(8~11), Physical beam used for 3th data set
            uint16_t beamData4  : 4; // BIT(12~15), Physical beam used for 4th data set
        } __attribute__((packed));

        struct CPstatus0
        {   
            uint16_t procIdle3   : 1;   // BIT(0),  Indicates that the processor Idles less than 3 percent
            uint16_t procIdle6   : 1;   // BIT(1),  Indicates that the processor Idles less than 6 percent
            uint16_t procIdle12  : 1;   // BIT(2),  Indicates that the processor Idles less than 12 percent
            uint16_t empty       : 12;  // BIT(3~14), unused
            uint16_t stat0inUse  : 1;   // BIT(15),  If this bit is set the rest of the word should be interpreted
        } __attribute__((packed));

        struct CPstatus
        {
            uint32_t unused1        : 1; // BIT(0)
            uint32_t bdScaling      : 1; // BIT(1), cm scaling of blanking distance
            uint32_t unused2        : 1; // BIT(2)
            uint32_t unused3        : 1; // BIT(3)
            uint32_t unused4        : 1; // BIT(4)
            uint32_t echoFrequency  : 5; // BIT(5~9)
            uint32_t boostRun       : 1; // BIT(10)
            uint32_t telemetry      : 1; // BIT(11)
            uint32_t echoIndex      : 4; // BIT(12~15)
            uint32_t activeConfig   : 1; // BIT(16)
            uint32_t lowVoltSkip    : 1; // BIT(17), 0=normal operation, 1=last measurement skipped due to low input voltage
            uint32_t prevWakeupState: 4; // BIT(18~21), 10=break, 11= RTC alarm, 00=bad power, 01=power applied
            uint32_t autoOrient     : 3; // BIT(22~24)
            uint32_t orient         : 3; // BIT(25~27)
            uint32_t wakeupState    : 4; // BIT(28~31), 10=break, 11= RTC alarm, 00=bad power, 01=power applied
        } __attribute__((packed));

        struct currentprofile
        {
            /***** Information Data*****/
            uint8_t version;                // Version number of the Data Record Definition.
            uint8_t data_offset;            // Number of bytes from start of record to start of data (velocity/amplitude/correlation)
            CPconfiguration configuration;  // Record Configuration Bit Mask
            uint32_t serial_num;
            
            /***** Sensor Data *****/
            uint8_t year;                   // Years since 1900 
            uint8_t month;                  // Jan =0, Feb= 1 ...
            uint8_t day;                    // (see struct tm definition)
            uint8_t hour;                   // (see struct tm definition)
            uint8_t minute;                 // (see struct tm definition)
            uint8_t seconds;                // (see struct tm definition)
            uint16_t microseconds;          // 100 usec
            uint16_t speed_sound;           // [0.1 m/s]
            int16_t temperature;            // [0.01 Deg Celsius]
            uint32_t pressure;              // [0.001 dBar]
            uint16_t heading;               // [0.01 degree]
            int16_t pitch;                  // [0.01 degree]
            int16_t roll;                   // [0.01 degree]
            CPbeams beam_system;            // Number of Cells; Coordinate system; Number of Beams;
            uint16_t cell_size;             // [1 mm]
            uint16_t blanking;              // [1 mm]
            uint8_t nominalCorrelation;     // The nominal correlation for the configured combination of cell size and velocity range. [%]
            uint8_t pressTemp;              // Temperature of Pressure sensor: T = (Val/5) - 4.0 [0.2 Deg Celslus]
            uint16_t battery;               // [0.1 Volt]
            int16_t mag3D[3];               // Magnetometer Raw data in 3-axis
            int16_t acc3D[3];               // Accelrometer Raw Data in 3-axis (16384 = 1.0)
            uint16_t ambVelocity;           // [10^(velocity scaling) m/s] Ambiguity velocity, corrected for sound velocity,(Velocity scaled according to Velocity Scaling
            CPdataset dataSetDescription;
            uint16_t transmitEnergy;
            int8_t velocityScaling;         // Used to scale velocity data.
            int8_t powerlevel;              // [dB] Configured power level
            int16_t magnTemperature;        // Magnetometer temperature reading
            int16_t rtcTemperature;         // [0.01 Deg Celsius] Real time clock temperature reading
            uint16_t error;
            CPstatus0 status0;
            CPstatus status;
            uint32_t ensembleCounter;       // Counts the number of ensembles in both averaged data and burst data

            /***** Cell Data *****/
            int16_t velData[4][20];         // [10^(velocity scaling) m/s ]
            uint8_t ampData[4][20];         // [1 count]
            uint8_t corData[4][20];         // [0-100]

        } __attribute__((packed));

    }  // nortekdvl_structs

} //ds_sensors
#endif //DS_SENSORS_NORTEKDVL_STRUCTS_H
