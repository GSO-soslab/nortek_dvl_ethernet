#################
## Header data ##
#################
std_msgs/Header header
time io_time
time system_time

####################
# Information Data #
####################

# Standard array with one piston transducer for each beam, 
# e.g., Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
# Newfangled phased-array DVL, e.g., RDI DVLs
uint8 DVL_TYPE_PISTON=0
uint8 DVL_TYPE_PHASED_ARRAY=1

uint8 dvl_type
uint8 version
uint8 data_offset
nortek_dvl_ethernet/NortekCPConfig configuration
uint32 serial_number

###################
### Sensor Data ###
###################
uint8 year
uint8 month
uint8 day
uint8 hour
uint8 minute
uint8 seconds
uint16 micro_seconds
float32 speed_sound # [m/s]
float32 temperature # [Degree Celsius]
float32 pressure    # [Bar]
float32 heading     # [Degree]          
float32 pitch       # [Degree]
float32 roll        # [Degree]
nortek_dvl_ethernet/NortekCPBeam beam_system
float32 cell_size    # [m]
float32 blanking     # [m]
uint8 nominal_correlation # [%]
float32 pressure_temperature # [Degree Celsius]
float32 battery       # [Voltage]
int16[3] mag3D
int16[3] acc3D        #[16384 = 1.0]
float32 ambVelocity   # [m/s]
nortek_dvl_ethernet/NortekCPDataset dataset_description
uint16 transmit_energy
int8 velocity_scale
float32 power_level   #[dB]
int16 mag_temperature #[uncalibrated]
int16 rtc_temperature #[Degree Celsius]
uint16 error
nortek_dvl_ethernet/NortekCPStatus0 status0
nortek_dvl_ethernet/NortekCPStatus status
uint32 ensemble_counter

###################
### Cell Data ###
###################
nortek_dvl_ethernet/NortekCPCell[] cells
