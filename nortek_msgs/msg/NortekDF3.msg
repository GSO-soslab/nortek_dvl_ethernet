#################
## Header data ##
#################
std_msgs/Header header
builtin_interfaces/Time io_time
builtin_interfaces/Time system_time

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
NortekProfileConfig configuration
uint32 serial_number

###################
### Sensor Data ###
###################
uint8 year
uint8 month
uint8 day
uint8 hour
uint8 minute
uint8 second
uint16 micro_second
float32 speed_sound                      #[m/s]
float32 temperature                      #[Degree Celsius]
float32 pressure                         #[Bar]
float32 heading                          #[Degree]          
float32 pitch                            #[Degree]
float32 roll                             #[Degree]
NortekProfileBeam beam_system
float32 cell_size                        #[m]
float32 blanking                         #[m]
uint8 nominal_correlation                #[%]
float32 pressure_temperature             #[Degree Celsius]
float32 battery                          #[Voltage]
int16[3] mag_3d
int16[3] acc_3d                          #[16384 = 1.0]
float32 ambiguity_velocity               # [m/s], corrected for sound velocity, scaled according to Velocity Scaling
NortekProfileDataset dataset_description
uint16 transmit_energy
int8 velocity_scale
float32 power_level                      #[dB]
int16 mag_temperature                    #[uncalibrated]
int16 rtc_temperature                    #[Degree Celsius]
uint16 error
NortekProfileStatus0 status0
NortekProfileStatus status
uint32 ensemble_counter

###################
### Cell Data ###
###################
NortekProfileCell[] cells
