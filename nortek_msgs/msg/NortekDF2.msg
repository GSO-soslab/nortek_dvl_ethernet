# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
builtin_interfaces/Time io_time
builtin_interfaces/Time system_time

################
# DF21/22 Mode #
################

#  All distances are in m, velocities in m/s, intensities in dB
###################
# Information Data #
###################

# Standard array with one piston transducer for each beam, 
# e.g., Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
uint8 DVL_TYPE_PISTON=0
# Newfangled phased-array DVL, e.g., RDI DVLs
uint8 DVL_TYPE_PHASED_ARRAY=1

uint8 dvl_type
uint8 version
uint8 data_offset
uint32 serial_num
uint8 year
uint8 month
uint8 day
uint8 hour
uint8 minute
uint8 second
uint16 micro_second
uint16 beam_num
uint32 error
NortekTrackStatus status
float32 speed_sound
float32 temperature
float32 pressure

#############
# Beam Data #
#############
float32[4] beam_vel  #Velocities for each beam
float32[4] beam_dist #Distances for each beam, vertical distance
float32[4] beam_fom  #Figure of merit for each beam, Measure of the velocity white noise level 
float32[4] beam_dt1  #DT1 for each beam
float32[4] beam_dt2  #DT2 for each beam
float32[4] beam_dura #Duration of velocity estimate for each beam

#############
# XYZ Data #
#############
float32 vel_x         #Velocity X
float32 vel_y         #Velocity Y
float32 vel_z1        #Velocity Z1
float32 vel_z2        #Velocity Z2
float32 fom_x         #Figure of Merit X
float32 fom_y         #Figure of Merit Y
float32 fom_z1        #Figure of Merit Z1
float32 fom_z2        #Figure of Merit Z2
float32 time_diff1_x  #Time from trigger to center of bottom echo
float32 time_diff1_y  #Same as above
float32 time_diff1_z1 #Same as above
float32 time_diff1_z2 #Same as above
float32 time_diff2_x  #Time from start of NMEA output msg to center of bottom echo
float32 time_diff2_y  #Same as above
float32 time_diff2_z1 #Same as above
float32 time_diff2_z2 #Same as above
float32 time_dura_x   #Duration of velocity estimate for each component
float32 time_dura_y   #Same as above
float32 time_dura_z1  #Same as above
float32 time_dura_z2  #Same as above

#################
# processed data #
#################
uint8 good_beams
float64 altitude # roughly measurement, averaged from 4 beams
float64 speed_gnd
float64 course_gnd


