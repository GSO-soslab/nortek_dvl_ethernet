########################################################################
################################ Header ################################
########################################################################
# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header #io_time
float64 dvl_time

########################################################################
############################### Setting ################################
########################################################################
# ENUM: Velocity mode
uint8 DVL_MODE_BOTTOM=1
uint8 DVL_MODE_WATER=2
# ENUM: Coordinates system 
uint8 DVL_COORD_BEAM=0
uint8 DVL_COORD_INSTRUMENT=1
uint8 DVL_COORD_SHIP=2
uint8 DVL_COORD_EARTH=3
# ENUM: DVL Type
uint8 DVL_TYPE_PISTON=0       # Piston transducer for each beam, e.g. Sonardyne Syrinx, Nortek DVLs, and RDI Workhorse
uint8 DVL_TYPE_PHASED_ARRAY=1 # Newfangled phased-array DVL, e.g. RDI pathfinder, pioneer, and tasman

########################################################################
uint8 dvl_type                # Type of DVL array employed (see above).  Necessary for soundspeed corrections
uint8 velocity_mode           # Velocity mode (see above)
uint8 coordinate_mode         # Coordinate system mode (see above)

########################################################################
################################ Data ##################################
########################################################################
geometry_msgs/Vector3 velocity # Velocities [m/s]
geometry_msgs/Vector3 quality  # XYZ FOM, Measure of the velocity white noise level [m/s]
float64[4] range               # the range from beam to target 
float64 avg_altitude           # Average of all good beam ranges: altitude = sum(range) / num_good_beams
float64 course_gnd             # Calculated from xy velocities: course_gnd = atan2(velocity.y, velocity.x)
float64 speed_gnd              # Calculated from xy velocities: speed_gnd = sqrt(velocity.x^2 + velocity.y^2)

########################################################################
############################ Property ##################################
########################################################################
float32 speed_sound            # Speed of sound (m/s) used by the DVL
float32 temperature            # DVL measured temperature
uint8 num_good_beams           # only use data when all beams are good (num_good_beams = 4) ? 
