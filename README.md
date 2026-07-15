# Nortek DVL(1000) Ethernet ROS driver.
It will publish the bottom track, water track and current profile data.

## Usage:
```sh
cd ~/your_ws/src
git clone -b ros2-devel https://github.com/GSO-soslab/nortek_dvl_ethernet
cd ~/your_ws
colcon build --packages-select nortek_dvl_ethernet
ros2 launch nortek_dvl_ethernet driver.launch.py
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `bottom_track` | `nortek_dvl_ethernet::NortekDF2` | Bottom track raw data (DF21) |
| `water_track` | `nortek_dvl_ethernet::NortekDF2` | Water track raw data (DF22) |
| `current_profile` | `nortek_dvl_ethernet::NortekDF3` | Current profile raw data (DF3) |
| `bt_velocity` | `geometry_msgs::TwistWithCovarianceStamped` | Bottom track velocity for navigation |
| `bt_pointcloud` | `sensor_msgs::PointCloud2` | Bottom track point cloud from 4 beams |
| `bt_altitude` | `sensor_msgs::PointStamped` | Bottom track altitude (averaged from 4 beams) |
| `wt_velocity` | `geometry_msgs::TwistWithCovarianceStamped` | Water track velocity for navigation |
| `pressure` | `sensor_msgs::FluidPressure` | Pressure data (Bar to Pascal conversion) |

## Configuration

| Parameter | Default | Description |
|-----------|---------|-------------|
| `udp_rx` | `9004` | UDP receive port |
| `udp_address` | `192.168.2.110` | DVL IP address |
| `buffer_size` | `512` | UDP buffer size |
| `beam_angle` | `25.0` | DVL beam angle (degrees) |
| `sound_speed` | `1500.0` | Sound speed (m/s) |
| `fluid_density` | `1023.0` | Fluid density (kg/m³) |
| `sensor_frame_id` | `nortek_dvl` | Frame ID for sensor messages |
| `world_frame_id` | `world` | World frame ID for odometry |
| `pressure_frame_id` | `nortek_dvl_pressure` | Frame ID for pressure sensor |
| `depth_cov` | `0.001` | Depth covariance |

## Acknowledgment
This package is inspired by [DVL driver](https://bitbucket.org/whoidsl/) from WHOI Deep Submergence Lab.
