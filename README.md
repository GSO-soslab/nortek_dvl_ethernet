# Nortek DVL(1000) Ethernet ROS driver.
It will publish the bottom track, water track and current profile data.

## Usage:
```sh
cd ~/your_ws/src
git clone -b noetic-devel https://github.com/GSO-soslab/nortek_dvl_ethernet
cd ~/your_ws
catkin build nortek_dvl_ethernet
roslaunch nortek_dvl_ethernet driver.launch
```

## Introduction:

- bottom track raw data 
    - topic: "bottom_track"
    - type: nortek_dvl_ethernet::NortekDF2
    - info: DF21 data structure
- water track raw data
    - topic: "water_track"
    - type: nortek_dvl_ethernet::NortekDF2
    - info: DF22 data structure
- current profile raw data
    - topic: "current_profile"
    - type: nortek_dvl_ethernet::NortekDF3
    - info: DF3 data structure
- bottom track velocity
    - topic: "bt_velocity"
    - type: geometry_msgs::TwistWithCovarianceStamped
    - info: the derived msg from raw bottom track data, used for navigation
- bottom track point cloud
    - topic: "bt_pointcloud"
    - type: sensor_msgs::PointCloud2
    - info: the derived msg from raw bottom track data, it's from 4 range measurement
- bottom track altitude
    - topic: "bt_altitude"
    - type: sensor_msgs::PointStamped
    - info: the derived msg from raw bottom track data, it's simpled averaged from 4 range measurement
- water track velocity
    - topic: "wt_velocity"
    - type: geometry_msgs::TwistWithCovarianceStamped
    - info: the derived msg from raw water track data, used for navigation
- pressure
    - topic: "pressure"
    - type: sensor_msgs::FluidPressure
    - info: the derived msg from raw bottom track and current profile data, Nortek output the gauge pressure and unit in Bar. FluidPressure need absolute pressure unit in Pascal.

## Acknowledgment
This package is inspired by [DVL driver](https://bitbucket.org/whoidsl/) from WHOI Deep Submergence Lab.
