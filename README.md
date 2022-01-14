# Nortek DVL(1000) Ethernet ROS driver.

## Test:
- Tested in Ubuntu 20.04, Ubuntu 18.04(Jetson ARM)

## Acknowledgment

This package is based on [ds_sensors](https://bitbucket.org/whoidsl/) from WHOI Deep Submergence Lab. They do really good job. Right now, still in ds_sensors mode. Another foked [original branch](https://github.com/GSO-soslab/whoi_ds)

## Modification
- add current profile
- added/changed msg files
- delete some files trying to make this only for Nortek DVL

## Build

```
catkin build ds_sensors
```

## Launch files

```
roslaunch ds_sensors nortekdvl1000.launch
```

## TODOs:
- change current profile cell size based on data, right now just manually set to **20**
