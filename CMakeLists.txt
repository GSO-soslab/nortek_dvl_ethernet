cmake_minimum_required(VERSION 3.0.2)
project(nortek_dvl_ethernet)


if(CMAKE_BUILD_TYPE STREQUAL "Debug") 	
  add_definitions(-DDEBUG) 
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads)
find_package(Eigen3 REQUIRED)

file(GLOB MSG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/**/*.msg)

foreach(I IN ITEMS ${MSG_FILES})
  get_filename_component(ITEM_NAME ${I} NAME)
  add_message_files(FILES ${ITEM_NAME})
endforeach()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nortek_dvl_ethernet
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node
  include/nortek_dvl_ethernet/udp_socket_handler.cpp
  include/nortek_dvl_ethernet/parser.cpp
  include/nortek_dvl_ethernet/ros_driver.cpp
  src/node.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)