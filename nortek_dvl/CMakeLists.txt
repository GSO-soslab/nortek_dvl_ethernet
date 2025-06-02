cmake_minimum_required(VERSION 3.8)
project(nortek_dvl)

####################################################################
## setup compiler                    
####################################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

####################################################################
## setup packages                    
####################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nortek_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

####################################################################
## build                  
####################################################################

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  nortek_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR} 
)

set(library_name ${PROJECT_NAME}_lib)
set(node_name ${PROJECT_NAME}_node)

# add library
add_library(${library_name} SHARED 
  include/nortek_dvl/udp_socket_handler.cpp
  include/nortek_dvl/parser.cpp
  include/nortek_dvl/ros_driver.cpp  
)
target_link_libraries(${library_name} ${Boost_LIBRARIES})
ament_target_dependencies(${library_name} ${dependencies})

# add exe
add_executable(${node_name} src/node.cpp)
target_link_libraries(${node_name} ${library_name} ${Boost_LIBRARIES})
ament_target_dependencies(${node_name} ${dependencies})

####################################################################
## install all the files                    
####################################################################

# lib
install(TARGETS 
  ${node_name} 
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# include
install(DIRECTORY include/
  DESTINATION include/
)

# launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# config
install(DIRECTORY 
  config 
  DESTINATION share/${PROJECT_NAME}
)

####################################################################
## export                    
####################################################################

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
