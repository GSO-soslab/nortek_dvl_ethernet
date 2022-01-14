cmake_minimum_required(VERSION 2.8.3)
project(ds_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ds_asio
  ds_param
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(ClangFormat)
if(CLANG_FORMAT_FOUND)
    add_custom_target(clang-format-ds-base
            COMMENT
            "Run clang-format on all project C++ sources"
            WORKING_DIRECTORY
            ${PROJECT_SOURCE_DIR}
            COMMAND
            find src
            include/ds_base
            -iname '*.h' -o -iname '*.cpp'
            | xargs ${CLANG_FORMAT_EXECUTABLE} -i
            )
endif(CLANG_FORMAT_FOUND)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
        FILES
        StringCommand.srv
        BoolCommand.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ds_base
  CATKIN_DEPENDS ds_asio ds_param
  # DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}

  include/ds_base/ds_global.h

  include/ds_base/util.h
  src/ds_base/util.cpp

  include/ds_base/ds_process.h
  src/ds_base/ds_process.cpp
  src/ds_base/ds_process_private.h
  src/ds_base/ds_process_private.cpp

  include/ds_base/ds_bus.h
  src/ds_base/ds_bus_private.h
  src/ds_base/ds_bus.cpp

  include/ds_base/ds_bus_device.h
  src/ds_base/ds_bus_device_private.h
  src/ds_base/ds_bus_device.cpp
  src/ds_base/ds_bus_device_private.cpp

  include/ds_base/sensor_base.h
  src/ds_base/sensor_base.cpp
  src/ds_base/sensor_base_private.h

)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ds_bus
        src/ds_bus_node.cpp)
target_link_libraries(ds_bus ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(ds_bus ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ds_base.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test_ds_process
            test/ds_process.test
            # src/test/utils.h
            # src/test/utils.cpp
            src/test/test_ds_process.cpp
            )
    target_link_libraries(test_ds_process ${PROJECT_NAME})
endif(CATKIN_ENABLE_TESTING)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
