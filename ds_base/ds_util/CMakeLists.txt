cmake_minimum_required(VERSION 2.8.3)
project(ds_util)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(INSERT CMAKE_MODULE_PATH 0 ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(ClangFormat)
if(CLANG_FORMAT_FOUND)
    add_custom_target(clang-format-ds-util
            COMMENT
            "Run clang-format on all project C++ sources"
            WORKING_DIRECTORY
            ${PROJECT_SOURCE_DIR}
            COMMAND
            find src/ds_util
            src/test
            include/ds_util
            -iname '*.h' -o -iname '*.cpp'
            | xargs ${CLANG_FORMAT_EXECUTABLE} -i
            )
endif(CLANG_FORMAT_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
   LIBRARIES ds_util
   CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/ds_util.h

  include/${PROJECT_NAME}/int_to_hex.h

  include/${PROJECT_NAME}/bcd_to_int.h
  src/${PROJECT_NAME}/bcd_to_int.cpp

  include/${PROJECT_NAME}/float_round.h
  src/${PROJECT_NAME}/float_round.cpp

  include/${PROJECT_NAME}/fofonoff_depth.h
  src/${PROJECT_NAME}/fofonoff_depth.cpp

  include/${PROJECT_NAME}/thruster.h
  src/${PROJECT_NAME}/thruster.cpp

  include/${PROJECT_NAME}/angle.h
  src/${PROJECT_NAME}/angle.cpp

  include/${PROJECT_NAME}/reference_smoothing.h
  src/${PROJECT_NAME}/reference_smoothing.cpp

  include/${PROJECT_NAME}/dynamics.h
  src/${PROJECT_NAME}/dynamics.cpp

  include/${PROJECT_NAME}/median.h
  src/${PROJECT_NAME}/median.cpp

  include/${PROJECT_NAME}/limit_change.h
  src/${PROJECT_NAME}/limit_change.cpp

  include/${PROJECT_NAME}/time_validate.h
  src/${PROJECT_NAME}/time_validate.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ds_util_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ds_util.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_fofonoff_depth src/test/test_fofonoff_depth.cpp)
    target_link_libraries(test_fofonoff_depth ${PROJECT_NAME})

    catkin_add_gtest(test_angle src/test/test_angle.cpp)
    target_link_libraries(test_angle ${PROJECT_NAME})

    catkin_add_gtest(test_int2hex src/test/test_int_to_hex.cpp)
    target_link_libraries(test_int2hex ${PROJECT_NAME})

    catkin_add_gtest(test_reference_smoothing src/test/test_reference_smoothing.cpp)
    target_link_libraries(test_reference_smoothing ${PROJECT_NAME})

    catkin_add_gtest(test_float_round src/test/test_float_round.cpp)
    target_link_libraries(test_float_round ${PROJECT_NAME})

    catkin_add_gtest(test_bcd_to_int src/test/test_bcd_to_int.cpp)
    target_link_libraries(test_bcd_to_int ${PROJECT_NAME})

    catkin_add_gtest(test_limit_change src/test/test_limit_change.cpp)
    target_link_libraries(test_limit_change ${PROJECT_NAME})

    catkin_add_gtest(test_time_validate src/test/test_time_validate.cpp)
    target_link_libraries(test_time_validate ${PROJECT_NAME})

endif(CATKIN_ENABLE_TESTING)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
