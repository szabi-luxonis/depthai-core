cmake_minimum_required(VERSION 2.8.3)
project(depthai_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs cv_bridge
    message_generation std_msgs)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

#find_package(depthai REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(FILES 
    CameraInfo.msg
    Entry.msg
    MetaOut.msg)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    std_msgs
    sensor_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  LIBRARIES
  CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Activate c++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()	
    message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} /usr/local/include/depthai)

link_directories(/usr/lib /usr/local/lib)

# Declare a C++ executable
add_executable(depthai_node
    src/depthai_node_base.cpp
    src/depthai_node.cpp
)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(depthai_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
    depthai ${PROJECT_NAME}_gencfg)

# Specify libraries to link a library or executable target against
target_link_libraries(depthai_node ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${OpenCV_LIBS} depthai usb-1.0)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS depthai_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES data/default.calib
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(FILES data/object_detection.json
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(FILES data/mobilenet_ssd.blob
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

install(FILES
    launch/depthai.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
