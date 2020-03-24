project(libdepthaitest)

cmake_minimum_required(VERSION 2.8.12)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++17")   
endif(CMAKE_COMPILER_IS_GNUCXX)

include_directories(
    .
    /usr/local/include/depthai
)

add_executable( libdepthaitest
    test.cpp
)

target_link_libraries( libdepthaitest
  depthai
  usb-1.0
  pthread
)
