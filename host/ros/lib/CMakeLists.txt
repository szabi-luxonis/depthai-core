if (CMAKE_VERSION VERSION_LESS 3.0)
  cmake_policy(SET CMP0048 OLD)
  project(depthai)
else()
  cmake_policy(SET CMP0048 NEW)
  project(depthai VERSION "1.0.1")
endif()

cmake_minimum_required(VERSION 2.8.12)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(dldt_dir ../../../shared/3rdparty/dldt)

add_definitions(-D__PC__)
add_definitions(-DUSE_USB_VSC) # for XLink communication
add_definitions(-DXLINK_USB_DATA_TIMEOUT=0)
add_definitions(-DXLINK_COMMON_TIMEOUT_MSEC=0)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++17")   
endif(CMAKE_COMPILER_IS_GNUCXX)

include(${dldt_dir}/inference-engine/thirdparty/movidius/XLink/XLink.cmake)

add_library(depthai STATIC
    src/depthai.cpp
    ../../core/pipeline/host_pipeline.cpp
    ../../core/pipeline/host_pipeline_config.cpp
    ../../core/pipeline/cnn_host_pipeline.cpp
    ../../core/types.cpp
    ../../core/host_json_helper.cpp
    ../../core/host_data_reader.cpp
    ../../core/disparity_stream_post_processor.cpp
    ../../core/device_support_listener.cpp
    ../../../shared/json_helper.cpp
    ../../../shared/stream/stream_info.cpp
    ../../../shared/xlink/xlink_wrapper.cpp
    ../../../shared/disparity_luts.cpp
	${XLINK_SOURCES}
)

#set_target_properties(depthai PROPERTIES SOVERSION 1)

set_target_properties(depthai PROPERTIES PUBLIC_HEADER "include/depthai.hpp")

target_include_directories(depthai PRIVATE
    ../../core/
    ../../../shared/
    ${XLINK_INCLUDE_DIRECTORIES}
    ../../../shared/3rdparty/boost_1_71_0/
    ../../../shared/3rdparty/json/include/
    ../../py_module/
    ../../core/nnet/
    ../../core/pipeline/
    include/)

# nlohman JSON
set(nlohmann_json_DIR ../../../shared/3rdparty/json)

include_directories(${nlohmann_json_DIR}/include/)
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/${nlohmann_json_DIR} ${CMAKE_CURRENT_BINARY_DIR}/json)

# nlohman JSON validator
set(BUILD_TESTS OFF CACHE INTERNAL "")
set(BUILD_EXAMPLES OFF CACHE INTERNAL "")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/3rdparty/json-schema-validator/ ${CMAKE_CURRENT_BINARY_DIR}/json-schema-validator)


# for commit hash
# TODO: maybe we shoud add it into another .cmake and than include here?
find_package(Git)

set(commit_version "unknown")

if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY "${local_dir}"
    OUTPUT_VARIABLE commit_version
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
else()
  message(STATUS "GIT module not found")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/../../../shared/version.hpp @ONLY)


# link libraries
target_link_libraries(depthai
    usb-1.0
    pthread
    nlohmann_json::nlohmann_json
    nlohmann_json_schema_validator
)

include(GNUInstallDirs)

set(DEPTHAI_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/depthai")

install(TARGETS depthai
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${DEPTHAI_INSTALL_INCLUDEDIR})

install(
	DIRECTORY "../../core/"
  DESTINATION ${DEPTHAI_INSTALL_INCLUDEDIR}
	FILES_MATCHING 
	PATTERN "*.hpp"
	PATTERN "*.h"
	PATTERN "3rdparty" EXCLUDE)

install(
	DIRECTORY "../../../shared/general/"
  DESTINATION "${DEPTHAI_INSTALL_INCLUDEDIR}/general"
	FILES_MATCHING 
	PATTERN "*.hpp"
	PATTERN "*.h")

install(
	DIRECTORY "../../../shared/stream/"
  DESTINATION "${DEPTHAI_INSTALL_INCLUDEDIR}/stream"
	FILES_MATCHING 
	PATTERN "*.hpp"
	PATTERN "*.h")

install(
	DIRECTORY "../../../shared/"
    DESTINATION "${DEPTHAI_INSTALL_INCLUDEDIR}"
	FILES_MATCHING 
	PATTERN "*.hpp"
  PATTERN "*.h"
  PATTERN "3rdparty" EXCLUDE
  PATTERN "general" EXCLUDE
  PATTERN "logger" EXCLUDE
  PATTERN "stream" EXCLUDE
  PATTERN "xlink" EXCLUDE)


configure_file(depthai.pc.in depthai.pc @ONLY)

install(FILES ${CMAKE_BINARY_DIR}/depthai.pc DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
