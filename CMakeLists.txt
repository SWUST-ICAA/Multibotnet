cmake_minimum_required(VERSION 3.0.2)
project(multibotnet)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  topic_tools
  roslib
)

## Find system dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

## Find ZeroMQ
find_library(ZMQ_LIBRARY NAMES zmq)
if(NOT ZMQ_LIBRARY)
  message(FATAL_ERROR "ZeroMQ library not found")
endif()

## Find LZ4 (for compression)
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(LZ4 liblz4)
  if(LZ4_FOUND)
    set(LZ4_LIBRARY ${LZ4_LIBRARIES})
    set(LZ4_INCLUDE_DIRS ${LZ4_INCLUDE_DIRS})
    link_directories(${LZ4_LIBRARY_DIRS})
  endif()
endif()

if(NOT LZ4_FOUND)
  find_library(LZ4_LIBRARY NAMES lz4)
  find_path(LZ4_INCLUDE_DIRS NAMES lz4.h)
endif()

if(LZ4_LIBRARY AND LZ4_INCLUDE_DIRS)
  message(STATUS "Found LZ4: ${LZ4_LIBRARY}")
else()
  message(WARNING "LZ4 library not found, compression support will be limited")
  set(LZ4_LIBRARY "")
endif()

## Find zlib (for compression)
find_package(ZLIB)
if(NOT ZLIB_FOUND)
  message(WARNING "zlib not found, compression support will be limited")
endif()

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_transport
    ${PROJECT_NAME}_managers
    ${PROJECT_NAME}_utils
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    geometry_msgs 
    sensor_msgs 
    nav_msgs 
    std_srvs
    topic_tools
    roslib
  DEPENDS
    YAML_CPP
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Add compile definitions
if(ZLIB_FOUND)
  add_definitions(-DHAVE_ZLIB)
  include_directories(${ZLIB_INCLUDE_DIRS})
endif()

if(LZ4_LIBRARY AND LZ4_INCLUDE_DIRS)
  add_definitions(-DHAVE_LZ4)
  include_directories(${LZ4_INCLUDE_DIRS})
endif()

## Core library
add_library(${PROJECT_NAME}_core
  src/core/message_factory.cpp
  src/core/service_factory.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## Transport library
add_library(${PROJECT_NAME}_transport
  src/transport/zmq_transport.cpp
  src/transport/compression.cpp
  src/transport/connection_pool.cpp
)
target_link_libraries(${PROJECT_NAME}_transport
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARY}
  ${ZLIB_LIBRARIES}
  ${LZ4_LIBRARY}
)

## Utils library
add_library(${PROJECT_NAME}_utils
  src/utils/thread_pool.cpp
  src/utils/logger.cpp
  src/utils/config_parser.cpp
)
target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## Managers library
add_library(${PROJECT_NAME}_managers
  src/managers/topic_manager.cpp
  src/managers/service_manager.cpp
)
target_link_libraries(${PROJECT_NAME}_managers
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_transport
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARY}
)

## Topic node executable
add_executable(${PROJECT_NAME}_topic_node
  src/nodes/topic_node.cpp
)
target_link_libraries(${PROJECT_NAME}_topic_node
  ${PROJECT_NAME}_managers
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_transport
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${ZMQ_LIBRARY}
)

## Service node executable
add_executable(${PROJECT_NAME}_service_node
  src/nodes/service_node.cpp
)
target_link_libraries(${PROJECT_NAME}_service_node
  ${PROJECT_NAME}_managers
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_transport
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${ZMQ_LIBRARY}
)

## Add cmake target dependencies of the executables
add_dependencies(${PROJECT_NAME}_topic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_service_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install
install(TARGETS 
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_transport
  ${PROJECT_NAME}_managers
  ${PROJECT_NAME}_utils
  ${PROJECT_NAME}_topic_node 
  ${PROJECT_NAME}_service_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add test files here
endif()