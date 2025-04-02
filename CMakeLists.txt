cmake_minimum_required(VERSION 3.0.2)
project(multibotnet)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_srvs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_library(ZMQ_LIBRARY NAMES zmq)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs nav_msgs std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_topic_node
  src/main_topic.cpp
  src/zmq_manager.cpp
)

add_executable(${PROJECT_NAME}_service_node
  src/main_service.cpp
  src/service_manager.cpp
)

target_link_libraries(${PROJECT_NAME}_topic_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${ZMQ_LIBRARY}
)

target_link_libraries(${PROJECT_NAME}_service_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${ZMQ_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}_topic_node ${PROJECT_NAME}_service_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
