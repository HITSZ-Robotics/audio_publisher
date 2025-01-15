cmake_minimum_required(VERSION 3.0.2)
project(audio_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
)

add_executable(audio_publisher_node_main src/audio_publisher_node.cpp src/audio_publisher_node_main.cpp)

target_link_libraries(audio_publisher_node_main
  ${catkin_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
)

install(TARGETS audio_publisher_node_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
