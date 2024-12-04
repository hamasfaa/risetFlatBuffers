cmake_minimum_required(VERSION 3.0.2)
project(robot)

# Specify the directory where live555 is located
set(LIVE555_DIR /home/hamasfa/Downloads/live)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  turtlesim
  message_generation
)

# Find pkg-config for GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_RTSP REQUIRED gstreamer-rtsp-server-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  PC2BS.msg
  BS2PC.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot
  CATKIN_DEPENDS roscpp turtlesim
  DEPENDS system_lib
)

# Include directories for your package, including live555 and GStreamer
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/hamasfa/flatbuffers/include
  /usr/local/include/liveMedia
  /usr/local/include/UsageEnvironment
  /usr/local/include/groupsock
  /usr/local/include/BasicUsageEnvironment
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_RTSP_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
)

link_directories(${GSTREAMER_LIBRARY_DIRS} ${GSTREAMER_RTSP_LIBRARY_DIRS} ${GSTREAMER_APP_LIBRARY_DIRS})

# Add definitions for GStreamer
add_definitions(${GSTREAMER_CFLAGS_OTHER})

# Add executable targets
add_executable(tes_node src/tes.cpp)
add_executable(comm_node src/comm.cpp)
add_executable(RTSP_node src/commRTSP.cpp)
add_executable(recvRTSP_node src/recvRTSP.cpp)
# add_executable(555_node src/tes555.cpp)

# Link libraries for each executable
target_link_libraries(tes_node
  ${catkin_LIBRARIES}
)

target_link_libraries(comm_node
  ${catkin_LIBRARIES}
  flatbuffers
)

target_link_libraries(RTSP_node
  ${catkin_LIBRARIES}
  flatbuffers
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_RTSP_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)

target_link_libraries(recvRTSP_node
  ${catkin_LIBRARIES}
  flatbuffers
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_RTSP_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
)
# Link Live555 libraries
# target_link_libraries(555_node
#   ${catkin_LIBRARIES}
#   /usr/local/lib/libliveMedia.a
#   /usr/local/lib/libgroupsock.a
#   /usr/local/lib/libUsageEnvironment.a
#   /usr/local/lib/libBasicUsageEnvironment.a
# )

