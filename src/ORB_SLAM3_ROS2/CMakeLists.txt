cmake_minimum_required(VERSION 3.5)
project(orbslam3)

# 1. Standard C++14 (Required by ORB-SLAM3)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 2. Set the Path to your existing ORB-SLAM3 "Engine"
# Adjust this path if your workspace structure is different
set(ORB_SLAM3_ROOT_DIR /home/roverto/ros2_ws/src/ros2_orb_slam3/orb_slam3)

# 3. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED) # Added for Pose publishing
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

# 4. Include Directories
include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}    # Added this
  ${sensor_msgs_INCLUDE_DIRS}  # Added this
  ${geometry_msgs_INCLUDE_DIRS} # Added this
)
# 5. Link to the ORB-SLAM3 Shared Libraries
# This tells the wrapper to use the .so files you already compiled
link_directories(
  ${ORB_SLAM3_ROOT_DIR}/lib
)

# 6. Define the Monocular Executable
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

# 7. Target Dependencies (Crucial for Jazzy/Ubuntu 24.04)
# Note: We link ORB_SLAM3 and Thirdparty libs manually if not using a system-wide install
target_link_libraries(mono
  /home/roverto/ros2_ws/install/ros2_orb_slam3/lib/ros2_orb_slam3/liborb_slam3_lib.so
  ${Pangolin_LIBRARIES}
  ${OpenCV_LIBS}
)

ament_target_dependencies(mono 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  geometry_msgs 
  message_filters
)

# 8. Installation
install(TARGETS mono
  DESTINATION lib/${PROJECT_NAME})

ament_package()
