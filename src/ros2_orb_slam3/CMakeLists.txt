cmake_minimum_required(VERSION 3.10)
project(ros2_orb_slam3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

# 2. Define the Library First
add_library(orb_slam3_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/Thirdparty/DBoW2/DUtils/Random.cpp
  orb_slam3/Thirdparty/DBoW2/DUtils/Timestamp.cpp
  orb_slam3/Thirdparty/DBoW2/DUtils/Random.h
)

# 3. Define the Executable Next
add_executable(mono_node_cpp
  src/mono_example.cpp
  src/common.cpp
)

# 4. NOW set the includes (Global for the whole project)
target_include_directories(orb_slam3_lib PRIVATE
  include
  orb_slam3
  orb_slam3/include
  orb_slam3/include/CameraModels
  orb_slam3/Thirdparty
  orb_slam3/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2
)

target_include_directories(mono_node_cpp PRIVATE
  include
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${cv_bridge_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2
)

# 5. Link everything (Plain style)
ament_target_dependencies(orb_slam3_lib
  rclcpp
  cv_bridge
  sensor_msgs
)

target_link_libraries(orb_slam3_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto 
)

ament_target_dependencies(mono_node_cpp
  rclcpp
  cv_bridge
  sensor_msgs
  image_transport
)

target_link_libraries(mono_node_cpp orb_slam3_lib)

# 6. Installation
# Install the C++ Compiled Targets
install(TARGETS mono_node_cpp orb_slam3_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python Scripts
install(PROGRAMS
  scripts/mono_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install folders (Vocabulary and Settings)
# This is often needed so the node can find the .txt and .yaml files
install(DIRECTORY orb_slam3/Vocabulary
  DESTINATION share/${PROJECT_NAME}
)

ament_package()