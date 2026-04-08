cmake_minimum_required(VERSION 3.0)
project(DBoW2)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
)

set(SOURCES
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp
  DUtils/Timestamp.cpp
)

add_library(DBoW2 SHARED ${SOURCES})
target_link_libraries(DBoW2 ${OpenCV_LIBS})
