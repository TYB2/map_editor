cmake_minimum_required(VERSION 2.8.3)

# Create Project - set the project name and version
project(map_editor)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

# Find Packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             geometry_msgs
             sensor_msgs
             nav_msgs
             visualization_msgs
             tf
             message_filters
             )   

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

# Include header files
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(map_editor
              include/mapping.h
              src/mapping.cpp
              src/main.cpp
             )


target_link_libraries(map_editor 
                      ${catkin_LIBRARIES}
                      Eigen3::Eigen
                      )