 # Copyright (C) 2024 David Skuddis - All Rights Reserved
 # You may use, distribute and modify this code under the
 # terms of the MIT license.
 #
 # You should have received a copy of the MIT license with
 # this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/



cmake_minimum_required(VERSION 3.0.2)
project(dmsa_slam_ros)

set(CMAKE_BUILD_TYPE release)

set(CMAKE_CXX_FLAGS_RELEASE "-O1")

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  pcl_conversions)

find_package(Boost 1.40.0 COMPONENTS filesystem system iostreams REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.3 REQUIRED COMPONENTS)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  message("OPENMP Found")
else()
  message("OPENMP NOT Found")
endif()

catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs tf pcl_conversions sensor_msgs rosbag
)

include_directories(
        include
        Thirdparty
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(dmsa_slam_ros
               src/main.cpp
               src/dmsa_slam_ros.cpp
               )


target_link_libraries(dmsa_slam_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
