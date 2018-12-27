cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_test1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_test1 pcl_test1.cpp)
target_link_libraries (pcl_test1 ${PCL_LIBRARIES})
