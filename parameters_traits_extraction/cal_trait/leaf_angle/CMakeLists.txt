cmake_minimum_required(VERSION 3.5 FATAL_ERROR)


project(width)


find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


add_executable (angle leaf_angle.cpp)

target_link_libraries (leaf_angle ${PCL_LIBRARIES})

