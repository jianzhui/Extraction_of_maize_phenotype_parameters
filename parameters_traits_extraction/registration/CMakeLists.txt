cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(icp5)
 
find_package(PCL 1.3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)  # 添加对 Boost 库的查找
 
include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})  # 包含 Boost 头文件目录
 
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})  # 添加 Boost 库文件目录
 
add_definitions(${PCL_DEFINITIONS})
 
add_executable(icp5 icp5.cpp)
 
target_link_libraries(icp5 ${PCL_LIBRARIES} ${Boost_LIBRARIES})  # 链接 Boost 库

