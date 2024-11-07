cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(read_cloud)

# 设置 LIBFREENECT2 的路径
set(LIBFREENECT2_INCLUDE_DIR "/home/jianzhui/libfreenect2/include/libfreenect2")

# 添加 LIBFREENECT2 的库文件路径
link_directories("/usr/lib")

# 查找 PCL 库
find_package(PCL 1.3 REQUIRED)

# 查找 OpenCV 库
find_package(OpenCV 4 REQUIRED)

# 包含 LIBFREENECT2、PCL 和 OpenCV 的头文件目录
include_directories(${PCL_INCLUDE_DIRS} ${LIBFREENECT2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

# 添加编译选项
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(read read.cpp)

# 链接 PCL、LIBFREENECT2 和 OpenCV 库
target_link_libraries(read ${PCL_LIBRARIES} ${OpenCV_LIBS} freenect2)  # 这里使用 freenect2
