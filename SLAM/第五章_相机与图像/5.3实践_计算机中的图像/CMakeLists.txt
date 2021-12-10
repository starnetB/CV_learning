# 添加C++ 11 标准支持
#cmset(CMAKE_CXX_FLAGS = "-std=c++11")
project(imageBasics)
cmake_minimum_required(VERSION 3.14)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)

# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics src/imageBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBRARIES})

add_executable(undistorImage src/undistorImage.cpp)
target_link_libraries(undistorImage ${OpenCV_LIBRARIES})